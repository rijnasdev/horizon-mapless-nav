#!/usr/bin/env python3
"""
decision_node.py — Reactive mapless navigation node for ROS2.

Subscribes to a depth image, splits it into left/center/right regions,
and publishes /cmd_vel using a simple reactive policy with smoothing,
stuck detection, and an emergency-stop safety layer.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from cv_bridge import CvBridge
import numpy as np


class DecisionNode(Node):

    def __init__(self):
        super().__init__("decision_node")

        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge()

        # --- QoS ---
        # Depth images: best-effort, small queue (sensor-like data, we only
        # care about the latest frame).
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Commands: reliable, small queue.
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", cmd_qos)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            sensor_qos,
        )
        self.estop_sub = self.create_subscription(
            Bool, "/emergency_stop", self.estop_callback, 10
        )

        self.control_timer = self.create_timer(
            self.control_period, self.publish_velocity
        )
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        # --- State ---
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.estop_active = False
        self.last_depth_time = None
        self.last_progress_time = time.time()
        self.last_center_distance = math.inf

        self.get_logger().info(
            f"Decision Node started (safe_distance={self.safe_distance}, "
            f"depth_topic={self.depth_topic})"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("safe_distance", 1.0)      # m, stop/turn threshold
        self.declare_parameter("danger_distance", 0.4)    # m, triggers recovery
        self.declare_parameter("max_linear_speed", 0.40)  # m/s
        self.declare_parameter("max_angular_speed", 0.60) # rad/s
        self.declare_parameter("recover_linear", -0.15)
        self.declare_parameter("recover_angular", 0.8)
        self.declare_parameter("control_period", 0.1)     # s (10 Hz)
        self.declare_parameter("linear_accel_limit", 0.5) # m/s^2 per control step (smoothing)
        self.declare_parameter("angular_accel_limit", 2.0)
        self.declare_parameter("roi_top", 0.45)            # fraction of image height
        self.declare_parameter("roi_bottom", 0.75)
        self.declare_parameter("depth_timeout", 0.5)       # s, no data -> stop
        self.declare_parameter("stuck_timeout", 3.0)       # s of no forward progress
        self.declare_parameter("min_valid_range", 0.1)     # m, filters sensor noise

        self.add_on_set_parameters_callback(self._on_params_changed)

    def _load_parameters(self):
        p = self.get_parameter
        self.depth_topic = p("depth_topic").value
        self.safe_distance = p("safe_distance").value
        self.danger_distance = p("danger_distance").value
        self.max_linear_speed = p("max_linear_speed").value
        self.max_angular_speed = p("max_angular_speed").value
        self.recover_linear = p("recover_linear").value
        self.recover_angular = p("recover_angular").value
        self.control_period = p("control_period").value
        self.linear_accel_limit = p("linear_accel_limit").value
        self.angular_accel_limit = p("angular_accel_limit").value
        self.roi_top = p("roi_top").value
        self.roi_bottom = p("roi_bottom").value
        self.depth_timeout = p("depth_timeout").value
        self.stuck_timeout = p("stuck_timeout").value
        self.min_valid_range = p("min_valid_range").value

    def _on_params_changed(self, params):
        # Allow live-tuning via `ros2 param set` without restarting the node.
        from rcl_interfaces.msg import SetParametersResult
        self._load_parameters()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def estop_callback(self, msg: Bool):
        if msg.data and not self.estop_active:
            self.get_logger().warn("EMERGENCY STOP engaged")
        elif not msg.data and self.estop_active:
            self.get_logger().info("Emergency stop cleared")
        self.estop_active = msg.data

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        self.last_depth_time = time.time()

        h, w = depth.shape
        roi = depth[int(h * self.roi_top):int(h * self.roi_bottom), :]

        left = roi[:, : w // 3]
        center = roi[:, w // 3 : 2 * w // 3]
        right = roi[:, 2 * w // 3 :]

        left_d = self._min_valid_distance(left)
        center_d = self._min_valid_distance(center)
        right_d = self._min_valid_distance(right)

        self._update_stuck_tracker(center_d)
        self.make_decision(left_d, center_d, right_d)

    # ------------------------------------------------------------------
    # Perception helpers
    # ------------------------------------------------------------------
    def _min_valid_distance(self, region: np.ndarray) -> float:
        valid = region[np.isfinite(region)]
        if valid.size == 0:
            return math.inf
        valid = valid[valid > self.min_valid_range]
        if valid.size == 0:
            return math.inf
        return float(np.min(valid))

    def _update_stuck_tracker(self, center_d: float):
        # "Progress" = center clearance is above the danger threshold, i.e.
        # the robot isn't jammed against an obstacle. Reset the clock
        # whenever that's true; if it stays jammed too long, force recovery.
        now = time.time()
        if center_d > self.danger_distance:
            self.last_progress_time = now
        self.last_center_distance = center_d

    # ------------------------------------------------------------------
    # Decision logic
    # ------------------------------------------------------------------
    def make_decision(self, left: float, center: float, right: float):
        stuck = (time.time() - self.last_progress_time) > self.stuck_timeout
        all_close = (
            left < self.danger_distance
            and center < self.danger_distance
            and right < self.danger_distance
        )

        if all_close or stuck:
            self._recover()
            return

        if center > self.safe_distance:
            # Clear ahead: go forward, scale speed down as clearance shrinks
            # toward safe_distance so braking isn't abrupt.
            clearance_ratio = min(
                1.0, (center - self.safe_distance) / self.safe_distance + 0.5
            )
            self.target_linear = self.max_linear_speed * clearance_ratio
            self.target_angular = 0.0
        else:
            # Obstacle ahead: stop translating, turn toward the more open side.
            self.target_linear = 0.0
            turn_dir = 1.0 if left > right else -1.0
            self.target_angular = turn_dir * self.max_angular_speed

    def _recover(self):
        self.get_logger().warn("Recovery behavior triggered (boxed in / stuck)")
        self.target_linear = self.recover_linear
        self.target_angular = self.recover_angular
        # Give the stuck timer a fresh window so recovery has time to work
        # before immediately re-triggering.
        self.last_progress_time = time.time()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def publish_velocity(self):
        # Safety: stale sensor data or explicit e-stop -> zero velocity.
        stale = (
            self.last_depth_time is None
            or (time.time() - self.last_depth_time) > self.depth_timeout
        )
        if self.estop_active or stale:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.current_linear = 0.0
            self.current_angular = 0.0
        else:
            self.current_linear = self._ramp(
                self.current_linear,
                self.target_linear,
                self.linear_accel_limit * self.control_period,
            )
            self.current_angular = self._ramp(
                self.current_angular,
                self.target_angular,
                self.angular_accel_limit * self.control_period,
            )

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)

    @staticmethod
    def _ramp(current: float, target: float, max_step: float) -> float:
        """Move `current` toward `target` by at most `max_step` (smoothing)."""
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "decision_node"
        status.hardware_id = "mapless_nav"

        stale = (
            self.last_depth_time is None
            or (time.time() - self.last_depth_time) > self.depth_timeout
        )

        if self.estop_active:
            status.level = DiagnosticStatus.ERROR
            status.message = "Emergency stop active"
        elif stale:
            status.level = DiagnosticStatus.WARN
            status.message = "No recent depth data"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Running"

        status.values = [
            KeyValue(key="center_distance_m", value=f"{self.last_center_distance:.2f}"),
            KeyValue(key="linear_cmd", value=f"{self.current_linear:.2f}"),
            KeyValue(key="angular_cmd", value=f"{self.current_angular:.2f}"),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a final zero-velocity command before shutting down.
        try:
            stop = Twist()
            node.cmd_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
main()