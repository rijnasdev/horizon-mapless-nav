import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

class MaplessNavBrain(Node):
    def __init__(self):
        super().__init__('mapless_nav_brain')
        
        # 1. Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # 2. Subscribe to your friend's depth image topic
        self.image_sub = self.create_subscription(
            Image,
            '/depth/image',               
            self.image_callback,
            10)
            
        # 3. Publisher to move the rover
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Mapless Navigation Brain is alive and listening...')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to an OpenCV/Numpy float array
            # Format '32FC1' means 32-bit floating point, 1 channel (depth values in meters)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Replace NaN or infinite values with a safe max distance (e.g., 5 meters)
        depth_image = np.nan_to_num(depth_image, nan=5.0, posinf=5.0, neginf=0.0)

        # Get the height and width of the image
        h, w = depth_image.shape

        # We only care about obstacles directly in front of the rover's horizon. 
        # Let's look at a horizontal band across the middle rows of the image.
        middle_row_start = int(h * 0.4)
        middle_row_end = int(h * 0.6)
        center_band = depth_image[middle_row_start:middle_row_end, :]

        # Slice the band into 3 horizontal sectors: Left, Center, Right
        third = int(w / 3)
        left_sector = center_band[:, 0:third]
        center_sector = center_band[:, third:2*third]
        right_sector = center_band[:, 2*third:w]

        # Find the MINIMUM distance to an object in each zone (how close is the nearest wall?)
        min_left = np.min(left_sector)
        min_center = np.min(center_sector)
        min_right = np.min(right_sector)

        # Print distances to terminal for debugging
        self.get_logger().info(f"Distances -> L: {min_left:.2f}m | C: {min_center:.2f}m | R: {min_right:.2f}m")

        # Create movement message
        twist = Twist()
        obstacle_threshold = 1.2  # Stop/turn if something is closer than 1.2 meters

        # --- RE-ACTIVE NAVIGATION DECISION LOGIC ---
        if min_center < obstacle_threshold:
            # Something is blocking our path directly ahead! Stop and turn.
            twist.linear.x = 0.0
            if min_left > min_right:
                self.get_logger().info("Obstacle ahead! Turning LEFT.")
                twist.angular.z = 0.5  # Positive turns counter-clockwise (left)
            else:
                self.get_logger().info("Obstacle ahead! Turning RIGHT.")
                twist.angular.z = -0.5 # Negative turns clockwise (right)
                
        elif min_left < obstacle_threshold:
            # Too close to something on the left, nudge right
            self.get_logger().info("Nudging right...")
            twist.linear.x = 0.2
            twist.angular.z = -0.3
            
        elif min_right < obstacle_threshold:
            # Too close to something on the right, nudge left
            self.get_logger().info("Nudging left...")
            twist.linear.x = 0.2
            twist.angular.z = 0.3
            
        else:
            # Path is wide open! Drive straight ahead.
            self.get_logger().info("Path clear. Moving forward.")
            twist.linear.x = 0.4
            twist.angular.z = 0.0

        # Publish the command to the rover simulation
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MaplessNavBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()