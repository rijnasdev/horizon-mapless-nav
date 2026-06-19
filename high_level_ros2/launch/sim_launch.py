from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():

    pkg = get_package_share_directory(
        "rover_navigation"
    )

    world = os.path.join(
        pkg,
        "worlds",
        "simple_baylands.sdf"
    )

    xacro_file = os.path.join(
        pkg,
        "urdf",
        "rover.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            world,
            "-r"
        ],
        output="screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            { "robot_description": robot_description }
        ],
        output="screen"
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "simple_baylands",
                    "-name", "rover",
                    "-topic", "robot_description"
                ],
                output="screen"
            )
        ]
    )

    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/simple_baylands/model/rover/link/base_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image"
        ],
        remappings=[
            (
                "/world/simple_baylands/model/rover/link/base_link/sensor/depth_camera/depth_image",
                "/depth/image"
            )
        ]
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/simple_baylands/model/rover/link/base_link/sensor/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        remappings=[
            (
                "/world/simple_baylands/model/rover/link/base_link/sensor/depth_camera/camera_info",
                "/depth/camera_info"
            )
        ]
    )

    navigation = Node(
        package="rover_navigation",
        executable="navigation_node",
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        depth_bridge,
        camera_info_bridge,
        navigation
    ])
