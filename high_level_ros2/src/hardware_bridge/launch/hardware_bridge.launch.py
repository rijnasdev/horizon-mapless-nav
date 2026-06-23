import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Locate files from the perception package where your URDF lives
    perception_dir = get_package_share_directory('perception')
    xacro_file = os.path.join(perception_dir, 'urdf', 'rover.urdf.xacro')
    
    # Process Xacro to plain XML string
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Locate your controllers configuration file
    hardware_bridge_dir = get_package_share_directory('hardware_bridge')
    controllers_yaml = os.path.join(hardware_bridge_dir, 'config', 'controllers.yaml')

    # Node to publish robot state transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # The core Controller Manager node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_raw}, controllers_yaml],
        output='screen'
    )

    # Command to spawn the Joint State Broadcaster
    load_joint_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Command to spawn your wheel driver controller
    load_rover_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'rover_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        load_joint_broadcaster,
        load_rover_controller
    ])