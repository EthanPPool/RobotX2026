from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uav_bringup'), 'config', 'software_test.yaml')

    return LaunchDescription([
        Node(
            package='uav_vehicle', executable='mock_mavros',
            name='mock_mavros', output='screen', parameters=[config]),
        Node(
            package='uav_perception', executable='mock_target_publisher',
            name='mock_target_publisher', output='screen', parameters=[config]),
        Node(
            package='uav_perception', executable='target_filter',
            name='target_filter', output='screen', parameters=[config]),
        Node(
            package='uav_mission', executable='target_mission',
            name='target_mission', output='screen', parameters=[config]),
        Node(
            package='uav_control', executable='velocity_guidance',
            name='velocity_guidance', output='screen', parameters=[config]),
        Node(
            package='uav_safety', executable='safety_supervisor',
            name='safety_supervisor', output='screen', parameters=[config]),
        Node(
            package='uav_vehicle', executable='vehicle_manager',
            name='vehicle_manager', output='screen', parameters=[config]),
        Node(
            package='uav_vehicle', executable='mavros_command_bridge',
            name='mavros_command_bridge', output='screen', parameters=[config]),
    ])
