import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('boat_mission'), 'config', 'por.yaml')
    return LaunchDescription([
        Node(
            package='boat_mission',
            executable='por_gate_mission',
            name='por_gate_mission',
            output='screen',
            parameters=[config],
        )
    ])
