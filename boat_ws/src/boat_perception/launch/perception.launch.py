import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('boat_perception'),
        'config',
        'lidar_perception.yaml',
    )

    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[config],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),
            ('scan', '/perception/scan'),
        ],
    )

    buoy_detector = Node(
        package='boat_perception',
        executable='buoy_detector',
        name='buoy_detector',
        output='screen',
        parameters=[config],
    )

    gate_detector = Node(
        package='boat_perception',
        executable='gate_detector',
        name='gate_detector',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([cloud_to_scan, buoy_detector, gate_detector])
