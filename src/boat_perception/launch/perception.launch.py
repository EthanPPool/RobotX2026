import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    buoy_config = os.path.join(
        get_package_share_directory('boat_perception'),
        'config',
        'buoy_detector_3d.yaml',
    )

    gate_config = os.path.join(
        get_package_share_directory('boat_perception'),
        'config',
        'gate_detector.yaml',
    )

    buoy_detector = Node(
        package='boat_perception',
        executable='buoy_detector_3d',
        name='buoy_detector_3d',
        output='screen',
        parameters=[buoy_config],
    )

    gate_detector = Node(
        package='boat_perception',
        executable='gate_detector',
        name='gate_detector',
        output='screen',
        parameters=[gate_config],
    )

    return LaunchDescription([buoy_detector, gate_detector])
