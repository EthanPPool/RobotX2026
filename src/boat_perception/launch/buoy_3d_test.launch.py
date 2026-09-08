import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('boat_perception'),
        'config',
        'buoy_detector_3d.yaml',
    )

    detector = Node(
        package='boat_perception',
        executable='buoy_detector_3d',
        name='buoy_detector_3d',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([detector])
