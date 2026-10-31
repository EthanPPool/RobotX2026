"""Run on the USV: local interpretation plus optional UAV input fusion."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('robotx_buoy_interpretation'),
                           'config', 'interpretation.yaml')
    config = LaunchConfiguration('config')
    mode = LaunchConfiguration('source_mode')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('source_mode', default_value='usv_only',
                              choices=['usv_only', 'assisted']),
        Node(package='robotx_buoy_interpretation', executable='usv_buoy_interpretation',
             name='usv_buoy_interpretation', parameters=[config], output='screen'),
        Node(package='robotx_buoy_interpretation', executable='buoy_tracker',
             name='buoy_tracker', parameters=[config, {'source_mode': mode}], output='screen'),
    ])
