"""Run on UAV Pi, or the host with the timestamped UAV optical TF/calibration."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('robotx_buoy_interpretation'),
                           'config', 'interpretation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='robotx_buoy_interpretation', executable='uav_buoy_interpretation',
             name='uav_buoy_interpretation',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
