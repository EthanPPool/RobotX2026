"""Independent camera producer for the existing mission perception contract.

Starts only HSV processing. No LiDAR driver, geometry subscriber, interpreter,
tracker, planner, or controller is started by this launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robotx_buoy_interpretation')
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', description='Actual USV camera Image topic'),
        DeclareLaunchArgument('hsv_config', default_value=os.path.join(
            share, 'config', 'hsv_clusters.yaml')),
        DeclareLaunchArgument('camera_info_topic', default_value='/usv/camera/color/camera_info'),
        DeclareLaunchArgument('detections_topic', default_value='/perception/usv/color_detections'),
        DeclareLaunchArgument('input_rectified', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('calibration_timeout', default_value='0.1'),
        DeclareLaunchArgument('allow_static_calibration', default_value='false', choices=['true', 'false']),
        Node(package='robotx_buoy_interpretation', executable='hsv_color_clusters',
             name='usv_hsv_clusters', output='screen',
             parameters=[LaunchConfiguration('hsv_config'), {
                 'image_topic': LaunchConfiguration('image_topic'),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                 'detections_topic': LaunchConfiguration('detections_topic'),
                 'input_rectified': ParameterValue(
                     LaunchConfiguration('input_rectified'), value_type=bool),
                 'calibration_timeout': ParameterValue(
                     LaunchConfiguration('calibration_timeout'), value_type=float),
                 'allow_static_calibration': ParameterValue(
                     LaunchConfiguration('allow_static_calibration'), value_type=bool),
             }]),
    ])
