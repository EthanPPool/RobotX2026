"""USV interpretation, with an optional HSV camera producer and UAV fusion."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robotx_buoy_interpretation')
    config = LaunchConfiguration('config')
    mode = LaunchConfiguration('source_mode')
    camera = LaunchConfiguration('camera_info_topic')
    detections = LaunchConfiguration('detections_topic')
    calibration = {
        'calibration_timeout': ParameterValue(LaunchConfiguration('calibration_timeout'), value_type=float),
        'allow_static_calibration': ParameterValue(LaunchConfiguration('allow_static_calibration'), value_type=bool),
    }
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'interpretation.yaml')),
        DeclareLaunchArgument('source_mode', default_value='usv_only', choices=['usv_only', 'assisted']),
        DeclareLaunchArgument('start_hsv', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hsv_config', default_value=os.path.join(share, 'config', 'hsv_clusters.yaml')),
        DeclareLaunchArgument('image_topic', default_value=''),
        # Shared overrides keep the producer and interpreter on the same streams.
        DeclareLaunchArgument('camera_info_topic', default_value='/usv/camera/color/camera_info'),
        DeclareLaunchArgument('detections_topic', default_value='/perception/usv/color_detections'),
        DeclareLaunchArgument('input_rectified', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('calibration_timeout', default_value='0.1'),
        DeclareLaunchArgument('allow_static_calibration', default_value='false', choices=['true', 'false']),
        Node(package='robotx_buoy_interpretation', executable='hsv_color_clusters',
             name='usv_hsv_clusters', output='screen',
             condition=IfCondition(LaunchConfiguration('start_hsv')),
             parameters=[LaunchConfiguration('hsv_config'), calibration, {
                 'image_topic': LaunchConfiguration('image_topic'),
                 'camera_info_topic': camera, 'detections_topic': detections,
                 'input_rectified': ParameterValue(LaunchConfiguration('input_rectified'), value_type=bool),
             }]),
        Node(package='robotx_buoy_interpretation', executable='usv_buoy_interpretation',
             name='usv_buoy_interpretation', condition=IfCondition(LaunchConfiguration('start_hsv')),
             parameters=[config, calibration, {
                 'camera_info_topic': camera, 'detections_topic': detections,
             }], output='screen'),
        # Preserve existing YAML-only behavior when HSV is disabled.
        Node(package='robotx_buoy_interpretation', executable='usv_buoy_interpretation',
             name='usv_buoy_interpretation', parameters=[config], output='screen',
             condition=UnlessCondition(LaunchConfiguration('start_hsv'))),
        Node(package='robotx_buoy_interpretation', executable='buoy_tracker',
             name='buoy_tracker', parameters=[config, {'source_mode': mode}], output='screen'),
    ])
