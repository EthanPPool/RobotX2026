"""Independent HSV processing and a red/green adapter for the two-gate stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robotx_buoy_interpretation')
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('start_hsv_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera_info_topic', default_value='/usv/camera/color/camera_info'),
        DeclareLaunchArgument('input_rectified', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_static_calibration', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hsv_config', default_value=os.path.join(share, 'config', 'hsv_clusters.yaml')),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'config', 'two_gate_colors.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hsv_camera.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_hsv_camera')),
            launch_arguments={n: LaunchConfiguration(n) for n in (
                'image_topic', 'camera_info_topic', 'input_rectified',
                'allow_static_calibration', 'hsv_config',
            )}.items()),
        Node(package='robotx_buoy_interpretation', executable='two_gate_colors',
             name='two_gate_colors', output='screen', parameters=[LaunchConfiguration('color_config'), {
                 'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                 'allow_static_calibration': ParameterValue(
                     LaunchConfiguration('allow_static_calibration'), value_type=bool),
             }]),
    ])
