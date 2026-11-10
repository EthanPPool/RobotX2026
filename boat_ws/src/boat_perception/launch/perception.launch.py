import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory(
        'boat_perception'
    )

    # Global geometry definitions for all supported buoy types.
    buoy_types_file = os.path.join(
        package_share,
        'config',
        'buoy_types.yaml',
    )

    gate_config = os.path.join(
        get_package_share_directory('boat_perception'),
        'config',
        'gate_detector.yaml',
    )

    # -------------------------------------------------------------------------
    # MULTI-TYPE 3D BUOY DETECTOR
    # -------------------------------------------------------------------------
    # Publish to the production /perception/objects topic so the existing
    # gate detector receives the new detector output without any modification.
    buoy_detector = Node(
        package='boat_perception',
        executable='buoy_detector_multi',
        name='buoy_detector_multi',
        output='screen',
        parameters=[
            {
                'buoy_types_file': buoy_types_file,

                # Production Task 1 topics.
                'objects_topic': '/perception/objects',
                'motion_compensation': True,
                'local_position_topic': '/mavros/local_position/pose',
                'markers_topic': '/perception/object_markers',

                # Keep engineering outputs available while testing.
                'classification_topic':
                    '/perception/buoy_classifications',
                'diagnostics_topic':
                    '/perception/buoy_cluster_diagnostics',

                # Temporal persistence values from our latest revision.
                'max_track_misses': 7,
                # Keep tracks internally for reacquisition, but never stamp a
                # predicted, unobserved buoy as a fresh gate measurement.
                'publish_misses': 0,
                'generic_confirm_hits': 3,
                'generic_release_bad_frames': 6,
                'generic_confidence_alpha': 0.60,
                'generic_confidence_decay': 0.94,
            }
        ],
    )

    gate_detector = Node(
        package='boat_perception',
        executable='gate_detector',
        name='gate_detector',
        output='screen',
        parameters=[gate_config, {
            'objects_topic': PythonExpression([
                "'/perception/colored_objects' if '", LaunchConfiguration('use_hsv_colors'),
                "' == 'true' else '/perception/objects'",
            ]),
            'require_red_green': ParameterValue(LaunchConfiguration('use_hsv_colors'), value_type=bool),
        }],
    )

    arguments = [
        DeclareLaunchArgument('use_hsv_colors', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_hsv_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('camera_info_topic', default_value='/usv/camera/color/camera_info'),
        DeclareLaunchArgument('input_rectified', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_static_calibration', default_value='false', choices=['true', 'false']),
    ]
    colors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robotx_buoy_interpretation'), 'launch', 'two_gate_colors.launch.py',
        ])), condition=IfCondition(LaunchConfiguration('use_hsv_colors')),
        launch_arguments={n: LaunchConfiguration(n) for n in (
            'start_hsv_camera', 'image_topic', 'camera_info_topic', 'input_rectified',
            'allow_static_calibration',
        )}.items())
    return LaunchDescription(arguments + [buoy_detector, colors, gate_detector])
