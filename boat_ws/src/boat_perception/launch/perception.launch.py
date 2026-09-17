import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
                'markers_topic': '/perception/object_markers',

                # Keep engineering outputs available while testing.
                'classification_topic':
                    '/perception/buoy_classifications',
                'diagnostics_topic':
                    '/perception/buoy_cluster_diagnostics',

                # Temporal persistence values from our latest revision.
                'max_track_misses': 7,
                'publish_misses': 4,
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
        parameters=[gate_config],
    )

    return LaunchDescription([buoy_detector, gate_detector])
