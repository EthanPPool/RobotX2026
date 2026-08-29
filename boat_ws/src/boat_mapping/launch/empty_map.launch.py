from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="boat_mapping",
            executable="empty_map_publisher",
            name="empty_map_publisher",
            output="screen",
            parameters=[{
                "frame_id": "map",
                "map_topic": "/map",
                "width": 100,
                "height": 100,
                "resolution": 1.0,
                "origin_x": -50.0,
                "origin_y": -50.0,
            }],
        )
    ])

