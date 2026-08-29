from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dashboard = Node(
        package='boat_dashboard',
        executable='dashboard',
        name='boat_dashboard',
        output='screen',
        parameters=[
            {
                'port': 8080,
                'gate_timeout': 0.50,
                'min_gate_confidence': 0.75,
            }
        ],
    )

    return LaunchDescription([
        dashboard
    ])
