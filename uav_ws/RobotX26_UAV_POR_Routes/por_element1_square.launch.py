from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RobotX 2026 POR Element #1:
    # 10 m square, positioned 5 m forward of the takeoff point.
    # Points are [forward_m, right_m] relative to heading at /por/start.
    return LaunchDescription([
        Node(
            package='uav_vehicle',
            executable='por_route_runner',
            name='por_route_runner',
            output='screen',
            parameters=[{
                'route_name': 'por_element1_square',
                'route_points': [
                    5.0,  5.0,   # P1
                    15.0, 5.0,   # P2
                    15.0, -5.0,  # P3
                    5.0, -5.0,   # P4
                ],
                # Handbook text requires ABOVE 3 m AGL.
                # The handbook graphic says 2-3 m, so keep this parameter easy to edit.
                'takeoff_altitude_m': 3.5,
                'waypoint_tolerance_m': 0.75,
                'altitude_tolerance_m': 0.50,
                'waypoint_hold_s': 1.0,
            }],
        ),
    ])
