from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RobotX 2026 POR Element #2 ("hour-glass" pattern).
    #
    # The handbook figure does not publish metric dimensions for Element #2.
    # These offsets reproduce the pictured geometry at a compact test scale and
    # are intentionally easy to edit before the submission flight.
    #
    # Points are [forward_m, right_m] relative to heading at /por/start.
    return LaunchDescription([
        Node(
            package='uav_vehicle',
            executable='por_route_runner',
            name='por_route_runner',
            output='screen',
            parameters=[{
                'route_name': 'por_element2_hourglass',
                'route_points': [
                    5.0,  2.5,   # P1: near/right
                    15.0, -2.5,  # P2: far/left
                    10.0, 7.5,   # P3: right
                    10.0, -7.5,  # P4: left
                ],
                'takeoff_altitude_m': 3.5,
                'waypoint_tolerance_m': 0.75,
                'altitude_tolerance_m': 0.50,
                'waypoint_hold_s': 1.0,
            }],
        ),
    ])
