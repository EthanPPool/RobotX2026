from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # RobotX 2026 POR Element #2 ("hour-glass" pattern).
    #
    # The handbook figure does not publish metric dimensions for Element #2.
    # These offsets reproduce the pictured geometry at a compact test scale and
    # are intentionally easy to edit before the submission flight.
    #
    # Points are [forward_m, right_m] relative to heading at /por/start.
    return LaunchDescription([
        DeclareLaunchArgument('force_arm', default_value='false'),
        Node(
            package='uav_vehicle',
            executable='por_route_runner',
            name='por_route_runner',
            output='screen',
            parameters=[{
                'force_arm': ParameterValue(
                    LaunchConfiguration('force_arm'), value_type=bool
                ),
                'route_name': 'por_element2_hourglass',
                'route_points': [
                    0.0,   5.0,    # P1
                    16.0, -5.0,    # P2
                    16.0,  5.0,    # P3
                    0.0,  -5.0,    # P4
                    0.0,   0.0,    # START/END
                ],
                'takeoff_altitude_m': 3.5,
                'waypoint_tolerance_m': 0.75,
                'altitude_tolerance_m': 0.50,
                'waypoint_hold_s': 1.0,
            }],
        ),
    ])
