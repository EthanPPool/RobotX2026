from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # RobotX 2026 POR Element #1:
    # 10 m square, positioned 5 m forward of the takeoff point.
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
                'route_name': 'por_element1_square',
                'route_points': [
                    0.0,   5.0,   # P1 - right end of near edge
                    10.0,  5.0,   # P2 - far right
                    10.0, -5.0,   # P3 - far left
                    0.0,  -5.0,   # P4 - left end of near edge
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
