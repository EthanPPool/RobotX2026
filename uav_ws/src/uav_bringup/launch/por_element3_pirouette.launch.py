from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # RobotX 2026 POR UAV Flight Pattern - Element #3.
    #
    # Defaults are chosen inside the handbook's published 30-40 m horizontal
    # and 30-40 m altitude ranges:
    #   P2/P3 radial distance from START ~= 32.5 m
    #   P2-P3 separation = 30 m
    #   high altitude = 35 m
    #
    # The P3 -> START geometry and altitude drop approximate the required
    # 45-degree descent.
    return LaunchDescription([
        DeclareLaunchArgument('force_arm', default_value='false'),
        Node(
            package='uav_vehicle',
            executable='por_element3_runner',
            name='por_element3_runner',
            output='screen',
            parameters=[{
                'force_arm': ParameterValue(
                    LaunchConfiguration('force_arm'), value_type=bool
                ),
                'route_name': 'por_element3_pirouette',
                'initial_hover_altitude_m': 3.5,
                'high_altitude_m': 35.0,
                'final_hover_altitude_m': 2.5,

                'p2_forward_m': 0.0,
                'p2_right_m': 32.5,
                'p3_forward_m': 0.0,
                'p3_right_m': -32.5,

                'pirouette_count': 4,
                'pirouette_yaw_rate_dps': 30.0,

                # The handbook says the assessor calls for the stationary pirouettes.
                # Hold at the midpoint until that call is received, then trigger with:
                #   ros2 service call /por/pirouette std_srvs/srv/Trigger "{}"
                'pirouette_wait_for_trigger': True,

                'descent_steps': 12,
                'waypoint_tolerance_m': 1.0,
                'altitude_tolerance_m': 0.75,
                'waypoint_hold_s': 1.5,
            }],
        ),
    ])
