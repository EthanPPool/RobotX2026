import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_mavros = LaunchConfiguration('start_mavros')
    fcu_url = LaunchConfiguration('fcu_url')
    autonomy_enabled = LaunchConfiguration('autonomy_enabled')

    mavros_share = get_package_share_directory('mavros')
    mavros_pluginlist = os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml')
    mavros_config = os.path.join(mavros_share, 'launch', 'apm_config.yaml')
    overrides = os.path.join(
        get_package_share_directory('boat_vehicle'), 'config', 'mavros_overrides.yaml'
    )
    bridge_config = os.path.join(
        get_package_share_directory('boat_vehicle'), 'config', 'bridge.yaml'
    )

    mavros_node = Node(
        condition=IfCondition(start_mavros),
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            mavros_pluginlist,
            mavros_config,
            overrides,
            {
                'fcu_url': fcu_url,
                'gcs_url': '',
                'system_id': 255,
                'component_id': 191,
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )

    bridge = Node(
        package='boat_vehicle',
        executable='mavros_command_bridge',
        name='mavros_command_bridge',
        output='screen',
        parameters=[bridge_config, {'autonomy_enabled': autonomy_enabled}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_mavros', default_value='true'),
        DeclareLaunchArgument('fcu_url', default_value='udp://0.0.0.0:14550@'),
        DeclareLaunchArgument(
            'autonomy_enabled',
            default_value='false',
            description='Explicit propulsion-command gate. Keep false for bench testing.',
        ),
        mavros_node,
        bridge,
    ])
