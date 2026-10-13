"""Common USV stack with a dashboard-owned, initially stopped mission process."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common = os.path.join(get_package_share_directory('boat_bringup'), 'launch', 'task1_bringup.launch.py')
    forwarded = ('start_sensors', 'start_perception', 'start_vehicle', 'start_mavros',
                 'start_esp32', 'esp32_serial_port', 'fcu_url')
    defaults = {'esp32_serial_port': 'auto', 'fcu_url': 'udp://0.0.0.0:14550@'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=defaults.get(k, 'true')) for k in forwarded],
        DeclareLaunchArgument('route_config', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(common), launch_arguments={
            **{k: LaunchConfiguration(k) for k in forwarded},
            'start_mission': 'false', 'start_dashboard_bridge': 'false',
        }.items()),
        Node(package='boat_dashboard_bridge', executable='bridge', name='boat_dashboard_bridge',
             parameters=[{'manage_missions': True, 'route_config': LaunchConfiguration('route_config')}],
             output='screen'),
    ])
