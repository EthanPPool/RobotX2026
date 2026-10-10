"""Pi flight stack with UAV-local command/telemetry topics on a shared domain."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    config = LaunchConfiguration('config')
    # Absolute names in existing nodes need remapping as well as node namespaces.
    remaps = [('/mavros/mavros/**', r'/uav/mavros/\1')]
    remaps += [(f'/{prefix}/**', f'/uav/{prefix}/' + r'\1')
               for prefix in ('mavros', 'vehicle', 'control', 'mission', 'safety', 'perception', 'por', 'oak')]
    remaps += [('/tf', '/uav/tf'), ('/tf_static', '/uav/tf_static')]
    flight_nodes = [
        ('uav_perception', 'target_filter', 'target_filter'),
        ('uav_mission', 'target_mission', 'target_mission'),
        ('uav_control', 'velocity_guidance', 'velocity_guidance'),
        ('uav_safety', 'safety_supervisor', 'safety_supervisor'),
        ('uav_safety', 'uav_geofence', 'uav_geofence'),
        ('uav_vehicle', 'vehicle_manager', 'vehicle_manager'),
        ('uav_vehicle', 'mavros_command_bridge', 'mavros_command_bridge'),
        ('uav_dashboard_bridge', 'bridge', 'uav_dashboard_bridge'),
    ]
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('target_system'),
            'namespace': '/uav',
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('uav_bringup'), 'config', 'coordinated_autonomy.yaml')),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyAMA0:57600'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@'),
        DeclareLaunchArgument('target_system', default_value='2', description='Match UAV autopilot SYSID_THISMAV'),
        GroupAction(actions=[SetRemap(src=src, dst=dst) for src, dst in remaps] + [mavros] + [
            Node(package=package, executable=executable, name=name, namespace='/uav',
                 parameters=[config], output='screen')
            for package, executable, name in flight_nodes]),
    ])
