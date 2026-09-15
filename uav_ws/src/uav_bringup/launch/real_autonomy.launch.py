from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')

    mavros_launch = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'apm.launch'
    )

    autonomy_launch = os.path.join(
        get_package_share_directory('uav_bringup'),
        'launch',
        'autonomy.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyAMA0:57600',
            description='Pixhawk MAVLink connection'
        ),

        DeclareLaunchArgument(
            'gcs_url',
            default_value=(
                'udp://0.0.0.0:14552@'
                '192.168.2.100:14552'
            ),
            description='Beeptop RobotX GCS MAVLink connection'
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'namespace': '/',
            }.items()
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(autonomy_launch)
        ),

        Node(
            package='uav_dashboard_bridge',
            executable='bridge',
            name='uav_dashboard_bridge',
            output='screen',
        ),
    ])
