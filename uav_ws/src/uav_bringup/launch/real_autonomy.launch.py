from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')

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

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'fcu_url': fcu_url,
            }.items()
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(autonomy_launch)
        ),
    ])
