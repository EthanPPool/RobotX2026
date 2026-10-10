from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(get_package_share_directory('uuv_follow'),'config','uuv_follow.yaml')),
        Node(package='uuv_follow', executable='follower', parameters=[config], output='screen'),
        Node(package='uuv_follow', executable='velocity_guard', parameters=[config], output='screen'),
    ])
