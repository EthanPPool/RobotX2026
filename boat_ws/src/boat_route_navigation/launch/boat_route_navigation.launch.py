from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(get_package_share_directory('boat_route_navigation'),'config','boat_route_navigation.yaml')),
        Node(package='boat_route_navigation', executable='route_mission', parameters=[config], output='screen'),
        Node(package='boat_route_navigation', executable='survey_recorder', parameters=[config], output='screen'),
    ])
