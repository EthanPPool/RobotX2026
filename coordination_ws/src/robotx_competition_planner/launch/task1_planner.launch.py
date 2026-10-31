"""Launch planner beside the interpretation stack; no propulsion is launched."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('robotx_competition_planner'),
                           'config', 'task1.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='robotx_competition_planner', executable='competition_planner',
             name='robotx_competition_planner',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
