from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(get_package_share_directory('uav_mapping'),'config','uav_mapping.yaml')),
        DeclareLaunchArgument('start_camera',default_value='false'),
        Node(package='uav_mapping',executable='downward_camera',parameters=[config],condition=IfCondition(LaunchConfiguration('start_camera')),output='screen'),
        Node(package='uav_mapping', executable='mapper', parameters=[config],
             remappings=[('/tf','/uav/tf'),('/tf_static','/uav/tf_static')], output='screen'),
    ])
