"""Incremental mission + professor controller; force hardware bridge is opt-in."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config=LaunchConfiguration('config');mode=LaunchConfiguration('mode')
    default=os.path.join(get_package_share_directory('robotx_professor_control'),'config','pathing.yaml')
    tier=PythonExpression(["'advanced' if '",mode,"' == 'duo' else 'core'"])
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=default),
        DeclareLaunchArgument('mode',default_value='usv',choices=['usv','duo']),
        DeclareLaunchArgument('start_force_bridge',default_value='false',choices=['true','false']),
        Node(package='robotx_competition_planner',executable='incremental_planner',
             name='robotx_competition_planner',parameters=[config,{'tier':tier}],output='screen'),
        Node(package='robotx_professor_control',executable='professor_controller',
             name='professor_controller',parameters=[config],output='screen'),
        Node(package='robotx_professor_control',executable='professor_force_bridge',
             name='mavros_command_bridge',parameters=[config],output='screen',
             condition=IfCondition(LaunchConfiguration('start_force_bridge'))),
    ])
