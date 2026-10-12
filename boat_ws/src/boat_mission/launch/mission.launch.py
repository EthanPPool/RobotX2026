#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('boat_mission'),
        'config',
        'task1_gate_mission.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="true"),
        Node(
            package='boat_mission',
            executable='task1_gate_mission',
            name='task1_gate_mission',
            output='screen',
            parameters=[config, {"enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool)}],
        )
    ])
