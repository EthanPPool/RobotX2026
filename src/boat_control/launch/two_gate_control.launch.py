#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('boat_control'),
        'config',
        'two_gate_follower.yaml',
    )

    parameters = [config] if os.path.isfile(config) else []

    return LaunchDescription([
        Node(
            package='boat_control',
            executable='two_gate_follower',
            name='two_gate_follower',
            output='screen',
            parameters=parameters,
        ),
    ])
