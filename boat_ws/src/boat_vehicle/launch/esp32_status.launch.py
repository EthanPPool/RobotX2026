#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='auto',
            description='ESP32 serial port or auto',
        ),

        Node(
            package='boat_vehicle',
            executable='esp32_status_bridge',
            name='esp32_status_bridge',
            output='screen',
            parameters=[
                {
                    'serial_port': serial_port,
                }
            ],
        ),
    ])
