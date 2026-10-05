#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_launch(package_name, launch_file):
    return PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            'launch',
            launch_file,
        )
    )


def generate_launch_description():
    start_sensors = LaunchConfiguration('start_sensors')
    start_perception = LaunchConfiguration('start_perception')
    start_mission = LaunchConfiguration('start_mission')
    start_control = LaunchConfiguration('start_control')
    start_vehicle = LaunchConfiguration('start_vehicle')
    start_dashboard_bridge = LaunchConfiguration('start_dashboard_bridge')
    start_mission_logger = LaunchConfiguration('start_mission_logger')
    start_esp32 = LaunchConfiguration('start_esp32')
    start_mavros = LaunchConfiguration('start_mavros')

    esp32_serial_port = LaunchConfiguration('esp32_serial_port')
    fcu_url = LaunchConfiguration('fcu_url')

    arguments = [
        DeclareLaunchArgument('start_sensors', default_value='true'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('start_vehicle', default_value='true'),
        DeclareLaunchArgument('start_dashboard_bridge', default_value='true'),
        DeclareLaunchArgument('start_mission_logger', default_value='true'),
        DeclareLaunchArgument('start_esp32', default_value='true'),
        DeclareLaunchArgument(
            'esp32_serial_port',
            default_value='auto',
            description='ESP32 serial port or auto.',
        ),
        DeclareLaunchArgument('start_mavros', default_value='true'),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://0.0.0.0:14550@',
            description='MAVROS connection to BlueOS/ArduRover.',
        ),
    ]

    sensors = IncludeLaunchDescription(
        package_launch('boat_bringup', 'boat_sensors.launch.py'),
        condition=IfCondition(start_sensors),
    )

    perception = IncludeLaunchDescription(
        package_launch('boat_perception', 'perception.launch.py'),
        condition=IfCondition(start_perception),
    )

    mission = IncludeLaunchDescription(
        package_launch('boat_mission', 'mission.launch.py'),
        condition=IfCondition(start_mission),
    )

    control = IncludeLaunchDescription(
        package_launch('boat_control', 'control.launch.py'),
        condition=IfCondition(start_control),
    )

    vehicle = IncludeLaunchDescription(
        package_launch('boat_vehicle', 'vehicle.launch.py'),
        condition=IfCondition(start_vehicle),
        launch_arguments={
            'start_mavros': start_mavros,
            'fcu_url': fcu_url,
        }.items(),
    )

    dashboard_bridge = Node(
        package='boat_dashboard_bridge',
        executable='bridge',
        name='boat_dashboard_bridge',
        output='screen',
        condition=IfCondition(start_dashboard_bridge),
    )

    mission_logger = Node(
        package='boat_dashboard_bridge',
        executable='mission_logger',
        name='mission_logger',
        output='screen',
        condition=IfCondition(start_mission_logger),
    )

    esp32_status = IncludeLaunchDescription(
        package_launch('boat_vehicle', 'esp32_status.launch.py'),
        condition=IfCondition(start_esp32),
        launch_arguments={'serial_port': esp32_serial_port}.items(),
    )

    message = LogInfo(
        msg=[
            '\n',
            '============================================================\n',
            ' RobotX Task 1 canonical stack\n',
            ' Perception : buoy_detector_multi -> gate_detector\n',
            ' Mission    : task1_gate_mission -> /mission/target\n',
            ' Control    : target_controller -> /control/cmd_vel\n',
            ' Vehicle    : mavros_command_bridge -> MAVROS/ArduRover\n',
            ' Dashboard  : TCP bridge + mission logger\n',
            ' ESP32      : status/light bridge\n',
            '\n',
            ' Legacy two_gate_follower/simple_gate_follower paths removed.\n',
            ' Startup remains SOFTWARE-STOPPED / DISARMED.\n',
            ' FCU = ',
            fcu_url,
            '\n',
            '============================================================',
        ]
    )

    return LaunchDescription(
        arguments
        + [
            message,
            sensors,
            perception,
            mission,
            control,
            vehicle,
            dashboard_bridge,
            mission_logger,
            esp32_status,
        ]
    )
