#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def package_launch(
    package_name,
    launch_file
):
    return PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(
                package_name
            ),
            'launch',
            launch_file,
        )
    )


def generate_launch_description():

    start_sensors = LaunchConfiguration(
        'start_sensors'
    )

    start_perception = LaunchConfiguration(
        'start_perception'
    )

    start_control = LaunchConfiguration(
        'start_control'
    )

    start_vehicle = LaunchConfiguration(
        'start_vehicle'
    )

    start_dashboard = LaunchConfiguration(
        'start_dashboard'
    )

    start_esp32 = LaunchConfiguration(
        'start_esp32'
    )

    esp32_serial_port = LaunchConfiguration(
        'esp32_serial_port'
    )

    start_mavros = LaunchConfiguration(
        'start_mavros'
    )

    autonomy_enabled = LaunchConfiguration(
        'autonomy_enabled'
    )

    fcu_url = LaunchConfiguration(
        'fcu_url'
    )

    arguments = [
        DeclareLaunchArgument(
            'start_sensors',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_perception',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_control',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_vehicle',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_dashboard',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_esp32',
            default_value='true',
            description=(
                'Start ESP32 status/light bridge.'
            ),
        ),

        DeclareLaunchArgument(
            'esp32_serial_port',
            default_value='auto',
            description=(
                'ESP32 serial port or auto.'
            ),
        ),

        DeclareLaunchArgument(
            'start_mavros',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'autonomy_enabled',
            default_value='false',
            description=(
                'Safety gate. '
                'Leave FALSE at startup.'
            ),
        ),

        DeclareLaunchArgument(
            'fcu_url',
            default_value=(
                'udp://0.0.0.0:14550@'
            ),
        ),
    ]

    sensors = IncludeLaunchDescription(
        package_launch(
            'boat_bringup',
            'boat_sensors.launch.py',
        ),
        condition=IfCondition(
            start_sensors
        ),
    )

    perception = IncludeLaunchDescription(
        package_launch(
            'boat_perception',
            'perception.launch.py',
        ),
        condition=IfCondition(
            start_perception
        ),
    )

    # This replaces BOTH:
    #
    #   por_gate_mission
    #   target_controller
    #
    # It directly produces the conservative body-frame
    # /control/cmd_vel used by the validated single-gate path.
    control = IncludeLaunchDescription(
        package_launch(
            'boat_control',
            'two_gate_control.launch.py',
        ),
        condition=IfCondition(
            start_control
        ),
    )

    vehicle = IncludeLaunchDescription(
        package_launch(
            'boat_vehicle',
            'vehicle.launch.py',
        ),
        condition=IfCondition(
            start_vehicle
        ),
        launch_arguments={
            'start_mavros':
                start_mavros,
            'autonomy_enabled':
                autonomy_enabled,
            'fcu_url':
                fcu_url,
        }.items(),
    )

    dashboard = IncludeLaunchDescription(
        package_launch(
            'boat_dashboard',
            'dashboard.launch.py',
        ),
        condition=IfCondition(
            start_dashboard
        ),
    )

    esp32_status = IncludeLaunchDescription(
        package_launch(
            'boat_vehicle',
            'esp32_status.launch.py',
        ),
        condition=IfCondition(
            start_esp32
        ),
        launch_arguments={
            'serial_port':
                esp32_serial_port,
        }.items(),
    )

    message = LogInfo(
        msg=[
            '\n'
            '============================================================\n'
            ' RobotX Task 1 unified stack\n'
            ' Sensors       : ON\n'
            ' Perception    : ON\n'
            ' Two-gate ctrl : ON\n'
            ' Vehicle safety: ON\n'
            ' Xbox operator : ON via dashboard/bridge\n'
            ' Dashboard     : ON\n'
            ' ESP32/light   : ON\n'
            '\n'
            ' OLD por_gate_mission: NOT STARTED\n'
            ' OLD target_controller: NOT STARTED\n'
            '\n'
            ' autonomy_enabled = ',
            autonomy_enabled,
            '\n'
            ' Startup remains SOFTWARE-STOPPED / DISARMED.\n'
            '============================================================'
        ]
    )

    return LaunchDescription(
        arguments
        + [
            message,
            sensors,
            perception,
            control,
            vehicle,
            dashboard,
            esp32_status,
        ]
    )
