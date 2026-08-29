#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    start_control = LaunchConfiguration('start_control')
    start_vehicle = LaunchConfiguration('start_vehicle')
    start_mavros = LaunchConfiguration('start_mavros')
    start_dashboard = LaunchConfiguration('start_dashboard')

    autonomy_enabled = LaunchConfiguration('autonomy_enabled')
    fcu_url = LaunchConfiguration('fcu_url')

    arguments = [

        DeclareLaunchArgument(
            'start_sensors',
            default_value='true',
            description=(
                'Start BlueBoat description and Unitree L2 sensor stack.'
            ),
        ),

        DeclareLaunchArgument(
            'start_perception',
            default_value='true',
            description=(
                'Start conservative buoy and gate perception.'
            ),
        ),

        DeclareLaunchArgument(
            'start_control',
            default_value='true',
            description=(
                'Start fail-stop simple gate follower.'
            ),
        ),

        DeclareLaunchArgument(
            'start_vehicle',
            default_value='true',
            description=(
                'Start MAVROS command safety bridge.'
            ),
        ),

        DeclareLaunchArgument(
            'start_mavros',
            default_value='true',
            description='Start MAVROS.',
        ),

        DeclareLaunchArgument(
            'start_dashboard',
            default_value='true',
            description=(
                'Start browser-based test dashboard.'
            ),
        ),

        DeclareLaunchArgument(
            'autonomy_enabled',
            default_value='false',
            description=(
                'Final MAVROS propulsion gate. '
                'MUST remain false at startup.'
            ),
        ),

        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://0.0.0.0:14550@',
            description=(
                'MAVROS connection to BlueOS/ArduRover.'
            ),
        ),
    ]

    # --------------------------------------------------------------
    # Sensors
    # --------------------------------------------------------------

    sensors = IncludeLaunchDescription(
        package_launch(
            'boat_bringup',
            'boat_sensors.launch.py',
        ),
        condition=IfCondition(start_sensors),
    )

    # --------------------------------------------------------------
    # Perception
    # --------------------------------------------------------------

    perception = IncludeLaunchDescription(
        package_launch(
            'boat_perception',
            'perception.launch.py',
        ),
        condition=IfCondition(start_perception),
    )

    # --------------------------------------------------------------
    # Simple fail-stop gate follower
    #
    # IMPORTANT:
    # This replaces the old mission state machine + TargetController
    # in this test configuration.
    # --------------------------------------------------------------

    simple_control = IncludeLaunchDescription(
        package_launch(
            'boat_control',
            'simple_gate_control.launch.py',
        ),
        condition=IfCondition(start_control),
    )

    # --------------------------------------------------------------
    # MAVROS + command bridge
    # --------------------------------------------------------------

    vehicle = IncludeLaunchDescription(
        package_launch(
            'boat_vehicle',
            'vehicle.launch.py',
        ),
        condition=IfCondition(start_vehicle),
        launch_arguments={
            'start_mavros': start_mavros,
            'autonomy_enabled': autonomy_enabled,
            'fcu_url': fcu_url,
        }.items(),
    )

    # --------------------------------------------------------------
    # Browser dashboard
    # --------------------------------------------------------------

    dashboard = IncludeLaunchDescription(
        package_launch(
            'boat_dashboard',
            'dashboard.launch.py',
        ),
        condition=IfCondition(start_dashboard),
    )

    # --------------------------------------------------------------
    # Startup safety message
    # --------------------------------------------------------------

    safety_message = LogInfo(
        msg=[
            '\n'
            '============================================================\n'
            ' RobotX SAFE SINGLE-GATE TEST STACK\n'
            '============================================================\n'
            '\n'
            ' OLD MISSION STATE MACHINE: NOT STARTED\n'
            ' OLD TARGET CONTROLLER:      NOT STARTED\n'
            '\n'
            ' Active control path:\n'
            '\n'
            ' LiDAR\n'
            '   -> Buoy Detector\n'
            '   -> Gate Detector\n'
            '   -> Simple Gate Follower\n'
            '   -> MAVROS Command Bridge\n'
            '   -> MAVROS / ArduRover\n'
            '\n'
            ' STARTUP SAFETY STATE:\n'
            '\n'
            ' Simple Gate Follower: DISABLED\n'
            ' Software Stop:        ACTIVE\n'
            ' Bridge Autonomy:      ',
            autonomy_enabled,
            '\n'
            '\n'
            ' Dashboard:\n'
            ' http://<JETSON-IP>:8080\n'
            '\n'
            ' STARTUP MUST PRODUCE ZERO PROPULSION.\n'
            '============================================================'
        ]
    )

    return LaunchDescription(
        arguments
        + [
            safety_message,
            sensors,
            perception,
            simple_control,
            vehicle,
            dashboard,
        ]
    )
