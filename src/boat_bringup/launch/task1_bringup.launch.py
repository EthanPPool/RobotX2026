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

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    start_sensors = LaunchConfiguration('start_sensors')
    start_perception = LaunchConfiguration('start_perception')
    start_mission = LaunchConfiguration('start_mission')
    start_control = LaunchConfiguration('start_control')
    start_vehicle = LaunchConfiguration('start_vehicle')
    start_mavros = LaunchConfiguration('start_mavros')

    autonomy_enabled = LaunchConfiguration('autonomy_enabled')
    fcu_url = LaunchConfiguration('fcu_url')

    arguments = [

        DeclareLaunchArgument(
            'start_sensors',
            default_value='true',
            description='Start BlueBoat description and Unitree L2 sensor stack.',
        ),

        DeclareLaunchArgument(
            'start_perception',
            default_value='true',
            description='Start buoy and gate perception pipeline.',
        ),

        DeclareLaunchArgument(
            'start_mission',
            default_value='true',
            description='Start the two-gate RobotX mission state machine.',
        ),

        DeclareLaunchArgument(
            'start_control',
            default_value='true',
            description='Start NavigationTarget to body velocity controller.',
        ),

        DeclareLaunchArgument(
            'start_vehicle',
            default_value='true',
            description='Start vehicle command bridge.',
        ),

        DeclareLaunchArgument(
            'start_mavros',
            default_value='true',
            description='Start MAVROS from boat_vehicle.',
        ),

        DeclareLaunchArgument(
            'autonomy_enabled',
            default_value='false',
            description=(
                'Final propulsion-command safety gate. '
                'Default FALSE for bench testing.'
            ),
        ),

        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://0.0.0.0:14550@',
            description='MAVROS connection to BlueOS/ArduRover.',
        ),
    ]

    # ------------------------------------------------------------------
    # Sensors / robot description
    # ------------------------------------------------------------------

    sensors = IncludeLaunchDescription(
        package_launch(
            'boat_bringup',
            'boat_sensors.launch.py',
        ),
        condition=IfCondition(start_sensors),
    )

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    perception = IncludeLaunchDescription(
        package_launch(
            'boat_perception',
            'perception.launch.py',
        ),
        condition=IfCondition(start_perception),
    )

    # ------------------------------------------------------------------
    # Mission
    # ------------------------------------------------------------------

    mission = IncludeLaunchDescription(
        package_launch(
            'boat_mission',
            'mission.launch.py',
        ),
        condition=IfCondition(start_mission),
    )

    # ------------------------------------------------------------------
    # Controller
    # ------------------------------------------------------------------

    control = IncludeLaunchDescription(
        package_launch(
            'boat_control',
            'control.launch.py',
        ),
        condition=IfCondition(start_control),
    )

    # ------------------------------------------------------------------
    # MAVROS + vehicle safety bridge
    # ------------------------------------------------------------------

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

    safety_message = LogInfo(
        msg=[
            '\n'
            '============================================================\n'
            ' RobotX Task 1 autonomy stack starting\n'
            ' autonomy_enabled = ',
            autonomy_enabled,
            '\n'
            ' FCU = ',
            fcu_url,
            '\n'
            ' Propulsion commands remain safety-gated by boat_vehicle.\n'
            '============================================================'
        ]
    )

    return LaunchDescription(
        arguments
        + [
            safety_message,
            sensors,
            perception,
            mission,
            control,
            vehicle,
        ]
    )
