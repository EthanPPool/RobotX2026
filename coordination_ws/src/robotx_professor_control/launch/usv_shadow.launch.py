"""USV perception, incremental planner and professor control.

Non-actuating integration launch. No vehicle bridge or arming.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def shared_launch(package, filename):
    return PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package),
            "launch",
            filename,
        )
    )


def generate_launch_description():
    pathing = LaunchConfiguration("pathing_config")
    interpretation = LaunchConfiguration("interpretation_config")

    pathing_default = os.path.join(
        get_package_share_directory("robotx_professor_control"),
        "config",
        "pathing.yaml",
    )
    interpretation_default = os.path.join(
        get_package_share_directory("robotx_buoy_interpretation"),
        "config",
        "interpretation.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pathing_config",
            default_value=pathing_default,
        ),
        DeclareLaunchArgument(
            "interpretation_config",
            default_value=interpretation_default,
        ),
        IncludeLaunchDescription(
            shared_launch(
                "robotx_buoy_interpretation",
                "interpretation.launch.py",
            ),
            launch_arguments={
                "config": interpretation,
                "source_mode": "usv_only",
            }.items(),
        ),
        IncludeLaunchDescription(
            shared_launch(
                "robotx_professor_control",
                "task1_professor.launch.py",
            ),
            launch_arguments={
                "config": pathing,
                "mode": "usv",
                "start_force_bridge": "false",
            }.items(),
        ),
    ])
