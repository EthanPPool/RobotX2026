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
        DeclareLaunchArgument("start_hsv", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument("camera_info_topic", default_value="/usv/camera/color/camera_info"),
        DeclareLaunchArgument("input_rectified", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("allow_static_calibration", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("calibration_timeout", default_value="0.1"),
        DeclareLaunchArgument("hsv_config", default_value=os.path.join(
            get_package_share_directory("robotx_buoy_interpretation"), "config", "hsv_clusters.yaml")),
        IncludeLaunchDescription(
            shared_launch(
                "robotx_buoy_interpretation",
                "interpretation.launch.py",
            ),
            launch_arguments={
                "config": interpretation,
                "source_mode": "usv_only",
                "start_hsv": LaunchConfiguration("start_hsv"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "input_rectified": LaunchConfiguration("input_rectified"),
                "hsv_config": LaunchConfiguration("hsv_config"),
                "allow_static_calibration": LaunchConfiguration("allow_static_calibration"),
                "calibration_timeout": LaunchConfiguration("calibration_timeout"),
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
