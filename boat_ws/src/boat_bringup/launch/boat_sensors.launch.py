import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnShutdown

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch argument:
    #
    #   lidar_rotation:=start
    #   lidar_rotation:=stop
    #   lidar_rotation:=none
    # ---------------------------------------------------------

    lidar_rotation = LaunchConfiguration('lidar_rotation')

    declare_lidar_rotation = DeclareLaunchArgument(
        'lidar_rotation',
        default_value='start',
        choices=['start', 'stop', 'none'],
        description='Command sent to the Unitree L2 rotation service'
    )


    # ---------------------------------------------------------
    # Boat description / robot_state_publisher
    # ---------------------------------------------------------

    boat_description_launch = os.path.join(
        get_package_share_directory('boat_description'),
        'launch',
        'description.launch.py'
    )

    boat_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            boat_description_launch
        )
    )


    # ---------------------------------------------------------
    # Unitree L2 driver
    #
    # Your Unitree package currently installs launch.py
    # directly into the package share directory.
    # ---------------------------------------------------------

    lidar_launch = os.path.join(
        get_package_share_directory('unitree_lidar_ros2'),
        'launch.py'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            lidar_launch
        )
    )


    # ---------------------------------------------------------
    # LiDAR rotation commands
    #
    # Wait briefly so the Unitree node and its services have
    # time to initialize before calling the service.
    # ---------------------------------------------------------

    start_rotation = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/unitree_lidar_ros2_node/start_rotation',
            'std_srvs/srv/Trigger',
            '{}'
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", lidar_rotation, "' == 'start'"
            ])
        )
    )

    stop_rotation = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/unitree_lidar_ros2_node/stop_rotation',
            'std_srvs/srv/Trigger',
            '{}'
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", lidar_rotation, "' == 'stop'"
            ])
        )
    )

    # ---------------------------------------------------------
    # Stop LiDAR rotation whenever this launch file shuts down
    # ---------------------------------------------------------

    stop_lidar_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg='Shutting down boat_sensors: stopping Unitree L2 rotation...'
                ),

                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'service',
                        'call',
                        '/unitree_lidar_ros2_node/stop_rotation',
                        'std_srvs/srv/Trigger',
                        '{}'
                    ],
                    output='screen'
                )
            ]
        )
    )

    rotation_command = TimerAction(
        period=4.0,
        actions=[
            start_rotation,
            stop_rotation
        ]
    )


    return LaunchDescription([
        declare_lidar_rotation,

        boat_description,
        lidar,

        rotation_command,

        stop_lidar_on_shutdown,
    ])
