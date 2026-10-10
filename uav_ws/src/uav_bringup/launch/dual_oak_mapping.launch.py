"""Bind the existing RGB source and the second, downward OAK-D on the Pi."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_nodes(context):
    primary = LaunchConfiguration('primary_device_id').perform(context).strip()
    downward = LaunchConfiguration('downward_device_id').perform(context).strip()
    if not primary or not downward or primary == downward:
        raise ValueError('Provide two DIFFERENT OAK-D DeviceID/MXIDs: primary_device_id and downward_device_id')
    config = LaunchConfiguration('config')
    return [
        Node(package='uav_perception', executable='oak_rgb_node', namespace='/uav',
             parameters=[{'device_id': primary, 'frame_id': 'uav/primary_optical'}],
             remappings=[('/oak/rgb/image_raw', '/uav/oak/rgb/image_raw')], output='screen'),
        Node(package='uav_mapping', executable='downward_camera',
             parameters=[config, {'camera_device_id': downward}], output='screen'),
        Node(package='uav_mapping', executable='mapper', parameters=[config],
             remappings=[('/tf', '/uav/tf'), ('/tf_static', '/uav/tf_static')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('primary_device_id'),
        DeclareLaunchArgument('downward_device_id'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('uav_mapping'), 'config', 'uav_mapping.yaml')),
        OpaqueFunction(function=camera_nodes),
    ])
