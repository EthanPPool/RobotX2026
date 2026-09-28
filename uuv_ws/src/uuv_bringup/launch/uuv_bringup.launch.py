#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('uuv_bringup')
    mavros_share = get_package_share_directory('mavros')
    config = os.path.join(bringup_share, 'config', 'uuv.yaml')

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    target_system = LaunchConfiguration('target_system')
    camera_source = LaunchConfiguration('camera_source')
    perception_backend = LaunchConfiguration('perception_backend')
    model_path = LaunchConfiguration('model_path')
    roboflow_model_id = LaunchConfiguration('roboflow_model_id')
    roboflow_api_url = LaunchConfiguration('roboflow_api_url')
    dashboard_port = LaunchConfiguration('dashboard_port')

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(mavros_share, 'launch', 'apm.launch')),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'tgt_system': target_system,
            'tgt_component': '1',
            'namespace': '/uuv/mavros',
        }.items(),
    )

    def node(package, executable, name, extra=None):
        params = [config]
        if extra:
            params.append(extra)
        return Node(package=package, executable=executable, name=name, output='screen', parameters=params)

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://192.168.4.10:14600@', description='UUV flight-controller MAVLink URL'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@', description='Optional MAVROS GCS forwarding URL'),
        DeclareLaunchArgument('target_system', default_value='3', description='MAVLink target system ID; match UUV FC SYSID'),
        DeclareLaunchArgument('camera_source', default_value='/dev/video0'),
        DeclareLaunchArgument('perception_backend', default_value='ultralytics', choices=['ultralytics', 'roboflow']),
        DeclareLaunchArgument('model_path', default_value='models/uuv_yolo.pt'),
        DeclareLaunchArgument('roboflow_model_id', default_value=''),
        DeclareLaunchArgument('roboflow_api_url', default_value='http://127.0.0.1:9001'),
        DeclareLaunchArgument('dashboard_port', default_value='8770'),
        mavros,
        node('uuv_camera', 'camera_node', 'uuv_camera', {'source': camera_source}),
        node('uuv_perception', 'yolo_detector', 'uuv_yolo_detector', {
            'backend': perception_backend,
            'model_path': model_path,
            'roboflow_model_id': roboflow_model_id,
            'roboflow_api_url': roboflow_api_url,
        }),
        node('uuv_vehicle', 'status_aggregator', 'uuv_status_aggregator'),
        node('uuv_safety', 'safety_supervisor', 'uuv_safety_supervisor'),
        node('uuv_mission', 'mission_manager', 'uuv_mission_manager'),
        node('uuv_vehicle', 'command_bridge', 'uuv_command_bridge'),
        node('uuv_dashboard_bridge', 'bridge', 'uuv_dashboard_bridge', {'port': ParameterValue(dashboard_port, value_type=int)}),
    ])
