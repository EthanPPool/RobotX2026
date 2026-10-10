"""Only synthetic /sim endpoints; no hardware MAVROS, camera or network transport."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topics = [
        '/tf','/tf_static','/uav/downward/image_raw','/uav/downward/camera_info',
        '/uav/mapping/buoys','/uav/mapping/georeferenced_image','/uav/mapping/status',
        '/boat/coordination/odometry','/uuv/coordination/odometry',
        '/perception/objects','/mission/target','/mission/state','/control/cmd_vel','/control/diagnostics',
        '/boat/route/status','/boat/route/path','/boat/route/set_enabled','/boat/route/reset',
        '/mavros/state','/mavros/setpoint_velocity/cmd_vel','/mavros/set_mode','/mavros/manual_control/send','/mavros/battery',
        '/operator/cmd_vel','/operator/deadman','/vehicle/software_estop','/vehicle/set_autonomy',
        '/vehicle/reset_low_voltage','/vehicle/battery_safety_status','/vehicle/software_stop_state','/vehicle/control_diagnostics',
        '/uuv/mavros/state','/uuv/safety/authorized','/uuv/mission/state','/uuv/follow/cmd_vel',
        '/uuv/follow/status','/uuv/follow/reset','/uuv/control/cmd_vel',
    ]
    remap = [(topic,'/sim'+topic) for topic in topics]
    common = {'datum_configured':True,'map_frame':'robotx_map',
              'origin_latitude':30.2,'origin_longitude':-92.0,'water_altitude':0.0}
    def node(package, executable, params=None):
        return Node(package=package,executable=executable,namespace='sim',output='screen',
                    parameters=[common,params or {}],remappings=remap)
    return LaunchDescription([
        node('robotx_coordination_sim','sim_world'),
        node('uav_mapping','mapper',{'georef_uncertainty':0.25}),
        node('boat_route_navigation','route_mission',{'goal_x':30.,'goal_y':0.,'bounds':[-10.,-20.,45.,20.]}),
        node('boat_control','target_controller'),
        node('boat_vehicle','mavros_command_bridge',{'battery_required_for_propulsion':False}),
        node('uuv_follow','follower'),
        node('uuv_follow','velocity_guard',{'output_topic':'/sim/uuv/mavros/setpoint_velocity/cmd_vel','require_frame_check':False}),
    ])
