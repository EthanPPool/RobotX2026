#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, NavSatFix
from std_msgs.msg import String


def q_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class UuvStatusAggregator(Node):
    def __init__(self):
        super().__init__('uuv_status_aggregator')
        self.state_timeout = float(self.declare_parameter('state_timeout', 1.5).value)
        self.depth_sign = float(self.declare_parameter('depth_sign', -1.0).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 5.0).value)
        self.data = {
            'id': 'uuv',
            'name': 'BlueROV2 Heavy',
            'type': 'UUV',

            'platform': 'BlueROV2 Heavy',
            'manufacturer': 'Blue Robotics',
            'autopilot': 'ArduSub',

            'frame_config': 2,
            'frame_name': 'Vectored_6DOF',

            'thruster_count': 8,
            'thruster_model': 'T200',

            'enclosure_material': 'acrylic',
            'platform_depth_rating_m': 100.0,

            'full_6dof_frame': True,

            'autonomy_command_axes': [
                'surge',
                'sway',
                'heave',
                'yaw_rate',
            ],

            'fc_stabilized_axes': [
                'roll',
                'pitch',
            ],

            'velocity_command_frame':
                'BODY_NED',

            'connected': False, 'online': False, 'vehicle_link': False,
            'jetson_link': True, 'mavlink_link': False, 'mavros_connected': False,
            'mavros_state_fresh': False, 'armed': False, 'mode': 'UNKNOWN',
            'system_status': None, 'voltage': None, 'battery_percent': None,
            'battery_remaining': None, 'battery_current': None,
            'latitude': None, 'longitude': None, 'altitude': None,
            'local_x': None, 'local_y': None, 'local_z': None, 'depth_m': None,
            'roll_deg': None, 'pitch_deg': None, 'yaw_deg': None, 'heading_deg': None,
            'velocity_north': None, 'velocity_east': None, 'velocity_up': None,
            'pressure_kpa': None, 'camera_alive': False, 'camera_fps': None,
            'perception_alive': False, 'perception_backend': None, 'perception_model': None,
            'perception_fps': None, 'detection_count': 0, 'top_detection_label': None,
            'top_detection_confidence': None, 'autonomy_enabled': False,
            'autonomy_state': 'IDLE', 'authorized': False, 'safety_state': 'UNKNOWN',
            'safety_reason': None,
        }
        self.last_state_rx = None
        self.pub = self.create_publisher(String, '/uuv/vehicle/status', 10)
        self.create_subscription(State, '/uuv/mavros/state', self._state, 10)
        self.create_subscription(BatteryState, '/uuv/mavros/battery', self._battery, 10)
        self.create_subscription(NavSatFix, '/uuv/mavros/global_position/global', self._gps, 10)
        self.create_subscription(PoseStamped, '/uuv/mavros/local_position/pose', self._pose, 10)
        self.create_subscription(TwistStamped, '/uuv/mavros/local_position/velocity_local', self._velocity, 10)
        self.create_subscription(Imu, '/uuv/mavros/imu/data', self._imu, 10)
        self.create_subscription(FluidPressure, '/uuv/depth/pressure', self._pressure, 10)
        for topic, cb in [
            ('/uuv/camera/status', self._json_merge),
            ('/uuv/perception/status', self._json_merge),
            ('/uuv/mission/state', self._json_merge),
            ('/uuv/safety/status', self._json_merge),
        ]:
            self.create_subscription(String, topic, cb, 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish)

    def _state(self, msg: State):
        self.last_state_rx = time.monotonic()
        self.data.update({
            'connected': bool(msg.connected), 'mavros_connected': bool(msg.connected),
            'mavlink_link': bool(msg.connected), 'armed': bool(msg.armed),
            'mode': str(msg.mode or 'UNKNOWN'), 'system_status': int(msg.system_status),
        })

    def _battery(self, msg: BatteryState):
        def finite_or_none(x):
            return float(x) if math.isfinite(float(x)) else None
        pct = finite_or_none(msg.percentage)
        if pct is not None and 0.0 <= pct <= 1.0:
            pct *= 100.0
        self.data['voltage'] = finite_or_none(msg.voltage)
        self.data['battery_current'] = finite_or_none(msg.current)
        self.data['battery_percent'] = pct
        self.data['battery_remaining'] = pct

    def _gps(self, msg: NavSatFix):
        self.data['latitude'] = float(msg.latitude) if math.isfinite(msg.latitude) else None
        self.data['longitude'] = float(msg.longitude) if math.isfinite(msg.longitude) else None
        self.data['altitude'] = float(msg.altitude) if math.isfinite(msg.altitude) else None

    def _pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.data.update({'local_x': float(p.x), 'local_y': float(p.y), 'local_z': float(p.z)})
        self.data['depth_m'] = max(0.0, self.depth_sign * float(p.z))
        r, pch, y = q_to_rpy(msg.pose.orientation)
        self.data.update({'roll_deg': r, 'pitch_deg': pch, 'yaw_deg': y, 'heading_deg': y % 360.0})

    def _imu(self, msg: Imu):
        r, pch, y = q_to_rpy(msg.orientation)
        self.data.update({'roll_deg': r, 'pitch_deg': pch, 'yaw_deg': y, 'heading_deg': y % 360.0})

    def _velocity(self, msg: TwistStamped):
        self.data['velocity_north'] = float(msg.twist.linear.x)
        self.data['velocity_east'] = float(msg.twist.linear.y)
        self.data['velocity_up'] = float(msg.twist.linear.z)

    def _pressure(self, msg: FluidPressure):
        if math.isfinite(msg.fluid_pressure):
            self.data['pressure_kpa'] = float(msg.fluid_pressure) / 1000.0

    def _json_merge(self, msg: String):
        try:
            obj = json.loads(msg.data)
            if isinstance(obj, dict):
                self.data.update(obj)
        except Exception:
            pass

    def _publish(self):
        age = None if self.last_state_rx is None else time.monotonic() - self.last_state_rx
        fresh = bool(age is not None and age <= self.state_timeout)
        self.data['mavros_state_fresh'] = fresh
        self.data['vehicle_link'] = bool(fresh and self.data.get('mavros_connected'))
        self.data['online'] = self.data['vehicle_link']
        self.data['state_age_sec'] = None if age is None else round(age, 3)
        msg = String(); msg.data = json.dumps(self.data, separators=(',', ':'))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UuvStatusAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
