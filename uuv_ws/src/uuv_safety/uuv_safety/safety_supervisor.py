#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from std_msgs.msg import Bool, String


class UuvSafetySupervisor(Node):
    """
    Safety authorization for the RobotX BlueROV2 Heavy.

    Autonomous propulsion is authorized only when:
      - MAVROS state is fresh
      - FC is connected
      - FC is armed
      - ArduSub is in an allowed autonomy mode
      - depth/local-position estimate is fresh
      - required camera is alive
      - required perception is alive
    """

    def __init__(self):
        super().__init__('uuv_safety_supervisor')

        self.allowed_modes = {
            x.strip().upper()
            for x in str(
                self.declare_parameter(
                    'allowed_modes',
                    'GUIDED'
                ).value
            ).split(',')
            if x.strip()
        }

        self.state_timeout = float(
            self.declare_parameter(
                'state_timeout',
                1.0
            ).value
        )

        self.depth_timeout = float(
            self.declare_parameter(
                'depth_timeout',
                1.0
            ).value
        )

        self.sensor_timeout = float(
            self.declare_parameter(
                'sensor_timeout',
                2.0
            ).value
        )

        self.depth_sign = float(
            self.declare_parameter(
                'depth_sign',
                -1.0
            ).value
        )

        self.require_depth = bool(
            self.declare_parameter(
                'require_depth',
                True
            ).value
        )

        self.require_camera = bool(
            self.declare_parameter(
                'require_camera',
                True
            ).value
        )

        self.require_perception = bool(
            self.declare_parameter(
                'require_perception',
                True
            ).value
        )

        self.state = None
        self.state_rx = None

        self.depth_m = None
        self.depth_rx = None

        self.camera = {}
        self.camera_rx = None

        self.perception = {}
        self.perception_rx = None

        self.auth_pub = self.create_publisher(
            Bool,
            '/uuv/safety/authorized',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/uuv/safety/status',
            10
        )

        self.create_subscription(
            State,
            '/uuv/mavros/state',
            self._state_cb,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/uuv/mavros/local_position/pose',
            self._pose_cb,
            10
        )

        self.create_subscription(
            String,
            '/uuv/camera/status',
            self._camera_cb,
            10
        )

        self.create_subscription(
            String,
            '/uuv/perception/status',
            self._perception_cb,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self._evaluate
        )

    def _state_cb(self, msg):
        self.state = msg
        self.state_rx = time.monotonic()

    def _pose_cb(self, msg):
        z = float(msg.pose.position.z)

        if math.isfinite(z):
            self.depth_m = max(
                0.0,
                self.depth_sign * z
            )
            self.depth_rx = time.monotonic()

    @staticmethod
    def _decode(msg):
        try:
            obj = json.loads(msg.data)

            if isinstance(obj, dict):
                return obj

        except Exception:
            pass

        return {}

    def _camera_cb(self, msg):
        self.camera = self._decode(msg)
        self.camera_rx = time.monotonic()

    def _perception_cb(self, msg):
        self.perception = self._decode(msg)
        self.perception_rx = time.monotonic()

    def _evaluate(self):
        now = time.monotonic()
        reasons = []

        state_fresh = (
            self.state_rx is not None
            and now - self.state_rx <= self.state_timeout
        )

        connected = bool(
            state_fresh
            and self.state is not None
            and self.state.connected
        )

        armed = bool(
            connected
            and self.state.armed
        )

        mode = (
            str(self.state.mode).upper()
            if self.state is not None
            else 'UNKNOWN'
        )

        mode_allowed = (
            connected
            and mode in self.allowed_modes
        )

        if not state_fresh:
            reasons.append(
                'flight-controller state stale'
            )

        elif not connected:
            reasons.append(
                'MAVROS not connected'
            )

        if connected and not armed:
            reasons.append(
                'vehicle not armed'
            )

        if connected and not mode_allowed:
            reasons.append(
                f'mode {mode} not allowed'
            )

        depth_fresh = (
            self.depth_rx is not None
            and now - self.depth_rx <= self.depth_timeout
            and self.depth_m is not None
            and math.isfinite(self.depth_m)
        )

        if self.require_depth and not depth_fresh:
            reasons.append(
                'depth/local-position estimate unavailable'
            )

        camera_fresh = (
            self.camera_rx is not None
            and now - self.camera_rx <= self.sensor_timeout
        )

        camera_ready = bool(
            camera_fresh
            and self.camera.get(
                'camera_alive',
                False
            )
        )

        if self.require_camera and not camera_ready:
            reasons.append(
                'camera unavailable'
            )

        perception_fresh = (
            self.perception_rx is not None
            and now - self.perception_rx <= self.sensor_timeout
        )

        perception_ready = bool(
            perception_fresh
            and self.perception.get(
                'perception_alive',
                False
            )
        )

        if (
            self.require_perception
            and not perception_ready
        ):
            reasons.append(
                'perception unavailable'
            )

        authorized = len(reasons) == 0

        auth = Bool()
        auth.data = authorized
        self.auth_pub.publish(auth)

        prearm_ready = bool(
            connected
            and (
                depth_fresh
                or not self.require_depth
            )
            and (
                camera_ready
                or not self.require_camera
            )
            and (
                perception_ready
                or not self.require_perception
            )
        )

        status = {
            'authorized': authorized,
            'safety_state':
                'AUTHORIZED'
                if authorized
                else 'BLOCKED',

            'safety_reason':
                'ready'
                if authorized
                else '; '.join(reasons),

            'platform': 'BlueROV2 Heavy',
            'frame_config': 2,
            'frame_name': 'Vectored_6DOF',

            'mode_allowed': mode_allowed,
            'prearm_ready': prearm_ready,

            'depth_ready': depth_fresh,
            'depth_m': self.depth_m,

            'camera_ready': camera_ready,
            'perception_ready': perception_ready,
        }

        out = String()
        out.data = json.dumps(
            status,
            separators=(',', ':')
        )

        self.status_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = UuvSafetySupervisor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
