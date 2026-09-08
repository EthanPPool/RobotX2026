#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from uav_interfaces.msg import MissionCommand


class TargetMission(Node):
    """Minimal high-level mission controller for stack integration.

    This node does NOT arm, change flight mode, take off, or land. It only emits
    bounded body-frame velocity requests after /mission/start is called.
    """

    def __init__(self):
        super().__init__('target_mission')
        self.declare_parameter('target_topic', '/perception/target')
        self.declare_parameter('command_topic', '/mission/command')
        self.declare_parameter('target_timeout', 0.5)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('kp_xy', 0.35)
        self.declare_parameter('kp_z', 0.35)
        self.declare_parameter('desired_forward_distance', 1.5)
        self.declare_parameter('xy_deadband', 0.15)
        self.declare_parameter('z_deadband', 0.15)
        self.declare_parameter('max_horizontal_speed', 0.75)
        self.declare_parameter('max_vertical_speed', 0.35)

        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.desired_forward = float(self.get_parameter('desired_forward_distance').value)
        self.xy_deadband = float(self.get_parameter('xy_deadband').value)
        self.z_deadband = float(self.get_parameter('z_deadband').value)
        self.max_horizontal = float(self.get_parameter('max_horizontal_speed').value)
        self.max_vertical = float(self.get_parameter('max_vertical_speed').value)

        self.active = False
        self.aborted = False
        self.target = None
        self.target_rx_time = None

        self.command_pub = self.create_publisher(
            MissionCommand, str(self.get_parameter('command_topic').value), 10)
        self.target_sub = self.create_subscription(
            PoseStamped, str(self.get_parameter('target_topic').value), self._target_cb, 10)
        self.start_srv = self.create_service(Trigger, '/mission/start', self._start)
        self.abort_srv = self.create_service(Trigger, '/mission/abort', self._abort)
        self.reset_srv = self.create_service(Trigger, '/mission/reset', self._reset)

        rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('Mission initialized in IDLE. Call /mission/start explicitly.')

    def _target_cb(self, msg):
        self.target = msg
        self.target_rx_time = self.get_clock().now()

    def _start(self, request, response):
        del request
        if self.aborted:
            response.success = False
            response.message = 'Mission is ABORTED; call /mission/reset first.'
            return response
        self.active = True
        response.success = True
        response.message = 'Mission ACTIVE. Vehicle bridge safety gate remains independent.'
        self.get_logger().warn(response.message)
        return response

    def _abort(self, request, response):
        del request
        self.active = False
        self.aborted = True
        response.success = True
        response.message = 'Mission ABORTED; HOLD commands will be emitted.'
        self.get_logger().error(response.message)
        return response

    def _reset(self, request, response):
        del request
        self.active = False
        self.aborted = False
        response.success = True
        response.message = 'Mission reset to IDLE.'
        return response

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _target_fresh(self):
        if self.target_rx_time is None:
            return False
        age = (self.get_clock().now() - self.target_rx_time).nanoseconds / 1e9
        return age <= self.target_timeout

    def _hold(self, source):
        cmd = MissionCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.command = MissionCommand.HOLD
        cmd.source = source
        return cmd

    def _tick(self):
        if self.aborted:
            self.command_pub.publish(self._hold('target_mission:ABORTED'))
            return
        if not self.active:
            self.command_pub.publish(self._hold('target_mission:IDLE'))
            return
        if not self._target_fresh() or self.target is None:
            self.command_pub.publish(self._hold('target_mission:TARGET_STALE'))
            return

        p = self.target.pose.position
        ex = float(p.x) - self.desired_forward
        ey = float(p.y)
        ez = float(p.z)

        vx = 0.0 if abs(ex) <= self.xy_deadband else self.kp_xy * ex
        vy = 0.0 if abs(ey) <= self.xy_deadband else self.kp_xy * ey
        vz = 0.0 if abs(ez) <= self.z_deadband else self.kp_z * ez

        norm = math.hypot(vx, vy)
        if norm > self.max_horizontal and norm > 0.0:
            scale = self.max_horizontal / norm
            vx *= scale
            vy *= scale

        cmd = MissionCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.command = MissionCommand.VELOCITY
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = self._clamp(vz, self.max_vertical)
        cmd.yaw_rate = 0.0
        cmd.source = 'target_mission:TRACK'
        self.command_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TargetMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
