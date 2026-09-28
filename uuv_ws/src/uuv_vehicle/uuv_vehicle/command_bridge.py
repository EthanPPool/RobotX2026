#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class UuvCommandBridge(Node):
    """Safety-gated velocity setpoint bridge with a command deadman."""

    def __init__(self):
        super().__init__('uuv_command_bridge')
        self.input_topic = str(self.declare_parameter('input_topic', '/uuv/control/cmd_vel').value)
        self.output_topic = str(self.declare_parameter('output_topic', '/uuv/mavros/setpoint_velocity/cmd_vel').value)
        self.authorized_topic = str(self.declare_parameter('authorized_topic', '/uuv/safety/authorized').value)
        self.deadman_sec = float(self.declare_parameter('deadman_sec', 0.35).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)
        self.max_linear_xy = float(self.declare_parameter('max_linear_xy', 0.5).value)
        self.max_linear_z = float(self.declare_parameter('max_linear_z', 0.3).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 0.5).value)

        self.authorized = False
        self.last_cmd = None
        self.last_cmd_time = None
        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.create_subscription(TwistStamped, self.input_topic, self._cmd_cb, 10)
        self.create_subscription(Bool, self.authorized_topic, self._authorized_cb, 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._tick)

    def _authorized_cb(self, msg: Bool):
        self.authorized = bool(msg.data)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, float(value)))

    def _cmd_cb(self, msg: TwistStamped):
        clean = TwistStamped()
        clean.header = msg.header
        clean.twist.linear.x = self._clamp(msg.twist.linear.x, self.max_linear_xy)
        clean.twist.linear.y = self._clamp(msg.twist.linear.y, self.max_linear_xy)
        clean.twist.linear.z = self._clamp(msg.twist.linear.z, self.max_linear_z)
        clean.twist.angular.z = self._clamp(msg.twist.angular.z, self.max_yaw_rate)
        self.last_cmd = clean
        self.last_cmd_time = time.monotonic()

    def _zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        return msg

    def _tick(self):
        fresh = bool(self.last_cmd_time is not None and time.monotonic() - self.last_cmd_time <= self.deadman_sec)
        if not self.authorized or not fresh or self.last_cmd is None:
            self.pub.publish(self._zero())
            return
        msg = self.last_cmd
        msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'base_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UuvCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(node._zero())
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
