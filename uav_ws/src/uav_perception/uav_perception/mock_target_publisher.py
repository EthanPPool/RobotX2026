#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MockTargetPublisher(Node):
    """Publishes a synthetic target for no-hardware integration testing."""

    def __init__(self):
        super().__init__('mock_target_publisher')
        self.declare_parameter('topic', '/perception/target_raw')
        self.declare_parameter('x', 2.0)
        self.declare_parameter('y', 0.5)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('rate', 5.0)
        self.pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('topic').value), 10)
        rate = max(0.5, float(self.get_parameter('rate').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(self.get_parameter('x').value)
        msg.pose.position.y = float(self.get_parameter('y').value)
        msg.pose.position.z = float(self.get_parameter('z').value)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
