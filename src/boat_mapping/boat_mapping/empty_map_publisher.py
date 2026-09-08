#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid


class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__("empty_map_publisher")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("width", 100)
        self.declare_parameter("height", 100)
        self.declare_parameter("resolution", 1.0)
        self.declare_parameter("origin_x", -50.0)
        self.declare_parameter("origin_y", -50.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.map_topic = self.get_parameter("map_topic").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(OccupancyGrid, self.map_topic, qos)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info(
            f"Publishing placeholder OccupancyGrid on {self.map_topic} in frame '{self.frame_id}'"
        )

    def publish_map(self):
        msg = OccupancyGrid()

        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        msg.info.map_load_time = now
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # 0 = free space. This is a placeholder, not a measured map.
        msg.data = [0] * (self.width * self.height)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmptyMapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
