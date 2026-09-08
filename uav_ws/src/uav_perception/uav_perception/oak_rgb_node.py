#!/usr/bin/env python3

import depthai as dai
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class OakRgbNode(Node):

    def __init__(self):
        super().__init__('oak_rgb_node')

        self.declare_parameter('frame_id', 'oak_rgb_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 15.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        self.publisher = self.create_publisher(
            Image,
            '/oak/rgb/image_raw',
            10,
        )

        self.bridge = CvBridge()

        self.pipeline = dai.Pipeline()

        self.camera = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        self.queue = self.camera.requestOutput(
            size=(self.width, self.height),
            fps=self.fps,
        ).createOutputQueue()

        self.pipeline.start()

        device = self.pipeline.getDefaultDevice()

        self.get_logger().info(
            f'OAK-D connected: {device.getDeviceId()}'
        )
        self.get_logger().info(
            f'USB speed: {device.getUsbSpeed()}'
        )
        self.get_logger().info(
            f'Publishing {self.width}x{self.height} @ {self.fps} FPS '
            f'on /oak/rgb/image_raw'
        )

        self.timer = self.create_timer(
            1.0 / self.fps,
            self.publish_frame,
        )

    def publish_frame(self):
        frame = self.queue.tryGet()

        if frame is None:
            return

        image = frame.getCvFrame()

        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8',
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OakRgbNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
