"""Run explicitly in a built Kilted environment, not the offline unit suite."""
import copy
import importlib.util
import os
import time
import unittest

import numpy as np
import rclpy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node as LaunchNode
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo
from robotx_coordination_interfaces.msg import BuoyColorDetectionArray

from robotx_buoy_interpretation.hsv_node import HsvColorClusters, detection_array


class RosHsvTests(unittest.TestCase):
    def test_camera_launch_starts_only_one_camera_processing_node(self):
        source = os.path.join(get_package_share_directory('robotx_buoy_interpretation'),
                              'launch', 'hsv_camera.launch.py')
        spec = importlib.util.spec_from_file_location('hsv_camera_launch', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        nodes = [e for e in module.generate_launch_description().entities
                 if isinstance(e, LaunchNode)]
        self.assertEqual(len(nodes), 1)

    def test_real_messages_and_producer(self):
        rclpy.init()
        producer = None
        executor = SingleThreadedExecutor()
        try:
            producer = HsvColorClustersWithOverrides()
            sink = rclpy.create_node('hsv_test_sink')
            executor.add_node(producer)
            executor.add_node(sink)
            received = []
            subscription = sink.create_subscription(BuoyColorDetectionArray,
                '/perception/usv/color_detections', received.append, 10)
            deadline = time.monotonic()+5
            while producer.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertGreater(producer.publisher.get_subscription_count(), 0)
            self.assertEqual(producer.get_subscriptions_info_by_topic('/perception/objects'), [])
            pixels = np.zeros((200, 300, 3), np.uint8)
            pixels[50:80, 60:90] = (0, 0, 255)
            image = CvBridge().cv2_to_imgmsg(pixels, encoding='bgr8')
            image.header.stamp = producer.get_clock().now().to_msg()
            image.header.frame_id = 'test_camera_optical'
            camera = CameraInfo()
            camera.header = copy.deepcopy(image.header)
            camera.width, camera.height = 300, 200
            camera.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            camera.p = [200., 0., 150., 0., 0., 200., 100., 0., 0., 0., 1., 0.]
            producer.on_image(image)  # Image before CameraInfo must wait.
            self.assertIsNotNone(producer.pending)
            producer.on_info(camera)
            deadline = time.monotonic()+2
            while not received and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(len(received), 1)
            result = received[0]
            self.assertEqual(result.header, image.header)
            self.assertTrue(result.image_rectified)
            self.assertEqual((result.image_width, result.image_height), (300, 200))
            self.assertEqual(len(result.detections), 1)
            self.assertEqual(result.detections[0].color, 'red')
            self.assertEqual(result.detections[0].light_state, 'unknown')
            producer.on_image(image)  # Replays cannot refresh the capture.
            self.assertIsNone(producer.pending)
            empty = detection_array(image, [])
            self.assertEqual(len(empty.detections), 0)
            self.assertEqual(empty.header, image.header)
            self.assertIsNotNone(subscription)
        finally:
            executor.shutdown()
            if producer is not None:
                producer.destroy_node()
            if 'sink' in locals():
                sink.destroy_node()
            rclpy.shutdown()


class HsvColorClustersWithOverrides(HsvColorClusters):
    def declare_parameter(self, name, value=None, *args, **kwargs):
        if name == 'image_topic':
            value = '/test/hsv/image'
        elif name == 'input_rectified':
            value = True
        elif name == 'max_image_age':
            value = 2.0
        return super().declare_parameter(name, value, *args, **kwargs)


if __name__ == '__main__':
    unittest.main()
