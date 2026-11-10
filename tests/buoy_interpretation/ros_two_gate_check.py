"""Kilted HSV -> typed colored objects -> existing gate detector, no hardware."""
from copy import deepcopy
import time
import unittest

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo
from boat_interfaces.msg import DetectedObject, DetectedObjectArray, GateArray
from boat_perception.gate_detector import GateDetector
from robotx_coordination_interfaces.msg import BuoyColorDetectionArray
from robotx_buoy_interpretation.gate_color_node import GateColorAdapter
from robotx_buoy_interpretation.ros_support import stamp_seconds
from ros_hsv_check import HsvColorClustersWithOverrides


class ColorGateDetector(GateDetector):
    def declare_parameter(self, name, value=None, *args, **kwargs):
        value = {'objects_topic': '/perception/colored_objects',
                 'require_red_green': True}.get(name, value)
        return super().declare_parameter(name, value, *args, **kwargs)


class CalibratedTransformFixture:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp))
        tf = TransformStamped()
        # Synthetic camera: optical right=-body Y, down=-body Z, forward=body X.
        tf.transform.rotation.x, tf.transform.rotation.y = .5, -.5
        tf.transform.rotation.z, tf.transform.rotation.w = .5, .5
        tf.transform.translation.y = .5
        return tf


class RosTwoGateTests(unittest.TestCase):
    def test_camera_to_two_gate_pipeline_and_source_dropouts(self):
        rclpy.init()
        executor = SingleThreadedExecutor()
        nodes = []
        try:
            camera = HsvColorClustersWithOverrides()
            adapter = GateColorAdapter()
            adapter.buffer = CalibratedTransformFixture()
            gate = ColorGateDetector()
            sink = rclpy.create_node('two_gate_color_test_sink')
            nodes = [camera, adapter, gate, sink]
            for node in nodes:
                executor.add_node(node)
            gates, colors, labels = [], [], []
            subscriptions = [
                sink.create_subscription(GateArray, '/perception/gates', gates.append, 10),
                sink.create_subscription(BuoyColorDetectionArray,
                    '/perception/usv/color_detections', colors.append, 10),
                sink.create_subscription(DetectedObjectArray,
                    '/perception/colored_objects', labels.append, 10),
            ]

            def spin_until(predicate, timeout=3.):
                deadline = time.monotonic()+timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(predicate())

            spin_until(lambda: camera.publisher.get_subscription_count() >= 2
                       and adapter.publisher.get_subscription_count() >= 2)
            pixels = np.zeros((400, 800, 3), np.uint8)
            pixels[185:210, 290:315] = (0, 255, 0)
            pixels[185:210, 485:510] = (0, 0, 255)

            def capture():
                image = CvBridge().cv2_to_imgmsg(pixels, encoding='bgr8')
                image.header.stamp = camera.get_clock().now().to_msg()
                image.header.frame_id = 'test_camera_optical'
                info = CameraInfo()
                info.header = deepcopy(image.header)
                info.width, info.height = 800, 400
                info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                info.p = [400., 0., 400., 0., 0., 400., 200., 0., 0., 0., 1., 0.]
                camera.on_info(info)
                adapter.on_info(info)
                return image

            def geometry(stamp):
                array = DetectedObjectArray()
                array.header.stamp = deepcopy(stamp)
                array.header.frame_id = 'base_link'
                for i, y in ((1, 1.22), (2, -1.22)):
                    obj = DetectedObject()
                    obj.id, obj.object_type, obj.confidence = i, 1, .8
                    obj.position.x, obj.position.y, obj.position.z = 5., y, .5
                    obj.size.x, obj.size.y, obj.size.z = .3, .4, 1.
                    array.objects.append(obj)
                return array

            for _ in range(2):
                image = capture()
                raw = geometry(image.header.stamp)
                before = len(labels)
                adapter.on_objects(raw)
                camera.on_image(image)
                spin_until(lambda: len(labels) > before)
                spin_until(lambda: gate.last_input_stamp == stamp_seconds(raw.header.stamp))
                self.assertEqual(labels[-1].header, raw.header)
                self.assertEqual([o.color for o in labels[-1].objects], [2, 1])
                self.assertEqual([o.color for o in raw.objects], [0, 0])
            spin_until(lambda: any(m.gates for m in gates))
            self.assertEqual(len(gate.confirmed.gates), 1)
            self.assertAlmostEqual(gate.confirmed.gates[0].width, 2.44, places=5)
            self.assertEqual(adapter.buffer.calls[-1][:2], ('test_camera_optical', 'base_link'))

            # No LiDAR input: camera still publishes colors, no positioned output.
            before_colors, before_labels = len(colors), len(labels)
            camera.on_image(capture())
            spin_until(lambda: len(colors) > before_colors)
            self.assertEqual(len(labels), before_labels)

            # No camera input: a fresh LiDAR scan is retained as unknown, not
            # colored from an old image. The red/green gate is withheld.
            deadline = time.monotonic()+.15
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            raw = geometry(camera.get_clock().now().to_msg())
            adapter.on_objects(raw)
            spin_until(lambda: len(labels) > before_labels)
            spin_until(lambda: gate.last_input_stamp == stamp_seconds(raw.header.stamp))
            self.assertEqual([o.color for o in labels[-1].objects], [0, 0])
            self.assertEqual(len(labels[-1].objects), 2)
            self.assertEqual(gate.confirmed.gates, [])
            self.assertEqual(len(subscriptions), 3)
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
