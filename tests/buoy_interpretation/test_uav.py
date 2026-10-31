"""Run with unittest; no ROS runtime required."""
import math
import unittest
from collections import deque
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from robotx_buoy_interpretation.contracts import Camera, Detection
from robotx_buoy_interpretation.uav import interpret_uav, select_capture_calibration


CAMERA = Camera(640, 480, 400., 400., 320., 240.)
DOWN = (1., 0., 0., 0.)  # Optical x east, y south, z down.


def detection(**overrides):
    values = dict(detection_id=7, u=320., v=230., width=20., height=20.,
                  color='blue', confidence=.9, light_state='flashing')
    values.update(overrides)
    return Detection(**values)


def project(detections=None, **overrides):
    values = dict(detections=[detection()] if detections is None else detections,
                  camera=CAMERA, position=(10., 20., 10.), quaternion=DOWN,
                  stamp=10., now=10.1)
    values.update(overrides)
    return interpret_uav(**values)


class UavProjectionTests(unittest.TestCase):
    def test_capture_calibration_never_uses_future_stale_or_wrong_frame(self):
        samples = [(9.5, 'optical', 'old'), (10.01, 'optical', 'future'),
                   (10., 'other_camera', 'wrong_frame'), (9.98, 'optical', 'capture')]
        self.assertEqual(select_capture_calibration(samples, 10., 'optical'), 'capture')
        self.assertIsNone(select_capture_calibration(samples[:3], 10., 'optical'))

    def test_static_calibration_requires_explicit_opt_in(self):
        samples = [(0., 'optical', 'static')]
        self.assertIsNone(select_capture_calibration(samples, 10., 'optical'))
        self.assertEqual(select_capture_calibration(samples, 10., 'optical', allow_static=True),
                         'static')
        samples.append((10., 'optical', 'timed'))
        self.assertEqual(select_capture_calibration(samples, 10., 'optical', allow_static=True),
                         'timed')

    def test_bottom_contact_anchor_and_capture_metadata(self):
        observation, = project()
        self.assertAlmostEqual(observation.x, 10.)
        self.assertAlmostEqual(observation.y, 20.)
        self.assertEqual(observation.z, 0.)
        self.assertEqual(observation.source, 'uav')
        self.assertEqual(observation.stamp, 10.)
        self.assertEqual(observation.color, 'blue')
        self.assertEqual(observation.light_state, 'flashing')
        self.assertGreaterEqual(observation.uncertainty, .75)

    def test_offset_pixel_and_camera_yaw_change_world_projection(self):
        observation, = project([detection(u=360.)])
        self.assertAlmostEqual(observation.x, 11.)
        self.assertAlmostEqual(observation.y, 20.)
        s = math.sqrt(.5)
        rotated, = project([detection(u=360.)], quaternion=(s, s, 0., 0.))
        self.assertAlmostEqual(rotated.x, 10.)
        self.assertAlmostEqual(rotated.y, 21.)

    def test_arbitrary_oblique_attitude_is_supported(self):
        q = (math.sin(math.radians(75)), 0., 0., math.cos(math.radians(75)))
        observation, = project(quaternion=q)
        self.assertAlmostEqual(observation.x, 10.)
        self.assertAlmostEqual(observation.y, 20.-10./math.sqrt(3.))

    def test_stale_future_and_zero_timestamps_rejected(self):
        for stamp, now in [(9., 10.), (10.01, 10.), (0., .1)]:
            with self.subTest(stamp=stamp, now=now), self.assertRaises(ValueError):
                project(stamp=stamp, now=now)

    def test_horizon_upward_and_range_rejections(self):
        self.assertEqual(project(quaternion=(0., 0., 0., 1.)), [])
        self.assertEqual(project(quaternion=(math.sqrt(.5), 0., 0., math.sqrt(.5))), [])
        self.assertEqual(project(max_range=5.), [])

    def test_invalid_camera_pose_rejected(self):
        for position in [(0., 0., 0.), (0., 0., -1.), (float('nan'), 0., 3.)]:
            with self.subTest(position=position), self.assertRaises(ValueError):
                project(position=position)

    def test_invalid_camera_and_quaternion_rejected_even_without_detections(self):
        with self.assertRaises(ValueError):
            project([], camera=Camera(640, 480, 0., 400., 320., 240.))
        with self.assertRaises(ValueError):
            project([], quaternion=(0., 0., 0., 0.))

    def test_bad_low_confidence_clipped_and_duplicate_objects_are_omitted(self):
        valid = detection()
        items = [detection(detection_id=1, confidence=.2), detection(detection_id=2, u=-1.),
                 detection(detection_id=3, v=478.), detection(detection_id=4, color='purple'),
                 valid, valid]
        self.assertEqual(len(project(items)), 1)

    def test_configurable_center_anchor_and_uncertainty_increase(self):
        bottom, = project()
        center, = project(anchor_fraction=0.)
        self.assertAlmostEqual(center.y, 20.25)
        wider, = project([detection(width=100.)])
        self.assertGreater(wider.uncertainty, bottom.uncertainty)
        near, = project([detection(u=360.)], max_buoy_height=0.)
        tall, = project([detection(u=360.)], max_buoy_height=2.)
        self.assertGreater(tall.uncertainty, near.uncertainty)


def _stamp(seconds):
    sec = math.floor(seconds)
    return SimpleNamespace(sec=sec, nanosec=round((seconds-sec)*1e9))


def _node_module():
    """Exercise actual shared wire helpers against minimal ROS transport stubs."""
    class FakeTime:
        def __init__(self, seconds=0.):
            self.seconds = seconds

        def to_msg(self):
            return _stamp(self.seconds)

        @staticmethod
        def from_msg(stamp):
            return stamp

    class ObservationMsg:
        def __init__(self):
            self.position = SimpleNamespace(x=0., y=0., z=0.)

    class ArrayMsg:
        def __init__(self):
            self.header = SimpleNamespace(stamp=_stamp(0.), frame_id='')
            self.reference_id = ''
            self.observations = []

    class TransformError(Exception):
        pass

    names = {
        'rclpy': {}, 'rclpy.duration': {'Duration': object},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'rclpy.time': {'Time': FakeTime},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'CameraInfo': object},
        'std_msgs': {}, 'std_msgs.msg': {'String': object},
        'tf2_ros': {'Buffer': object, 'TransformException': TransformError,
                    'TransformListener': object},
        'robotx_coordination_interfaces': {},
        'robotx_coordination_interfaces.msg': {
            'BuoyColorDetectionArray': object, 'BuoyObservationArray': ArrayMsg,
            'BuoyObservation': ObservationMsg},
    }
    modules = {}
    for name, members in names.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        modules[name] = module
    base = Path(__file__).resolve().parents[2] / 'coordination_ws/src/robotx_buoy_interpretation/robotx_buoy_interpretation'
    with patch.dict(sys.modules, modules):
        helper_name = 'robotx_buoy_interpretation.ros_support'
        helper_spec = importlib.util.spec_from_file_location(helper_name, base / 'ros_support.py')
        helper = importlib.util.module_from_spec(helper_spec)
        helper_spec.loader.exec_module(helper)
        with patch.dict(sys.modules, {helper_name: helper}):
            spec = importlib.util.spec_from_file_location(
                'robotx_buoy_interpretation._uav_node_test', base / 'uav_node.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
    module.TestTransformError = TransformError
    return module


def _image_msg(stamp=10., frame='uav_optical', width=640, height=480):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(stamp), frame_id=frame),
        image_width=width, image_height=height, image_rectified=True, detections=[detection()])


def _info(stamp=10., frame='uav_optical', width=640, height=480):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(stamp), frame_id=frame),
        width=width, height=height, r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
        p=[400., 0., 320., 0., 0., 400., 240., 0., 0., 0., 1., 0.])


def _node(module):
    node = module.UavInterpretation.__new__(module.UavInterpretation)
    node.enabled = True
    node.frame = 'robotx_map'
    node.ref = 'surveyed_datum_id'
    node.timeout = .5
    node.max_range = 60.
    node.min_confidence = .6
    node.uncertainty = .75
    node.pixel_uncertainty = 2.
    node.anchor_fraction = .5
    node.radius = .2
    node.height_bound = 1.5
    node.calibration_timeout = .1
    node.allow_static_calibration = False
    node.calibrations = deque(maxlen=60)
    node.on_info(_info())
    node.pending = None
    node.last_received_stamp = 0.
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10100000000))
    node.messages = []
    node.publisher = SimpleNamespace(publish=node.messages.append)
    node.states = []
    node.status = node.states.append
    node.transforms = []

    def lookup(target, source, stamp):
        node.transforms.append((target, source, stamp))
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=10., y=20., z=10.),
            rotation=SimpleNamespace(x=1., y=0., z=0., w=0.)))

    node.buffer = SimpleNamespace(lookup_transform=lookup)
    return node


class UavRosWrapperTests(unittest.TestCase):
    def test_rotated_rectification_is_not_used_with_original_optical_tf(self):
        module = _node_module()
        node = _node(module)
        node.calibrations[-1][2].r = [0., -1., 0., 1., 0., 0., 0., 0., 1.]
        node.on_detections(_image_msg())
        node.process_pending()
        self.assertEqual(node.messages, [])
        self.assertTrue(node.states[-1].startswith('REJECTED:'))

    def test_raw_pixels_are_not_silently_interpreted_as_rectified(self):
        module = _node_module()
        node = _node(module)
        image = _image_msg()
        image.image_rectified = False
        node.on_detections(image)
        node.process_pending()
        self.assertEqual(node.messages, [])
        self.assertTrue(node.states[-1].startswith('REJECTED:'))

    def test_capture_tf_and_common_reference_output_use_actual_wire_helpers(self):
        module = _node_module()
        node = _node(module)
        image = _image_msg()
        node.on_detections(image)
        node.process_pending()
        self.assertEqual(len(node.transforms), 1)
        self.assertEqual(node.transforms[0][:2], ('robotx_map', 'uav_optical'))
        self.assertIs(node.transforms[0][2], image.header.stamp)
        self.assertEqual(len(node.messages), 1)
        output = node.messages[0]
        self.assertEqual(output.reference_id, 'surveyed_datum_id')
        self.assertEqual(output.header.frame_id, 'robotx_map')
        self.assertIs(output.header.stamp, image.header.stamp)
        observation, = output.observations
        self.assertEqual(observation.source, 'uav')
        self.assertEqual(observation.color, 'blue')
        self.assertEqual(observation.observed_at.sec, 10)
        self.assertAlmostEqual(observation.position.x, 10.)
        self.assertIsNone(node.pending)

    def test_dimension_and_projection_calibration_rejections(self):
        module = _node_module()
        for width, projection_offset in [(320, 0.), (640, -10.)]:
            with self.subTest(width=width, projection_offset=projection_offset):
                node = _node(module)
                if projection_offset:
                    node.calibrations[-1][2].p[3] = projection_offset
                node.on_detections(_image_msg(width=width))
                node.process_pending()
                self.assertEqual(node.messages, [])
                self.assertIsNone(node.pending)
                self.assertTrue(node.states[-1].startswith('REJECTED:'))

    def test_camera_header_mismatch_waits_without_using_other_camera(self):
        module = _node_module()
        node = _node(module)
        node.on_detections(_image_msg(frame='other_optical'))
        node.process_pending()
        self.assertEqual(node.messages, [])
        self.assertEqual(node.transforms, [])
        self.assertEqual(node.states[-1], 'WAITING_FOR_CAPTURE_CALIBRATION')

    def test_future_then_valid_then_replayed_capture(self):
        module = _node_module()
        node = _node(module)
        node.on_detections(_image_msg(stamp=100.))
        self.assertEqual(node.last_received_stamp, 0.)
        self.assertIsNone(node.pending)
        image = _image_msg()
        node.on_detections(image)
        node.process_pending()
        node.on_detections(image)
        node.process_pending()
        self.assertEqual(len(node.messages), 1)
        self.assertEqual(node.states[-1], 'REPLAY_OR_OUT_OF_ORDER')

    def test_missing_tf_retries_only_until_original_capture_expires(self):
        module = _node_module()
        node = _node(module)
        node.on_detections(_image_msg())

        def missing(*args):
            raise module.TestTransformError('No acquisition-time TF')

        node.buffer.lookup_transform = missing
        node.process_pending()
        self.assertIsNotNone(node.pending)
        self.assertEqual(node.states[-1], 'WAITING_FOR_CAPTURE_TF')
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11000000000))
        node.process_pending()
        self.assertIsNone(node.pending)
        self.assertEqual(node.messages, [])
        self.assertEqual(node.states[-1], 'CAPTURE_STALE_OR_FUTURE')

    def test_disabled_datum_cannot_publish_observations(self):
        module = _node_module()
        node = _node(module)
        node.enabled = False
        node.on_detections(_image_msg())
        node.process_pending()
        self.assertIsNone(node.pending)
        self.assertEqual(node.messages, [])
        self.assertEqual(node.states[-1], 'DATUM_NOT_CONFIGURED')


if __name__ == '__main__':
    unittest.main()
