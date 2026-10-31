"""USV image/geometry association behavior without a ROS installation."""
import math
import unittest
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
from collections import deque

# Load numerical dependencies outside patch.dict so mocked ROS imports do not
# unload and reimport NumPy between transport tests.
import numpy as np
import robotx_coordination.geometry

from robotx_buoy_interpretation.contracts import Camera, Detection
from robotx_buoy_interpretation.usv import (
    GeometryCandidate, geometry_observations, interpret_usv)


CAMERA = Camera(640, 480, 400.0, 400.0, 320.0, 240.0)


def candidate(identifier=1, optical=(0.0, 0.0, 10.0), position=(12.0, 8.0, 0.0)):
    return GeometryCandidate(identifier, position, optical, 0.25, 0.2, 0.95)


def detection(identifier=1, u=320.0, color='green', confidence=0.9, width=30.0):
    return Detection(identifier, u, 240.0, width, 30.0, color, confidence)


def interpret(detections, candidates, **kwargs):
    return interpret_usv(detections, candidates, CAMERA, 10.0, 9.95, 10.1, **kwargs)


def test_color_joins_map_geometry_without_replacing_position():
    observation, = interpret([detection()], [candidate()])
    assert (observation.x, observation.y, observation.z) == (12.0, 8.0, 0.0)
    assert observation.color == 'green'
    assert observation.stamp == 9.95
    assert observation.source == 'usv'
    assert observation.confidence == 0.9


def test_unmatched_behind_camera_and_out_of_view_remain_unknown_obstacles():
    result = interpret([detection()], [candidate(1),
        candidate(2, optical=(0.0, 0.0, -2.0)),
        candidate(3, optical=(50.0, 0.0, 1.0)),
        candidate(4, optical=(0.0, 0.0, 0.0))])
    assert [o.color for o in result] == ['green', 'unknown', 'unknown', 'unknown']


def test_two_geometry_candidates_in_one_box_do_not_borrow_same_color():
    result = interpret([detection()], [candidate(1), candidate(2, (0.1, 0.0, 10.0))])
    assert [o.color for o in result] == ['unknown', 'unknown']


def test_two_overlapping_boxes_do_not_guess_assignment():
    result = interpret([detection(1, color='red'), detection(2, u=325.0)], [candidate()])
    assert result[0].color == 'unknown'


def test_independent_buoys_have_one_to_one_color_assignments():
    result = interpret([detection(1, u=280.0, color='red'), detection(2, u=360.0)],
                       [candidate(1, (-1.0, 0.0, 10.0)), candidate(2, (1.0, 0.0, 10.0))])
    assert [o.color for o in result] == ['red', 'green']


def test_low_confidence_detection_does_not_erase_geometry():
    result = interpret([detection(confidence=0.3)], [candidate()])
    assert result[0].color == 'unknown'


def test_stale_future_and_unsynchronized_captures_rejected():
    for image_stamp, geometry_stamp, now in [
            (10.0, 9.8, 10.1), (10.0, 9.95, 11.0), (11.0, 11.0, 10.0),
            (10.01, 10.01, 10.0), (0.0, 0.0, 0.0), (math.nan, 10.0, 10.1)]:
        with unittest.TestCase().assertRaises(ValueError):
            interpret_usv([detection()], [candidate()], CAMERA,
                          image_stamp, geometry_stamp, now)


def test_pixel_gate_limits_large_bounding_boxes():
    result = interpret([detection(u=400.0, width=300.0)], [candidate()],
                       max_pixel_distance=40.0)
    assert result[0].color == 'unknown'


def test_geometry_only_retains_all_obstacles_without_camera():
    result = geometry_observations([candidate(1), candidate(2)], 10.0, 10.1)
    assert len(result) == 2
    assert all(o.color == 'unknown' and o.stamp == 10.0 for o in result)


def test_invalid_geometry_and_duplicate_ids_rejected():
    with unittest.TestCase().assertRaises(ValueError):
        interpret([], [candidate(1), candidate(1)])
    with unittest.TestCase().assertRaises(ValueError):
        interpret([], [candidate(position=(math.nan, 0.0, 0.0))])
    with unittest.TestCase().assertRaises(ValueError):
        interpret([detection(1), detection(1)], [candidate()])


def test_invalid_calibration_rejected():
    with unittest.TestCase().assertRaises(ValueError):
        interpret_usv([], [], Camera(640, 480, 0.0, 400.0, 320.0, 240.0),
                      10.0, 10.0, 10.1)


def _node_module():
    """Load adapter against minimal transport stubs; real ROS remains untested."""
    names = {
        'rclpy': {}, 'rclpy.duration': {'Duration': object},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'rclpy.time': {'Time': SimpleNamespace(from_msg=lambda value: value)},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'CameraInfo': object},
        'std_msgs': {}, 'std_msgs.msg': {'String': object},
        'tf2_ros': {'Buffer': object, 'TransformException': RuntimeError,
                    'TransformListener': object},
        'boat_interfaces': {}, 'boat_interfaces.msg': {'DetectedObjectArray': object},
        'robotx_coordination_interfaces': {},
        'robotx_coordination_interfaces.msg': {'BuoyColorDetectionArray': object,
                                               'BuoyObservationArray': object},
        'robotx_buoy_interpretation.ros_support': {
            'camera_from_info': lambda *args: CAMERA,
            'detections_from_msg': lambda msg: msg.detections,
            'observation_array': lambda *args: args,
            'stamp_seconds': lambda stamp: float(stamp),
            'transform_parts': lambda tf: (tf.translation, tf.quaternion)},
    }
    modules = {}
    for name, members in names.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        modules[name] = module
    path = Path(__file__).resolve().parents[2] / 'coordination_ws/src/robotx_buoy_interpretation/robotx_buoy_interpretation/usv_node.py'
    spec = importlib.util.spec_from_file_location('robotx_buoy_interpretation._usv_node_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def _geometry_msg(stamp=10.0, frame='robotx_map'):
    obj = SimpleNamespace(id=1, object_type=1, confidence=0.9,
        position=SimpleNamespace(x=1.0, y=0.0, z=0.0),
        size=SimpleNamespace(x=0.5, y=0.5))
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id=frame),
                           objects=[obj])


def test_ros_geometry_only_publishes_without_camera_or_model():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.enabled = True
    node.frame = 'robotx_map'
    node.timeout = 0.5
    node.uncertainty = 0.25
    node.pending_geometry = _geometry_msg()
    node.pending = None
    node.now_seconds = lambda: 10.1
    captured = []
    node._publish = lambda observations, stamp: captured.append((observations, stamp))
    node.status = lambda reason: None
    node.process_pending()
    assert node.pending_geometry is None
    assert len(captured) == 1
    assert captured[0][0][0].color == 'unknown'
    assert captured[0][0][0].stamp == 10.0


def test_ros_map_transform_uses_geometry_capture_not_latest_pose():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.frame = 'robotx_map'
    node.uncertainty = 0.25
    calls = []
    transform = SimpleNamespace(translation=(10.0, 20.0, 0.0),
        quaternion=(0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4)))
    def lookup(target, source, stamp):
        calls.append((target, source, stamp))
        return transform
    node.buffer = SimpleNamespace(lookup_transform=lookup)
    mapped, = node._mapped(_geometry_msg(stamp=9.95, frame='base_link'))
    assert calls == [('robotx_map', 'base_link', 9.95)]
    assert math.isclose(mapped.map_position[0], 10.0, abs_tol=1e-9)
    assert math.isclose(mapped.map_position[1], 21.0, abs_tol=1e-9)


def test_ros_future_geometry_does_not_poison_monotonic_capture_guard():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.timeout = 0.5
    node.now_seconds = lambda: 10.0
    node.status = lambda reason: None
    node.last_geometry_stamp = 9.0
    node.on_geometry(_geometry_msg(stamp=100.0))
    assert node.last_geometry_stamp == 9.0


def test_ros_future_image_does_not_poison_monotonic_capture_guard():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.timeout = 0.5
    node.now_seconds = lambda: 10.0
    node.status = lambda reason: None
    node.last_image_stamp = 9.0
    node.pending = None
    node.on_detection(SimpleNamespace(header=SimpleNamespace(stamp=100.0)))
    assert node.last_image_stamp == 9.0
    assert node.pending is None


def test_ros_calibration_history_selects_capture_time_not_latest_arrival():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.calibrations = deque(maxlen=60)
    node.calibration_timeout = 0.1
    node.allow_static_calibration = False
    before = SimpleNamespace(header=SimpleNamespace(stamp=9.98, frame_id='optical'))
    after = SimpleNamespace(header=SimpleNamespace(stamp=10.02, frame_id='optical'))
    other_frame = SimpleNamespace(header=SimpleNamespace(stamp=10.0, frame_id='other'))
    for info in (before, after, other_frame):
        node.on_info(info)
    assert node.calibration_at(10.0, 'optical') is before
    assert node.calibration_at(10.2, 'optical') is None


def test_ros_zero_stamped_static_calibration_requires_explicit_opt_in():
    module = _node_module()
    node = module.UsvInterpreter.__new__(module.UsvInterpreter)
    node.calibrations = deque(maxlen=60)
    node.calibration_timeout = 0.1
    node.allow_static_calibration = False
    static = SimpleNamespace(header=SimpleNamespace(stamp=0.0, frame_id='optical'))
    node.on_info(static)
    assert node.calibration_at(10.0, 'optical') is None
    node.allow_static_calibration = True
    assert node.calibration_at(10.0, 'optical') is static


def load_tests(loader, tests, pattern):
    suite = unittest.TestSuite()
    for name, function in sorted(globals().items()):
        if name.startswith('test_') and callable(function):
            suite.addTest(unittest.FunctionTestCase(function))
    return suite


if __name__ == '__main__':
    unittest.main()
