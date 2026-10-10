"""Exercise real node callbacks with message/clock adapters, not ROS or DDS."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

SRC = Path(__file__).resolve().parents[1] / 'src'
sys.path[:0] = [str(SRC / 'boat_perception'), str(SRC / 'boat_mission')]


def point():
    return S(x=0., y=0., z=0.)


def header():
    return S(stamp=S(sec=0, nanosec=0), frame_id='base_link')


def message(**fields):
    class Message:
        def __init__(self):
            for key, value in fields.items():
                setattr(self, key, value() if callable(value) else copy.deepcopy(value))
    return Message


Gate = message(header=header, left_marker=point, right_marker=point, center=point, width=0., confidence=0.)
GateArray = message(header=header, gates=list)
Pose = message(header=header, pose=lambda: S(position=point(), orientation=S(x=0., y=0., z=0., w=1.)))
Target = message(header=header, target=point, desired_speed=0., stop=False)


class Instant:
    seconds = 10.
    def __init__(self):
        self.nanoseconds = round(self.seconds * 1e9)
    def __sub__(self, other):
        return S(nanoseconds=self.nanoseconds - other.nanoseconds)
    def to_msg(self):
        sec, ns = divmod(self.nanoseconds, 10**9)
        return S(sec=sec, nanosec=ns)


class Publisher:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class Node:
    parameters = {}
    def __init__(self, name):
        self.subscriptions = []
    def declare_parameter(self, name, default):
        return S(value=self.parameters.get(name, default))
    def get_clock(self):
        return S(now=Instant)
    def create_publisher(self, *args):
        return Publisher()
    def create_subscription(self, cls, topic, callback, qos):
        self.subscriptions.append(topic)
    def create_service(self, *args):
        pass
    def create_timer(self, *args):
        return None
    def get_logger(self):
        return S(info=lambda *args: None, warning=lambda *args: None)


def module(name, **items):
    m = types.ModuleType(name)
    m.__dict__.update(items)
    return m


def load_nodes():
    string = message(data='')
    stubs = {'rclpy': module('rclpy'), 'rclpy.node': module('rclpy.node', Node=Node),
             'rclpy.qos': module('rclpy.qos', qos_profile_sensor_data=None),
             'geometry_msgs': module('geometry_msgs'),
             'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=Pose,
                 PointStamped=message(header=header, point=point),
                 Vector3Stamped=message(header=header, vector=point)),
             'mavros_msgs': module('mavros_msgs'),
             'mavros_msgs.msg': module('mavros_msgs.msg', State=message()),
             'std_msgs': module('std_msgs'),
             'std_msgs.msg': module('std_msgs.msg', String=string, Float64=message(data=0.)),
             'std_srvs': module('std_srvs'),
             'std_srvs.srv': module('std_srvs.srv', SetBool=object, Trigger=object),
             'visualization_msgs': module('visualization_msgs'),
             'visualization_msgs.msg': module('visualization_msgs.msg', Marker=object, MarkerArray=object),
             'boat_interfaces': module('boat_interfaces'),
             'boat_interfaces.msg': module('boat_interfaces.msg', Gate=Gate, GateArray=GateArray,
                 NavigationTarget=Target, DetectedObjectArray=message(header=header, objects=list))}
    loaded = []
    with patch.dict(sys.modules, stubs):
        for name, relative in [('mission_callback_test', 'boat_mission/boat_mission/task1_gate_mission.py'),
                               ('detector_callback_test', 'boat_perception/boat_perception/gate_detector.py')]:
            spec = importlib.util.spec_from_file_location(name, SRC / relative)
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            loaded.append(m)
    return loaded


class CallbackTests(unittest.TestCase):
    def setUp(self):
        Instant.seconds = 10.
        Node.parameters = {}
        self.mission_module, self.detector_module = load_nodes()
        self.mission = self.mission_module.Task1GateMission()
        self.mission.publish_debug_telemetry = lambda: None
        self.mission.vehicle_state = S(connected=True, armed=True, mode='GUIDED')
        pose = Pose()
        pose.header.stamp = Instant().to_msg()
        self.mission.local_position_callback(pose)

    def batch(self, x=5., stamp=10.):
        array = GateArray()
        array.header.stamp = S(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))
        gate = Gate()
        gate.header = copy.deepcopy(array.header)
        gate.left_marker.x = gate.right_marker.x = gate.center.x = x
        gate.left_marker.y, gate.right_marker.y = 1.22, -1.22
        gate.width, gate.confidence = 2.44, .9
        array.gates = [gate]
        return array

    def test_real_mission_callback_rejects_switch_to_far_gate(self):
        self.mission.gates_callback(self.batch())
        Instant.seconds = 10.1
        self.mission.gates_callback(self.batch(12., 10.1))
        self.assertEqual(self.mission.tracked_midpoint, (5., 0.))
        self.assertEqual(self.mission.gate_selection_reason, 'LOCKED_GATE_NOT_OBSERVED')

    def test_real_mission_chooses_near_pair_in_full_candidate_array(self):
        near, far = self.batch(5.), self.batch(9.)
        far.gates[0].confidence = .99
        near.gates = far.gates + near.gates
        self.mission.gates_callback(near)
        self.assertEqual(self.mission.tracked_midpoint, (5., 0.))

    def test_real_mission_transforms_delayed_measurement_with_historical_pose(self):
        Instant.seconds = 10.1
        pose = Pose()
        pose.header.stamp = Instant().to_msg()
        pose.pose.position.x = 1.
        self.mission.local_position_callback(pose)
        self.mission.gates_callback(self.batch(5., 10.))
        self.assertEqual(self.mission.tracked_midpoint, (5., 0.))

    def test_gate_without_measurement_pose_cannot_be_acquired(self):
        self.mission.pose_history.samples.clear()
        self.mission.gates_callback(self.batch())
        self.assertIsNone(self.mission.tracked_midpoint)
        self.assertEqual(self.mission.gate_selection_reason, 'MEASUREMENT_POSE_UNAVAILABLE')

    def test_stale_gate_republishing_cannot_refresh_track(self):
        batch = self.batch()
        self.mission.gates_callback(batch)
        Instant.seconds = 13.
        self.mission.gates_callback(batch)
        pose = Pose()
        pose.header.stamp = Instant().to_msg()
        self.mission.local_position_callback(pose)
        self.mission.update()
        self.assertTrue(self.mission.target_pub.messages[-1].stop)
        self.assertEqual(self.mission.last_reason, 'TRACK_GATE_MEASUREMENT_STALE')

    def test_fresh_locked_gate_recovers_after_gap(self):
        self.mission.gates_callback(self.batch())
        Instant.seconds = 13.
        pose = Pose()
        pose.header.stamp = Instant().to_msg()
        self.mission.local_position_callback(pose)
        self.mission.gates_callback(self.batch(5.1, 13.))
        self.mission.update()
        self.assertFalse(self.mission.target_pub.messages[-1].stop)
        self.assertEqual(self.mission.tracked_midpoint, (5.1, 0.))

    def test_passage_ignores_other_gate_candidates(self):
        self.mission.gates_callback(self.batch())
        self.mission.phase = self.mission_module.MissionPhase.PASS_GATE
        Instant.seconds = 10.1
        self.mission.gates_callback(self.batch(12., 10.1))
        self.assertEqual(self.mission.tracked_midpoint, (5., 0.))

    def test_inconsistent_frame_or_timestamp_is_rejected(self):
        for change in ('frame', 'stamp', 'width'):
            batch = self.batch()
            if change == 'frame':
                batch.gates[0].header.frame_id = 'lidar_link'
            elif change == 'stamp':
                batch.gates[0].header.stamp.sec = 9
            else:
                batch.gates[0].width = float('nan')
            self.mission.gates_callback(batch)
            self.assertIsNone(self.mission.tracked_midpoint)

    def test_default_mission_uses_candidate_array_only(self):
        self.assertIn('/perception/gates', self.mission.subscriptions)
        self.assertNotIn('/perception/gate', self.mission.subscriptions)

    def test_real_detector_requires_distinct_frames_and_expires_output(self):
        Node.parameters = dict(min_gate_width=1.83, max_gate_width=3.05,
                               nominal_gate_width=2.44, confirm_hits=2)
        detector = self.detector_module.GateDetector()
        detector.publish_empty_markers = lambda header: None
        detector.publish_gate_markers = lambda gate: None
        objects = S(header=header(), objects=[S(id=i, object_type=1, color=0,
            position=S(x=5., y=y, z=0.), confidence=.8) for i, y in [(1, 1.22), (2, -1.22)]])
        objects.header.stamp.sec = 10
        detector.objects_callback(objects)
        detector.objects_callback(objects)
        self.assertEqual(detector.confirmed.gates, [])
        Instant.seconds = 10.1
        objects.header.stamp = Instant().to_msg()
        detector.objects_callback(objects)
        detector.publish_tracked_gate()
        self.assertEqual(len(detector.gates_pub.messages[-1].gates), 1)
        self.assertEqual(detector.gates_pub.messages[-1].header.stamp.nanosec, 100000000)
        Instant.seconds = 13.
        detector.publish_tracked_gate()
        self.assertEqual(detector.gates_pub.messages[-1].gates, [])
        self.assertEqual(detector.pair_hits, {})


if __name__ == '__main__':
    unittest.main()
