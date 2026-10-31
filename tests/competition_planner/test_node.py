"""Real planner callbacks with small ROS transport/message stubs."""
from collections import deque
from dataclasses import replace
import importlib.util
import json
from pathlib import Path as FilePath
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from robotx_competition_planner.clearance import Scan
from robotx_competition_planner.mission import SafePassage
from robotx_competition_planner.models import PlannerConfig, Pose
from robotx_competition_planner.survey import Survey
from fixtures import BOUNDARY, markers, pose


def stamp(t):
    return NS(sec=int(t), nanosec=round((t-int(t))*1e9))


def header(t=10., frame='robotx_map'):
    return NS(stamp=stamp(t), frame_id=frame)


class FakeNow:
    def __init__(self, time):
        self.nanoseconds = round(time*1e9)

    def to_msg(self):
        return stamp(self.nanoseconds/1e9)


def module():
    class FakeTime:
        @staticmethod
        def from_msg(msg):
            return msg

    class Preview:
        def __init__(self):
            self.header, self.samples = header(), []

    class Path:
        def __init__(self):
            self.header, self.poses = header(), []

    class PoseStamped:
        def __init__(self):
            self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(w=0.))

    class TransformError(Exception):
        pass

    defs = {'rclpy': {}, 'rclpy.duration': {'Duration': object},
            'rclpy.node': {'Node': object}, 'rclpy.qos': {'qos_profile_sensor_data': object()},
            'rclpy.time': {'Time': FakeTime}, 'geometry_msgs': {},
            'geometry_msgs.msg': {'PoseStamped': PoseStamped}, 'nav_msgs': {},
            'nav_msgs.msg': {'Odometry': object, 'Path': Path}, 'sensor_msgs': {},
            'sensor_msgs.msg': {'LaserScan': object}, 'std_msgs': {},
            'std_msgs.msg': {'String': type('String', (), {})}, 'std_srvs': {},
            'std_srvs.srv': {'SetBool': object, 'Trigger': object},
            'tf2_ros': {'Buffer': object, 'TransformException': TransformError, 'TransformListener': object},
            'robotx_coordination_interfaces': {}, 'robotx_coordination_interfaces.msg': {
                'BuoyTrackArray': object, 'TrajectoryPreview': Preview,
                'TrajectorySample': type('TrajectorySample', (), {})}}
    mods = {}
    for name, values in defs.items():
        m = ModuleType(name)
        m.__dict__.update(values)
        mods[name] = m
    path = FilePath(__file__).resolve().parents[2]/'coordination_ws/src/robotx_competition_planner/robotx_competition_planner/node.py'
    with patch.dict(sys.modules, mods):
        spec = importlib.util.spec_from_file_location('robotx_competition_planner._node_test', path)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
    return m


def wire_map(ms=None, t=10., ref='datum'):
    ms = markers() if ms is None else ms
    return NS(header=header(t), reference_id=ref, source_mode='assisted', tracks=[NS(
        track_id=m.track_id, position=NS(x=m.x, y=m.y), radius=m.radius,
        uncertainty=m.uncertainty, color=m.color, light_state=m.light_state,
        confidence=m.confidence, color_observed_at=stamp(m.semantic_stamp),
        semantic_conflict=False, color_source=m.color_source) for m in ms])


def node(tier='core'):
    m = module()
    n = m.CompetitionPlanner.__new__(m.CompetitionPlanner)
    n.time = 10.
    n.get_clock = lambda: NS(now=lambda: FakeNow(n.time))
    n.frame, n.ref, n.enabled = 'robotx_map', 'datum', True
    n.config, n.boundary, n.tier = PlannerConfig(), BOUNDARY, tier
    n.mission = SafePassage(n.config, BOUNDARY, tier)
    n.pose_timeout = n.map_timeout = n.scan_timeout = .5
    n.semantic_age, n.max_pose_variance = 90., 1.
    n.report_ack_stamp = n.report_map_stamp = 0.
    n.report_signature = None
    n.survey = Survey(9., 90., tier == 'advanced')
    n.latest_map = None
    n.pose = pose()
    n.scan = Scan(tuple([float('inf')]*720), -3.141592653589793,
        3.141592653589793/360, .1, 40., -6., 0., 0., 10.)
    n.external_reason = 'NOT_STARTED'
    for key in ('preview', 'route', 'events', 'status'):
        array = []
        setattr(n, key+'_messages', array)
        setattr(n, key+'_pub', NS(publish=array.append))
    n.on_map(wire_map(markers('uav' if tier == 'advanced' else 'usv')))
    n.buffer = NS(lookup_transform=lambda target, source, at: NS(transform=NS(
        translation=NS(x=-6., y=0., z=1.), rotation=NS(x=0., y=0., z=0., w=1.))))
    return n


def response():
    return NS(success=False, message='')


class NodeTests(unittest.TestCase):
    def test_start_and_tick_publish_three_timed_samples_with_shared_reference(self):
        n = node()
        self.assertTrue(n.start(None, response()).success)
        n.time += .1
        n.tick()
        out = n.preview_messages[-1]
        self.assertTrue(out.valid)
        self.assertEqual(out.reference_id, 'datum')
        self.assertEqual(out.header.frame_id, 'robotx_map')
        self.assertEqual([s.offset_seconds for s in out.samples], [0., .1, .2])
        self.assertTrue(n.route_messages[-1].poses)

    def test_start_has_no_uav_requirement_at_core_and_is_disabled_without_datum(self):
        n = node()
        n.enabled = False
        r = n.start(None, response())
        self.assertFalse(r.success)
        self.assertEqual(r.message, 'DATUM_NOT_CONFIGURED')

    def test_missing_lidar_invalidates_preview_and_freezes_clock(self):
        n = node()
        n.start(None, response())
        n.scan = None
        n.time += .1
        n.tick()
        self.assertFalse(n.preview_messages[-1].valid)
        self.assertEqual(n.mission.elapsed, 0.)

    def test_future_or_wrong_frame_map_cannot_replace_good_snapshot(self):
        n = node()
        original = n.latest_map
        n.on_map(wire_map(t=100.))
        n.on_map(wire_map(t=10.1, ref='different-datum'))
        self.assertIs(n.latest_map, original)

    def test_acknowledgement_is_bound_to_reported_survey_and_consumed(self):
        n = node('advanced')
        self.assertFalse(n.start(None, response()).success)
        n.reported(NS(data=True), response())
        self.assertTrue(n.start(None, response()).success)
        self.assertIsNone(n.report_signature)
        n.stop(None, response())
        self.assertFalse(n.preview_messages[-1].valid)
        self.assertFalse(n.survey.semantics)

    def test_changed_survey_after_report_cannot_start_with_old_ack(self):
        n = node('advanced')
        n.reported(NS(data=True), response())
        changed = tuple(replace(m, color='red', semantic_stamp=10.1) if m.track_id == 3 else m
                        for m in markers('uav'))
        n.time = 10.1
        n.on_map(wire_map(changed, t=10.1))
        self.assertIn('ACKNOWLEDGEMENT', n.start(None, response()).message)

    def test_advanced_continues_after_aerial_semantics_age_out_of_live_tracker(self):
        n = node('advanced')
        n.reported(NS(data=True), response())
        n.start(None, response())
        for i in range(1, 62):
            n.time = 10.+i*.1
            s = n.mission.trajectory.sample(n.mission.elapsed)
            n.pose = Pose(s.x, s.y, s.yaw, n.time)
            n.scan = replace(n.scan, x=s.x, y=s.y, yaw=s.yaw, stamp=n.time)
            unknown = tuple(replace(m, color='unknown', light_state='unknown',
                confidence=0., semantic_stamp=0., color_source='') for m in markers('uav'))
            n.on_map(wire_map(unknown, t=n.time))
            n.tick()
        self.assertTrue(n.preview_messages[-1].valid)
        self.assertEqual(n.mission.course.markers[0].role, 'entry')

    def test_new_local_obstacle_on_route_withholds_reference(self):
        n = node()
        n.start(None, response())
        obstacle = replace(markers()[6], track_id=11, x=-4.5, y=0.)
        n.time = 10.1
        n.on_map(wire_map(markers()+(obstacle,), t=10.1))
        n.tick()
        self.assertFalse(n.preview_messages[-1].valid)
        self.assertIn('REPLAN', n.preview_messages[-1].reason)

    def test_braking_clearance_horizon_extends_beyond_two_reference_steps(self):
        n = node()
        n.start(None, response())
        self.assertGreater(len(n.clearance_samples()), 3)

    def test_map_and_pose_staleness_cannot_be_hidden_by_new_preview_headers(self):
        n = node()
        n.start(None, response())
        n.time = 11.
        n.tick()
        self.assertFalse(n.preview_messages[-1].valid)
        self.assertEqual(n.preview_messages[-1].reason, 'POSE_STALE')

    def test_scan_pose_lookup_uses_acquisition_timestamp(self):
        n = node()
        calls = []
        old_lookup = n.buffer.lookup_transform
        n.buffer.lookup_transform = lambda *args: (calls.append(args) or old_lookup(*args))
        n.time = 10.1
        scan = NS(header=header(10.1, 'lidar_frame'), ranges=[float('inf')]*720,
            angle_min=-3.141592653589793, angle_increment=3.141592653589793/360,
            range_min=.1, range_max=40.)
        n.on_scan(scan)
        self.assertEqual(calls[0][:2], ('robotx_map', 'lidar_frame'))
        self.assertIs(calls[0][2], scan.header.stamp)
        self.assertAlmostEqual(n.scan.stamp, 10.1)


if __name__ == '__main__':
    unittest.main()
