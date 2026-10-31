from dataclasses import replace
import math
import unittest

from robotx_competition_planner.mission import SafePassage
from robotx_competition_planner.models import PlannerConfig, Pose
from fixtures import BOUNDARY, markers, pose


class MissionTests(unittest.TestCase):
    def mission(self, tier='core'):
        return SafePassage(PlannerConfig(), BOUNDARY, tier)

    def test_core_starts_without_uav_and_advanced_requires_aerial_provenance_and_report(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'test')
        self.assertEqual(m.state, 'APPROACH_ENTRY')
        advanced = self.mission('advanced')
        with self.assertRaisesRegex(ValueError, 'UAV'):
            advanced.start(markers(), pose(), 10., 'test', True)
        with self.assertRaisesRegex(ValueError, 'ACKNOWLEDGEMENT'):
            advanced.start(markers('uav'), pose(), 10., 'test')
        advanced.start(markers('uav'), pose(), 10., 'test', True)
        self.assertEqual(advanced.state, 'APPROACH_ENTRY')

    def test_unknown_flash_state_incomplete_or_old_survey_not_guessed(self):
        cases = [markers()[:-1], tuple(replace(x, light_state='unknown') if x.role == 'entry' else x for x in markers()), markers(stamp=1.)]
        for ms in cases:
            m = self.mission()
            with self.assertRaises(ValueError):
                m.start(ms, pose(), 10., 'test')
            self.assertEqual(m.state, 'IDLE')
            self.assertIsNone(m.course)

    def test_advanced_semantics_remain_attempt_scoped_after_uav_departs(self):
        m = self.mission('advanced')
        m.start(markers('uav'), pose(), 10., 'run', True)
        for i in range(1, 101):
            sample = m.trajectory.sample(m.elapsed)
            self.assertTrue(m.update(Pose(sample.x, sample.y, sample.yaw, 10.+i*.1), 10.+i*.1))
        self.assertEqual(m.course.markers[0].semantic_stamp, 10.)
        self.assertNotEqual(m.state, 'STOPPED')

    def test_local_clearance_loss_freezes_reference_clock(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'test')
        self.assertFalse(m.update(pose(10.1), 10.1, False))
        self.assertEqual(m.elapsed, 0.)

    def test_stationary_vessel_does_not_complete_based_on_time(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'test')
        for i in range(1, 1001):
            m.update(pose(10.+i*.1), 10.+i*.1)
        self.assertEqual(m.state, 'APPROACH_ENTRY')
        self.assertLess(m.elapsed, m.trajectory.duration)

    def test_pose_jump_and_clock_gap_invalidate_cycle(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'test')
        self.assertFalse(m.update(Pose(15., 0., 0., 10.1), 10.1))
        self.assertEqual(m.reason, 'POSE_JUMP')
        self.assertFalse(m.update(pose(15.), 15.))
        self.assertEqual(m.reason, 'CLOCK_OR_UPDATE_GAP')

    def test_perfect_tracking_completes_all_measured_phases(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'run')
        states, now = [m.state], 10.
        for _ in range(40000):
            sample = m.trajectory.sample(m.elapsed)
            now += .1
            m.update(Pose(sample.x, sample.y, sample.yaw, now), now)
            if states[-1] != m.state:
                states.append(m.state)
            if m.state == 'COMPLETE':
                break
        self.assertEqual(states, ['APPROACH_ENTRY', 'CIRCLE_ENTRY', 'TRANSIT', 'CIRCLE_EXIT', 'COMPLETE'])
        self.assertEqual(m.passed_mask, m.course.full_mask)

    def test_circle_endpoint_without_actual_revolution_cannot_advance(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'run')
        m.phase_index = 1
        m._phase(0.)
        m.elapsed = m.trajectory.duration
        s = m.trajectory.end
        m.last_pose = Pose(s.x, s.y, s.yaw, 10.)
        m.update(Pose(s.x, s.y, s.yaw, 10.1), 10.1)
        self.assertEqual(m.state, 'CIRCLE_ENTRY')

    def test_wrong_side_measured_passage_stops_attempt(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'run')
        m.phase_index = 2
        m._phase(0.)
        # Green is south of this actual west-to-east crossing: wrong side.
        m.last_pose = Pose(8.9, 4., 0., 10.)
        m.trajectory = __import__('robotx_competition_planner.trajectory', fromlist=['polyline']).polyline(
            ((8.9, 4.), (9.2, 4.)), 0., m.config)
        self.assertFalse(m.update(Pose(9.1, 4., 0., 10.1), 10.1))
        self.assertEqual(m.state, 'STOPPED')

    def test_stop_and_new_attempt_do_not_share_progress(self):
        m = self.mission()
        m.start(markers(), pose(), 10., 'one')
        m.passed_mask = 3
        m.stop()
        m.start(markers(stamp=11.), pose(11.), 11., 'two')
        self.assertEqual(m.passed_mask, 0)
        self.assertEqual(m.run_id, 'two')


if __name__ == '__main__':
    unittest.main()
