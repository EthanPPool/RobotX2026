"""Run with python3 -m unittest discover -s boat_ws/tests (no ROS required)."""
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace as S
import unittest

SRC = Path(__file__).resolve().parents[1] / 'src'
sys.path[:0] = [str(SRC / 'boat_perception'), str(SRC / 'boat_mission')]
from boat_perception.gate_candidates import find_pairs
from boat_perception.pose_history import PoseHistory, to_map, to_body
from boat_mission.gate_selection import GateGeometry, GateSelector


def buoy(i, x, y, confidence=0.8, color=0):
    return S(id=i, position=S(x=x, y=y, z=0.2), object_type=1,
             confidence=confidence, color=color)


def geometry(x, y=0, confidence=0.8, width=2.44):
    return GateGeometry((x, y + width / 2), (x, y - width / 2), confidence, 10.0)


class PairTests(unittest.TestCase):
    def test_both_near_and_confident_far_gate_are_reported(self):
        pairs = find_pairs([buoy(1, 3, 1.22), buoy(2, 3, -1.22),
                            buoy(3, 9, 1.22, .95), buoy(4, 9, -1.22, .95)])
        self.assertEqual([p['key'] for p in pairs], [(1, 2), (3, 4)])

    def test_input_order_does_not_change_selection(self):
        objects = [buoy(i * 2 + side + 1, x, y) for i, x in enumerate((2., 6., 10.))
                   for side, y in enumerate((1.22, -1.22))]
        keys = [p['key'] for p in find_pairs(objects)]
        random.Random(7).shuffle(objects)
        self.assertEqual([p['key'] for p in find_pairs(objects)], keys)

    def test_many_buoys_have_bounded_deterministic_output(self):
        objects = [buoy(i, 2 + i % 7, i // 7 - 10) for i in range(140)]
        pairs = find_pairs(objects, max_buoys=32, max_candidates=12)
        self.assertLessEqual(len(pairs), 12)
        self.assertEqual([p['key'] for p in pairs],
                         [p['key'] for p in find_pairs(list(reversed(objects)),
                                                     max_buoys=32, max_candidates=12)])

    def test_duplicate_tracks_cannot_form_extra_markers(self):
        pairs = find_pairs([buoy(1, 5, 1.22), buoy(2, 5, -1.22),
                            buoy(3, 5.01, 1.21), buoy(4, 5.01, -1.21)])
        self.assertEqual(len(pairs), 1)

    def test_buoy_limit_keeps_near_gate_before_confident_far_gate(self):
        pairs = find_pairs([buoy(1, 3, 1.22), buoy(2, 3, -1.22),
                            buoy(3, 9, 1.22, .99), buoy(4, 9, -1.22, .99)],
                           max_buoys=2)
        self.assertEqual([p['key'] for p in pairs], [(1, 2)])

    def test_weak_pairs_do_not_consume_candidate_limit(self):
        offset = 2.44 / math.sqrt(8)
        pairs = find_pairs([buoy(1, 3 + offset, offset, .6),
                            buoy(2, 3 - offset, -offset, .6),
                            buoy(3, 9, 1.22), buoy(4, 9, -1.22)],
                           min_pair_confidence=.75, max_candidates=1)
        self.assertEqual([p['key'] for p in pairs], [(3, 4)])

    def test_invalid_or_incomplete_geometry_is_rejected(self):
        for objects in ([buoy(1, 5, 0)], [buoy(1, 5, 2), buoy(2, 5, -2)],
                        [buoy(1, 5, 1.22, float('nan')), buoy(2, 5, -1.22)],
                        [buoy(1, -5, 1.22), buoy(2, -5, -1.22)]):
            with self.subTest(objects=objects):
                self.assertEqual(find_pairs(objects), [])

    def test_yaw_preserves_valid_pair(self):
        points = [to_body((5, y), (0, .5, math.radians(15))) for y in (1.22, -1.22)]
        self.assertEqual(len(find_pairs([buoy(i + 1, *p) for i, p in enumerate(points)])), 1)

    def test_obstructed_opening_is_rejected(self):
        pairs = find_pairs([buoy(1, 5, 1.22), buoy(2, 5, -1.22), buoy(3, 5, 0)])
        self.assertEqual(pairs, [])

    def test_color_requirement_needs_actual_red_and_green(self):
        self.assertEqual(find_pairs([buoy(1, 5, 1.22), buoy(2, 5, -1.22)],
                                    require_red_green=True), [])
        self.assertEqual(len(find_pairs([buoy(1, 5, 1.22, color=1),
                                        buoy(2, 5, -1.22, color=2)],
                                       require_red_green=True)), 1)


class IdentityTests(unittest.TestCase):
    def test_nearest_gate_wins_over_confidence(self):
        selector = GateSelector()
        self.assertEqual(selector.choose([geometry(12, confidence=.99), geometry(5)],
                                         (0, 0, 0)).center, (5., 0.))

    def test_competing_gate_cannot_steal_selected_gate(self):
        selector = GateSelector()
        selector.choose([geometry(5)], (0, 0, 0))
        self.assertIsNone(selector.choose([geometry(12, confidence=.99)], (2, 0, 0)))
        self.assertEqual(selector.anchor.center, (5., 0.))

    def test_gradual_drift_cannot_move_fixed_anchor_to_another_gate(self):
        selector = GateSelector()
        selector.choose([geometry(5)], (0, 0, 0))
        self.assertIsNotNone(selector.choose([geometry(5.6)], (0, 0, 0)))
        self.assertIsNone(selector.choose([geometry(6.2)], (0, 0, 0)))

    def test_marker_order_can_reverse_without_losing_identity(self):
        selector = GateSelector()
        original = geometry(5)
        selector.choose([original], (0, 0, 0))
        reversed_gate = GateGeometry(original.starboard, original.port, .9, 11)
        self.assertIs(selector.choose([reversed_gate], (0, 0, 0)), reversed_gate)

    def test_passed_gate_is_not_counted_again(self):
        selector = GateSelector()
        first = selector.choose([geometry(5)], (0, 0, 0))
        selector.complete(first)
        chosen = selector.choose([geometry(5.1), geometry(12)], (0, 0, 0))
        self.assertEqual(chosen.center, (12., 0.))

    def test_lateral_and_behind_gates_are_not_initial_targets(self):
        self.assertIsNone(GateSelector().choose([geometry(-3), geometry(1, 5)], (0, 0, 0)))

    def test_reset_discards_completed_gate_exclusions(self):
        selector = GateSelector()
        selector.complete(geometry(5))
        selector.reset()
        self.assertEqual(selector.choose([geometry(5)], (0, 0, 0)).center, (5., 0.))


class PoseTests(unittest.TestCase):
    def test_translation_and_rotation_rebase_stationary_buoy(self):
        old, new = (0, 0, math.radians(15)), (1, .5, math.radians(-20))
        observed = to_body((5, 1.22), old)
        rebased = to_body(to_map(observed, old), new)
        for actual, expected in zip(rebased, to_body((5, 1.22), new)):
            self.assertAlmostEqual(actual, expected)

    def test_delayed_scan_uses_its_measurement_pose(self):
        history = PoseHistory()
        history.add(10, 0, 0, math.radians(15))
        history.add(10.1, 0, 0, math.radians(5))
        point = to_body((5, 1.22), history.at(10))
        self.assertAlmostEqual(to_map(point, history.at(10))[0], 5.)
        self.assertGreater(math.dist(to_map(point, history.at(10.1)), (5, 1.22)), .5)

    def test_yaw_interpolation_crosses_wrap_without_full_turn(self):
        history = PoseHistory()
        history.add(10, 0, 0, math.radians(179))
        history.add(10.1, 1, 1, math.radians(-179))
        pose = history.at(10.05)
        self.assertAlmostEqual(pose[0], .5)
        self.assertAlmostEqual(abs(pose[2]), math.pi)

    def test_missing_or_stale_pose_is_not_extrapolated(self):
        history = PoseHistory()
        self.assertIsNone(history.at(10))
        history.add(10, 0, 0, 0)
        self.assertIsNone(history.at(11))
        self.assertIsNone(history.at(0))

    def test_out_of_order_and_nonfinite_poses_are_rejected(self):
        history = PoseHistory()
        history.add(10, 0, 0, 0)
        self.assertFalse(history.add(9, 0, 0, 0))
        self.assertFalse(history.add(11, float('nan'), 0, 0))

    def test_gap_in_pose_stream_is_not_interpolated(self):
        history = PoseHistory()
        history.add(10, 0, 0, 0)
        history.add(12, 0, 0, 0)
        self.assertIsNone(history.at(11))


if __name__ == '__main__':
    unittest.main()
