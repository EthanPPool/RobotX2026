from dataclasses import replace
import math
import unittest

from robotx_competition_planner.models import Marker, PlannerConfig, Pose
from robotx_competition_planner.routing import Course
from fixtures import BOUNDARY, markers, pose


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.config = PlannerConfig()
        self.course = Course(markers(), BOUNDARY, self.config)

    def test_full_course_route_has_correct_orbits_and_passes_every_required_marker(self):
        plan = self.course.plan(pose())
        self.assertEqual(plan.entry_orbit.direction, -1)
        self.assertEqual(plan.exit_orbit.direction, 1)
        self.assertGreater(len(plan.transit), 2)  # Avoid central black obstacle.
        bits = 0
        for a, b in zip(plan.transit, plan.transit[1:]):
            self.assertTrue(self.course.segment_clear(a, b))
            legal, passed = self.course.passages(a, b)
            self.assertTrue(legal)
            bits |= passed
        self.assertEqual(bits, self.course.full_mask)

    def test_green_is_port_and_red_starboard_in_enu(self):
        c = Course((Marker(1, 5., 2., color='green', light_state='flashing'),
                    Marker(2, 5., -2., color='red', light_state='flashing')),
                    BOUNDARY, self.config)
        legal, bits = c.passages((0., 0.), (10., 0.))
        self.assertTrue(legal)
        self.assertEqual(bits, c.full_mask)
        self.assertFalse(c.passages((10., 0.), (0., 0.))[0])

    def test_closest_same_side_buoy_controls_abeam_passage(self):
        c = Course((Marker(1, 5., 4., color='red', light_state='flashing'),
                    Marker(2, 5., 2., color='black', light_state='inactive')),
                    BOUNDARY, self.config)
        self.assertEqual(c.passages((0., 0.), (10., 0.)), (True, 0))

    def test_distant_beacon_does_not_count_as_passed(self):
        c = Course((Marker(1, 5., 12., color='green', light_state='flashing'),
                    Marker(2, 5., -12., color='red', light_state='flashing')),
                    BOUNDARY, self.config)
        self.assertEqual(c.passages((0., 0.), (10., 0.)), (True, 0))

    def test_hull_and_uncertainty_clearance_are_included(self):
        m = self.course.markers[0]
        r = self.course.inflated(m)
        self.assertFalse(self.course.point_clear((m.x+r-.01, m.y)))
        self.assertFalse(self.course.segment_clear((-4., 0.), (4., 0.)))

    def test_boundary_is_not_silently_fabricated(self):
        for boundary in ((), ((0., 0.), (5., 0.), (2., 1.), (5., 5.), (0., 5.))):
            with self.assertRaises(ValueError):
                Course(markers(), boundary, self.config)

    def test_duplicate_ids_bad_geometry_and_search_budget_are_rejected(self):
        for bad in (markers()+(markers()[0],), (replace(markers()[0], x=float('nan')),)+markers()[1:]):
            with self.assertRaises(ValueError):
                Course(bad, BOUNDARY, self.config)
        c = Course(markers(), BOUNDARY, replace(self.config, max_search_states=1))
        with self.assertRaisesRegex(ValueError, 'budget'):
            c.plan(pose())

    def test_orbit_checks_complete_circumference_not_just_start_point(self):
        m = self.course.markers[0]
        c = Course((m, replace(self.course.markers[1], x=3., y=0.)), BOUNDARY, self.config)
        self.assertFalse(c.orbit_clear(m, 3.))

    def test_missing_blue_state_is_not_guessed(self):
        ms = tuple(replace(m, light_state='unknown') if m.role == 'entry' else m for m in markers())
        with self.assertRaises(ValueError):
            Course(ms, BOUNDARY, self.config).plan(pose())

    def test_route_is_rotation_invariant_not_a_global_x_gate_rule(self):
        rotated = tuple(replace(m, x=-m.y, y=m.x) for m in markers())
        boundary = tuple((-y, x) for x, y in BOUNDARY)
        c = Course(rotated, boundary, self.config)
        p = c.plan(Pose(0., -6., math.pi/2, 10.))
        mask = 0
        for a, b in zip(p.transit, p.transit[1:]):
            legal, bits = c.passages(a, b)
            self.assertTrue(legal)
            mask |= bits
        self.assertEqual(mask, c.full_mask)


if __name__ == '__main__':
    unittest.main()
