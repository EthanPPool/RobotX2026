import unittest

from robotx_competition_planner.models import Marker, PlannerConfig
from robotx_competition_planner.routing import Course
from fixtures import pose


class EightMarkerTests(unittest.TestCase):
    def test_all_eight_colored_markers_are_supported_without_gate_pair_assumptions(self):
        ms = [Marker(1, 0., 0., color='blue', light_state='flashing'),
              Marker(2, 36., 0., color='blue', light_state='steady')]
        for i, x in enumerate((8., 14., 21., 28.)):
            ms.extend((Marker(3+2*i, x, 5., color='green', light_state='flashing'),
                       Marker(4+2*i, x, -5., color='red', light_state='flashing')))
        course = Course(ms, ((-10., -14.), (46., -14.), (46., 14.), (-10., 14.)), PlannerConfig())
        plan = course.plan(pose())
        mask = 0
        for a, b in zip(plan.transit, plan.transit[1:]):
            legal, bits = course.passages(a, b)
            self.assertTrue(legal)
            mask |= bits
        self.assertEqual(len(plan.required_ids), 8)
        self.assertEqual(mask, 255)


if __name__ == '__main__':
    unittest.main()
