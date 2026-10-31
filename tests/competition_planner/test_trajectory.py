import math
import unittest

from robotx_competition_planner.models import PlannerConfig
from robotx_competition_planner.routing import Orbit
from robotx_competition_planner.trajectory import polyline, orbit


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.config = PlannerConfig()

    def assert_limits(self, trajectory):
        for i in range(501):
            s = trajectory.sample(trajectory.duration*i/500)
            self.assertLessEqual(math.hypot(s.vx, s.vy), self.config.speed+1e-8)
            self.assertLessEqual(math.hypot(s.ax, s.ay), self.config.acceleration+1e-8)
            self.assertLessEqual(abs(s.yaw_rate), self.config.yaw_rate+1e-8)
            self.assertLessEqual(abs(s.yaw_acceleration), self.config.yaw_acceleration+1e-8)

    def test_corner_turns_have_zero_surge_and_c2_boundaries(self):
        t = polyline(((0., 0.), (3., 0.), (3., 3.)), 0., self.config)
        self.assertIn('turn', [p.kind for p in t.primitives])
        for p in t.primitives:
            for s in (p.sample(0.), p.sample(p.duration)):
                self.assertAlmostEqual(math.hypot(s.vx, s.vy), 0.)
                self.assertAlmostEqual(math.hypot(s.ax, s.ay), 0.)
                self.assertAlmostEqual(s.yaw_rate, 0.)
                self.assertAlmostEqual(s.yaw_acceleration, 0.)
            if p.kind == 'turn':
                s = p.sample(p.duration/2)
                self.assertEqual((s.vx, s.vy), (0., 0.))
        self.assert_limits(t)

    def test_continuous_yaw_crosses_pi_without_two_pi_jump(self):
        t = polyline(((0., 0.), (-3., -.03)), math.pi-.01, self.config)
        self.assertLess(abs(t.end.yaw-(math.pi-.01)), .1)
        self.assert_limits(t)

    def test_clockwise_and_counterclockwise_full_orbits_are_analytic(self):
        for direction in (-1, 1):
            arc = Orbit((3., 4.), 3., .2, direction)
            t = orbit(arc, .2+direction*math.pi/2, self.config)
            self.assert_limits(t)
            for i in range(101):
                s = t.sample(t.duration*i/100)
                self.assertAlmostEqual(math.hypot(s.x-3., s.y-4.), 3.)
            self.assertAlmostEqual(t.end.yaw-t.initial.yaw, direction*2*math.pi)

    def test_preview_has_exact_three_offsets_and_does_not_wrap_at_end(self):
        t = polyline(((0., 0.), (1., 0.)), 0., self.config)
        p = t.preview(t.duration-.05, .1)
        self.assertEqual(tuple(s.offset for s in p), (0., .1, .2))
        self.assertAlmostEqual(p[1].x, 1.)
        self.assertAlmostEqual(p[2].x, 1.)
        self.assertAlmostEqual(p[2].vx, 0.)
        with self.assertRaises(ValueError):
            t.preview(0., 0.)

    def test_derivatives_agree_with_local_position_differences(self):
        for t in (polyline(((0., 0.), (3., 2.)), math.atan2(2., 3.), self.config),
                  orbit(Orbit((0., 0.), 3., 0., 1), math.pi/2, self.config)):
            at, h = t.duration*.42, 1e-3
            a, b, c = t.sample(at-h), t.sample(at), t.sample(at+h)
            self.assertAlmostEqual((c.x-a.x)/(2*h), b.vx, places=6)
            self.assertAlmostEqual((c.y-a.y)/(2*h), b.vy, places=6)
            self.assertAlmostEqual((c.vx-a.vx)/(2*h), b.ax, places=6)


if __name__ == '__main__':
    unittest.main()
