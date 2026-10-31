from dataclasses import replace
import math
import unittest

from robotx_competition_planner.clearance import Scan
from robotx_competition_planner.trajectory import Sample


class ClearanceTests(unittest.TestCase):
    def setUp(self):
        self.scan = Scan(tuple([math.inf]*720), -math.pi, math.pi/360, .1, 30., 0., 0., 0., 10.)
        self.points = (Sample(0., 0., 0.), Sample(1., 0., 0.), Sample(2., 0., 0.))

    def test_full_clear_scan_accepts_swept_hull(self):
        self.assertTrue(self.scan.preview_clear(self.points, 1., 10.1))

    def test_nan_unknown_and_close_obstacles_are_not_clear_water(self):
        for value in (float('nan'), 0., .5, -math.inf, 1000.):
            ranges = list(self.scan.ranges)
            ranges[360] = value
            self.assertFalse(replace(self.scan, ranges=tuple(ranges)).preview_clear(self.points, 1., 10.1))

    def test_stale_future_and_out_of_range_preview_rejected(self):
        self.assertFalse(self.scan.preview_clear(self.points, 1., 11.))
        self.assertFalse(self.scan.preview_clear(self.points, 1., 9.9))
        self.assertFalse(self.scan.preview_clear((Sample(30., 0., 0.),), 1., 10.1))

    def test_incomplete_field_of_view_cannot_clear_unobserved_hull(self):
        half = replace(self.scan, ranges=self.scan.ranges[:360], angle_min=-math.pi/2)
        self.assertFalse(half.preview_clear(self.points, 1., 10.1))

    def test_capture_frame_transform_is_used(self):
        shifted = replace(self.scan, x=10., y=20., yaw=math.pi/2)
        points = (Sample(10., 20., 0.), Sample(10., 22., 0.))
        self.assertTrue(shifted.preview_clear(points, 1., 10.1))


if __name__ == '__main__':
    unittest.main()
