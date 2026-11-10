"""Measured HSV pixels feed the existing association/tracker/planner contracts."""
import unittest

import numpy as np

from robotx_buoy_interpretation.contracts import Camera, Detection
from robotx_buoy_interpretation.hsv_clusters import Segmenter
from robotx_buoy_interpretation.tracking import BuoyTracker
from robotx_buoy_interpretation.usv import (
    GeometryCandidate, geometry_observations, interpret_usv,
)
from robotx_competition_planner.incremental import IncrementalPassage
from robotx_competition_planner.models import Marker, PlannerConfig


class HsvMissionPipelineTests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera(300, 200, 200., 200., 150., 100.)
        self.segmenter = Segmenter()
        # Projection of the LiDAR candidate is at the synthetic lamp center.
        self.geometry = GeometryCandidate(
            42, (5., 0., 0.), ((74.5-150.)/200.*5., (64.5-100.)/200.*5., 5.),
            radius=.25, uncertainty=.1,
        )

    def detections(self, pixels):
        clusters, _ = self.segmenter.detect(pixels)
        return [Detection(i, c['u'], c['v'], c['width'], c['height'],
                          c['color'], c['confidence']) for i, c in enumerate(clusters)]

    def markers(self, tracker, now):
        return tuple(Marker(r['track_id'], r['x'], r['y'], r['radius'],
            r['uncertainty'], r['color'], r['light_state'], r['confidence'],
            r['color_stamp'], r['color_source']) for r in tracker.snapshot(now))

    def test_hsv_blue_to_existing_temporal_tracker_to_incremental_exit(self):
        pixels = np.zeros((200, 300, 3), np.uint8)
        pixels[50:80, 60:90] = (255, 0, 0)
        tracker = BuoyTracker(reference_id='datum')
        planner = IncrementalPassage(PlannerConfig(),
            ((-20., -20.), (20., -20.), (20., 20.), (-20., 20.)))
        for i in range(13):
            now = 10.+i*.25
            observed = interpret_usv(self.detections(pixels), [self.geometry],
                self.camera, now, now, now)
            tracker.ingest(observed, 'robotx_map', 'datum', now)
            planner.observe(self.markers(tracker, now))
            if i == 0:
                self.assertIsNone(planner.endpoint('exit'))
        endpoint = planner.endpoint('exit')
        self.assertIsNotNone(endpoint)
        self.assertEqual(endpoint.color_source, 'usv')
        self.assertEqual(endpoint.light_state, 'steady')
        self.assertEqual(endpoint.point, (5., 0.))
        self.assertIsNone(planner.endpoint('entry'))

    def test_camera_and_lidar_outputs_are_independent(self):
        pixels = np.zeros((200, 300, 3), np.uint8)
        pixels[50:80, 60:90] = (0, 0, 255)
        detections = self.detections(pixels)
        self.assertEqual(detections[0].color, 'red')
        self.assertEqual(interpret_usv(detections, [], self.camera, 10., 10., 10.), [])
        geometry = geometry_observations([self.geometry], 10., 10.)
        self.assertEqual(geometry[0].color, 'unknown')
        self.assertEqual((geometry[0].x, geometry[0].y), (5., 0.))

    def test_missing_hsv_cluster_is_not_an_inactive_or_flashing_beacon(self):
        tracker = BuoyTracker(reference_id='datum')
        on = np.zeros((200, 300, 3), np.uint8)
        on[50:80, 60:90] = (0, 0, 255)
        missing = np.zeros_like(on)
        for i in range(25):
            now = 10.+i*.25
            pixels = on if int(i*.25) % 2 == 0 else missing
            observed = interpret_usv(self.detections(pixels), [self.geometry],
                self.camera, now, now, now)
            tracker.ingest(observed, 'robotx_map', 'datum', now)
        marker = self.markers(tracker, now)[0]
        self.assertEqual(marker.color, 'red')
        self.assertEqual(marker.light_state, 'unknown')
        self.assertEqual(marker.role, 'obstacle')


if __name__ == '__main__':
    unittest.main()
