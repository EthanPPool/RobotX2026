"""HSV -> copied LiDAR colors -> actual gate detector/mission callbacks."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import unittest

import numpy as np

from robotx_buoy_interpretation.contracts import Camera, Detection
from robotx_buoy_interpretation.gate_colors import color_objects, unknown_objects
from robotx_buoy_interpretation.hsv_clusters import Segmenter

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def objects():
    return S(header=S(frame_id='base_link', stamp=S(sec=10, nanosec=0)), objects=[
        S(id=i, object_type=1, color=0, confidence=.8,
          position=S(x=5., y=y, z=.5), size=S(x=.3, y=.4, z=1.))
        for i, y in ((1, 1.22), (2, -1.22))])


def optical(point):
    x, y, z = point
    return (-y, .5-z, x)


class TwoGateColorsTests(unittest.TestCase):
    def setUp(self):
        self.camera = Camera(800, 400, 400., 400., 400., 200.)
        self.pairs = load('color_gate_pairs', 'boat_ws/src/boat_perception/boat_perception/gate_candidates.py')

    def detections(self, left=(0, 255, 0), right=(0, 0, 255)):
        pixels = np.zeros((400, 800, 3), np.uint8)
        pixels[185:210, 290:315] = left
        pixels[185:210, 485:510] = right
        clusters, _ = Segmenter().detect(pixels)
        return [Detection(i, c['u'], c['v'], c['width'], c['height'],
                          c['color'], c['confidence']) for i, c in enumerate(clusters)]

    def test_hsv_red_green_labels_form_existing_gate_without_flashing(self):
        raw = objects()
        result = color_objects(raw, self.detections(), self.camera, optical)
        self.assertEqual([o.color for o in result.objects], [2, 1])
        self.assertEqual([o.color for o in raw.objects], [0, 0])
        self.assertEqual(result.header, raw.header)
        for before, after in zip(raw.objects, result.objects):
            self.assertEqual((before.id, before.position, before.size),
                             (after.id, after.position, after.size))
        pairs = self.pairs.find_pairs(result.objects, require_red_green=True)
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]['width'], 2.44)

    def test_real_gate_detector_and_two_gate_mission_receive_hsv_gate(self):
        helpers = load('color_gate_callback_helpers', 'boat_ws/tests/test_gate_callbacks.py')
        fixture = helpers.CallbackTests('test_real_mission_chooses_near_pair_in_full_candidate_array')
        fixture.setUp()
        helpers.Node.parameters = {'require_red_green': True, 'confirm_hits': 2}
        detector = fixture.detector_module.GateDetector()
        result = color_objects(objects(), self.detections(), self.camera, optical)
        detector.objects_callback(result)
        self.assertEqual(detector.confirmed.gates, [])
        detector.objects_callback(result)
        self.assertEqual(detector.confirmed.gates, [])  # Same scan is not a hit.
        helpers.Instant.seconds = 10.1
        result.header.stamp = helpers.Instant().to_msg()
        detector.objects_callback(result)
        self.assertEqual(len(detector.confirmed.gates), 1)
        fixture.mission.gates_callback(detector.confirmed)
        self.assertEqual(fixture.mission.tracked_midpoint, (5., 0.))
        self.assertEqual(fixture.mission.gates_required, 2)

    def test_no_camera_and_blue_or_same_color_cannot_make_red_green_gate(self):
        for detections in ([], self.detections(left=(255, 0, 0)),
                           self.detections(left=(0, 0, 255))):
            result = color_objects(objects(), detections, self.camera, optical)
            self.assertEqual(self.pairs.find_pairs(result.objects, require_red_green=True), [])
        result = unknown_objects(objects())
        self.assertEqual(len(result.objects), 2)
        self.assertEqual([o.color for o in result.objects], [0, 0])

    def test_ambiguous_projection_never_borrows_color(self):
        raw = objects()
        raw.objects[1].position = deepcopy(raw.objects[0].position)
        result = color_objects(raw, self.detections(), self.camera, optical)
        self.assertEqual([o.color for o in result.objects], [0, 0])
        raw = objects()
        detections = self.detections()
        detections.append(Detection(99, detections[0].u, detections[0].v,
                                    25., 25., 'blue', 1.))
        result = color_objects(raw, detections, self.camera, optical)
        self.assertEqual(sum(o.color != 0 for o in result.objects), 1)

    def test_behind_camera_and_invalid_geometry_are_uncolored(self):
        raw = objects()
        result = color_objects(raw, self.detections(), self.camera, lambda p: (0., 0., -1.))
        self.assertEqual([o.color for o in result.objects], [0, 0])
        raw.objects[0].size.z = float('nan')
        result = color_objects(raw, self.detections(), self.camera, optical)
        self.assertEqual(result.objects[0].color, 0)


if __name__ == '__main__':
    unittest.main()
