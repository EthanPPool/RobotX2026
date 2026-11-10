"""Synthetic pixel, calibration, timestamp and downstream association checks."""
from copy import deepcopy
from types import SimpleNamespace as Box
import unittest

import cv2
import numpy as np

from robotx_buoy_interpretation.hsv_clusters import (
    DEFAULT_RANGES, Limits, Segmenter, rectify, capture_valid)
from robotx_buoy_interpretation.contracts import Camera, Detection
from robotx_buoy_interpretation.usv import GeometryCandidate, interpret_usv


def image(hue=0, saturation=255, value=255, size=20):
    hsv = np.zeros((200, 300, 3), np.uint8)
    hsv[50:50+size, 60:60+size] = (hue, saturation, value)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def info():
    return Box(header=Box(frame_id='camera_optical'), width=300, height=200,
        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
        p=[200., 0., 150., 0., 0., 200., 100., 0., 0., 0., 1., 0.],
        k=[200., 0., 150., 0., 200., 100., 0., 0., 1.],
        d=[0.]*5, distortion_model='plumb_bob', binning_x=0, binning_y=0,
        roi=Box(x_offset=0, y_offset=0, width=0, height=0))


class HsvTests(unittest.TestCase):
    def test_rgb_and_red_wrap(self):
        for hue, expected in ((0, 'red'), (15, 'red'), (165, 'red'), (179, 'red'),
                              (60, 'green'), (120, 'blue')):
            with self.subTest(hue=hue):
                detections, masks = Segmenter().detect(image(hue))
                self.assertEqual(len(detections), 1)
                d = detections[0]
                self.assertEqual(d['color'], expected)
                self.assertEqual((d['u'], d['v'], d['width'], d['height']),
                                 (69.5, 59.5, 20., 20.))
                self.assertEqual(d['area'], 400)
                self.assertEqual(set(masks), {'red', 'green', 'blue'})

    def test_saturation_value_and_area_rejection(self):
        for hue in (0, 60, 120):
            for kwargs in ({'saturation': 200}, {'value': 100}, {'size': 19}):
                self.assertEqual(Segmenter().detect(image(hue, **kwargs))[0], [])
        for hue in (30, 90, 150):
            self.assertEqual(Segmenter().detect(image(hue))[0], [])
        self.assertEqual(Segmenter().detect(np.zeros((200, 300, 3), np.uint8))[0], [])

    def test_multiple_clusters_and_maximum_area(self):
        pixels = image()
        pixels[100:125, 100:125] = (255, 0, 0)
        clusters, _ = Segmenter().detect(pixels)
        self.assertEqual([d['color'] for d in clusters], ['blue', 'red'])
        self.assertEqual(len(Segmenter(Limits(max_detections=1)).detect(pixels)[0]), 1)
        self.assertEqual(Segmenter().detect(np.full((200, 300, 3), (0, 0, 255), np.uint8))[0], [])

    def test_led_rim_and_border(self):
        pixels = np.zeros((200, 300, 3), np.uint8)
        cv2.circle(pixels, (100, 100), 30, (0, 0, 255), 4)
        self.assertEqual(len(Segmenter().detect(pixels)[0]), 1)
        pixels[:] = 0
        pixels[:20, :20] = (0, 0, 255)
        self.assertEqual(len(Segmenter().detect(pixels)[0]), 1)
        self.assertEqual(Segmenter(Limits(reject_border=True)).detect(pixels)[0], [])

    def test_invalid_limits_and_ranges(self):
        for cfg in (Limits(kernel=2), Limits(min_area=0), Limits(min_area=1.5),
                    Limits(min_fill=float('nan')), Limits(max_area_fraction=2)):
            with self.assertRaises(ValueError):
                Segmenter(cfg)
        for red in ([180, 0, 0, 200, 255, 255], [0, 1], [float('nan')]*6):
            with self.assertRaises(ValueError):
                Segmenter(ranges=dict(DEFAULT_RANGES, red=red))

    def test_rectification_and_rejections(self):
        pixels = image()
        self.assertTrue(np.array_equal(rectify(pixels, info()), pixels))
        for fault in ('size', 'frame', 'rotation', 'baseline', 'binning', 'roi', 'model', 'nan'):
            camera = deepcopy(info())
            if fault == 'size': camera.width -= 1
            elif fault == 'frame': camera.header.frame_id = ''
            elif fault == 'rotation': camera.r[0] = 0.
            elif fault == 'baseline': camera.p[3] = 1.
            elif fault == 'binning': camera.binning_x = 2
            elif fault == 'roi': camera.roi.x_offset = 1
            elif fault == 'model': camera.distortion_model = 'fisheye'
            elif fault == 'nan': camera.k[0] = float('nan')
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                rectify(pixels, camera)

    def test_capture_deadlines(self):
        self.assertTrue(capture_valid(10_000_000_000, 10_100_000_000, 0, .5))
        for stamp, now, last in ((0, 1, 0), (10, 9, 0), (10, 11, 10),
                                 (1_000_000_000, 2_000_000_000, 0)):
            self.assertFalse(capture_valid(stamp, now, last, .5))

    def test_real_interpreter_association(self):
        clusters, _ = Segmenter().detect(image())
        d = clusters[0]
        detection = Detection(0, d['u'], d['v'], d['width'], d['height'],
                              d['color'], d['confidence'])
        camera = Camera(300, 200, 200., 200., 150., 100.)
        optical = ((d['u']-150.)/200.*5., (d['v']-100.)/200.*5., 5.)
        geometry = GeometryCandidate(42, (4., 2., 0.), optical)
        result = interpret_usv([detection], [geometry], camera, 10., 10., 10.1)
        self.assertEqual(result[0].color, 'red')
        self.assertEqual((result[0].x, result[0].y), (4., 2.))
        # Image-only detections cannot manufacture a positioned observation.
        self.assertEqual(interpret_usv([detection], [], camera, 10., 10., 10.1), [])


if __name__ == '__main__':
    unittest.main()
