from dataclasses import replace
import unittest

from robotx_buoy_interpretation.beacon import BeaconHistory
from robotx_buoy_interpretation.contracts import Observation
from robotx_buoy_interpretation.tracking import BuoyTracker


class BeaconTests(unittest.TestCase):
    def test_two_on_off_cycles_identify_flashing_not_single_color_sample(self):
        for color in ('red', 'green', 'blue'):
            h = BeaconHistory()
            self.assertIsNone(h.update(color, 10., .9))
            answer = None
            for i in range(1, 21):
                t = i*.25
                answer = h.update(color if int(t)%2 == 0 else 'black', 10.+t, .9)
            self.assertEqual(answer[:2], (color, 'flashing'))

    def test_steady_blue_and_inactive_black_need_contiguous_history(self):
        for color, state in (('blue', 'steady'), ('black', 'inactive')):
            h = BeaconHistory()
            for i in range(12):
                answer = h.update(color, 10.+i*.25, .95)
            self.assertEqual(answer[:2], (color, state))

    def test_missing_frames_and_unknown_colors_do_not_mean_off(self):
        h = BeaconHistory()
        h.update('blue', 10., .9)
        self.assertIsNone(h.update('unknown', 11., .9))
        self.assertIsNone(h.update('blue', 12., .9))
        self.assertEqual(len(h.runs), 1)
        self.assertIsNone(h.update('black', 12., .9))

    def test_arbitrary_fast_color_changes_are_not_accepted_as_task_flash(self):
        h = BeaconHistory()
        for i in range(10):
            answer = h.update('blue' if i%2 else 'black', 10.+i*.1, .9)
        self.assertIsNone(answer)

    def test_tracker_retains_active_color_through_measured_off_phase(self):
        t = BuoyTracker(reference_id='datum')
        out = None
        for i in range(25):
            elapsed = i*.25
            color = 'blue' if int(elapsed)%2 == 0 else 'black'
            o = Observation('buoy', 'usv', 10.+elapsed, 5., 0., 0., color, .95, .2)
            t.ingest([o], 'robotx_map', 'datum', 10.+elapsed)
            out = t.snapshot(10.+elapsed)[0]
            if elapsed >= 4.:
                self.assertEqual(out['color'], 'blue')
                self.assertEqual(out['light_state'], 'flashing')
                self.assertEqual(out['color_source'], 'usv')

    def test_explicit_teammate_temporal_classification_takes_precedence(self):
        t = BuoyTracker(reference_id='datum')
        o = Observation('buoy', 'usv', 10., 5., 0., 0., 'blue', .95, .2,
                        light_state='flashing')
        t.ingest([o], 'robotx_map', 'datum', 10.)
        self.assertEqual(t.snapshot(10.)[0]['light_state'], 'flashing')


if __name__ == '__main__':
    unittest.main()
