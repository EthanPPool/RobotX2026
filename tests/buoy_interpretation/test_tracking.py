import unittest
from dataclasses import replace

from robotx_buoy_interpretation.contracts import Observation
from robotx_buoy_interpretation.tracking import BuoyTracker


def buoy(source='usv', stamp=10., x=5., color='red', observation_id='1', **kwargs):
    return Observation(observation_id, source, stamp, x, 0., 0., color,
                       kwargs.get('confidence', 0.9), kwargs.get('uncertainty', 0.25),
                       kwargs.get('radius', 0.3), kwargs.get('light_state', 'unknown'))


class TrackingTests(unittest.TestCase):
    def make(self, mode='assisted', **kwargs):
        return BuoyTracker(reference_id='datum', source_mode=mode, **kwargs)

    def add(self, tracker, *obs, now=10.):
        return tracker.ingest(list(obs), 'robotx_map', 'datum', now)

    def test_usv_only_rejects_uav_without_waiting_for_it(self):
        t = self.make('usv_only')
        self.assertEqual(self.add(t, buoy('uav'))['rejected'], 1)
        self.assertEqual(self.add(t, buoy())['accepted'], 1)
        self.assertTrue(t.snapshot(10.)[0]['actionable'])

    def test_same_buoy_cross_vehicle_merges_without_averaging(self):
        t = self.make()
        self.add(t, buoy('uav', x=5.2, uncertainty=0.8))
        self.add(t, buoy('usv', x=5., uncertainty=0.2))
        tracks = t.snapshot(10.)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['x'], 5.)
        self.assertEqual(tracks[0]['active_sources'], ['uav', 'usv'])
        self.assertGreaterEqual(tracks[0]['radius']+tracks[0]['uncertainty'], 1.3-1e-8)

    def test_conflict_withholds_route_semantics(self):
        t = self.make()
        self.add(t, buoy('usv', color='red'))
        self.add(t, buoy('uav', color='green'))
        r = t.snapshot(10.)[0]
        self.assertTrue(r['semantic_conflict'])
        self.assertFalse(r['actionable'])
        self.assertEqual(r['color'], 'unknown')

    def test_uav_dropout_preserves_live_usv_path_input(self):
        t = self.make()
        self.add(t, buoy('uav', color='green'))
        self.add(t, buoy('usv', color='red'))
        self.add(t, buoy('usv', stamp=16., color='red'), now=16.)
        r = t.snapshot(16.)[0]
        self.assertTrue(r['actionable'])
        self.assertEqual(r['color'], 'red')
        self.assertEqual(r['active_sources'], ['usv'])

    def test_replay_does_not_refresh_stamp(self):
        t = self.make()
        self.add(t, buoy())
        self.assertEqual(self.add(t, buoy(), now=10.5)['rejected'], 1)
        r = t.snapshot(12.)[0]
        self.assertFalse(r['actionable'])
        self.assertEqual(r['position_stamp'], 10.)

    def test_new_geometry_preserves_older_semantics_until_expiry(self):
        t = self.make()
        self.add(t, buoy())
        self.add(t, buoy(stamp=10.2, color='unknown'), now=10.2)
        r = t.snapshot(10.2)[0]
        self.assertEqual(r['color'], 'red')
        self.assertEqual(r['color_stamp'], 10.)
        self.assertEqual(r['position_stamp'], 10.2)
        self.add(t, buoy(stamp=12., color='unknown'), now=12.)
        self.assertFalse(t.snapshot(12.)[0]['actionable'])

    def test_same_time_and_delayed_semantic_enrichment(self):
        t = self.make()
        self.add(t, buoy(color='unknown'))
        self.assertEqual(self.add(t, buoy())['accepted'], 1)
        self.add(t, buoy(stamp=10.2, color='unknown'), now=10.2)
        self.assertEqual(self.add(t, buoy(stamp=10.1, color='green'), now=10.2)['accepted'], 1)
        r = t.snapshot(10.2)[0]
        self.assertEqual(r['position_stamp'], 10.2)
        self.assertEqual(r['color_stamp'], 10.1)

    def test_recycled_image_id_is_not_a_persistent_track_id(self):
        t = self.make()
        self.add(t, buoy(x=5., observation_id='0'))
        self.add(t, buoy(stamp=10.1, x=15., observation_id='0'), now=10.1)
        self.assertEqual(len(t.snapshot(10.1)), 2)

    def test_reference_mismatch_stale_future_and_invalid_rejected(self):
        t = self.make()
        self.assertEqual(t.ingest([buoy()], 'odom', 'datum', 10.)['accepted'], 0)
        self.assertEqual(t.ingest([buoy()], 'robotx_map', 'other', 10.)['accepted'], 0)
        for b in (buoy(stamp=8.), buoy(stamp=10.1), buoy(x=float('nan')),
                  buoy(uncertainty=3.)):
            self.assertEqual(self.add(t, b)['accepted'], 0)

    def test_tight_cluster_association_rejects_ambiguous_observation(self):
        t = self.make()
        self.add(t, buoy(x=5.), buoy(x=6., observation_id='2'))
        self.assertEqual(self.add(t, buoy('uav', x=5.5))['rejected'], 1)

    def test_same_frame_candidates_cannot_collapse(self):
        t = self.make()
        r = self.add(t, buoy(), buoy(x=5.2, observation_id='2'))
        self.assertEqual((r['accepted'], r['rejected']), (2, 0))
        self.assertEqual(len(t.snapshot(10.)), 2)
        self.assertEqual(self.add(t, buoy(stamp=10.1),
            buoy(stamp=10.1, x=5.2, observation_id='2'), now=10.1)['accepted'], 2)

    def test_duplicate_observation_id_in_batch_is_rejected(self):
        t = self.make()
        r = self.add(t, buoy(), buoy(x=8.))
        self.assertEqual((r['accepted'], r['rejected']), (1, 1))

    def test_association_prefers_active_position_over_old_precise_uav(self):
        t = self.make()
        self.add(t, buoy('uav', uncertainty=0.01))
        self.add(t, buoy('usv', x=5.6))
        self.add(t, buoy('usv', stamp=16., x=5.6), now=16.)
        self.assertEqual(self.add(t, buoy('usv', stamp=16.1, x=6.), now=16.1)['accepted'], 1)
        self.assertEqual(len(t.snapshot(16.1)), 1)

    def test_unknown_buoy_is_an_obstacle_not_a_route_marker(self):
        t = self.make()
        self.add(t, buoy(color='unknown'))
        r = t.snapshot(12.)[0]
        self.assertEqual(r['color'], 'unknown')
        self.assertFalse(r['actionable'])
        self.assertAlmostEqual(r['radius'], 0.3)

    def test_light_state_conflict_is_not_a_color_conflict_only(self):
        t = self.make()
        self.add(t, buoy('usv', color='blue', light_state='steady'))
        self.add(t, buoy('uav', color='blue', light_state='flashing'))
        self.assertTrue(t.snapshot(10.)[0]['semantic_conflict'])

    def test_capacity_retention_and_clock_rewind_are_bounded(self):
        t = self.make(max_tracks=1)
        self.add(t, buoy())
        self.assertEqual(self.add(t, buoy(x=20.))['rejected'], 1)
        self.assertEqual(t.snapshot(131.), [])
        self.add(t, buoy(stamp=132.), now=132.)
        self.assertEqual(t.snapshot(100.), [])


if __name__ == '__main__':
    unittest.main()
