from dataclasses import replace
import unittest

from robotx_competition_planner.survey import Survey
from fixtures import markers


class SurveyTests(unittest.TestCase):
    def test_multiple_images_combine_and_geometry_refines_without_erasing_color(self):
        s = Survey(10., 90., True)
        a = markers('uav', 11.)
        s.update(a[:5], 11.)
        s.update(a[5:], 20.)
        s.update(tuple(replace(m, x=m.x+.1, color='unknown', confidence=0.,
                               light_state='unknown', semantic_stamp=0.) for m in a), 25.)
        out = s.snapshot(25.)
        self.assertEqual(len(out), 10)
        self.assertEqual(out[0].color, 'blue')
        self.assertAlmostEqual(out[0].x, .1)
        self.assertEqual(out[0].semantic_stamp, 11.)

    def test_new_attempt_rejects_pre_epoch_semantics(self):
        s = Survey(20.)
        s.update(markers(stamp=19.), 20.)
        self.assertTrue(all(m.color == 'unknown' for m in s.snapshot(20.)))

    def test_aerial_survey_cannot_be_filled_from_usv_color_assignments(self):
        s = Survey(10., aerial=True)
        s.update(markers('usv', 11.), 11.)
        self.assertTrue(all(m.color == 'unknown' for m in s.snapshot(11.)))

    def test_conflict_invalidates_prior_semantics_and_age_is_not_refreshed(self):
        s = Survey(10., max_age=5.)
        s.update(markers(stamp=11.), 11.)
        s.update(markers(stamp=11.), 12., conflicting_ids=(1,))
        self.assertEqual(s.snapshot(12.)[0].color, 'unknown')
        self.assertTrue(all(m.color == 'unknown' for m in s.snapshot(17.)))


if __name__ == '__main__':
    unittest.main()
