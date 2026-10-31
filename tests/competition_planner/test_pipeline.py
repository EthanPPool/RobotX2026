import unittest

from robotx_buoy_interpretation.contracts import Observation
from robotx_buoy_interpretation.tracking import BuoyTracker
from robotx_competition_planner.survey import Survey
from robotx_competition_planner.models import Marker, PlannerConfig
from robotx_competition_planner.mission import SafePassage
from fixtures import BOUNDARY, markers, pose


class PipelineTests(unittest.TestCase):
    def test_color_frames_to_shared_tracks_to_aerial_survey_to_route(self):
        tracker = BuoyTracker(reference_id='datum', source_mode='assisted')
        survey = Survey(9., aerial=True)
        for i in range(13):
            now = 10.+i*.5
            observations = []
            for m in markers('uav'):
                color = m.color
                if m.light_state == 'flashing' and int(i*.5)%2:
                    color = 'black'
                observations.append(Observation(str(m.track_id), 'uav', now,
                    m.x, m.y, 0., color, .95, .15, .25, 'unknown'))
            tracker.ingest(observations, 'robotx_map', 'datum', now)
            mapped = tuple(Marker(r['track_id'], r['x'], r['y'], r['radius'], r['uncertainty'],
                r['color'], r['light_state'], r['confidence'], r['color_stamp'], r['color_source'])
                for r in tracker.snapshot(now))
            survey.update(mapped, now)
        mission = SafePassage(PlannerConfig(), BOUNDARY, 'advanced')
        mission.start(survey.snapshot(now), pose(now), now, 'integration-test', True, 90.)
        self.assertEqual(mission.state, 'APPROACH_ENTRY')
        self.assertEqual(len(mission.plan.required_ids), 4)
        self.assertEqual(len(mission.preview()), 3)


if __name__ == '__main__':
    unittest.main()
