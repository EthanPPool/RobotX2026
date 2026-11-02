from dataclasses import replace
import unittest
from robotx_competition_planner.models import Marker,PlannerConfig,Pose
from robotx_competition_planner.incremental import IncrementalPassage,LocalCourse
from fixtures import BOUNDARY


class IncrementalCoreTests(unittest.TestCase):
    def test_empty_and_entry_only_maps_start_without_expected_count(self):
        for markers in ((),(Marker(1,0.,0.,.25,.1,'blue','flashing',.95,10.,'usv'),)):
            m=IncrementalPassage(PlannerConfig(),BOUNDARY);m.observe(markers)
            m.start(Pose(-6.,0.,0.,10.),10.)
            self.assertIsNotNone(m.trajectory)

    def test_closest_unknown_is_not_a_free_side_passage(self):
        c=LocalCourse((Marker(1,5.,2.),),BOUNDARY,PlannerConfig())
        self.assertFalse(c.passages((0.,0.),(10.,0.))[0])

    def test_new_obstacle_invalidates_local_reference(self):
        m=IncrementalPassage(PlannerConfig(),BOUNDARY)
        m.observe((Marker(1,0.,0.,.25,.1,'blue','flashing',.95,10.,'usv'),))
        p=Pose(-6.,0.,0.,10.);m.start(p,10.);old=m.trajectory
        m.observe((Marker(2,-4.,0.,.25,.1,'black','inactive',.95,11.,'usv'),))
        m.prepare(replace(p,stamp=11.),11.)
        self.assertIsNot(m.trajectory,old)
        self.assertTrue(any(e['event']=='REPLAN_MAP_CHANGED' for e in m.history))

    def test_aerial_provenance_does_not_replace_precise_local_geometry(self):
        m=IncrementalPassage(PlannerConfig(),BOUNDARY,'advanced')
        m.observe((Marker(1,0.,0.,.25,.1),))
        m.observe_aerial((Marker(99,.1,.1,.2,.6,'blue','flashing',.95,10.,'uav'),))
        self.assertEqual(m.endpoint('entry').point,(0.,0.))
        self.assertEqual(m.endpoint('entry').color_source,'uav')

    def test_static_semantic_memory_survives_dropout_but_conflict_invalidates_it(self):
        m=IncrementalPassage(PlannerConfig(),BOUNDARY)
        seen=Marker(15,0.,0.,.25,.1,'blue','flashing',.95,10.,'usv');m.observe((seen,))
        unseen=replace(seen,color='unknown',light_state='unknown',confidence=0.)
        m.observe((unseen,));self.assertIsNotNone(m.endpoint('entry'))
        m.observe((unseen,),conflicts=(15,));self.assertIsNone(m.endpoint('entry'))

    def test_advanced_reports_are_external_by_default(self):
        m=IncrementalPassage(PlannerConfig(),BOUNDARY,'advanced')
        m.observe((Marker(1,0.,0.,.25,.1,'blue','flashing',.95,10.,'uav'),
            Marker(2,26.,0.,.25,.1,'blue','steady',.95,10.,'uav')))
        p=Pose(-6.,0.,0.,10.);m.start(p,10.)
        self.assertEqual(m.state,'HOLD');self.assertFalse(m.entry_reported)
        m.acknowledge_report();m.prepare(replace(p,stamp=11.),11.)
        self.assertEqual(m.state,'APPROACH_ENTRY')


if __name__=='__main__':unittest.main()
