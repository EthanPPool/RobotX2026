import math
import unittest

from robotx_competition_planner.models import Marker,PlannerConfig,Pose
from robotx_competition_planner.routing import Course,Orbit
from robotx_competition_planner.mission import SafePassage
from fixtures import markers,BOUNDARY


class ApproachRulesTests(unittest.TestCase):
    def test_reverse_start_approach_is_side_legal(self):
        c=Course(markers(),BOUNDARY,PlannerConfig())
        p=c.plan(Pose(32.,0.,math.pi,10.))
        self.assertTrue(all(c.passages(a,b,local_only=True)[0] for a,b in zip(p.approach,p.approach[1:])))

    def test_runtime_all_phases_enforce_legality_without_crediting_approach(self):
        c=Course((Marker(1,5.,2.,color='red',light_state='flashing'),
                  Marker(2,30.,10.,color='black',light_state='inactive')),BOUNDARY,PlannerConfig())
        for state in SafePassage.PHASES:
            m=SafePassage(PlannerConfig(),BOUNDARY)
            m.course=c;m.state=state;m.last_pose=Pose(4.99,0.,0.,10.);m.last_now=10.
            m.update(Pose(5.01,0.,0.,10.1),10.1)
            self.assertEqual(m.state,'STOPPED',state)
            self.assertEqual(m.passed_mask,0)

    def test_wrong_side_orbit_is_rejected_even_with_geometric_clearance(self):
        c=Course((Marker(1,0.,0.,color='blue',light_state='flashing'),
                  Marker(2,5.,0.,color='red',light_state='flashing')),BOUNDARY,PlannerConfig())
        self.assertTrue(c.orbit_clear(c.markers[0],3.))
        self.assertFalse(c.orbit_sides_clear(Orbit((0.,0.),3.,0.,-1)))

    def test_nearest_same_side_buoy_not_required_to_be_abeam(self):
        c=Course((Marker(1,5.,4.,color='red',light_state='flashing'),
                  Marker(2,4.,1.,color='black',light_state='inactive')),BOUNDARY,PlannerConfig())
        self.assertEqual(c.passages((0.,0.),(10.,0.)),(True,0))

    def test_transit_legality_has_no_range_exemption(self):
        c=Course((Marker(1,5.,12.,color='red',light_state='flashing'),
                  Marker(2,30.,-10.,color='black',light_state='inactive')),BOUNDARY,PlannerConfig())
        self.assertFalse(c.passages((0.,0.),(10.,0.))[0])


if __name__=='__main__':unittest.main()
