from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import robotx_competition_planner
from robotx_competition_planner.models import Marker
from robotx_competition_planner.incremental import IncrementalPassage
from test_node import module,node,wire_map,response,FakeNow,stamp,header


def incremental_node(tier='core',partial=True):
    base=module()
    path=Path(__file__).resolve().parents[2]/'coordination_ws/src/robotx_competition_planner/robotx_competition_planner/incremental_node.py'
    with patch.dict(sys.modules,{'robotx_competition_planner.node':base}),patch.object(robotx_competition_planner,'node',base,create=True):
        spec=importlib.util.spec_from_file_location('robotx_competition_planner._incremental_node_test',path)
        adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)
    old=node(tier)
    n=adapter.IncrementalPlanner.__new__(adapter.IncrementalPlanner);n.__dict__.update(old.__dict__)
    n.get_clock=lambda:NS(now=lambda:FakeNow(n.time))
    n.planning_margin=.5;n.observation_distance=8.;n.mission=n.make_mission();n.last_engine_event=0
    source=n.latest_map;n.latest_map=None
    if partial:source.tracks=source.tracks[:1]
    n.on_map(source)
    return n


class IncrementalNodeTests(unittest.TestCase):
    def test_partial_map_starts_and_publishes_local_three_step_reference(self):
        n=incremental_node();self.assertEqual(len(n.mission.belief),1)
        self.assertTrue(n.start(None,response()).success)
        n.time+=.1;n.pose=replace(n.pose,stamp=n.time);n.tick()
        p=n.preview_messages[-1]
        self.assertTrue(p.valid,p.reason)
        self.assertEqual(len(p.samples),3)
        self.assertEqual([s.offset_seconds for s in p.samples],[0.,.1,.2])
        self.assertEqual(p.reference_id,'datum')

    def test_stale_map_invalidates_reference_and_freezes_local_clock(self):
        n=incremental_node();n.start(None,response());before=n.mission.elapsed
        n.time+=.8;n.pose=replace(n.pose,stamp=n.time);n.scan=replace(n.scan,stamp=n.time)
        n.tick()
        self.assertFalse(n.preview_messages[-1].valid)
        self.assertEqual(n.preview_messages[-1].reason,'BUOY_MAP_STALE')
        self.assertEqual(n.mission.elapsed,before)

    def test_new_obstacle_in_map_replans_instead_of_using_attempt_snapshot(self):
        n=incremental_node();n.start(None,response());old=n.mission.trajectory
        n.time+=1.;n.pose=replace(n.pose,stamp=n.time);n.scan=replace(n.scan,stamp=n.time)
        entry=Marker(1,0.,0.,.25,.15,'blue','flashing',.95,n.time,'usv')
        block=Marker(2,-4.,0.,.25,.1,'black','inactive',.95,n.time,'usv')
        n.on_map(wire_map((entry,block),t=n.time));n.tick()
        self.assertIsNot(n.mission.trajectory,old)
        self.assertTrue(any(e['event']=='REPLAN_MAP_CHANGED' for e in n.mission.history))

    def test_advanced_start_needs_no_full_map_but_holds_for_external_report(self):
        n=incremental_node('advanced',partial=False)
        self.assertTrue(n.start(None,response()).success)
        self.assertEqual(n.mission.state,'HOLD')
        self.assertFalse(n.mission.entry_reported)
        self.assertTrue(n.reported(NS(data=True),response()).success)
        self.assertTrue(n.mission.entry_reported)

    def test_aerial_callback_enriches_labels_without_losing_local_geometry(self):
        n=incremental_node('advanced')
        # Replace with local geometry-only input, then receive actual aerial evidence.
        n.mission=IncrementalPassage(n.config,n.boundary,'advanced')
        n.mission.observe((Marker(1,0.,0.,.25,.1),))
        observation=NS(source='uav',observed_at=stamp(10.),position=NS(x=.1,y=.1),
            radius=.2,uncertainty=.6,color='blue',light_state='flashing',confidence=.95)
        n.on_aerial(NS(header=header(),reference_id='datum',observations=[observation]))
        m=n.mission.endpoint('entry')
        self.assertIsNotNone(m);self.assertEqual(m.point,(0.,0.));self.assertEqual(m.color_source,'uav')


if __name__=='__main__':unittest.main()
