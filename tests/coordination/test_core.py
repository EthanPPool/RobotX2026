import math
from dataclasses import replace

import cv2
import numpy as np
import pytest

from robotx_coordination.geometry import (Footprint, Obstacle, Pose, fresh, geodetic_to_enu,
    ray_to_water, reference_id, rotation, yaw_quaternion)
from robotx_coordination.mapping import SurveyMemory, project_image
from robotx_coordination.planning import SurveyPlanner, route_target
from robotx_coordination.following import TrailFollower


DOWN = (math.sqrt(0.5),-math.sqrt(0.5),0.,0.)
AREA = Footprint(((-10.,-10.),(40.,-10.),(40.,10.),(-10.,10.)),10.)


def test_georeference_center_and_axes():
    np.testing.assert_allclose(ray_to_water((0,0,1),(10,20,10),DOWN),(10,20,0),atol=1e-9)
    np.testing.assert_allclose(ray_to_water((1,0,1),(10,20,10),DOWN),(10,10,0),atol=1e-9)
    np.testing.assert_allclose(ray_to_water((0,-1,1),(10,20,10),DOWN),(20,20,0),atol=1e-9)


def test_georeference_rotated_vehicle():
    # Downward optical ray at a 90 degree rotated body.
    r = rotation(yaw_quaternion(math.pi/2)) @ rotation(DOWN)
    p = r @ np.array([0.,-0.5,1.])
    np.testing.assert_allclose(p,(0.,0.5,-1.),atol=1e-9)


@pytest.mark.parametrize('pose,q,ray',[((0,0,-1),DOWN,(0,0,1)),((0,0,10),(0,0,0,1),(0,0,1)),
    ((0,0,10),(0,0,0,0),(0,0,1)),((math.nan,0,10),DOWN,(0,0,1))])
def test_invalid_camera_geometry_stops(pose,q,ray):
    with pytest.raises(ValueError):ray_to_water(ray,pose,q)


def test_geodetic_shared_datum():
    origin=(30.2,-92.,12.)
    np.testing.assert_allclose(geodetic_to_enu(*origin,origin),0.,atol=1e-6)
    east=geodetic_to_enu(30.2,-91.99999,12.,origin)
    north=geodetic_to_enu(30.20001,-92.,12.,origin)
    assert east[0]>0.9 and abs(east[1])<0.01
    assert north[1]>1. and abs(north[0])<0.01
    assert reference_id(*origin)!=reference_id(30.2,-92.,13.)


def test_image_to_map_all_colors():
    image=np.zeros((240,320,3),np.uint8)
    for pos,color in [((80,120),(0,0,255)),((160,120),(0,255,0)),((240,120),(0,255,255))]:
        cv2.circle(image,pos,5,color,-1)
    k=np.array([[160.,0.,160.],[0.,160.,120.],[0.,0.,1.]])
    obs,footprint=project_image(image,k,[0.]*5,(0.,0.,10.),DOWN,10.,0.25)
    assert {o.color for o in obs}=={'red','green','yellow'}
    assert len(obs)==3 and len(footprint.points)==4
    by_color={o.color:o for o in obs}
    assert by_color['red'].y==pytest.approx(5.)
    assert by_color['yellow'].y==pytest.approx(-5.)
    assert all(abs(o.x)<0.01 for o in obs)


def test_camera_distortion_is_used():
    image=np.zeros((240,320,3),np.uint8);cv2.circle(image,(260,120),5,(0,0,255),-1)
    k=np.array([[160.,0.,160.],[0.,160.,120.],[0.,0.,1.]])
    a,_=project_image(image,k,[0.]*5,(0.,0.,10.),DOWN,10.)
    b,_=project_image(image,k,[0.2,0.,0.,0.,0.],(0.,0.,10.),DOWN,10.)
    assert abs(b[0].y)<abs(a[0].y)


def test_memory_replay_and_conservative_missing_detection():
    memory=SurveyMemory(ttl=1.)
    assert memory.add([Obstacle(0,0,stamp=10.)],AREA,10.)
    assert not memory.add([],AREA,10.1)
    assert memory.add([],replace(AREA,stamp=12.),12.)
    assert len(memory.obstacles)==1  # Occlusion must not erase a known buoy.
    assert all(f.stamp==12. for f in memory.footprints)


def test_close_same_color_buoys_are_not_merged_within_one_image():
    memory=SurveyMemory()
    assert memory.add([Obstacle(0.,0.,stamp=10.),Obstacle(.5,0.,stamp=10.)],AREA,10.)
    assert len(memory.obstacles)==2


def test_track_updates_preserve_old_obstacle_extent():
    memory=SurveyMemory()
    memory.add([Obstacle(0.,0.,radius=.3,stamp=10.)],AREA,10.)
    memory.add([Obstacle(.5,0.,radius=.3,stamp=11.)],replace(AREA,stamp=11.),11.)
    assert len(memory.obstacles)==1 and memory.obstacles[0].radius>=.8


def test_planner_routes_around_buoy():
    planner=SurveyPlanner([AREA],[Obstacle(10.,0.,0.5,0.25)],(-8.,-8.,38.,8.),0.5,1.)
    path=planner.plan((0.,0.),(20.,0.))
    assert len(path)>=3
    assert all(planner.segment_safe(a,b) for a,b in zip(path,path[1:]))
    assert any(abs(p[1])>1.75 for p in path)


def test_unknown_water_and_full_vehicle_footprint_blocked():
    planner=SurveyPlanner([AREA],[],(-15.,-15.,45.,15.),0.5,1.)
    assert not planner.allowed((-9.5,0.))  # Center visible, vehicle disk outside survey.
    assert not planner.plan((0,0),(41,0))
    assert not SurveyPlanner([],[],(-5,-5,5,5)).plan((0,0),(2,0))


def test_obstacle_wall_no_route():
    obstacles=[Obstacle(10.,float(y),0.5,0.5) for y in range(-10,11,2)]
    p=SurveyPlanner([AREA],obstacles,(-8.,-8.,38.,8.))
    assert p.plan((0.,0.),(20.,0.))==[]


def test_segment_circle_check_does_not_skip_small_obstacle():
    p=SurveyPlanner([AREA],[Obstacle(0.25,0.,0.01,0.)],(-8.,-8.,38.,8.),0.5,0.1)
    assert not p.segment_safe((0.,0.),(0.5,0.))


@pytest.mark.parametrize('obstacle',[Obstacle(math.nan,0),Obstacle(1,0,-1),Obstacle(1,0,1,math.inf)])
def test_malformed_map_rejected(obstacle):
    with pytest.raises(ValueError):SurveyPlanner([AREA],[obstacle],(-8,-8,38,8))


def test_excessive_map_bounds_rejected():
    with pytest.raises(ValueError):SurveyPlanner([AREA],[],(-1000,-1000,1000,1000),0.01)


def test_invalid_survey_polygon_rejected():
    crossed=Footprint(((0.,0.),(2.,2.),(0.,2.),(2.,0.)),10.)
    with pytest.raises(ValueError):SurveyPlanner([crossed],[],(-8,-8,38,8))


def test_target_turns_to_rearward_waypoint():
    p=SurveyPlanner([AREA],[],(-8.,-8.,38.,8.))
    x,y,speed,state=route_target((0,0),math.pi,[(0,0),(20,0)],p)
    assert x>0 and abs(y)>=1 and speed==0 and state=='ROUTE_TRACKING'


def test_local_obstacle_stops_current_path():
    p=SurveyPlanner([AREA],[Obstacle(0.75,0.,0.1,0.)],(-8.,-8.,38.,8.),0.5,0.5)
    assert route_target((0.,0.),0.,[(0.,0.),(20.,0.)],p)[-1]=='LOCAL_OBSTACLE_STOP'


def pose(x,y,z,stamp,q=None,frame='robotx_map',variance=0.):
    return Pose((x,y,z),q or yaw_quaternion(0.),stamp,frame,variance)


def warmed_follower():
    f=TrailFollower()
    for i in range(81):
        assert f.observe(pose(i*0.05,0.,0.,10.+i*0.05),10.+i*0.05)
    return f


def test_uuv_trail_and_depth_control():
    f=warmed_follower()
    cmd,state,target=f.command(pose(0.,0.,-1.,14.),14.,True,True)
    assert state=='FOLLOWING' and target[0]<=1.+1e-6
    assert cmd[0]>0 and cmd[2]<0 and math.hypot(*cmd[:2])<=0.4


def test_uuv_waits_for_observed_trail():
    f=TrailFollower();f.observe(pose(0,0,0,10),10)
    assert f.command(pose(-3,0,-1.5,10),10,True,True)[1]=='WAIT_FOR_USV_TRAIL'


@pytest.mark.parametrize('own,now,enabled,authorized,reason',[
    (pose(0,0,-1.5,14),14,False,True,'DISABLED_OR_UNAUTHORIZED'),
    (pose(0,0,-1.5,14),14,True,False,'DISABLED_OR_UNAUTHORIZED'),
    (pose(0,0,-1.5,15),15,True,True,'USV_POSE_STALE'),
    (pose(0,0,-1.5,13),14,True,True,'UUV_LOCALIZATION_UNAVAILABLE'),
    (pose(0,0,-1.5,14,frame='map'),14,True,True,'UUV_LOCALIZATION_UNAVAILABLE'),
    (pose(0,0,-1.5,14,variance=2),14,True,True,'UUV_LOCALIZATION_UNAVAILABLE'),
    (pose(0,0,-6,14),14,True,True,'DEPTH_ENVELOPE'),
    (pose(3.5,0,-1.5,14),14,True,True,'USV_SEPARATION_STOP'),
])
def test_uuv_faults_zero(own,now,enabled,authorized,reason):
    cmd,state,_=warmed_follower().command(own,now,enabled,authorized)
    assert cmd==(0.,0.,0.,0.) and state==reason


def test_uuv_jump_and_link_gap_latch_until_reset():
    f=warmed_follower()
    assert not f.observe(pose(20,0,0,14.05),14.05)
    assert f.command(pose(0,0,-1.5,14.05),14.05,True,True)[1]=='LOCALIZATION_DISCONTINUITY'
    f.reset();assert f.observe(pose(0,0,0,15),15)
    assert not f.observe(pose(0.1,0,0,16),16)


def test_replayed_usv_pose_does_not_refresh_follower():
    f=warmed_follower();assert not f.observe(pose(4.,0.,0.,14.),14.4)
    assert f.command(pose(0.,0.,-1.5,14.6),14.6,True,True)[1]=='USV_POSE_STALE'


def test_follow_trail_corner_has_no_long_shortcut():
    f=TrailFollower(distance=1.)
    points=[(i*.05,0.) for i in range(61)]+[(3.,i*.05) for i in range(1,61)]
    for i,(x,y) in enumerate(points):
        t=10+i*.05;assert f.observe(pose(x,y,0.,t),t)
    own=pose(1.,0.,-1.5,16.)
    cmd,state,target=f.command(own,16.,True,True)
    assert state=='FOLLOWING' and abs(cmd[1])<1e-6 and target[0]<2.


@pytest.mark.parametrize('stamp,now,expected',[(10,10,True),(9,10,False),(11,10,False),(math.nan,10,False)])
def test_stamp_freshness(stamp,now,expected):
    assert fresh(stamp,now,0.5)==expected
