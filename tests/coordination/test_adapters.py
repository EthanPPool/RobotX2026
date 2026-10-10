"""ROS adapter Python behavior on a shim, including source/receipt deadlines."""
import math
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace as Box

import pytest

from robotx_coordination.geometry import Footprint, Obstacle, Pose, finite, fresh, reference_id, yaw
from robotx_coordination.planning import SurveyPlanner, route_target
from boat_stack_shim import ActualBoatStack, Node, Publisher, TwistStamped, load_class, load_module

ROOT=Path(__file__).resolve().parents[2]
SUPPORT=load_module(ROOT/'coordination_ws/src/robotx_coordination/robotx_coordination/ros_support.py',
    {'Point':Box,'Pose':Pose,'fresh':fresh})


def stamp(seconds):return Box(sec=int(seconds),nanosec=round((seconds-int(seconds))*1e9))


def odom(t=10.,x=0.,y=0.):
    return Box(header=Box(stamp=stamp(t),frame_id='robotx_map'),
        pose=Box(pose=Box(position=Box(x=x,y=y,z=0.),orientation=Box(x=0.,y=0.,z=0.,w=1.)),covariance=[0.]*36))


def route():
    context={name:SUPPORT[name] for name in ['Received','now_sec','odometry_pose','point','seconds']}
    context.update({'Footprint':Footprint,'Obstacle':Obstacle,'finite':finite,'fresh':fresh,'reference_id':reference_id,
        'yaw':yaw,'SurveyPlanner':SurveyPlanner,'route_target':route_target,'Odometry':Box,'Path':Box,'PoseStamped':Box,
        'DetectedObjectArray':Box,'BuoyMap':Box,'deque':deque})
    cls=load_class(ROOT/'boat_ws/src/boat_route_navigation/boat_route_navigation/route_mission.py','RouteMission',context)
    node=cls();node.configured=True;node.enabled=True;return node


def map_message(t=10.):
    corners=[Box(x=x,y=y) for x,y in [(-9.,-19.),(59.,-19.),(59.,19.),(-9.,19.)]]
    return Box(header=Box(stamp=stamp(t),frame_id='robotx_map'),reference_id=reference_id(0.,0.,0.),
        origin_latitude=0.,origin_longitude=0.,water_altitude=0.,buoys=[],
        footprints=[Box(corners=corners,observed_at=stamp(t))])


@pytest.mark.parametrize('wrong', ['frame','id','datum','nonfinite_datum'])
def test_route_rejects_map_reference(wrong):
    Node.clock=10.;node=route();m=map_message()
    if wrong=='frame':m.header.frame_id='uav_local'
    elif wrong=='id':m.reference_id='wrong'
    elif wrong=='datum':m.water_altitude=1.
    else:m.origin_latitude=math.nan
    node.on_map(m);assert node.map.message is None


@pytest.mark.parametrize('missing,reason', [('pose','USV_LOCALIZATION_STALE'),('map','UAV_MAP_STALE'),('local','LOCAL_PERCEPTION_STALE')])
def test_route_required_input_loss(missing,reason):
    Node.clock=10.;node=route()
    if missing!='pose':node.on_pose(odom())
    if missing!='map':node.on_map(map_message())
    if missing!='local':node.local.update(Box(header=Box(stamp=stamp(10.),frame_id='base_link'),objects=[]),10.)
    node.tick();assert node.target_pub.last.stop is True and node.state_pub.last.data==reason


def test_replayed_map_does_not_refresh_source_age():
    Node.clock=10.;node=route();node.on_map(map_message());Node.clock=14.
    node.on_pose(odom(14.));node.local.update(Box(header=Box(stamp=stamp(14.),frame_id='base_link'),objects=[]),14.)
    node.on_map(map_message());node.tick();assert node.state_pub.last.data=='UAV_MAP_STALE'


def guard():
    context={name:SUPPORT[name] for name in ['Received','now_sec','seconds']}
    context['AsyncParameterClient']=lambda *args:Box(services_are_ready=lambda:False)
    node=load_class(ROOT/'uuv_ws/src/uuv_follow/uuv_follow/velocity_guard.py','VelocityGuard',context)()
    node.authorized=True;node.auth_rx=time.monotonic();node.frame_verified=True;node.frame_rx=time.monotonic()
    state=Box(connected=True,armed=True,mode='GUIDED')
    node.state.update(state,Node.clock)
    msg=TwistStamped();msg.header.stamp=stamp(Node.clock);msg.header.frame_id='robotx_map'
    msg.twist.linear.x=0.1;msg.twist.linear.y=0.1
    node.cmd.update(msg,Node.clock)
    return node


@pytest.mark.parametrize('fault',['frame','nonfinite','cmd_stale','auth_stale','wrong_mode','disarmed','state_stale','frame_not_verified','frame_check_stale'])
def test_uuv_guard_faults_zero(fault):
    Node.clock=10.;node=guard()
    if fault=='frame':node.cmd.message.header.frame_id='base_link'
    elif fault=='nonfinite':node.cmd.message.twist.linear.x=math.nan
    elif fault=='cmd_stale':node.cmd.stamp=9.
    elif fault=='auth_stale':node.auth_rx-=1.
    elif fault=='wrong_mode':node.state.message.mode='ALT_HOLD'
    elif fault=='disarmed':node.state.message.armed=False
    elif fault=='state_stale':node.state.stamp=9.
    elif fault=='frame_not_verified':node.frame_verified=False
    else:node.frame_rx-=4.
    node.tick();cmd=node.pub.last.twist
    assert (cmd.linear.x,cmd.linear.y,cmd.linear.z,cmd.angular.z)==(0.,0.,0.,0.)


def test_uuv_guard_verified_world_velocity():
    Node.clock=10.;node=guard();node.tick();cmd=node.pub.last
    assert cmd.header.frame_id=='robotx_map' and cmd.twist.linear.x==pytest.approx(.1)


def test_real_boat_bridge_completes_with_loiter_and_revoked_authority():
    Node.clock=10.;stack=ActualBoatStack(ROOT)
    assert stack.step(10.1,(2.,0.,.12,'ROUTE_TRACKING'))[0]>0.
    assert stack.step(10.2,(0.,0.,0.,'MISSION_COMPLETE'))==(0.,0.)
    assert stack.mode=='LOITER' and stack.bridge.get_parameter('autonomy_enabled').value is False


def test_map_loss_stops_while_boat_autonomy_is_active():
    Node.clock=10.;stack=ActualBoatStack(ROOT)
    assert stack.step(10.1,(2.,0.,.12,'ROUTE_TRACKING'))[0]>0.
    assert stack.step(10.2,(0.,0.,0.,'UAV_MAP_STALE'))==(0.,0.)


def test_mapper_preserves_photo_optical_frame():
    import cv2
    import numpy as np
    from robotx_coordination.mapping import SurveyMemory, project_image
    context={name:SUPPORT[name] for name in ['now_sec','seconds','quaternion','point']}
    class FakeTime:
        def __init__(self,seconds):self.value=seconds
        def to_msg(self):return stamp(self.value)
        @staticmethod
        def from_msg(value):return value
    class TransformError(Exception):pass
    class BridgeError(Exception):pass
    context.update({'Pose':Box,'Time':FakeTime,'BuoyMap':lambda:Box(buoys=[],footprints=[]),'GeoImage':Box,
        'SurveyBuoy':Box,'SurveyFootprint':Box,'CompressedImage':Box,'fresh':fresh,'cv2':cv2,
        'project_image':project_image,'TransformException':TransformError,'CvBridgeError':BridgeError})
    cls=load_class(ROOT/'uav_ws/src/uav_mapping/uav_mapping/mapper.py','UavMapper',context)
    node=cls.__new__(cls);Node.__init__(node,'mapper_test');Node.clock=10.
    node.max_rate=2.;node.last_run=-1e9;node.enabled=True;node.max_age=.5;node.frame='robotx_map'
    node.origin=(0.,0.,0.);node.ref=reference_id(*node.origin);node.uncertainty=.25;node.max_range=100.
    image=np.zeros((240,320,3),np.uint8)
    node.info=Box(header=Box(frame_id='uav/optical'),width=320,height=240,distortion_model='plumb_bob',
        k=[160.,0.,160.,0.,160.,120.,0.,0.,1.],d=[0.]*5)
    q=Box(x=math.sqrt(.5),y=-math.sqrt(.5),z=0.,w=0.)
    transform=Box(transform=Box(translation=Box(x=0.,y=0.,z=10.),rotation=q))
    node.buffer=Box(lookup_transform=lambda *args:transform)
    node.bridge=Box(imgmsg_to_cv2=lambda *args,**kwargs:image)
    node.memory=SurveyMemory();node.map_pub=Publisher();node.image_pub=Publisher();node.status_pub=Publisher()
    msg=Box(header=Box(stamp=stamp(10.),frame_id='uav/optical'),width=320,height=240)
    node.pending_image=msg;node.process_image(msg)
    assert node.map_pub.last.header.frame_id=='robotx_map'
    assert node.image_pub.last.image.header.frame_id=='uav/optical'
    assert msg.header.frame_id=='uav/optical'


def test_sim_startup_waits_for_guided_state_publication():
    context={'SetBool':Box(Request=Box)}
    cls=load_class(ROOT/'coordination_ws/src/robotx_coordination_sim/robotx_coordination_sim/sim_world.py','SimWorld',context)
    node=cls.__new__(cls);Node.__init__(node,'world_test')
    calls=[];client=Box(service_is_ready=lambda:True,call_async=lambda request:calls.append(request) or Box())
    node.started=time.monotonic()-4.;node.complete=False;node.stage=2;node.clients=[client]*3
    node.pending=None;node.boat_mode='HOLD';node.guided_since=None
    node.bootstrap();assert node.boat_mode=='GUIDED' and not calls
    node.bootstrap();assert not calls
    node.guided_since=time.monotonic()-.3;node.bootstrap();assert len(calls)==1 and calls[0].data is True
