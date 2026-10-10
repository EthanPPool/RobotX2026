#!/usr/bin/env python3
"""Repeat camera->map->route->actual boat controller/bridge + concurrent UUV.

Uses synthetic BGR images and shared world localization, not real camera images
or hydrodynamics. Writes JSON/CSV evidence and a failure exit status.
"""
import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path

import cv2
import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'coordination_ws/src/robotx_coordination'))
from robotx_coordination.following import TrailFollower
from robotx_coordination.geometry import Pose, rotation, yaw_quaternion
from robotx_coordination.mapping import project_image
from robotx_coordination.planning import SurveyPlanner, route_target
from boat_stack_shim import ActualBoatStack, Node


def run(seed,repo,trace=False):
    rng=random.Random(seed)
    camera=(15.,0.,30.);q=(math.sqrt(.5),-math.sqrt(.5),0.,0.)
    k=np.array([[240.,0.,320.],[0.,240.,240.],[0.,0.,1.]])
    truth=[(8.,1.+rng.uniform(-.3,.3),'red'),(12.,-1.5+rng.uniform(-.3,.3),'green'),
           (16.,rng.uniform(-.3,.3),'yellow'),(22.,2.+rng.uniform(-.3,.3),'red'),(26.,-2.+rng.uniform(-.3,.3),'green')]
    image=np.zeros((480,640,3),np.uint8);image[:]=(80,45,20)
    colors={'red':(0,0,255),'green':(0,255,0),'yellow':(0,255,255)}
    for x,y,color in truth:
        ray=rotation(q).T@(np.array([x,y,0.])-camera);uv=k@ray;u,v=uv[:2]/uv[2]
        cv2.circle(image,(round(u),round(v)),4,colors[color],-1)
    obstacles,footprint=project_image(image,k,[0.]*5,camera,q,10.,0.25)
    planner=SurveyPlanner([footprint],obstacles,(-10.,-20.,45.,20.),0.5,1.)
    boat=np.array([0.,rng.uniform(-.4,.4)]);angle=rng.uniform(-.3,.3)
    path=planner.plan(tuple(boat),(30.,0.))
    if not path:return {'seed':seed,'passed':False,'reason':'no_initial_route'},[]
    follower=TrailFollower();uuv=np.array([-3.,boat[1],-1.5]);uuv_angle=0.
    Node.clock=10.;stack=ActualBoatStack(repo)
    minimum=math.inf;depth_error=0.;target_errors=[];rows=[];done=False
    dt=0.2
    stopped_steps=0
    for step in range(2200):
        t=10.+step*dt
        boat_pose=Pose((*boat,0.),yaw_quaternion(angle),t)
        follower.observe(boat_pose,t)
        own=Pose(tuple(uuv),yaw_quaternion(uuv_angle),t)
        cmd,state,target=follower.command(own,t,True,True)
        requested=route_target(tuple(boat),angle,path,planner)
        if requested[-1]=='LOCAL_OBSTACLE_STOP':
            path=planner.plan(tuple(boat),(30.,0.));requested=route_target(tuple(boat),angle,path,planner)
        if requested[-1] in ('LOCAL_OBSTACLE_STOP','NO_SAFE_ROUTE'):
            stopped_steps+=1
            if stopped_steps>=3:
                return {'seed':seed,'passed':False,'reason':'route_stuck','position':boat.tolist()},rows
        else:
            stopped_steps=0
        v,w=stack.step(t,requested)
        if requested[-1]=='MISSION_COMPLETE':
            done=True
            assert v==0 and w==0 and stack.mode=='LOITER' and not stack.bridge.get_parameter('autonomy_enabled').value
            break
        old_boat=boat.copy();angle+=w*dt;boat+=np.array([math.cos(angle),math.sin(angle)])*v*dt
        if not planner.segment_safe(tuple(old_boat),tuple(boat)):
            return {'seed':seed,'passed':False,'reason':'unsafe_executed_segment'},rows
        uuv+=np.array(cmd[:3])*dt;uuv_angle+=cmd[3]*dt
        minimum=min(minimum,min(math.hypot(boat[0]-x,boat[1]-y)-0.5 for x,y,_ in truth))
        depth_error=max(depth_error,abs(uuv[2]+1.5))
        if target is not None and step>100:
            target_errors.append(math.dist(uuv[:2],target[:2]))
        if trace and step%5==0:
            rows.append({'t':t,'boat_x':boat[0],'boat_y':boat[1],'uuv_x':uuv[0],'uuv_y':uuv[1],'uuv_z':uuv[2],'boat_v':v,'boat_yaw_rate':w,'uuv_state':state})
    final_t=t
    # Concurrent stale-telemetry and authorization failures must produce all-zero commands.
    checks=[]
    checks.append(follower.command(Pose(tuple(uuv),yaw_quaternion(uuv_angle),final_t+1.),final_t+1.,True,True)[0]==(0.,0.,0.,0.))
    checks.append(follower.command(Pose(tuple(uuv),yaw_quaternion(uuv_angle),final_t),final_t,True,False)[0]==(0.,0.,0.,0.))
    checks.append(stack.step(final_t+0.05,(0.,0.,0.,'UAV_MAP_STALE'))==(0.,0.))
    return {'seed':seed,'passed':bool(done and minimum>=1. and all(checks)),
        'completed':done,'duration_sim_sec':final_t-10.,'buoys_detected':len(obstacles),'path_waypoints':len(path),
        'goal_error_m':math.dist(boat,(30.,0.)),'minimum_buoy_edge_clearance_m':minimum,
        'uuv_max_depth_error_m':depth_error,'uuv_mean_tracking_error_m':float(np.mean(target_errors)) if target_errors else None,
        'fault_stops':all(checks),'completion_mode':stack.mode},rows


def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=int,default=100);p.add_argument('--seed',type=int,default=20261010)
    p.add_argument('--repo',type=Path,default=REPO);p.add_argument('--output',type=Path,default=Path('coordination_results'))
    args=p.parse_args()
    if not 1<=args.runs<=10000:p.error('runs must be in 1..10000')
    args.output.mkdir(parents=True,exist_ok=True);results=[]
    for i in range(args.runs):
        result,rows=run(args.seed+i,args.repo,trace=i==0);results.append(result)
        if rows:
            with (args.output/'first_run.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        with (args.output/'runs.jsonl').open('a') as f:f.write(json.dumps(result,allow_nan=False)+'\n')
        if not result['passed']:print('FAILED',result,flush=True)
        elif (i+1)%10==0:print(f'{i+1}/{args.runs} passed',flush=True)
    summary={'runs':args.runs,'passed':sum(r['passed'] for r in results),'seed':args.seed,
        'scope':'synthetic images, kinematics, actual Python boat controller/bridge on ROS API shim; not DDS/SITL/hardware',
        'results':results}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(f"{summary['passed']}/{args.runs} passed; {args.output/'summary.json'}")
    return 0 if summary['passed']==args.runs else 1


if __name__=='__main__':raise SystemExit(main())
