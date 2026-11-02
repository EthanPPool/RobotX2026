"""ROS transport for the incremental engine. Publishes references, never motors."""
from dataclasses import replace
import json
import math
import uuid

from . import node as transport
from .incremental import IncrementalPassage
from .models import Marker,Pose
from .trajectory import Sample
from .survey import Survey


class IncrementalPlanner(transport.CompetitionPlanner):
    def __init__(self):
        super().__init__()
        self.planning_margin=float(self.declare_parameter('extra_planning_margin',.5).value)
        self.observation_distance=float(self.declare_parameter('observation_distance',8.).value)
        if not 0<=self.planning_margin<=2 or not 2<=self.observation_distance<=20:
            raise ValueError('Invalid incremental planning margins')
        self.mission=self.make_mission();self.last_engine_event=0
        from robotx_coordination_interfaces.msg import BuoyObservationArray
        self.create_subscription(BuoyObservationArray,'/perception/buoys/uav',self.on_aerial,10)

    def make_mission(self):
        config=replace(self.config,clearance=self.config.clearance+self.planning_margin)
        return IncrementalPassage(config,self.boundary,self.tier,observation_distance=self.observation_distance,
            physical_clearance=self.config.clearance)

    def on_map(self,msg):
        previous=self.latest_map
        super().on_map(msg)
        if self.latest_map is msg and self.latest_map is not previous:
            try:self.mission.observe(self.markers(),tuple(int(b.track_id) for b in msg.tracks if b.semantic_conflict))
            except ValueError:self.external_reason='MAP_GEOMETRY_REJECTED'

    def on_aerial(self,msg):
        now=self.now_seconds()
        if msg.header.frame_id!=self.frame or msg.reference_id!=self.ref:return
        if not self.fresh(transport.seconds(msg.header.stamp),now,self.map_timeout):return
        markers=[]
        for i,o in enumerate(msg.observations[:100]):
            if o.source!='uav' or o.uncertainty>2. or not self.fresh(transport.seconds(o.observed_at),now,self.map_timeout):continue
            try:
                m=Marker(i,o.position.x,o.position.y,o.radius,o.uncertainty,o.color,o.light_state,
                    o.confidence,transport.seconds(o.observed_at),'uav');m.validate();markers.append(m)
            except ValueError:continue
        self.mission.observe_aerial(markers)

    def start(self,request,response):
        now=self.now_seconds()
        try:
            if not self.enabled:raise ValueError('DATUM_NOT_CONFIGURED')
            if self.pose is None or not self.fresh(self.pose.stamp,now,self.pose_timeout):raise ValueError('WAITING_FOR_SHARED_FRAME_POSE')
            if self.latest_map is None or not self.fresh(transport.seconds(self.latest_map.header.stamp),now,self.map_timeout):
                raise ValueError('WAITING_FOR_FRESH_BUOY_MAP')
            self.mission.observe(self.markers(),tuple(int(b.track_id) for b in self.latest_map.tracks if b.semantic_conflict))
            if self.tier=='advanced':
                self.mission.entry_reported=bool(0<=now-self.report_ack_stamp<=self.semantic_age and
                    self.report_signature and all(k in self.mission.belief and self.mission.belief[k].role==role
                        for k,role in self.report_signature))
            self.mission.start(self.pose,now,uuid.uuid4().hex)
            self.publish_route()
            response.success,response.message=True,'Incremental mission active; observations may be incomplete'
        except ValueError as exc:response.success,response.message=False,str(exc)
        return response

    def reported(self,request,response):
        now=self.now_seconds()
        if not request.data:
            self.mission.entry_reported=False
            self.report_ack_stamp=0.;self.report_signature=None
            response.success,response.message=True,'Report acknowledgement cleared'
        elif self.latest_map is None or not self.fresh(transport.seconds(self.latest_map.header.stamp),now,self.map_timeout):
            response.success,response.message=False,'WAITING_FOR_FRESH_BUOY_MAP'
        elif self.mission.endpoint('entry') is None or self.mission.endpoint('exit') is None:
            response.success,response.message=False,'WAITING_FOR_IDENTIFIED_ENTRY_AND_EXIT'
        else:
            self.mission.acknowledge_report()
            self.report_ack_stamp=now
            self.report_signature=tuple((m.track_id,m.role) for m in self.mission.belief.values() if m.color!='unknown')
            response.success,response.message=True,'External successful-report acknowledgement recorded'
        return response

    def reset(self,request,response):
        self.mission=self.make_mission();self.last_engine_event=0
        self.survey=Survey(self.now_seconds(),self.semantic_age,self.tier=='advanced')
        self.external_reason='NOT_STARTED';self.tick()
        response.success,response.message=True,'Incremental map and progress reset'
        return response

    def publish_route(self):
        out=transport.Path();out.header.frame_id=self.frame;out.header.stamp=self.get_clock().now().to_msg()
        for line in self.mission.route_lines():
            for x,y in line['points']:
                p=transport.PoseStamped();p.header=out.header
                p.pose.position.x,p.pose.position.y,p.pose.orientation.w=float(x),float(y),1.
                out.poses.append(p)
        self.route_pub.publish(out)

    def clearance_samples(self):
        if not self.mission.trajectory:
            p=self.pose or Pose(0.,0.,0.,max(self.now_seconds(),1e-9))
            return (Sample(p.x,p.y,p.yaw),)
        horizon=2*self.config.preview_dt+self.config.speed/self.config.acceleration+self.scan_timeout
        count=math.ceil(horizon/self.config.preview_dt)
        return tuple(self.mission.trajectory.sample(self.mission.elapsed+i*self.config.preview_dt) for i in range(count+1))

    def tick(self):
        now=self.now_seconds();active=self.mission.state in self.mission.PHASES
        pose_ok=self.pose is not None and self.fresh(self.pose.stamp,now,self.pose_timeout)
        map_ok=self.latest_map is not None and self.fresh(transport.seconds(self.latest_map.header.stamp),now,self.map_timeout)
        valid=False;reason=self.mission.reason
        if active and pose_ok and map_ok:self.mission.prepare(self.pose,now)
        local=bool(self.scan and self.scan.preview_clear(self.clearance_samples(),
            self.config.hull_radius+self.config.clearance,now,self.scan_timeout))
        if active and pose_ok:
            self.mission.update(self.pose,now,map_ok and local)
            valid=map_ok and local and self.mission.state in self.mission.PHASES
        if active:reason='POSE_STALE' if not pose_ok else 'BUOY_MAP_STALE' if not map_ok else 'LOCAL_CLEARANCE_UNKNOWN_OR_BLOCKED' if not local else self.mission.reason
        samples=self.mission.preview()
        if not samples:
            p=self.pose or Pose(0.,0.,0.,max(now,1e-9))
            samples=tuple(Sample(p.x,p.y,p.yaw,offset=i*self.config.preview_dt) for i in range(3))
        if valid and self.pose:
            previous=self.pose
            # Side/unknown guards cover the braking horizon as well as t+2dt.
            for s in self.clearance_samples():
                proposed=Pose(s.x,s.y,s.yaw,now)
                if not self.mission.motion_legal(previous,proposed):
                    valid=False;reason='LOCAL_REFERENCE_REPLAN_REQUIRED'
                    self.mission.trajectory=None;self.mission.task=None;break
                previous=proposed
        out=transport.TrajectoryPreview()
        out.header.stamp,out.header.frame_id=self.get_clock().now().to_msg(),self.frame
        out.reference_id,out.run_id,out.phase=self.ref,self.mission.run_id,self.mission.state
        out.valid,out.reason,out.dt=bool(valid),reason,self.config.preview_dt
        for s in samples:
            item=transport.TrajectorySample();item.offset_seconds=s.offset
            for name in ('x','y','yaw','vx','vy','ax','ay','yaw_rate','yaw_acceleration'):setattr(item,name,float(getattr(s,name)))
            out.samples.append(item)
        self.preview_pub.publish(out)
        status=transport.String()
        status.data=json.dumps(dict(state=self.mission.state,reason=reason,reference_valid=bool(valid),
            run_id=self.mission.run_id,planning_mode='incremental',mapped=len(self.mission.belief),
            classified=sum(m.color!='unknown' for m in self.mission.belief.values()),
            entry_complete=self.mission.entry_done,replans=self.mission.replans,tier=self.tier,
            passed_mask=self.mission.passed_mask,passed_ids=sorted(self.mission.passed_ids),
            circle_progress=self.mission.circle_progress),allow_nan=False)
        self.status_pub.publish(status)
        for e in self.mission.history:
            if e['seq']>self.last_engine_event:
                self.emit(e['event'],revision=e['revision']);self.last_engine_event=e['seq']
        if active:self.publish_route()


def main(args=None):
    transport.rclpy.init(args=args);node=IncrementalPlanner()
    try:transport.rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.mission.stop('SHUTDOWN');node.tick();node.destroy_node();transport.rclpy.shutdown()
