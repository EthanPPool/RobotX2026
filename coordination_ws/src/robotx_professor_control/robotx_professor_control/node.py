"""ROS controller: three path references + causal measured state -> Ft3/Ft4.

Default shadow configuration deliberately records unconfirmed model assumptions.
It never publishes cmd_vel, voltage, PWM, arming, or FCU mode requests.
"""
import json
import math
import uuid
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robotx_coordination.geometry import rotation,reference_id
from robotx_coordination_interfaces.msg import TrajectoryPreview
from robotx_control_interfaces.msg import ThrusterForces
from .law import ForceIntegrator,rates
from .tracking import Binding,Motion,MotionHistory,PoseHistory
from .protocol import Preview,fresh


def seconds(stamp):return stamp.sec+stamp.nanosec/1e9


class ProfessorController(Node):
    def __init__(self):
        super().__init__('professor_controller')
        p=lambda n,d:self.declare_parameter(n,d).value
        self.frame=str(p('map_frame','robotx_map'))
        self.datum_configured=bool(p('datum_configured',False))
        self.reference_id=reference_id(float(p('origin_latitude',0.)),float(p('origin_longitude',0.)),float(p('water_altitude',0.)))
        self.confirmed=all([bool(p(k,False)) for k in ('units_confirmed_si','axis_binding_confirmed','nominal_trim_confirmed')])
        self.pose_timeout=float(p('pose_timeout',.15));self.reference_timeout=float(p('reference_timeout',.15))
        self.sync_tolerance=float(p('capture_sync_tolerance',.05))
        self.max_variance=float(p('max_pose_variance',.25))
        self.max_acceleration=float(p('max_measured_acceleration',2.))
        self.max_yaw_acceleration=float(p('max_measured_yaw_acceleration',2.))
        self.period=float(p('control_dt',.05))
        limits=(self.pose_timeout,self.reference_timeout,self.max_variance,self.max_acceleration,self.max_yaw_acceleration,self.period)
        if not all(math.isfinite(v) and v>0 for v in limits) or not 0<=self.sync_tolerance<=.1 or self.period>.2:
            raise ValueError('INVALID_CONTROLLER_TIMING_OR_STATE_LIMITS')
        self.binding=Binding(*(float(p(k,d)) for k,d in (('cross_track_lookahead_m',5.),('max_along_error_m',1.),('max_cross_error_m',1.),('max_yaw_error_rad',.35))))
        self.integrator=ForceIntegrator(*(float(p(k,d)) for k,d in (('max_force_n',10.),('max_force_rate_nps',20.),('nominal_right_force_n',0.),('nominal_left_force_n',0.))))
        source=p('velocity_source','pose_history')
        if source not in ('pose_history','odometry_twist'):raise ValueError('INVALID_VELOCITY_SOURCE')
        self.use_twist=source=='odometry_twist'
        self.body=str(p('body_frame','boat/base_link'))
        tau=float(p('derivative_filter_tau',.25))
        self.history=MotionHistory(tau,.2) if self.use_twist else PoseHistory(.2,tau)
        self.motion=self.preview=None;self.run_id='';self.last_tick=None;self.last_integrated_pose=0.
        self.sequence=0;self.controller_id=uuid.uuid4().hex
        self.error='WAITING_FOR_STATE_AND_REFERENCE';self.last_reference_stamp=0.
        self.publisher=self.create_publisher(ThrusterForces,'/control/thruster_forces',10)
        self.diagnostics=self.create_publisher(String,'/control/professor_status',10)
        self.create_subscription(Odometry,p('odometry_topic','/usv/coordination/odometry'),self.on_pose,qos_profile_sensor_data)
        self.create_subscription(TrajectoryPreview,'/mission/trajectory_preview',self.on_reference,10)
        self.create_timer(self.period,self.tick)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def on_pose(self,msg):
        try:
            t=seconds(msg.header.stamp)
            if msg.header.frame_id!=self.frame or not fresh(t,self.now(),self.pose_timeout):raise ValueError('POSE_FRAME_OR_TIME_INVALID')
            cov=[msg.pose.covariance[i] for i in (0,7,35)]
            if not all(math.isfinite(v) and 0<=v<=self.max_variance for v in cov):raise ValueError('POSE_UNCERTAIN')
            q=msg.pose.pose.orientation;matrix=rotation((q.x,q.y,q.z,q.w))
            if matrix[2,2]<math.cos(.2):raise ValueError('PLANAR_ATTITUDE_LIMIT')
            point=msg.pose.pose.position;yaw=math.atan2(matrix[1,0],matrix[0,0])
            vx=vy=w=0.
            if self.use_twist:
                if msg.child_frame_id!=self.body:raise ValueError('TWIST_BODY_FRAME_MISMATCH')
                c,s=math.cos(yaw),math.sin(yaw);v=msg.twist.twist
                vx,vy,w=c*v.linear.x-s*v.linear.y,s*v.linear.x+c*v.linear.y,v.angular.z
            value=self.history.update(Motion(t,point.x,point.y,yaw,vx,vy,w))
            self.motion=value
            if value is None:self.error='WAITING_FOR_CAUSAL_DERIVATIVE_HISTORY'
        except (ValueError,TypeError,OverflowError) as exc:
            self.motion=None;self.history.reset();self.error=str(exc)

    def on_reference(self,msg):
        try:
            t=seconds(msg.header.stamp)
            if t<=self.last_reference_stamp:raise ValueError('REFERENCE_TIME_REWIND')
            samples=tuple(Motion(t+s.offset_seconds,s.x,s.y,s.yaw,s.vx,s.vy,s.yaw_rate,s.ax,s.ay,s.yaw_acceleration) for s in msg.samples)
            preview=Preview(t,msg.header.frame_id,msg.reference_id,msg.run_id,msg.phase,msg.valid,msg.dt,samples)
            preview.validate(self.now(),self.frame,self.reference_id,self.reference_timeout)
            self.preview=preview;self.last_reference_stamp=t
        except (ValueError,TypeError,OverflowError) as exc:
            self.preview=None;self.error=str(exc);self.integrator.reset()

    def tick(self):
        now=self.now();out=ThrusterForces();self.sequence+=1
        out.header.stamp=self.get_clock().now().to_msg();out.header.frame_id=self.frame
        out.valid_until=copy.deepcopy(out.header.stamp)
        out.reference_id=self.reference_id;out.run_id=self.preview.run_id if self.preview else self.run_id
        out.controller_id=self.controller_id
        out.sequence=self.sequence;out.model_contract_confirmed=self.confirmed;out.valid=False
        out.right_force_n=out.left_force_n=0.;details={}
        try:
            if not self.datum_configured:raise ValueError('DATUM_NOT_CONFIGURED')
            if self.motion is None or self.preview is None:raise ValueError(self.error)
            self.preview.validate(now,self.frame,self.reference_id,self.reference_timeout)
            if not fresh(self.motion.stamp,now,self.pose_timeout):raise ValueError('POSE_STALE')
            if max(abs(self.motion.ax),abs(self.motion.ay))>self.max_acceleration or abs(self.motion.yaw_acceleration)>self.max_yaw_acceleration:
                raise ValueError('MEASURED_ACCELERATION_LIMIT')
            if self.run_id!=self.preview.run_id:
                self.integrator.reset();self.run_id=self.preview.run_id;self.last_tick=None
            reference=self.preview.at_measurement(self.motion.stamp,self.sync_tolerance)
            axes,details=self.binding.axes(self.motion,reference)
            rate=rates(*axes)
            dt=self.period if self.last_tick is None else now-self.last_tick
            forces,limited=self.integrator.step(rate,dt)
            out.right_force_n,out.left_force_n=forces;out.valid=True
            expiry=min(now+2*self.period,self.preview.stamp+self.reference_timeout,
                       self.motion.stamp+self.pose_timeout,self.preview.samples[-1].stamp)
            ticks=math.floor(expiry*1e9)
            out.valid_until.sec=ticks//1000000000;out.valid_until.nanosec=ticks%1000000000
            out.reason='LIMITED' if limited else 'TRACKING' if self.confirmed else 'SHADOW_MODEL_CONTRACT_UNCONFIRMED'
            details.update(right_rate=rate[0],left_rate=rate[1],limited=limited,derivative_mode='causal_continuous')
        except (ValueError,TypeError,OverflowError) as exc:
            self.integrator.reset();out.reason=str(exc)
        self.last_tick=now
        self.publisher.publish(out)
        status=String();status.data=json.dumps(dict(valid=out.valid,reason=out.reason,model_contract_confirmed=self.confirmed,
            right_force_n=out.right_force_n,left_force_n=out.left_force_n,run_id=out.run_id,**details),allow_nan=False)
        self.diagnostics.publish(status)


def main(args=None):
    rclpy.init(args=args);node=ProfessorController()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.preview=None;node.error='SHUTDOWN';node.tick();node.destroy_node();rclpy.shutdown()
