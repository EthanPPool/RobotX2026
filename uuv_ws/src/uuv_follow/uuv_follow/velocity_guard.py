"""Stamped, fail-closed world-velocity guard before the existing UUV bridge."""
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Bool

from robotx_coordination.ros_support import Received, now_sec, seconds


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('uuv_follow_velocity_guard')
        p=lambda n,d:self.declare_parameter(n,d).value
        self.frame = p('map_frame','robotx_map')
        self.require_frame_check = bool(p('require_frame_check',True))
        self.frame_verified = False;self.frame_rx = None;self.pending = None
        self.parameter_client = AsyncParameterClient(self,p('mavros_velocity_node','/uuv/mavros/setpoint_velocity'))
        self.cmd = Received(); self.state = Received(); self.authorized = False; self.auth_rx = None
        self.pub = self.create_publisher(TwistStamped,p('output_topic','/uuv/control/cmd_vel'),10)
        self.create_subscription(TwistStamped,'/uuv/follow/cmd_vel',lambda m:self.cmd.update(m,seconds(m.header.stamp)),10)
        self.create_subscription(State,'/uuv/mavros/state',lambda m:self.state.update(m,seconds(m.header.stamp)),10)
        self.create_subscription(Bool,'/uuv/safety/authorized',self.on_auth,10)
        self.timer=self.create_timer(0.05,self.tick)
        self.create_timer(1.,self.verify_frame)

    def verify_frame(self):
        if not self.require_frame_check:
            return
        if self.pending is not None:
            if not self.pending.done():
                if self.frame_rx is not None and time.monotonic()-self.frame_rx>3.:
                    self.frame_verified=False
                return
            try:
                result=self.pending.result()
                self.frame_verified=bool(result and len(result.values)==1 and result.values[0].string_value=='LOCAL_NED')
                self.frame_rx=time.monotonic()
            except Exception:
                self.frame_verified=False
            self.pending=None
        if self.parameter_client.services_are_ready():
            self.pending=self.parameter_client.get_parameters(['mav_frame'])

    def on_auth(self,msg):
        self.authorized=msg.data;self.auth_rx=time.monotonic()

    def tick(self):
        now=now_sec(self);out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg();out.header.frame_id=self.frame
        state_ok=self.state.valid(now,0.5) and self.state.message.connected and self.state.message.armed and self.state.message.mode=='GUIDED'
        frame_ok=not self.require_frame_check or (self.frame_verified and self.frame_rx is not None and time.monotonic()-self.frame_rx<3.)
        if frame_ok and self.authorized and self.auth_rx is not None and time.monotonic()-self.auth_rx<0.35 and state_ok and self.cmd.valid(now,0.25):
            m=self.cmd.message
            values=[m.twist.linear.x,m.twist.linear.y,m.twist.linear.z,m.twist.angular.z]
            if m.header.frame_id==self.frame and all(math.isfinite(v) for v in values):
                norm=math.hypot(values[0],values[1]);scale=min(1.,0.4/max(norm,1e-9))
                out.twist.linear.x=values[0]*scale;out.twist.linear.y=values[1]*scale
                out.twist.linear.z=max(-0.2,min(0.2,values[2]));out.twist.angular.z=max(-0.25,min(0.25,values[3]))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args);node=VelocityGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.pub.publish(TwistStamped());node.destroy_node();rclpy.shutdown()
