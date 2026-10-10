"""UUV trail/depth mission, subordinate to existing mission enable and safety."""
import json
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robotx_coordination.following import TrailFollower
from robotx_coordination.ros_support import Received, now_sec, odometry_pose, seconds


class UuvFollower(Node):
    def __init__(self):
        super().__init__('uuv_usv_follower')
        p = lambda n,d:self.declare_parameter(n,d).value
        self.configured = bool(p('datum_configured',False))
        self.core = TrailFollower(distance=float(p('follow_distance',3.)),depth=float(p('follow_depth',1.5)),
            max_depth=float(p('max_depth',5.)),max_xy=float(p('max_xy_speed',0.4)),max_z=float(p('max_z_speed',0.2)),
            frame=p('map_frame','robotx_map'),max_variance=float(p('max_pose_variance',1.)))
        self.own = Received(); self.state = Received()
        self.authorized = False; self.auth_rx = None; self.enabled = False; self.enable_rx = None
        self.pub = self.create_publisher(TwistStamped,p('output_topic','/uuv/follow/cmd_vel'),10)
        self.status = self.create_publisher(String,'/uuv/follow/status',10)
        self.create_subscription(Odometry,p('usv_pose_topic','/boat/coordination/odometry'),self.on_boat,qos_profile_sensor_data)
        self.create_subscription(Odometry,p('uuv_pose_topic','/uuv/coordination/odometry'),lambda m:self.own.update(m,seconds(m.header.stamp)),qos_profile_sensor_data)
        self.create_subscription(State,'/uuv/mavros/state',lambda m:self.state.update(m,seconds(m.header.stamp)),10)
        self.create_subscription(Bool,'/uuv/safety/authorized',self.on_auth,10)
        self.create_subscription(String,'/uuv/mission/state',self.on_mission,10)
        self.create_service(Trigger,'/uuv/follow/reset',self.reset)
        self.timer = self.create_timer(0.05,self.tick)

    def on_boat(self,msg):
        self.core.observe(odometry_pose(msg),now_sec(self))

    def on_auth(self,msg):
        self.authorized = msg.data; self.auth_rx = time.monotonic()

    def on_mission(self,msg):
        try:
            data = json.loads(msg.data)
            self.enabled = data.get('autonomy_enabled') is True
            self.enable_rx = time.monotonic()
        except (ValueError,AttributeError):
            self.enabled = False

    def reset(self,request,response):
        if self.enabled:
            response.success = False; response.message = 'Disable UUV mission before resetting trail'
        else:
            self.core.reset(); response.success = True; response.message = 'Trail reset'
        return response

    def tick(self):
        now, wall = now_sec(self), time.monotonic()
        state_ok = self.state.valid(now,0.5) and self.state.message.connected and self.state.message.armed and self.state.message.mode == 'GUIDED'
        auth = self.authorized and self.auth_rx is not None and wall-self.auth_rx < 0.35 and state_ok
        enabled = self.configured and self.enabled and self.enable_rx is not None and wall-self.enable_rx < 0.5
        own = odometry_pose(self.own.message) if self.own.valid(now,0.5) else None
        cmd, reason, target = self.core.command(own,now,enabled,auth)
        # LOCAL_NED MAVROS plugin consumes ROS ENU world velocity regardless of header.
        out = TwistStamped(); out.header.stamp = self.get_clock().now().to_msg(); out.header.frame_id = self.core.frame
        out.twist.linear.x,out.twist.linear.y,out.twist.linear.z,out.twist.angular.z = cmd
        self.pub.publish(out)
        report = String(); report.data = json.dumps({'follow_state':reason,'target':target,'velocity_enu':cmd})
        self.status.publish(report)


def main(args=None):
    rclpy.init(args=args); node = UuvFollower()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.pub.publish(TwistStamped());node.destroy_node();rclpy.shutdown()
