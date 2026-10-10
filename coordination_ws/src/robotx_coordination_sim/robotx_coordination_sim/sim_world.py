"""Kinematic ROS fixture; all interfaces are under /sim and no MAVLink is opened."""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from boat_interfaces.msg import DetectedObject, DetectedObjectArray
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from robotx_coordination.geometry import rotation, yaw_quaternion
from robotx_coordination.ros_support import point, seconds


class SimWorld(Node):
    def __init__(self):
        super().__init__('coordination_sim_world')
        self.bridge = CvBridge(); self.boat = np.array([0.,0.]); self.angle = 0.
        self.uuv = np.array([-3.,0.,-1.5]); self.uuv_angle = 0.
        self.boat_cmd = None; self.boat_rx = None; self.uuv_cmd = None; self.uuv_rx = None
        self.boat_mode = 'HOLD'; self.complete = False; self.stage = 0; self.pending = None; self.last_step = time.monotonic()
        self.guided_since = None
        self.obstacles = [(8.,1.,'red'),(12.,-1.5,'green'),(16.,0.,'yellow'),(22.,2.,'red'),(26.,-2.,'green')]
        self.camera_position = (15.,0.,30.); self.camera_q = (math.sqrt(0.5),-math.sqrt(0.5),0.,0.)
        self.k = np.array([[240.,0.,320.],[0.,240.,240.],[0.,0.,1.]])
        self.odom_boat = self.create_publisher(Odometry,'/sim/boat/coordination/odometry',10)
        self.odom_uuv = self.create_publisher(Odometry,'/sim/uuv/coordination/odometry',10)
        self.state_boat = self.create_publisher(State,'/sim/mavros/state',10)
        self.state_uuv = self.create_publisher(State,'/sim/uuv/mavros/state',10)
        self.auth_pub = self.create_publisher(Bool,'/sim/uuv/safety/authorized',10)
        self.enable_pub = self.create_publisher(String,'/sim/uuv/mission/state',10)
        self.local_pub = self.create_publisher(DetectedObjectArray,'/sim/perception/objects',10)
        self.image_pub = self.create_publisher(Image,'/sim/uav/downward/image_raw',2)
        self.info_pub = self.create_publisher(CameraInfo,'/sim/uav/downward/camera_info',2)
        self.report_pub = self.create_publisher(String,'/sim/test/status',10)
        self.tf = TransformBroadcaster(self)
        self.drop = {name:False for name in ('camera','usv_pose','uuv_pose','local_perception','uuv_safety')}
        for name in self.drop:
            self.create_service(SetBool,'/sim/test/drop_'+name,lambda req,res,key=name:self.set_drop(key,req,res))
        self.create_subscription(TwistStamped,'/sim/mavros/setpoint_velocity/cmd_vel',self.on_boat_cmd,10)
        self.create_subscription(TwistStamped,'/sim/uuv/mavros/setpoint_velocity/cmd_vel',self.on_uuv_cmd,10)
        self.create_subscription(String,'/sim/mission/state',self.on_mission,10)
        self.create_service(SetMode,'/sim/mavros/set_mode',self.set_mode)
        self.clients = [self.create_client(SetBool,name) for name in
                        ['/sim/boat/route/set_enabled','/sim/vehicle/software_estop','/sim/vehicle/set_autonomy']]
        self.started = time.monotonic(); self.timer = self.create_timer(0.05,self.tick)
        self.camera_timer = self.create_timer(0.5,self.capture)

    def on_boat_cmd(self,msg):
        self.boat_cmd = msg; self.boat_rx = time.monotonic()

    def on_uuv_cmd(self,msg):
        self.uuv_cmd = msg; self.uuv_rx = time.monotonic()

    def on_mission(self,msg):
        if msg.data.startswith('MISSION_COMPLETE'):
            self.complete = True

    def set_mode(self,req,res):
        self.boat_mode = req.custom_mode; res.mode_sent = True; return res

    def set_drop(self,key,req,res):
        self.drop[key]=req.data;res.success=True;res.message=key+(' dropped' if req.data else ' restored');return res

    def publish_odom(self,pub,xyz,angle,body):
        msg = Odometry(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'robotx_map'
        msg.child_frame_id = body; msg.pose.pose.position = point(xyz)
        q = yaw_quaternion(angle)
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w = q
        pub.publish(msg)

    def bootstrap(self):
        if time.monotonic()-self.started < 3. or self.complete or self.stage >= len(self.clients):
            return
        if self.pending is not None:
            if not self.pending.done():
                return
            result = self.pending.result()
            if result is None or not result.success:
                self.get_logger().error('Simulation startup service rejected'); self.stage = 99;return
            self.pending = None;self.stage += 1
            return
        if self.clients[self.stage].service_is_ready():
            if self.stage == 2:
                if self.boat_mode != 'GUIDED':
                    self.boat_mode = 'GUIDED';self.guided_since = time.monotonic();return
                # Give DDS subscribers fresh GUIDED state before requesting
                # autonomy; changing this fixture's field is not an FCU ACK.
                if self.guided_since is None or time.monotonic()-self.guided_since<0.25:
                    return
            req = SetBool.Request(); req.data = self.stage != 1
            self.pending = self.clients[self.stage].call_async(req)

    def capture(self):
        if self.drop['camera']:
            return
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped(); tf.header.stamp = stamp;tf.header.frame_id = 'robotx_map';tf.child_frame_id = 'uav/downward_optical'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = self.camera_position
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = self.camera_q
        self.tf.sendTransform(tf)
        image = np.zeros((480,640,3),dtype=np.uint8); image[:] = (80,45,20)
        colors = {'red':(0,0,255),'green':(0,255,0),'yellow':(0,255,255)}
        rot = rotation(self.camera_q)
        for x,y,color in self.obstacles:
            optical = rot.T @ (np.array([x,y,0.])-self.camera_position)
            uv = self.k @ optical;u,v = uv[:2]/uv[2]
            cv2.circle(image,(round(u),round(v)),4,colors[color],-1)
        info = CameraInfo();info.header.stamp = stamp;info.header.frame_id = tf.child_frame_id
        info.width=640;info.height=480;info.k=list(self.k.ravel());info.d=[0.]*5;info.distortion_model='plumb_bob'
        self.info_pub.publish(info)
        msg=self.bridge.cv2_to_imgmsg(image,'bgr8');msg.header=info.header;self.image_pub.publish(msg)

    def tick(self):
        now=time.monotonic();dt=min(0.1,now-self.last_step);self.last_step=now
        if self.boat_cmd is not None and self.boat_rx is not None and now-self.boat_rx<0.3 and self.boat_mode=='GUIDED':
            v,w=self.boat_cmd.twist.linear.x,self.boat_cmd.twist.angular.z
            self.angle+=w*dt;self.boat+=np.array([math.cos(self.angle),math.sin(self.angle)])*v*dt
        if self.uuv_cmd is not None and self.uuv_rx is not None and now-self.uuv_rx<0.3:
            c=self.uuv_cmd.twist;self.uuv+=np.array([c.linear.x,c.linear.y,c.linear.z])*dt;self.uuv_angle+=c.angular.z*dt
        if not self.drop['usv_pose']:
            self.publish_odom(self.odom_boat,(*self.boat,0.),self.angle,'boat/base_link')
        if not self.drop['uuv_pose']:
            self.publish_odom(self.odom_uuv,self.uuv,self.uuv_angle,'uuv/base_link')
        for pub,mode in [(self.state_boat,self.boat_mode),(self.state_uuv,'GUIDED')]:
            state=State();state.header.stamp=self.get_clock().now().to_msg();state.connected=True;state.armed=True;state.mode=mode;pub.publish(state)
        if not self.drop['uuv_safety']:
            auth=Bool();auth.data=not self.complete;self.auth_pub.publish(auth)
        enabled=String();enabled.data=json.dumps({'autonomy_enabled':not self.complete});self.enable_pub.publish(enabled)
        objects=DetectedObjectArray();objects.header.stamp=self.get_clock().now().to_msg();objects.header.frame_id='base_link'
        c,s=math.cos(self.angle),math.sin(self.angle)
        for x,y,color in self.obstacles:
            dx,dy=x-self.boat[0],y-self.boat[1];bx,by=c*dx+s*dy,-s*dx+c*dy
            if 0<bx<8 and abs(by)<5:
                o=DetectedObject();o.position=point((bx,by,0.));o.size.x=0.5;o.size.y=0.5;o.confidence=0.95;objects.objects.append(o)
        if not self.drop['local_perception']:
            self.local_pub.publish(objects)
        self.bootstrap()
        report=String();report.data=json.dumps({'bootstrap_stage':self.stage,'boat':self.boat.tolist(),'uuv':self.uuv.tolist(),
                'complete':self.complete,'mode':self.boat_mode,'software_fixture':True});self.report_pub.publish(report)


def main(args=None):
    rclpy.init(args=args);node=SimWorld()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
