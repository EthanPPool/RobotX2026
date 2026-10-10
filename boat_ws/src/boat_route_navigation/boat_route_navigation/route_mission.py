"""Survey map + onboard obstacle observations -> existing NavigationTarget path."""
import json
import math
import time
from collections import deque

import rclpy
from boat_interfaces.msg import DetectedObjectArray, NavigationTarget
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from robotx_coordination.geometry import Footprint, Obstacle, finite, fresh, reference_id, yaw
from robotx_coordination.planning import SurveyPlanner, route_target
from robotx_coordination.ros_support import Received, now_sec, odometry_pose, point, seconds
from robotx_coordination_interfaces.msg import BuoyMap


class RouteMission(Node):
    def __init__(self):
        super().__init__('survey_route_mission')
        p = lambda n,d:self.declare_parameter(n,d).value
        self.frame = p('map_frame','robotx_map')
        self.origin = (p('origin_latitude',0.),p('origin_longitude',0.),p('water_altitude',0.))
        self.ref = reference_id(*self.origin)
        self.configured = bool(p('datum_configured',False))
        self.bounds = list(p('bounds',[-10.,-20.,60.,20.]))
        self.resolution = float(p('resolution',0.5)); self.clearance = float(p('clearance',1.))
        self.goal = (float(p('goal_x',40.)),float(p('goal_y',0.)))
        self.map_timeout = float(p('map_timeout',3.)); self.survey_ttl = float(p('survey_ttl',60.))
        self.local_timeout = float(p('local_sensor_timeout',0.5))
        self.speed = float(p('desired_speed',0.12)); self.replan_period = float(p('replan_period',1.))
        if not finite(*self.goal,self.map_timeout,self.local_timeout,self.speed,self.replan_period,self.survey_ttl) or min(self.map_timeout,self.local_timeout,self.replan_period,self.survey_ttl) <= 0 or not 0 < self.speed <= 0.15:
            raise ValueError('Invalid mission limits')
        SurveyPlanner([],[],self.bounds,self.resolution,self.clearance)
        self.enabled = False; self.complete = False
        self.map = Received(); self.pose = Received(); self.local = Received()
        self.pose_history = deque(maxlen=100)
        self.path = []; self.last_plan = -1e9; self.last_map_stamp = None
        self.target_pub = self.create_publisher(NavigationTarget,'/mission/target',10)
        self.state_pub = self.create_publisher(String,'/mission/state',10)
        self.report_pub = self.create_publisher(String,'/boat/route/status',10)
        self.path_pub = self.create_publisher(Path,'/boat/route/path',10)
        self.create_subscription(BuoyMap,'/uav/mapping/buoys',self.on_map,10)
        self.create_subscription(Odometry,'/boat/coordination/odometry',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(DetectedObjectArray,p('objects_topic','/perception/objects'),lambda m:self.local.update(m,seconds(m.header.stamp)),qos_profile_sensor_data)
        self.create_service(SetBool,'/boat/route/set_enabled',self.set_enabled)
        self.create_service(Trigger,'/boat/route/reset',self.reset)
        self.timer = self.create_timer(0.05,self.tick)

    def on_map(self,msg):
        try:
            datum = reference_id(msg.origin_latitude,msg.origin_longitude,msg.water_altitude)
        except ValueError:
            return
        if (msg.header.frame_id != self.frame or msg.reference_id != self.ref or datum != self.ref):
            return
        self.map.update(msg,seconds(msg.header.stamp))

    def on_pose(self,msg):
        if self.pose.update(msg,seconds(msg.header.stamp)):
            self.pose_history.append(odometry_pose(msg))

    def set_enabled(self,req,res):
        if req.data and self.complete:
            res.success = False; res.message = 'Reset completed route before enabling'
        else:
            self.enabled = req.data; res.success = True; res.message = 'Route enabled' if req.data else 'Route disabled'
        return res

    def reset(self,req,res):
        self.enabled = False; self.complete = False; self.path = []
        res.success = True; res.message = 'Route reset; enable explicitly'; return res

    def publish(self,reason,x=0.,y=0.,speed=0.):
        target = NavigationTarget(); target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'base_link'; target.target = point((x,y,0.))
        target.stop = reason != 'ROUTE_TRACKING'; target.desired_speed = speed
        self.target_pub.publish(target)
        state = String(); state.data = reason; self.state_pub.publish(state)
        report = String(); report.data = json.dumps({'route_state':reason,'waypoints':len(self.path),'reference_id':self.ref})
        self.report_pub.publish(report)

    def tick(self):
        now = now_sec(self)
        if self.complete:
            self.publish('MISSION_COMPLETE'); return
        if not self.enabled or not self.configured:
            self.publish('ROUTE_DISABLED'); return
        if not self.pose.valid(now,0.5):
            self.publish('USV_LOCALIZATION_STALE'); return
        pose = odometry_pose(self.pose.message)
        if not pose.valid(self.frame,now,0.5,1.):
            self.publish('USV_LOCALIZATION_INVALID'); return
        if not self.map.valid(now,self.map_timeout):
            self.publish('UAV_MAP_STALE'); return
        if not self.local.valid(now,self.local_timeout) or self.local.message.header.frame_id != 'base_link':
            self.publish('LOCAL_PERCEPTION_STALE'); return
        mapped = self.map.message
        # Bounded hostile/malformed input handling; unknown or invalid data stops.
        if len(mapped.buoys)>500 or len(mapped.footprints)>300:
            self.publish('MAP_TOO_LARGE'); return
        footprints = [Footprint(tuple((p.x,p.y) for p in f.corners),seconds(f.observed_at)) for f in mapped.footprints
                      if fresh(seconds(f.observed_at),now,self.survey_ttl)]
        obstacles = [Obstacle(b.position.x,b.position.y,b.radius,b.uncertainty,b.color,seconds(b.observed_at)) for b in mapped.buoys]
        observation_pose = min(self.pose_history,key=lambda p:abs(p.stamp-self.local.stamp))
        if abs(observation_pose.stamp-self.local.stamp)>0.15:
            self.publish('LOCAL_POSE_TIME_MISMATCH'); return
        angle = yaw(observation_pose.quaternion); c,s = math.cos(angle),math.sin(angle)
        for obj in self.local.message.objects:
            p, size = obj.position,obj.size
            if not finite(p.x,p.y,size.x,size.y,obj.confidence) or min(size.x,size.y)<0:
                self.publish('LOCAL_OBSERVATION_INVALID'); return
            # All reported obstacles matter, regardless of color or selected gate.
            x = observation_pose.position[0]+c*p.x-s*p.y
            y = observation_pose.position[1]+s*p.x+c*p.y
            obstacles.append(Obstacle(x,y,max(0.2,size.x/2,size.y/2),0.25))
        try:
            planner = SurveyPlanner(footprints,obstacles,self.bounds,self.resolution,self.clearance)
            if not self.path or time.monotonic()-self.last_plan >= self.replan_period:
                self.path = planner.plan(pose.position[:2],self.goal)
                self.last_plan = time.monotonic()
                path = Path(); path.header.stamp = self.get_clock().now().to_msg(); path.header.frame_id = self.frame
                for x,y in self.path:
                    p = PoseStamped(); p.header = path.header; p.pose.position = point((x,y,0.));p.pose.orientation.w = 1.
                    path.poses.append(p)
                self.path_pub.publish(path)
            x,y,speed,reason = route_target(pose.position[:2],yaw(pose.quaternion),self.path,planner,self.speed)
        except (ValueError,OverflowError):
            self.publish('INVALID_MAP'); return
        if reason == 'MISSION_COMPLETE':
            self.complete = True; self.enabled = False
        self.publish(reason,x,y,speed)


def main(args=None):
    rclpy.init(args=args);node = RouteMission()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.publish('ROUTE_SHUTDOWN');node.destroy_node();rclpy.shutdown()
