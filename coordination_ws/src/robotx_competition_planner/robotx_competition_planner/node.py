"""Jetson competition planner. Publishes references, never actuator commands."""
from dataclasses import replace
import json
import math
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from robotx_coordination.geometry import reference_id, rotation, yaw_quaternion
from robotx_coordination_interfaces.msg import BuoyTrackArray, TrajectoryPreview, TrajectorySample

from .clearance import Scan
from .mission import SafePassage
from .models import Marker, PlannerConfig, Pose
from .routing import Course
from .trajectory import Sample
from .survey import Survey


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


class CompetitionPlanner(Node):
    def __init__(self):
        super().__init__('robotx_competition_planner')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = str(p('map_frame', 'robotx_map'))
        self.enabled = bool(p('datum_configured', False))
        self.ref = reference_id(float(p('origin_latitude', 0.)),
            float(p('origin_longitude', 0.)), float(p('water_altitude', 0.)))
        kwargs = {name: p(name, field.default) for name, field in PlannerConfig.__dataclass_fields__.items()}
        self.config = PlannerConfig(**kwargs)
        self.config.validate()
        flat = list(p('course_boundary_xy', [0.0]))
        self.boundary = tuple(zip(flat[::2], flat[1::2])) if len(flat) % 2 == 0 else ()
        self.tier = str(p('tier', 'core'))
        self.mission = SafePassage(self.config, self.boundary, self.tier,
                                  int(p('expected_buoys', 10)))
        self.pose_timeout = float(p('pose_timeout', .5))
        self.map_timeout = float(p('map_timeout', .5))
        self.scan_timeout = float(p('scan_timeout', .5))
        self.semantic_age = float(p('survey_semantic_age', 90.))
        self.max_pose_variance = float(p('max_pose_variance', 1.))
        if not all(math.isfinite(x) and x > 0 for x in (self.pose_timeout, self.map_timeout,
                self.scan_timeout, self.semantic_age, self.max_pose_variance)):
            raise ValueError('Invalid freshness or pose variance limits')
        self.latest_map = self.pose = self.scan = None
        self.survey = Survey(self.now_seconds(), self.semantic_age, self.tier == 'advanced',
                             self.config.min_confidence)
        self.report_ack_stamp = 0.
        self.report_map_stamp = 0.
        self.report_signature = None
        self.external_reason = 'NOT_STARTED'
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, self)
        self.preview_pub = self.create_publisher(TrajectoryPreview, '/mission/trajectory_preview', 10)
        self.route_pub = self.create_publisher(Path, '/mission/task1/path', 10)
        self.status_pub = self.create_publisher(String, '/mission/task1/status', 10)
        self.events_pub = self.create_publisher(String, '/mission/task1/events', 10)
        self.create_subscription(BuoyTrackArray, '/mission/buoys', self.on_map, 10)
        self.create_subscription(Odometry, p('odometry_topic', '/usv/coordination/odometry'),
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(LaserScan, p('scan_topic', '/scan'), self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/mission/task1/start', self.start)
        self.create_service(Trigger, '/mission/task1/stop', self.stop)
        self.create_service(Trigger, '/mission/task1/reset', self.reset)
        self.create_service(SetBool, '/mission/task1/survey_reported', self.reported)
        self.create_timer(.05, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def fresh(stamp, now, timeout):
        return math.isfinite(stamp) and stamp > 0 and 0 <= now-stamp <= timeout

    def on_map(self, msg):
        if msg.header.frame_id != self.frame or msg.reference_id != self.ref:
            self.external_reason = 'MAP_REFERENCE_MISMATCH'
            return
        stamp = seconds(msg.header.stamp)
        if not self.fresh(stamp, self.now_seconds(), self.map_timeout):
            return
        if self.latest_map is not None and stamp <= seconds(self.latest_map.header.stamp):
            return
        self.latest_map = msg
        try:
            self.survey.update(self.markers(), self.now_seconds(),
                tuple(int(b.track_id) for b in msg.tracks if b.semantic_conflict))
        except ValueError:
            self.external_reason = 'SURVEY_GEOMETRY_REJECTED'

    def on_pose(self, msg):
        now, stamp = self.now_seconds(), seconds(msg.header.stamp)
        if msg.header.frame_id != self.frame or not self.fresh(stamp, now, self.pose_timeout):
            return
        if self.pose is not None and stamp <= self.pose.stamp:
            return
        try:
            covariance = [float(msg.pose.covariance[i]) for i in (0, 7)]
            if not all(math.isfinite(v) and 0 <= v <= self.max_pose_variance for v in covariance):
                raise ValueError('Uncertain pose')
            q = msg.pose.pose.orientation
            matrix = rotation((q.x, q.y, q.z, q.w))
            point = msg.pose.pose.position
            pose = Pose(point.x, point.y, math.atan2(matrix[1, 0], matrix[0, 0]), stamp)
            pose.validate()
            self.pose = pose
        except ValueError:
            self.external_reason = 'POSE_REJECTED'

    def on_scan(self, msg):
        stamp, now = seconds(msg.header.stamp), self.now_seconds()
        if not self.fresh(stamp, now, self.scan_timeout):
            return
        if self.scan is not None and stamp <= self.scan.stamp:
            return
        try:
            transform = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                                      Time.from_msg(msg.header.stamp))
            q, t = transform.transform.rotation, transform.transform.translation
            matrix = rotation((q.x, q.y, q.z, q.w))
            if matrix[2, 2] < math.cos(.15):
                raise ValueError('Planar scan orientation unsupported')
            scan = Scan(tuple(msg.ranges), msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, t.x, t.y,
                        math.atan2(matrix[1, 0], matrix[0, 0]), stamp)
            scan.validate()
            self.scan = scan
        except (TransformException, ValueError):
            self.external_reason = 'WAITING_FOR_CAPTURE_TIME_SCAN_TF'

    def markers(self):
        if self.latest_map is None:
            return ()
        result = []
        for b in self.latest_map.tracks:
            color = b.color if not b.semantic_conflict else 'unknown'
            result.append(Marker(int(b.track_id), b.position.x, b.position.y,
                b.radius, b.uncertainty, color, b.light_state, b.confidence,
                seconds(b.color_observed_at), b.color_source))
        return tuple(result)

    def emit(self, event, **data):
        msg = String()
        msg.data = json.dumps(dict(event=event, run_id=self.mission.run_id, **data), allow_nan=False)
        self.events_pub.publish(msg)

    def reported(self, request, response):
        # This signal must come from the OCS/RoboCommand integration after its
        # actual successful report, never from elapsed time or UAV presence.
        if request.data and self.latest_map is not None:
            self.report_ack_stamp = self.now_seconds()
            self.report_map_stamp = seconds(self.latest_map.header.stamp)
            self.report_signature = self.survey_signature(self.survey.snapshot(self.now_seconds()))
            response.success, response.message = True, 'Survey-report acknowledgement recorded'
        else:
            self.report_ack_stamp = self.report_map_stamp = 0.
            self.report_signature = None
            response.success, response.message = True, 'Survey-report acknowledgement cleared'
        return response

    @staticmethod
    def survey_signature(markers):
        return tuple(sorted((m.track_id, m.color, m.light_state, m.color_source) for m in markers))

    def start(self, request, response):
        now = self.now_seconds()
        try:
            if not self.enabled:
                raise ValueError('DATUM_NOT_CONFIGURED')
            if self.mission.state == 'COMPLETE':
                raise ValueError('RESET_REQUIRED_FOR_NEW_ATTEMPT')
            if self.pose is None or not self.fresh(self.pose.stamp, now, self.pose_timeout):
                raise ValueError('WAITING_FOR_SHARED_FRAME_POSE')
            if self.latest_map is None or not self.fresh(seconds(self.latest_map.header.stamp), now, self.map_timeout):
                raise ValueError('WAITING_FOR_FRESH_BUOY_MAP')
            # A recent report acknowledgement belongs to this survey, not to an
            # earlier attempt. It is consumed on successful start/reset/stop.
            reported = (0 <= now-self.report_ack_stamp <= self.semantic_age
                        and self.report_map_stamp > 0
                        and self.report_signature == self.survey_signature(self.survey.snapshot(now)))
            self.mission.start(self.survey.snapshot(now), self.pose, now, uuid.uuid4().hex,
                               reported, self.semantic_age)
            self.report_ack_stamp = self.report_map_stamp = 0.
            self.report_signature = None
            self.external_reason = 'TRACKING'
            self.publish_route()
            self.emit('ATTEMPT_PLANNED', tier=self.tier,
                      markers=[dict(id=m.track_id, x=m.x, y=m.y, color=m.color,
                                    light_state=m.light_state) for m in self.mission.course.markers])
            response.success, response.message = True, 'Route and timed references prepared'
        except ValueError as exc:
            self.external_reason = str(exc)
            response.success, response.message = False, str(exc)
        return response

    def stop(self, request, response):
        self.mission.stop()
        self.report_ack_stamp = self.report_map_stamp = 0.
        self.report_signature = None
        self.survey = Survey(self.now_seconds(), self.semantic_age, self.tier == 'advanced',
                             self.config.min_confidence)
        self.tick()
        response.success, response.message = True, 'Planner reference invalidated'
        return response

    def reset(self, request, response):
        self.mission = SafePassage(self.config, self.boundary, self.tier, self.mission.expected_buoys)
        self.report_ack_stamp = self.report_map_stamp = 0.
        self.report_signature = None
        self.survey = Survey(self.now_seconds(), self.semantic_age, self.tier == 'advanced',
                             self.config.min_confidence)
        self.external_reason = 'NOT_STARTED'
        self.tick()
        response.success, response.message = True, 'Attempt snapshot and progress cleared'
        return response

    def current_course_clear(self):
        if self.mission.course is None:
            return False
        current = {m.track_id: m for m in self.markers()}
        updated = []
        for old in self.mission.course.markers:
            new = current.pop(old.track_id, None)
            if new is not None:
                relevant = self.tier == 'core' or new.color_source == 'uav'
                if relevant and new.confidence >= self.config.min_confidence and new.role not in ('obstacle', old.role):
                    self.external_reason = 'SURVEY_CHANGED_NEW_ATTEMPT_REQUIRED'
                    return False
                updated.append(replace(old, x=new.x, y=new.y,
                    radius=new.radius, uncertainty=new.uncertainty))
            else:
                updated.append(old)
        # Newly detected geometry is an obstacle regardless of its unknown color.
        updated.extend(replace(m, color='unknown', light_state='unknown') for m in current.values())
        try:
            course = Course(updated, self.boundary, self.config)
            # Validate the nominal path against refined/new LiDAR geometry.
            phase = self.mission.phase_index
            paths = ([self.mission.plan.approach] if phase == 0 else [])
            if phase <= 2:
                paths.append(self.mission.plan.transit)
            if any(not course.segment_clear(a, b) for path in paths for a, b in zip(path, path[1:])):
                self.external_reason = 'ROUTE_BLOCKED_REPLAN_REQUIRED'
                return False
            arcs = ([self.mission.plan.entry_orbit] if phase <= 1 else [])+[self.mission.plan.exit_orbit]
            for arc in arcs:
                if any(abs(math.dist(arc.center, m.point)-arc.radius) <= course.inflated(m)
                       for m in course.markers):
                    self.external_reason = 'ORBIT_BLOCKED_REPLAN_REQUIRED'
                    return False
            self.mission.course = course
            return True
        except ValueError:
            self.external_reason = 'MAP_GEOMETRY_REJECTED'
            return False

    def publish_route(self):
        out = Path()
        out.header.frame_id, out.header.stamp = self.frame, self.get_clock().now().to_msg()
        plan = self.mission.plan
        points = list(plan.approach)
        for arc, path in ((plan.entry_orbit, plan.transit), (plan.exit_orbit, ())):
            points.extend((arc.center[0]+arc.radius*math.cos(arc.start_angle+arc.direction*2*math.pi*i/100),
                           arc.center[1]+arc.radius*math.sin(arc.start_angle+arc.direction*2*math.pi*i/100))
                          for i in range(101))
            points.extend(path)
        for x, y in points:
            p = PoseStamped()
            p.header = out.header
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = float(x), float(y), 1.
            out.poses.append(p)
        self.route_pub.publish(out)

    def tick(self):
        now = self.now_seconds()
        active = self.mission.state in self.mission.PHASES
        valid, reason = False, self.external_reason
        if active:
            pose_ok = self.pose is not None and self.fresh(self.pose.stamp, now, self.pose_timeout)
            map_ok = self.latest_map is not None and self.fresh(seconds(self.latest_map.header.stamp), now, self.map_timeout)
            clear = map_ok and self.current_course_clear()
            samples = self.mission.preview()
            local = self.scan is not None and self.scan.preview_clear(self.clearance_samples(),
                self.config.hull_radius+self.config.clearance, now, self.scan_timeout)
            if pose_ok:
                previous_state = self.mission.state
                valid = self.mission.update(self.pose, now, pose_ok and clear and local)
                if self.mission.state != previous_state:
                    self.emit('PHASE_CHANGED', phase=self.mission.state)
                samples = self.mission.preview()
                valid = valid and self.scan is not None and self.scan.preview_clear(self.clearance_samples(),
                    self.config.hull_radius+self.config.clearance, now, self.scan_timeout)
            reason = ('POSE_STALE' if not pose_ok else 'BUOY_MAP_STALE' if not map_ok
                      else self.external_reason if not clear else
                      'LOCAL_CLEARANCE_UNKNOWN_OR_BLOCKED' if not local else self.mission.reason)
        else:
            samples = self.mission.preview()
            reason = self.mission.reason if self.mission.state != 'IDLE' else self.external_reason
        if not samples:
            point = self.pose or Pose(0., 0., 0., max(now, 1e-9))
            samples = tuple(Sample(point.x, point.y, point.yaw, offset=i*self.config.preview_dt) for i in range(3))
        out = TrajectoryPreview()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), self.frame
        out.reference_id, out.run_id, out.phase = self.ref, self.mission.run_id, self.mission.state
        out.valid, out.reason, out.dt = bool(valid), reason, self.config.preview_dt
        for s in samples:
            item = TrajectorySample()
            item.offset_seconds = s.offset
            for name in ('x', 'y', 'yaw', 'vx', 'vy', 'ax', 'ay', 'yaw_rate', 'yaw_acceleration'):
                setattr(item, name, float(getattr(s, name)))
            out.samples.append(item)
        self.preview_pub.publish(out)
        status = String()
        status.data = json.dumps(dict(state=self.mission.state, reason=reason,
            reference_valid=bool(valid), run_id=self.mission.run_id, tier=self.tier,
            passed_mask=self.mission.passed_mask, circle_progress=self.mission.circle_progress), allow_nan=False)
        self.status_pub.publish(status)

    def clearance_samples(self):
        # Check farther than the two-step controller preview: include a full
        # nominal braking interval and scan latency before publishing a reference.
        horizon = (2*self.config.preview_dt+self.config.speed/self.config.acceleration
                   +self.scan_timeout)
        count = math.ceil(horizon/self.config.preview_dt)
        return tuple(self.mission.trajectory.sample(self.mission.elapsed+i*self.config.preview_dt)
                     for i in range(count+1))


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mission.stop('SHUTDOWN')
        node.tick()
        node.destroy_node()
        rclpy.shutdown()
