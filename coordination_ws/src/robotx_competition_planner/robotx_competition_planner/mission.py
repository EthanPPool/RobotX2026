"""Attempt-scoped semantics and measured-progress Safe Passage execution."""
import math

from .models import angle_difference
from .routing import Course
from .trajectory import orbit, polyline


class SafePassage:
    PHASES = ('APPROACH_ENTRY', 'CIRCLE_ENTRY', 'TRANSIT', 'CIRCLE_EXIT')

    def __init__(self, config, boundary, tier='core', expected_buoys=10):
        config.validate()
        if tier not in ('core', 'advanced') or expected_buoys < 2:
            raise ValueError('Invalid tier or expected buoy count')
        self.config, self.boundary = config, boundary
        self.tier, self.expected_buoys = tier, expected_buoys
        self.state, self.reason = 'IDLE', 'NOT_STARTED'
        self.course = self.plan = self.trajectory = None
        self.phase_index, self.elapsed, self.passed_mask = 0, 0., 0
        self.circle_progress = 0.
        self.last_pose = self.last_now = self.last_angle = None
        self.run_id = ''

    def start(self, markers, pose, now, run_id, survey_reported=False, semantic_age=5.):
        pose.validate()
        if self.state in self.PHASES:
            raise ValueError('Attempt already active; stop/reset before starting a new attempt')
        if len(markers) != self.expected_buoys:
            raise ValueError('WAITING_FOR_COMPLETE_BUOY_SURVEY')
        if len({m.track_id for m in markers}) != len(markers):
            raise ValueError('Duplicate survey track IDs')
        for m in markers:
            m.validate()
            recognized = m.role != 'obstacle' or (m.color == 'black' and m.light_state == 'inactive')
            if not recognized or m.confidence < self.config.min_confidence:
                raise ValueError('WAITING_FOR_CONFIRMED_BEACON_STATES')
            if not 0 <= now-m.semantic_stamp <= semantic_age:
                raise ValueError('SURVEY_SEMANTICS_STALE')
            if self.tier != 'core' and m.color_source != 'uav':
                raise ValueError('WAITING_FOR_UAV_CLASSIFIED_SURVEY')
        if self.tier != 'core' and not survey_reported:
            raise ValueError('WAITING_FOR_SURVEY_REPORT_ACKNOWLEDGEMENT')
        course = Course(markers, self.boundary, self.config)
        plan = course.plan(pose)
        # Commit the snapshot only after complete validation and successful planning.
        self.course, self.plan, self.run_id = course, plan, run_id
        self.phase_index, self.elapsed, self.passed_mask = 0, 0., 0
        self.last_pose, self.last_now = pose, now
        self._phase(pose.yaw)

    def _phase(self, initial_yaw):
        self.state = self.PHASES[self.phase_index]
        self.elapsed, self.circle_progress, self.last_angle = 0., 0., None
        if self.state == 'APPROACH_ENTRY':
            self.trajectory = polyline(self.plan.approach, initial_yaw, self.config)
        elif self.state == 'TRANSIT':
            self.trajectory = polyline(self.plan.transit, initial_yaw, self.config)
        else:
            arc = self.plan.entry_orbit if self.state == 'CIRCLE_ENTRY' else self.plan.exit_orbit
            self.trajectory = orbit(arc, initial_yaw, self.config)
        self.reason = 'TRACKING'

    def stop(self, reason='OPERATOR_STOP'):
        self.state, self.reason = 'STOPPED', reason

    def preview(self):
        if self.trajectory is None:
            return ()
        return self.trajectory.preview(self.elapsed, self.config.preview_dt)

    def update(self, pose, now, permitted=True):
        pose.validate()
        if self.state not in self.PHASES:
            return False
        dt = now-self.last_now
        if not math.isfinite(dt) or dt < 0 or dt > 1.:
            self.reason = 'CLOCK_OR_UPDATE_GAP'
            self.last_now = now
            self.last_pose = pose
            return False
        previous = self.last_pose
        self.last_now, self.last_pose = now, pose
        if pose.stamp < previous.stamp:
            self.reason = 'POSE_TIME_REWIND'
            return False
        if math.dist(previous.point, pose.point) > self.config.speed*max(dt, 0.01)*3+0.5:
            self.reason = 'POSE_JUMP'
            return False
        if not permitted:
            self.reason = 'WAITING_FOR_FRESH_LOCAL_CLEARANCE'
            return False
        reference = self.trajectory.sample(self.elapsed)
        error = math.dist(pose.point, (reference.x, reference.y))
        heading_error = abs(angle_difference(pose.yaw, reference.yaw))
        if error > self.config.tracking_hard_limit:
            self.reason = 'TRACKING_ERROR_REPLAN_REQUIRED'
            return False
        if self.state.startswith('CIRCLE'):
            arc = self.plan.entry_orbit if self.state == 'CIRCLE_ENTRY' else self.plan.exit_orbit
            angle = math.atan2(pose.y-arc.center[1], pose.x-arc.center[0])
            if abs(math.dist(pose.point, arc.center)-arc.radius) <= self.config.tracking_soft_limit:
                if self.last_angle is not None:
                    delta = angle_difference(angle, self.last_angle)
                    if abs(delta) <= self.config.yaw_rate*max(dt, .01)*3+.05:
                        self.circle_progress = max(0., self.circle_progress+arc.direction*delta)
                self.last_angle = angle
            else:
                self.last_angle = None
        elif self.state == 'TRANSIT':
            legal, bits = self.course.passages(previous.point, pose.point, pose.yaw)
            if not legal:
                self.stop('WRONG_SIDE_PASSAGE')
                return False
            self.passed_mask |= bits
        # Freeze the moving reference if the real boat falls behind; keep its
        # current nearby reference valid so feedback can recover tracking error.
        if error <= self.config.tracking_soft_limit and heading_error <= .6:
            self.elapsed = min(self.trajectory.duration, self.elapsed+dt)
            self.reason = 'TRACKING'
        else:
            self.reason = 'REFERENCE_PAUSED_FOR_TRACKING'
        endpoint = self.trajectory.end
        reached = (math.dist(pose.point, (endpoint.x, endpoint.y)) <= self.config.arrival_tolerance
                   and abs(angle_difference(pose.yaw, endpoint.yaw)) <= self.config.heading_tolerance)
        finished = self.elapsed >= self.trajectory.duration and reached
        if self.state.startswith('CIRCLE'):
            finished = finished and self.circle_progress >= 2*math.pi-0.05
        if self.state == 'TRANSIT':
            finished = finished and self.passed_mask == self.course.full_mask
            if self.elapsed >= self.trajectory.duration and reached and not finished:
                self.reason = 'MEASURED_PASSAGES_INCOMPLETE'
        if finished:
            self.phase_index += 1
            if self.phase_index == len(self.PHASES):
                self.state, self.reason = 'COMPLETE', 'MEASURED_MISSION_COMPLETE'
                return False
            self._phase(endpoint.yaw)
        return True
