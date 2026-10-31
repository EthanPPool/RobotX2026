"""Unified mission input. UAV absence never gates the USV-only stream."""
import json

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from robotx_coordination.geometry import reference_id
from robotx_coordination_interfaces.msg import BuoyObservationArray, BuoyTrack, BuoyTrackArray

from .contracts import Observation
from .ros_support import stamp_seconds
from .tracking import BuoyTracker


class BuoyTrackerNode(Node):
    def __init__(self):
        super().__init__('buoy_tracker')
        p = lambda key, default: self.declare_parameter(key, default).value
        self.enabled = bool(p('datum_configured', False))
        self.frame = str(p('map_frame', 'robotx_map'))
        self.reference = reference_id(float(p('origin_latitude', 0.)),
            float(p('origin_longitude', 0.)), float(p('water_altitude', 0.)))
        self.mode = str(p('source_mode', 'usv_only'))
        self.tracker = BuoyTracker(self.frame, self.reference, self.mode,
            merge_distance=float(p('merge_distance', 0.7)),
            usv_timeout=float(p('usv_timeout', 1.5)),
            uav_timeout=float(p('uav_timeout', 5.)),
            retention=float(p('obstacle_retention', 120.)),
            min_confidence=float(p('min_confidence', 0.6)),
            max_uncertainty=float(p('max_uncertainty', 2.)),
            max_tracks=int(p('max_tracks', 300)))
        self.pub = self.create_publisher(BuoyTrackArray, '/mission/buoys', 10)
        self.status_pub = self.create_publisher(String, '/mission/buoy_interpretation/status', 10)
        self.create_subscription(BuoyObservationArray, '/perception/buoys/usv',
            lambda msg: self.receive(msg, 'usv'), 10)
        if self.mode == 'assisted':
            self.create_subscription(BuoyObservationArray, '/perception/buoys/uav',
                lambda msg: self.receive(msg, 'uav'), 10)
        self.last_update = {'reason': 'WAITING_FOR_OBSERVATIONS'}
        self.create_timer(0.1, self.publish)

    def receive(self, msg, expected_source):
        if not self.enabled:
            return
        observations = []
        for b in msg.observations:
            if b.source != expected_source:
                self.last_update = {'reason': 'SOURCE_MISMATCH'}
                return
            observations.append(Observation(b.observation_id, b.source,
                stamp_seconds(b.observed_at), b.position.x, b.position.y, b.position.z,
                b.color, b.confidence, b.uncertainty, b.radius, b.light_state))
        self.last_update = self.tracker.ingest(observations, msg.header.frame_id,
            msg.reference_id, self.get_clock().now().nanoseconds / 1e9)

    def publish(self):
        now = self.get_clock().now()
        rows = self.tracker.snapshot(now.nanoseconds / 1e9) if self.enabled else []
        out = BuoyTrackArray()
        out.header.stamp, out.header.frame_id = now.to_msg(), self.frame
        out.reference_id, out.source_mode = self.reference, self.mode
        for r in rows:
            b = BuoyTrack()
            b.track_id = r['track_id']
            b.position.x, b.position.y, b.position.z = r['x'], r['y'], r['z']
            for key in ('uncertainty', 'radius', 'color', 'confidence', 'light_state',
                        'sources', 'active_sources', 'actionable', 'semantic_conflict'):
                setattr(b, key, r[key])
            b.position_observed_at = Time(seconds=r['position_stamp']).to_msg()
            b.color_observed_at = Time(seconds=r['color_stamp']).to_msg()
            out.tracks.append(b)
        self.pub.publish(out)
        state = 'DATUM_NOT_CONFIGURED' if not self.enabled else (
            'UAV_ASSISTED' if any('uav' in r['active_sources'] for r in rows)
            else 'USV_ONLY' if any('usv' in r['active_sources'] for r in rows)
            else 'NO_FRESH_OBSERVATIONS')
        status = String()
        status.data = json.dumps(dict(state=state, source_mode=self.mode,
            track_count=len(rows), actionable_count=sum(r['actionable'] for r in rows),
            last_update=self.last_update), allow_nan=False)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
