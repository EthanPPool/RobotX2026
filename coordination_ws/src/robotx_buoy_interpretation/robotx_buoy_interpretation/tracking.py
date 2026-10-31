"""Bounded shared map with optional UAV evidence and acquisition-time aging.

    A retained obstacle is not proof of currently observed water. This map never
    declares free space. Semantic freshness is independent of map publication.
"""
from dataclasses import dataclass, field, replace
import math

from .contracts import Observation
from .beacon import BeaconHistory


@dataclass
class Track:
    track_id: int
    evidence: dict = field(default_factory=dict)
    semantics: dict = field(default_factory=dict)
    beacons: dict = field(default_factory=dict)


class BuoyTracker:
    def __init__(self, frame='robotx_map', reference_id='', source_mode='usv_only',
                 merge_distance=0.7, usv_timeout=1.5, uav_timeout=5.0,
                 retention=120.0, min_confidence=0.6, max_uncertainty=2.0,
                 max_tracks=300):
        if source_mode not in ('usv_only', 'assisted'):
            raise ValueError('source_mode must be usv_only or assisted')
        values = (merge_distance, usv_timeout, uav_timeout, retention, max_uncertainty)
        if (not frame or not reference_id or not all(math.isfinite(v) and v > 0
                for v in values) or not math.isfinite(min_confidence)
                or not 0 <= min_confidence <= 1 or max_tracks < 1
                or retention < max(usv_timeout, uav_timeout)):
            raise ValueError('Invalid tracker configuration')
        self.frame, self.reference_id, self.source_mode = frame, reference_id, source_mode
        self.merge_distance, self.retention = merge_distance, retention
        self.timeouts = {'usv': usv_timeout, 'uav': uav_timeout}
        self.min_confidence, self.max_uncertainty = min_confidence, max_uncertainty
        self.max_tracks, self.next_id = max_tracks, 1
        self.tracks, self.last_clock = [], None

    def _age(self, now):
        if not math.isfinite(now):
            raise ValueError('Invalid current time')
        if self.last_clock is not None and now < self.last_clock:
            # Bag/simulation clock rewind invalidates temporal association.
            self.tracks.clear()
        self.last_clock = now
        for t in self.tracks:
            t.evidence = {s: o for s, o in t.evidence.items()
                          if 0 <= now - o.stamp <= self.retention}
            t.semantics = {s: o for s, o in t.semantics.items()
                           if 0 <= now - o.stamp <= self.retention}
        self.tracks = [t for t in self.tracks if t.evidence]

    def _position(self, track, now):
        # Prefer the most precise observation; ties prefer the newest.
        active = [o for o in track.evidence.values()
                  if 0 <= now-o.stamp <= self.timeouts[o.source]]
        return min(active or list(track.evidence.values()),
                   key=lambda o: (o.uncertainty, -o.stamp))

    def ingest(self, observations, frame, reference_id, now):
        self._age(now)
        if frame != self.frame or reference_id != self.reference_id:
            return {'accepted': 0, 'rejected': len(observations), 'reason': 'REFERENCE_MISMATCH'}
        accepted, rejected, used_tracks, used_observations = 0, 0, set(), set()
        for o in observations:
            try:
                o.validate()
            except (ValueError, TypeError):
                rejected += 1
                continue
            if (o.source == 'uav' and self.source_mode == 'usv_only') or not (
                    0 <= now - o.stamp <= self.timeouts[o.source]) or o.uncertainty > self.max_uncertainty:
                rejected += 1
                continue
            key = (o.source, o.observation_id)
            if key in used_observations:
                rejected += 1
                continue
            used_observations.add(key)
            # Image detection IDs need only be unique within a frame. They are
            # not persistent buoy identities, and cannot cause a track to jump.
            nearby = [t for t in self.tracks if math.hypot(
                self._position(t, now).x-o.x, self._position(t, now).y-o.y) <= self.merge_distance]
            if len(nearby) > 1:
                # LiDAR already tracks candidate IDs: use an active, distance-
                # gated identity hint to maintain two close local obstacles.
                # Per-image UAV IDs cannot resolve a spatial ambiguity.
                hints = [t for t in nearby if o.source == 'usv' and 'usv' in t.evidence
                         and t.evidence['usv'].observation_id == o.observation_id
                         and 0 <= now-t.evidence['usv'].stamp <= self.timeouts['usv']]
                if len(hints) != 1:
                    rejected += 1
                    continue
                track = hints[0]
            else:
                track = nearby[0] if nearby else None
            if track is not None and track.track_id in used_tracks:
                # Two distinct candidates in one image/scan must remain two
                # obstacles even inside the cross-source association radius.
                track = None
            if track is None:
                if len(self.tracks) >= self.max_tracks:
                    rejected += 1
                    continue
                track = Track(self.next_id)
                self.next_id += 1
                self.tracks.append(track)
            if o.light_state == 'unknown' and o.confidence >= self.min_confidence:
                history = track.beacons.setdefault(o.source, BeaconHistory())
                interpreted = history.update(o.color, o.stamp, o.confidence)
                if interpreted is not None:
                    o = replace(o, color=interpreted[0], light_state=interpreted[1],
                                confidence=interpreted[2])
            previous = track.evidence.get(o.source)
            if previous is not None:
                if o.stamp < previous.stamp:
                    old_semantic = track.semantics.get(o.source)
                    if (o.color == 'unknown' or o.confidence < self.min_confidence
                            or (old_semantic is not None and o.stamp <= old_semantic.stamp)):
                        rejected += 1
                        continue
                    # Inference for an earlier image may arrive after newer
                    # geometry. Enrich semantics without rewinding position.
                    track.semantics[o.source] = o
                    used_tracks.add(track.track_id)
                    accepted += 1
                    continue
                if o.stamp == previous.stamp:
                    # Delayed model inference may enrich geometry at its original
                    # time. Replays and contradictory same-time changes cannot age it.
                    enriching = (previous.color == 'unknown' and o.color != 'unknown'
                                 and o.confidence >= self.min_confidence)
                    if not enriching:
                        rejected += 1
                        continue
            track.evidence[o.source] = o
            if o.color != 'unknown' and o.confidence >= self.min_confidence:
                track.semantics[o.source] = o
            used_tracks.add(track.track_id)
            accepted += 1
        return {'accepted': accepted, 'rejected': rejected, 'reason': 'UPDATED'}

    def snapshot(self, now):
        self._age(now)
        result = []
        for t in self.tracks:
            active = [o for o in t.evidence.values()
                      if 0 <= now-o.stamp <= self.timeouts[o.source]]
            # Current local geometry wins over a more precise but aged survey.
            p = min(active or list(t.evidence.values()),
                    key=lambda o: (o.uncertainty, -o.stamp))
            semantic = [o for o in t.semantics.values()
                        if 0 <= now-o.stamp <= self.timeouts[o.source]]
            colors = {o.color for o in semantic}
            light_states = {o.light_state for o in semantic if o.light_state != 'unknown'}
            conflict = len(colors) > 1 or len(light_states) > 1
            s = max(semantic, key=lambda o: o.stamp) if semantic else None
            color = s.color if s is not None and not conflict else 'unknown'
            light_state = (next(iter(light_states)) if len(light_states) == 1 and not conflict
                           else 'unknown')
            # Enclose position disagreement rather than averaging a local obstacle away.
            radius = max(o.radius + math.hypot(o.x-p.x, o.y-p.y) + o.uncertainty
                         for o in t.evidence.values())
            result.append(dict(track_id=t.track_id, x=p.x, y=p.y, z=p.z,
                uncertainty=p.uncertainty, radius=max(p.radius, radius-p.uncertainty),
                color=color, confidence=s.confidence if color != 'unknown' else 0.,
                light_state=light_state, position_stamp=p.stamp,
                color_stamp=s.stamp if s is not None else 0.,
                sources=sorted(t.evidence), active_sources=sorted(o.source for o in active),
                actionable=bool(active and color != 'unknown'), semantic_conflict=conflict,
                color_source=s.source if color != 'unknown' else '', position_source=p.source))
        return result
