"""Associate rectified color detections with acquisition-time LiDAR geometry.

Camera coordinates use ROS optical right/down/forward. Map coordinates are ENU.
Unmatched geometry is retained as unknown obstacles; ambiguous color is withheld.
"""
from dataclasses import dataclass
import math

from .contracts import Observation


@dataclass(frozen=True)
class GeometryCandidate:
    candidate_id: int
    map_position: tuple
    optical_position: tuple | None = None
    radius: float = 0.3
    uncertainty: float = 0.25
    confidence: float = 1.0

    def validate(self):
        if (len(self.map_position) != 3 or
                not all(math.isfinite(v) for v in self.map_position) or
                not all(math.isfinite(v) for v in
                    (self.radius, self.uncertainty, self.confidence)) or
                self.radius <= 0 or self.uncertainty < 0 or
                not 0 <= self.confidence <= 1):
            raise ValueError('Invalid geometric buoy candidate')
        if self.optical_position is not None and (len(self.optical_position) != 3
                or not all(math.isfinite(v) for v in self.optical_position)):
            raise ValueError('Invalid optical candidate position')


def _fresh(stamp, now, timeout):
    return (all(math.isfinite(v) for v in (stamp, now, timeout)) and stamp > 0
            and timeout > 0 and 0 <= now - stamp <= timeout)


def _validate_candidates(candidates):
    seen = set()
    for candidate in candidates:
        candidate.validate()
        if candidate.candidate_id in seen:
            raise ValueError('Duplicate geometry candidate ID')
        seen.add(candidate.candidate_id)


def _observation(candidate, stamp, detection=None):
    return Observation(
        str(candidate.candidate_id), 'usv', stamp, *candidate.map_position,
        detection.color if detection is not None else 'unknown',
        min(candidate.confidence, detection.confidence) if detection is not None
            else candidate.confidence,
        candidate.uncertainty, candidate.radius,
        detection.light_state if detection is not None else 'unknown')


def geometry_observations(candidates, geometry_stamp, now, timeout=0.5):
    """Publish geometry even when camera/model is absent; never invent color."""
    if not _fresh(geometry_stamp, now, timeout):
        raise ValueError('Stale or future geometry capture')
    _validate_candidates(candidates)
    return [_observation(candidate, geometry_stamp) for candidate in candidates]


def interpret_usv(detections, candidates, camera, image_stamp, geometry_stamp,
                  now, timeout=0.5, max_sync=0.1, min_confidence=0.6,
                  max_pixel_distance=80.0, bbox_padding=8.0):
    """Return one observation per geometry candidate with unambiguous colors.

    Both coordinate representations must already be transformed at their capture
    times by the caller. Every accepted edge must be unique on both sides of the
    association graph: two projected buoys within one box or overlapping color
    boxes produce unknown assignments. This is intentionally conservative.
    """
    camera.validate()
    if not all(math.isfinite(v) for v in
            (max_sync, min_confidence, max_pixel_distance, bbox_padding)) or (
            max_sync < 0 or not 0 <= min_confidence <= 1 or
            max_pixel_distance <= 0 or bbox_padding < 0):
        raise ValueError('Invalid association configuration')
    if not _fresh(image_stamp, now, timeout) or not _fresh(geometry_stamp, now, timeout):
        raise ValueError('Stale or future capture')
    if abs(image_stamp - geometry_stamp) > max_sync + 1e-9:
        raise ValueError('Unsynchronized camera and geometry')
    _validate_candidates(candidates)
    seen = set()
    for detection in detections:
        detection.validate(camera)
        if detection.detection_id in seen:
            raise ValueError('Duplicate image detection ID')
        seen.add(detection.detection_id)
    edges = {i: [] for i in range(len(candidates))}
    reverse_edges = {j: [] for j in range(len(detections))}
    for i, candidate in enumerate(candidates):
        if candidate.optical_position is None:
            continue
        x, y, z = candidate.optical_position
        if z <= 1e-6:
            continue
        u, v = camera.fx*x/z + camera.cx, camera.fy*y/z + camera.cy
        if not 0 <= u < camera.width or not 0 <= v < camera.height:
            continue
        for j, detection in enumerate(detections):
            if detection.confidence < min_confidence or detection.color == 'unknown':
                continue
            if (abs(u-detection.u) <= detection.width/2 + bbox_padding and
                    abs(v-detection.v) <= detection.height/2 + bbox_padding and
                    math.hypot(u-detection.u, v-detection.v) <= max_pixel_distance):
                edges[i].append(j)
                reverse_edges[j].append(i)
    stamp = min(image_stamp, geometry_stamp)
    observations = []
    for i, candidate in enumerate(candidates):
        selected = None
        if len(edges[i]) == 1 and len(reverse_edges[edges[i][0]]) == 1:
            selected = detections[edges[i][0]]
        observations.append(_observation(candidate, stamp, selected))
    return observations
