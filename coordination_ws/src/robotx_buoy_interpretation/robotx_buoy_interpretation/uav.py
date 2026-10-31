"""Project teammate-model detections into the surveyed ENU water plane.

The default anchor is the bottom-center of the box, an approximation to the
water contact point for an oblique camera. Set anchor_fraction=0 for a calibrated
center anchor (e.g. a nadir image), or have the model adapter place boxes around
the actual water contact. Box extent and a buoy-height bound enlarge the output
uncertainty; these are configured estimates, not a substitute for field calibration.
"""
import math

import numpy as np

from robotx_coordination.geometry import ray_to_water, rotation

from .contracts import Observation


def select_capture_calibration(samples, stamp, frame, max_age=0.1, allow_static=False):
    """Select (sample_stamp, optical_frame, payload) from a bounded history.

    Only calibration at or before image capture is applicable. Zero-stamped
    static calibration requires explicit opt-in, and is secondary to a matching
    timed sample. Payload is opaque so the selection is testable without ROS.
    """
    if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(max_age) or max_age <= 0:
        raise ValueError('Invalid calibration capture time or age limit')
    candidates = [sample for sample in samples if sample[1] == frame
                  and 0 < sample[0] <= stamp and stamp-sample[0] <= max_age]
    if candidates:
        return max(candidates, key=lambda item: item[0])[2]
    if allow_static:
        return next((sample[2] for sample in reversed(samples)
                     if sample[1] == frame and sample[0] == 0), None)
    return None


def interpret_uav(detections, camera, position, quaternion, stamp, now,
                  timeout=0.5, max_range=60.0, min_confidence=0.6,
                  position_uncertainty=0.75, pixel_uncertainty=2.0,
                  anchor_fraction=0.5, min_radius=0.2, max_buoy_height=1.5):
    """Return positioned observations; reject invalid captures, omit bad objects.

    `position` and `quaternion` are map<-camera optical at acquisition time. The
    map water surface must be z=0. The caller is responsible for a configured
    datum and capture-time calibration/TF. No future measurement is predicted.
    """
    camera.validate()
    values = (stamp, now, timeout, max_range, min_confidence,
              position_uncertainty, pixel_uncertainty, anchor_fraction,
              min_radius, max_buoy_height)
    if (not all(math.isfinite(v) for v in values) or stamp <= 0
            or min(timeout, max_range, min_radius) <= 0
            or min(position_uncertainty, pixel_uncertainty, max_buoy_height) < 0
            or not 0 <= min_confidence <= 1 or not -0.5 <= anchor_fraction <= 0.5):
        raise ValueError('Invalid UAV interpretation limits')
    if stamp > now or now - stamp > timeout:
        raise ValueError('UAV capture stale or in the future')
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all() or position[2] <= 0:
        raise ValueError('UAV camera must have a finite pose above the water')
    rotation(quaternion)  # Reject an invalid orientation even for an empty array.

    def project(u, v):
        return ray_to_water(((u-camera.cx)/camera.fx, (v-camera.cy)/camera.fy, 1.),
                            position, quaternion, max_range)

    result = []
    used_ids = set()
    for detection in detections:
        try:
            detection.validate(camera)
            if detection.confidence < min_confidence or detection.detection_id in used_ids:
                continue
            used_ids.add(detection.detection_id)
            u = detection.u
            v = detection.v + anchor_fraction * detection.height
            # A clipped anchor is not an observed water contact; never clamp it.
            if not 0 <= v < camera.height:
                continue
            point = project(u, v)
            # Project box corners and pixel-error offsets. Reject boxes straddling
            # the horizon instead of reporting unrealistically precise positions.
            samples = [(u-pixel_uncertainty, v), (u+pixel_uncertainty, v),
                       (u, v-pixel_uncertainty), (u, v+pixel_uncertainty)]
            samples += [(detection.u+dx*detection.width/2,
                         detection.v+dy*detection.height/2)
                        for dx in (-1, 1) for dy in (-1, 1)]
            spread = max(float(np.linalg.norm(project(a, b)[:2]-point[:2]))
                         for a, b in samples)
            height_parallax = (float(np.linalg.norm(point[:2]-position[:2]))
                               * max_buoy_height / position[2])
            observation = Observation(
                observation_id=str(detection.detection_id), source='uav',
                stamp=stamp, x=float(point[0]), y=float(point[1]), z=0.,
                color=detection.color, confidence=detection.confidence,
                uncertainty=position_uncertainty+spread+height_parallax,
                radius=min_radius, light_state=detection.light_state)
            observation.validate()
            result.append(observation)
        except (ValueError, TypeError, OverflowError):
            continue
    return result
