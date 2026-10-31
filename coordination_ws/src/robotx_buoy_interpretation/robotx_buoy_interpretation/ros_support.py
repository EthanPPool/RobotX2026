"""Wire conversions; geometry and tracking stay independently testable."""
import math

from rclpy.time import Time
from robotx_coordination_interfaces.msg import BuoyObservation, BuoyObservationArray

from .contracts import Camera, Detection


def stamp_seconds(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def camera_from_info(info, width, height, frame):
    """Require a rectified pinhole image and matching optical calibration.

    CameraInfo D may describe the raw image. The rectified P is authoritative.
    Stereo projection offsets are unsupported: detections must use the reference
    optical camera, not the shifted right-camera projection.
    """
    if info.header.frame_id != frame or not frame:
        raise ValueError('Calibration optical frame mismatch')
    if info.width != width or info.height != height:
        raise ValueError('Model pixels and camera calibration dimensions differ')
    rectification = tuple(float(v) for v in info.r)
    identity = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
    if (len(rectification) != 9 or not all(math.isfinite(v) for v in rectification)
            or any(abs(a-b) > 1e-9 for a, b in zip(rectification, identity))):
        raise ValueError('Nonidentity rectification requires a matching rectified optical TF')
    p = tuple(float(v) for v in info.p)
    if (len(p) != 12 or not all(math.isfinite(v) for v in p)
            or abs(p[3]) > 1e-9 or abs(p[7]) > 1e-9
            or any(abs(p[i]) > 1e-9 for i in (1, 4, 8, 9, 11))
            or abs(p[10] - 1) > 1e-9):
        raise ValueError('Unsupported rectified projection matrix')
    result = Camera(width, height, p[0], p[5], p[2], p[6])
    result.validate()
    return result


def detections_from_msg(msg):
    if not getattr(msg, 'image_rectified', False):
        raise ValueError('Raw pixels are unsupported: rectify the model image/coordinates first')
    result = [Detection(int(d.detection_id), d.u, d.v, d.width, d.height,
                        d.color, d.confidence, d.light_state or 'unknown')
              for d in msg.detections]
    if len({d.detection_id for d in result}) != len(result):
        raise ValueError('Duplicate detection IDs in image')
    return result


def transform_parts(tf):
    p, q = tf.transform.translation, tf.transform.rotation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


def observation_array(observations, frame, reference_id, stamp_msg):
    result = BuoyObservationArray()
    result.header.frame_id = frame
    result.header.stamp = stamp_msg
    result.reference_id = reference_id
    for o in observations:
        o.validate()
        item = BuoyObservation()
        item.observation_id, item.source = o.observation_id, o.source
        item.observed_at = Time(seconds=o.stamp).to_msg()
        item.position.x, item.position.y, item.position.z = map(float, (o.x, o.y, o.z))
        item.color, item.confidence = o.color, float(o.confidence)
        item.uncertainty, item.radius = float(o.uncertainty), float(o.radius)
        item.light_state = o.light_state
        result.observations.append(item)
    return result
