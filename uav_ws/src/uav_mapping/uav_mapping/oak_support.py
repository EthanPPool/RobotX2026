"""Validation independent of ROS; no guessed device or camera calibration."""
import math


def require_device_id(value):
    value = str(value).strip()
    if not value:
        raise ValueError('Set camera_device_id to the downward OAK-D DeviceID/MXID')
    return value


def acquisition_nanoseconds(ros_now, host_now, captured, max_age):
    """Convert a host-synchronized DepthAI monotonic exposure time to ROS epoch."""
    age = (host_now - captured).total_seconds()
    if not math.isfinite(age) or age < -0.001 or age > max_age:
        raise ValueError('OAK-D acquisition timestamp is stale or in the future')
    result = int(ros_now) - round(max(0.0, age) * 1e9)
    if result <= 0:
        raise ValueError('Invalid ROS clock for OAK-D timestamp')
    return result


def frame_calibration(transform):
    if transform is None:
        raise ValueError('OAK-D frame has no calibration metadata')
    # Intrinsics include the actual sensor crop/resize for THIS frame.
    k = [float(value) for row in transform.getIntrinsicMatrix() for value in row]
    d = [float(value) for value in transform.getDistortionCoefficients()]
    model = transform.getDistortionModel()
    if getattr(model, 'name', str(model)) != 'Perspective':
        raise ValueError('Mapper requires a calibrated perspective OAK-D RGB lens')
    if (len(k) != 9 or len(d) not in (4, 5, 8, 12, 14)
            or not all(math.isfinite(v) for v in k + d)
            or k[0] <= 0 or k[4] <= 0 or abs(k[8] - 1.0) > 1e-5):
        raise ValueError('Missing or invalid OAK-D frame calibration')
    return k, d, 'rational_polynomial' if len(d) >= 8 else 'plumb_bob'
