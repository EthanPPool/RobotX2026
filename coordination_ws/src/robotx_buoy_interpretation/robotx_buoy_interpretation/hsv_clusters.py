"""Image-only RGB light clusters. No metric position or light-state inference."""
from dataclasses import dataclass
import math

import cv2
import numpy as np

# OpenCV hue is 0..179. Red wraps around the ends.
# Red settings were tested on the laptop; green/blue share S/V as initial values.
DEFAULT_RANGES = {
    'red': [0, 230, 120, 15, 255, 255, 165, 230, 120, 179, 255, 255],
    'green': [35, 230, 120, 85, 255, 255],
    'blue': [95, 230, 120, 135, 255, 255],
}


@dataclass(frozen=True)
class Limits:
    min_area: int = 400
    max_area_fraction: float = 0.40
    min_fill: float = 0.10
    min_solidity: float = 0.30
    min_aspect: float = 0.20
    max_aspect: float = 5.0
    kernel: int = 1
    max_detections: int = 100
    reject_border: bool = False

    def validate(self):
        for name in ('min_area', 'kernel', 'max_detections'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError('Expected positive integer: ' + name)
        if self.kernel % 2 != 1 or self.kernel > 31 or self.max_detections > 1000:
            raise ValueError('Invalid kernel/detection bound')
        if not all(math.isfinite(v) for v in (
                self.max_area_fraction, self.min_fill, self.min_solidity,
                self.min_aspect, self.max_aspect)):
            raise ValueError('Nonfinite cluster limits')
        if (not 0 < self.max_area_fraction <= 1 or not 0 <= self.min_fill <= 1
                or not 0 <= self.min_solidity <= 1
                or not 0 < self.min_aspect <= self.max_aspect):
            raise ValueError('Invalid cluster limits')


class Segmenter:
    def __init__(self, limits=Limits(), ranges=None):
        limits.validate()
        self.limits = limits
        self.ranges = {c: list(v) for c, v in
                       (DEFAULT_RANGES if ranges is None else ranges).items()}
        if set(self.ranges) != {'red', 'green', 'blue'}:
            raise ValueError('Expected red, green and blue ranges')
        for values in self.ranges.values():
            if not values or len(values) % 6:
                raise ValueError('HSV ranges require lower/upper triples')
            for i in range(0, len(values), 6):
                for lo, hi, cap in zip(values[i:i+3], values[i+3:i+6], (179, 255, 255)):
                    if (not math.isfinite(lo) or not math.isfinite(hi)
                            or int(lo) != lo or int(hi) != hi
                            or not 0 <= lo <= hi <= cap):
                        raise ValueError('Invalid HSV bounds')
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (limits.kernel, limits.kernel))

    def detect(self, bgr):
        if (not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8
                or bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 1):
            raise ValueError('Expected nonempty uint8 BGR image')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        height, width = bgr.shape[:2]
        results, masks = [], {}
        cfg = self.limits
        for color, values in self.ranges.items():
            raw = np.zeros((height, width), np.uint8)
            for i in range(0, len(values), 6):
                raw |= cv2.inRange(hsv, np.array(values[i:i+3], np.uint8),
                                  np.array(values[i+3:i+6], np.uint8))
            mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            masks[color] = mask
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for label in range(1, count):
                x, y, w, h, area = map(int, stats[label])
                if not cfg.min_area <= area <= cfg.max_area_fraction * width * height:
                    continue
                if cfg.reject_border and (x == 0 or y == 0 or x+w == width or y+h == height):
                    continue
                if not cfg.min_aspect <= w/h <= cfg.max_aspect:
                    continue
                component = np.uint8(labels[y:y+h, x:x+w] == label) * 255
                contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                if not contours:
                    continue
                contour = max(contours, key=cv2.contourArea)
                hull_area = cv2.contourArea(cv2.convexHull(contour))
                solidity = cv2.contourArea(contour)/hull_area if hull_area > 0 else 0.
                if area/(w*h) < cfg.min_fill or solidity < cfg.min_solidity:
                    continue
                # Pixel support after morphology, not a calibrated buoy probability.
                # Shape acceptance is separate, so min_solidity=.30 does not
                # silently become the interpreter's min_confidence=.60.
                score = float(np.count_nonzero(
                    (component != 0) & (raw[y:y+h, x:x+w] != 0))/area)
                results.append(dict(color=color, u=x+(w-1)/2, v=y+(h-1)/2,
                                    width=float(w), height=float(h),
                                    area=area, confidence=score))
        results.sort(key=lambda d: (-d['area'], d['color'], d['v'], d['u']))
        return results[:cfg.max_detections], masks


def rectify(bgr, info, already_rectified=False):
    """Reference camera only: identity R, no stereo offset, no ROI/binning."""
    h, w = bgr.shape[:2]
    if info.width != w or info.height != h or not info.header.frame_id:
        raise ValueError('Missing frame or mismatched calibration dimensions')
    r = np.asarray(info.r, dtype=float).reshape(3, 3)
    p = np.asarray(info.p, dtype=float).reshape(3, 4)
    if (not np.isfinite(r).all() or not np.isfinite(p).all()
            or not np.allclose(r, np.eye(3), rtol=0, atol=1e-9)
            or p[0, 0] <= 0 or p[1, 1] <= 0
            or not np.allclose(p[:, 3], 0, rtol=0, atol=1e-9)
            or not np.allclose(p[2], [0, 0, 1, 0], rtol=0, atol=1e-9)
            or abs(p[0, 1]) > 1e-9 or abs(p[1, 0]) > 1e-9):
        raise ValueError('Unsupported rectified projection')
    if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
            or any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height))):
        raise ValueError('ROI/binning requires adjusted calibration')
    if already_rectified:
        return bgr
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    if (not np.isfinite(k).all() or not np.isfinite(d).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1], rtol=0, atol=1e-9)):
        raise ValueError('Invalid raw intrinsics')
    expected = {'plumb_bob': (4, 5), 'rational_polynomial': (8, 12, 14)}
    if len(d) not in expected.get(info.distortion_model, ()):
        raise ValueError('Unsupported distortion model/coefficients')
    mx, my = cv2.initUndistortRectifyMap(k, d, r, p[:, :3], (w, h), cv2.CV_32FC1)
    return cv2.remap(bgr, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def capture_valid(stamp_ns, now_ns, last_ns, timeout):
    return stamp_ns > 0 and stamp_ns > last_ns and 0 <= now_ns-stamp_ns <= timeout*1e9
