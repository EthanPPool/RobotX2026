"""Attach unambiguous image colors to copies of measured two-gate objects."""
from copy import deepcopy
import itertools
import math

import numpy as np


def unknown_objects(message):
    result = deepcopy(message)
    for obj in result.objects:
        obj.color = 0
    return result


def color_objects(message, detections, camera, optical_point,
                  min_confidence=.6, padding=8.):
    """Project LiDAR cuboids, not guessed pixel-to-distance estimates.

    optical_point transforms a geometry-capture body point into the camera at
    image capture. Geometry, IDs, dimensions and header remain LiDAR-owned.
    A detection and object must each have exactly one association edge.
    Blue has no enum in the two-gate message and remains COLOR_UNKNOWN.
    """
    camera.validate()
    if not math.isfinite(padding) or padding < 0 or not 0 <= min_confidence <= 1:
        raise ValueError('Invalid color association limits')
    if len({obj.id for obj in message.objects}) != len(message.objects):
        raise ValueError('Duplicate LiDAR object IDs')
    if len({d.detection_id for d in detections}) != len(detections):
        raise ValueError('Duplicate image detection IDs')
    for detection in detections:
        detection.validate(camera)
    result = unknown_objects(message)
    edges = {i: [] for i in range(len(message.objects))}
    reverse = {j: [] for j in range(len(detections))}
    for i, obj in enumerate(message.objects):
        p, s = obj.position, obj.size
        if (obj.object_type != 1 or not all(math.isfinite(v) for v in
                (p.x, p.y, p.z, s.x, s.y, s.z, obj.confidence))
                or min(s.x, s.y, s.z) <= 0 or not 0 <= obj.confidence <= 1):
            continue
        pixels = []
        for dx, dy, dz in itertools.product((-.5, .5), repeat=3):
            point = np.asarray(optical_point((p.x+dx*s.x, p.y+dy*s.y, p.z+dz*s.z)))
            if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 1e-6:
                pixels = []
                break
            pixels.append((camera.fx*point[0]/point[2]+camera.cx,
                           camera.fy*point[1]/point[2]+camera.cy))
        if not pixels:
            continue
        left, top = min(p[0] for p in pixels)-padding, min(p[1] for p in pixels)-padding
        right, bottom = max(p[0] for p in pixels)+padding, max(p[1] for p in pixels)+padding
        if right < 0 or bottom < 0 or left >= camera.width or top >= camera.height:
            continue
        for j, d in enumerate(detections):
            if (d.color in ('red', 'green', 'blue') and d.confidence >= min_confidence
                    and left <= d.u <= right and top <= d.v <= bottom):
                edges[i].append(j)
                reverse[j].append(i)
    for i, matches in edges.items():
        if len(matches) == 1 and len(reverse[matches[0]]) == 1:
            d = detections[matches[0]]
            result.objects[i].color = {'red': 1, 'green': 2}.get(d.color, 0)
            if result.objects[i].color:
                result.objects[i].confidence = min(result.objects[i].confidence, d.confidence)
    return result
