"""Calibrated downward image processing and bounded semantic survey memory."""
import math

import cv2
import numpy as np

from .geometry import Footprint, Obstacle, fresh, ray_to_water


def detect_buoys(image, min_area=15):
    """Baseline HSV detector; outdoor thresholds require site calibration.

    Return all red/green/yellow connected components, never a selected pair.
    Unknown-color buoys need an external trained detector before field use.
    """
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected BGR8 image')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    bands = {'red': [(0,10),(170,179)], 'green': [(35,85)], 'yellow': [(18,34)]}
    detections = []
    for color, intervals in bands.items():
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        for low, high in intervals:
            mask |= cv2.inRange(hsv, (low,100,70), (high,255,255))
        n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        for i in range(1,n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= min_area:
                width, height = stats[i,cv2.CC_STAT_WIDTH], stats[i,cv2.CC_STAT_HEIGHT]
                detections.append((float(centers[i,0]), float(centers[i,1]), color,
                                   float(max(width,height)/2)))
    return detections


def project_image(image, k, d, position, quaternion, stamp, uncertainty=0.5, max_range=100.):
    k = np.asarray(k, dtype=float).reshape(3,3)
    d = np.asarray(d, dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0,0] <= 0 or k[1,1] <= 0:
        raise ValueError('Missing or invalid camera calibration')
    height, width = image.shape[:2]
    if width < 20 or height < 20:
        raise ValueError('Image too small')
    def project(u,v):
        ray = cv2.undistortPoints(np.array([[[u,v]]],dtype=np.float64),k,d)[0,0]
        return ray_to_water((ray[0],ray[1],1.),position,quaternion,max_range)
    border = 5.
    corners = ((border,border),(width-1-border,border),
               (width-1-border,height-1-border),(border,height-1-border))
    footprint = Footprint(tuple(tuple(project(u,v)[:2]) for u,v in corners),stamp)
    obstacles = []
    for u,v,color,radius_px in detect_buoys(image):
        p = project(u,v)
        edge = project(min(width-1,u+radius_px),v)
        radius = max(0.2,float(np.linalg.norm(edge[:2]-p[:2])))
        obstacles.append(Obstacle(float(p[0]),float(p[1]),radius,uncertainty,color,stamp))
    return obstacles, footprint


class SurveyMemory:
    def __init__(self, ttl=60., merge_distance=0.7, max_tracks=500, max_footprints=300):
        self.ttl, self.merge_distance = ttl, merge_distance
        self.max_tracks, self.max_footprints = max_tracks, max_footprints
        self.obstacles, self.footprints = [], []
        self.last_stamp = None

    def add(self, obstacles, footprint, now):
        if not fresh(footprint.stamp, now, 2.) or (self.last_stamp is not None and footprint.stamp <= self.last_stamp):
            return False
        # Never erase a previously seen buoy because one later image missed it.
        # Aging footprints makes the region unknown; stale obstacles remain
        # conservative until an explicit new survey process is started.
        self.footprints = [f for f in self.footprints if fresh(f.stamp,now,self.ttl)]
        for o in obstacles:
            index = next((i for i,t in enumerate(self.obstacles)
                          if t.stamp < o.stamp and t.color == o.color and
                          math.hypot(t.x-o.x,t.y-o.y)<self.merge_distance), None)
            if index is None:
                self.obstacles.append(o)
            else:
                previous = self.obstacles[index]
                radius = max(previous.radius,o.radius+math.hypot(previous.x-o.x,previous.y-o.y))
                self.obstacles[index] = Obstacle(previous.x,previous.y,radius,
                    max(previous.uncertainty,o.uncertainty),o.color,o.stamp)
        if len(self.obstacles)>self.max_tracks:
            raise ValueError('Survey track capacity exceeded; start a new survey')
        self.footprints.append(footprint)
        # A hovering camera repeatedly observes the same polygon; keep one
        # current footprint rather than burdening planning with duplicates.
        self.footprints = [f for f in self.footprints[:-1]
                           if len(f.points)!=len(footprint.points) or
                           any(math.dist(a,b)>0.05 for a,b in zip(f.points,footprint.points))]+[footprint]
        self.footprints = self.footprints[-self.max_footprints:]
        self.last_stamp = footprint.stamp
        return True
