"""Bounded A* on surveyed water only, with continuous segment clearance."""
import heapq
import math

import numpy as np

from .geometry import finite, point_in_polygon, segment_distance


class SurveyPlanner:
    def __init__(self, footprints, obstacles, bounds, resolution=0.5, clearance=1., max_cells=100000):
        self.footprints = footprints
        self.obstacles = obstacles
        self.bounds = tuple(bounds)
        self.resolution, self.clearance = float(resolution), float(clearance)
        if not finite(*bounds, resolution, clearance) or resolution <= 0 or clearance <= 0:
            raise ValueError('Invalid planning limits')
        self.nx = int(math.floor((bounds[2]-bounds[0])/resolution))+1
        self.ny = int(math.floor((bounds[3]-bounds[1])/resolution))+1
        if self.nx < 2 or self.ny < 2 or self.nx*self.ny > max_cells:
            raise ValueError('Invalid or excessive planning area')
        for o in obstacles:
            if not finite(o.x, o.y, o.radius, o.uncertainty) or min(o.radius, o.uncertainty) < 0:
                raise ValueError('Invalid obstacle')
        for f in footprints:
            if not 3 <= len(f.points) <= 16 or not all(finite(*p) for p in f.points):
                raise ValueError('Invalid survey polygon')
            turns = []
            for i,a in enumerate(f.points):
                b,c=f.points[(i+1)%len(f.points)],f.points[(i+2)%len(f.points)]
                turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
            if not (all(t>1e-9 for t in turns) or all(t<-1e-9 for t in turns)):
                raise ValueError('Survey polygon must be strictly convex')
        self._free = {}

    def allowed(self, p):
        x, y = p
        if not finite(x, y) or not (self.bounds[0] <= x <= self.bounds[2] and self.bounds[1] <= y <= self.bounds[3]):
            return False
        # Conservatively require the whole vehicle disk inside one observed footprint.
        covered = any(point_in_polygon((x, y), list(f.points)) and
                      all(segment_distance((x, y), a, b) >= self.clearance
                          for a, b in zip(f.points, f.points[1:]+f.points[:1]))
                      for f in self.footprints)
        return covered and all(math.hypot(x-o.x, y-o.y) > self.clearance+o.radius+o.uncertainty
                               for o in self.obstacles)

    def segment_safe(self, a, b):
        # Exact circle clearance; additionally sample observed-region boundaries.
        if any(segment_distance((o.x, o.y), a, b) <= self.clearance+o.radius+o.uncertainty for o in self.obstacles):
            return False
        n = max(1, int(math.ceil(math.dist(a, b)/(self.resolution/4))))
        return all(self.allowed((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))

    def point(self, cell):
        return (self.bounds[0]+cell[0]*self.resolution, self.bounds[1]+cell[1]*self.resolution)

    def cell(self, point):
        return (round((point[0]-self.bounds[0])/self.resolution), round((point[1]-self.bounds[1])/self.resolution))

    def free(self, cell):
        if cell not in self._free:
            self._free[cell] = 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny and self.allowed(self.point(cell))
        return self._free[cell]

    def plan(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        if not self.allowed(start) or not self.allowed(goal):
            return []
        source, destination = self.cell(start), self.cell(goal)
        if not self.free(source) or not self.free(destination):
            return []
        if not self.segment_safe(start, self.point(source)) or not self.segment_safe(self.point(destination), goal):
            return []
        if self.segment_safe(start, goal):
            return [start, goal]
        queue, costs, parent, closed = [(0., source)], {source: 0.}, {}, set()
        while queue:
            _, current = heapq.heappop(queue)
            if current in closed:
                continue
            closed.add(current)
            if current == destination:
                cells = [current]
                while current != source:
                    current = parent[current]
                    cells.append(current)
                raw = [start]+[self.point(c) for c in reversed(cells)]+[goal]
                # Greedy visibility shortcut; every resulting segment is checked.
                path, i = [raw[0]], 0
                while i < len(raw)-1:
                    j = len(raw)-1
                    while j > i+1 and not self.segment_safe(raw[i], raw[j]):
                        j -= 1
                    path.append(raw[j]); i = j
                return path
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (current[0]+dx, current[1]+dy)
                if not self.free(nxt) or nxt in closed:
                    continue
                if dx and dy and (not self.free((current[0]+dx,current[1])) or not self.free((current[0],current[1]+dy))):
                    continue
                if not self.segment_safe(self.point(current), self.point(nxt)):
                    continue
                cost = costs[current]+math.hypot(dx, dy)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], parent[nxt] = cost, current
                    h = math.dist(nxt, destination)
                    heapq.heappush(queue, (cost+h, nxt))
        return []


def route_target(position, yaw, path, planner, speed=0.12, acceptance=0.35, lookahead=1.):
    """Return a body-relative target compatible with the existing target_controller."""
    if not path or not planner.allowed(position):
        return (0., 0., 0., 'NO_SAFE_ROUTE')
    if math.dist(position, path[-1]) <= acceptance:
        return (0., 0., 0., 'MISSION_COMPLETE')
    # Pick the nearest path segment, then walk forward by lookahead arc length.
    best = None
    for i, (a, b) in enumerate(zip(path, path[1:])):
        a, b, p = np.array(a), np.array(b), np.array(position)
        d = b-a
        t = max(0., min(1., float(np.dot(p-a,d)/np.dot(d,d)))) if np.dot(d,d) > 1e-12 else 0.
        q = a+t*d
        item = (float(np.linalg.norm(p-q)), i, q)
        if best is None or item[0] < best[0]:
            best = item
    if best is None:
        return (0., 0., 0., 'NO_SAFE_ROUTE')
    _, i, target = best
    remaining = lookahead
    for endpoint in path[i+1:]:
        delta = np.asarray(endpoint)-target
        length = float(np.linalg.norm(delta))
        if length >= remaining and length > 0:
            target = target+remaining*delta/length
            break
        target = np.asarray(endpoint); remaining -= length
    if not planner.segment_safe(position, target):
        # Arc lookahead can cut the inside of a tight corner. Aim at the
        # corner itself (or reacquire the nearest clear point) before turning.
        corner = np.asarray(path[i+1])
        if planner.segment_safe(position,corner):
            target = corner
        elif planner.segment_safe(position,best[2]) and math.dist(position,best[2])>0.05:
            target = best[2]
        else:
            return (0., 0., 0., 'LOCAL_OBSTACLE_STOP')
    dx, dy = target-np.asarray(position)
    x = math.cos(yaw)*dx+math.sin(yaw)*dy
    y = -math.sin(yaw)*dx+math.cos(yaw)*dy
    if x <= 0:
        # Existing controller refuses rearward targets: request an in-place turn.
        x, y = 0.001, math.copysign(max(abs(y), 1.), y or 1.)
        speed = 0.
    return float(x), float(y), float(speed), 'ROUTE_TRACKING'
