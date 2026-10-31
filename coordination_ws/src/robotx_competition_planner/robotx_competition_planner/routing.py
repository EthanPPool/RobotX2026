"""Bounded visibility search with signed, nearest-abeam passage constraints.

Red markers remain starboard (right), green remain port (left). This is not a
red/green gate-pair selector and does not assume any particular number of gates.
"""
from dataclasses import dataclass
import heapq
import math

from robotx_coordination.geometry import point_in_polygon, segment_distance
from .models import finite


@dataclass(frozen=True)
class Orbit:
    center: tuple
    radius: float
    start_angle: float
    direction: int

    @property
    def start(self):
        return (self.center[0]+self.radius*math.cos(self.start_angle),
                self.center[1]+self.radius*math.sin(self.start_angle))


@dataclass(frozen=True)
class MissionPlan:
    approach: tuple
    entry_orbit: Orbit
    transit: tuple
    exit_orbit: Orbit
    required_ids: tuple


class Course:
    def __init__(self, markers, boundary, config):
        config.validate()
        self.config = config
        self.markers = tuple(markers)
        self.boundary = tuple(tuple(p) for p in boundary)
        if not 2 <= len(self.markers) <= config.max_markers:
            raise ValueError('Invalid or excessive buoy count')
        if len({m.track_id for m in self.markers}) != len(self.markers):
            raise ValueError('Duplicate buoy track IDs')
        for m in self.markers:
            m.validate()
        if (not 3 <= len(self.boundary) <= 32
                or not all(len(p) == 2 and finite(*p) for p in self.boundary)):
            raise ValueError('Configure a finite convex course boundary')
        turns = []
        for a, b, c in zip(self.boundary, self.boundary[1:]+self.boundary[:1],
                           self.boundary[2:]+self.boundary[:2]):
            turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
        if not (all(t > 1e-9 for t in turns) or all(t < -1e-9 for t in turns)):
            raise ValueError('Course boundary must be strictly convex and ordered')
        self.semantic = tuple(m for m in self.markers if m.role in ('red', 'green'))
        if len(self.semantic) > config.max_semantic_markers:
            raise ValueError('Semantic search bound exceeded')
        self.bits = {m.track_id: 1 << i for i, m in enumerate(self.semantic)}
        self.full_mask = (1 << len(self.semantic))-1

    def inflated(self, marker):
        c = self.config
        return marker.radius+marker.uncertainty+c.hull_radius+c.clearance

    def inside(self, point):
        return (finite(*point) and point_in_polygon(point, list(self.boundary))
            and all(segment_distance(point, a, b) > self.config.hull_radius+self.config.clearance
                    for a, b in zip(self.boundary, self.boundary[1:]+self.boundary[:1])))

    def point_clear(self, point):
        return self.inside(point) and all(math.dist(point, m.point) > self.inflated(m)
                                        for m in self.markers)

    def segment_clear(self, a, b):
        # Convex course: if the two inflated endpoints are inside, the complete
        # center segment stays inside. Obstacle clearance is checked analytically.
        return self.inside(a) and self.inside(b) and all(
            segment_distance(m.point, a, b) > self.inflated(m) for m in self.markers)

    def passages(self, a, b, heading=None):
        """Return (legal, bits); closest on each side at abeam is authoritative.

        For graph edges, heading is the forward segment direction. Measured
        transitions can provide actual vessel heading to check realized passages.
        """
        length = math.dist(a, b)
        if length < 1e-9:
            return True, 0
        dx, dy = ((b[0]-a[0])/length, (b[1]-a[1])/length) if heading is None else (
            math.cos(heading), math.sin(heading))
        bits = 0
        for marker in self.semantic:
            ax, ay = marker.x-a[0], marker.y-a[1]
            bx, by = marker.x-b[0], marker.y-b[1]
            along_a, along_b = ax*dx+ay*dy, bx*dx+by*dy
            if not (along_a >= 0 and along_b <= 0):
                continue
            fraction = along_a/(along_a-along_b) if along_a != along_b else 0.
            p = (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]))
            lateral = dx*(marker.y-p[1])-dy*(marker.x-p[0])
            if abs(lateral) > self.config.passage_range:
                continue
            closer = False
            for other in self.markers:
                if other.track_id == marker.track_id:
                    continue
                ox, oy = other.x-p[0], other.y-p[1]
                side, longitudinal = dx*oy-dy*ox, dx*ox+dy*oy
                if (side*lateral > 0 and abs(side) < abs(lateral)-1e-8
                        and abs(longitudinal) <= self.config.abeam_tolerance):
                    closer = True
                    break
            if closer:
                continue
            if (marker.role == 'red' and lateral >= 0) or (
                    marker.role == 'green' and lateral <= 0):
                return False, bits
            bits |= self.bits[marker.track_id]
        return True, bits

    def orbit_clear(self, marker, radius):
        if radius <= self.inflated(marker):
            return False
        if not self.inside(marker.point):
            return False
        edge_margin = radius+self.config.hull_radius+self.config.clearance
        if any(segment_distance(marker.point, a, b) <= edge_margin
               for a, b in zip(self.boundary, self.boundary[1:]+self.boundary[:1])):
            return False
        return all(other.track_id == marker.track_id or
                   abs(math.dist(marker.point, other.point)-radius) > self.inflated(other)
                   for other in self.markers)

    def orbit(self, marker, near, direction):
        c = self.config
        radius = max(c.orbit_radius, self.inflated(marker)+0.2)
        while radius <= c.max_orbit_radius+1e-9:
            if self.orbit_clear(marker, radius):
                return Orbit(marker.point, radius,
                    math.atan2(near[1]-marker.y, near[0]-marker.x), direction)
            radius += 0.5
        raise ValueError('No complete collision-free orbit within configured radius')

    def route(self, start, goal, enforce_sides=False, required_mask=None):
        if not self.point_clear(start) or not self.point_clear(goal):
            raise ValueError('Route endpoint is outside clearance or course boundary')
        required = self.full_mask if required_mask is None else required_mask
        if not enforce_sides:
            required = 0
        nodes = [tuple(start), tuple(goal)]
        n = self.config.graph_samples
        for marker in self.markers:
            # Circumscribed samples: chords between adjacent nodes also stay
            # outside the inflated obstacle, unlike points on the circle itself.
            r = (self.inflated(marker)+0.08)/math.cos(math.pi/n)
            for i in range(n):
                p = (marker.x+r*math.cos(2*math.pi*i/n),
                     marker.y+r*math.sin(2*math.pi*i/n))
                if self.point_clear(p):
                    nodes.append(p)
        cache = {}

        def edges(index):
            if index not in cache:
                result = []
                for j, point in enumerate(nodes):
                    if index == j or not self.segment_clear(nodes[index], point):
                        continue
                    legal, bits = self.passages(nodes[index], point) if enforce_sides else (True, 0)
                    if legal:
                        result.append((j, bits & required, math.dist(nodes[index], point)))
                cache[index] = result
            return cache[index]

        queue, costs, parents = [(math.dist(start, goal), 0., 0, 0)], {(0, 0): 0.}, {}
        expanded = 0
        while queue:
            _, cost, i, mask = heapq.heappop(queue)
            if cost > costs[(i, mask)]+1e-9:
                continue
            expanded += 1
            if expanded > self.config.max_search_states:
                raise ValueError('Route search budget exhausted')
            if i == 1 and mask == required:
                state, path = (i, mask), []
                while state in parents:
                    path.append(nodes[state[0]])
                    state = parents[state]
                return tuple([nodes[0]]+list(reversed(path)))
            for j, bits, distance in edges(i):
                new_mask, new_cost = mask | bits, cost+distance
                state = (j, new_mask)
                if new_cost+1e-9 < costs.get(state, math.inf):
                    costs[state], parents[state] = new_cost, (i, mask)
                    heapq.heappush(queue, (new_cost+math.dist(nodes[j], goal), new_cost, j, new_mask))
        raise ValueError('No route satisfies clearance and required buoy-side passages')

    def plan(self, pose):
        entry = [m for m in self.markers if m.role == 'entry']
        exit_ = [m for m in self.markers if m.role == 'exit']
        if len(entry) != 1 or len(exit_) != 1 or not self.semantic:
            raise ValueError('Need unique flashing-blue entry, steady-blue exit and passage markers')
        entry_orbit = self.orbit(entry[0], pose.point, -1)
        exit_orbit = self.orbit(exit_[0], entry[0].point, 1)
        approach = self.route(pose.point, entry_orbit.start)
        transit = self.route(entry_orbit.start, exit_orbit.start, True)
        return MissionPlan(approach, entry_orbit, transit, exit_orbit,
                           tuple(m.track_id for m in self.semantic))
