"""Enumerate bounded geometric gate hypotheses without choosing a mission gate."""
import itertools
import math


def find_pairs(objects, *, min_confidence=0.60, min_width=1.83, max_width=3.05,
               nominal_width=2.44, width_tolerance=0.75, max_skew_deg=55.0,
               min_x=0.75, max_x=10.0, max_buoys=64, max_candidates=64,
               require_red_green=False, min_pair_confidence=0.0):
    # Input order cannot change the ranking. Deduplicate IDs and nearly
    # coincident returns so duplicate tracks cannot become separate markers.
    eligible = []
    for obj in objects:
        p = obj.position
        if (obj.object_type == 1 and
                all(math.isfinite(v) for v in (p.x, p.y, p.z, obj.confidence)) and
                min_confidence <= obj.confidence <= 1.0 and min_x <= p.x <= max_x):
            eligible.append(obj)
    eligible.sort(key=lambda o: (math.hypot(o.position.x, o.position.y), -o.confidence, o.id))
    buoys, ids = [], set()
    for obj in eligible:
        if obj.id in ids or any(math.hypot(obj.position.x - b.position.x,
                                         obj.position.y - b.position.y) < 0.25 for b in buoys):
            continue
        ids.add(obj.id)
        buoys.append(obj)
        if len(buoys) >= max_buoys:
            break  # Bound deduplication, pair enumeration and obstruction checks.
    result = []
    for a, b in itertools.combinations(buoys, 2):
        # Unknown color is never evidence of red/green. This option is useful
        # only when an upstream sensor actually provides reliable colors.
        if require_red_green and {a.color, b.color} != {1, 2}:
            continue
        dx, dy = a.position.x - b.position.x, a.position.y - b.position.y
        width = math.hypot(dx, dy)
        if not min_width <= width <= max_width:
            continue
        skew = math.atan2(abs(dx), abs(dy))
        if skew > math.radians(max_skew_deg):
            continue
        cx, cy = (a.position.x + b.position.x) / 2, (a.position.y + b.position.y) / 2
        if not min_x <= cx <= max_x:
            continue
        # Reject a proposed opening containing another observed buoy.
        segment = (b.position.x - a.position.x, b.position.y - a.position.y)
        obstructed = False
        for other in buoys:
            if other.id in (a.id, b.id):
                continue
            ox, oy = other.position.x - a.position.x, other.position.y - a.position.y
            projection = (ox * segment[0] + oy * segment[1]) / (width * width)
            perpendicular = abs(ox * segment[1] - oy * segment[0]) / width
            if 0.05 < projection < 0.95 and perpendicular < 0.25:
                obstructed = True
                break
        if obstructed:
            continue
        score = (0.45 * (a.confidence + b.confidence) / 2 +
                 0.25 * math.cos(skew) + 0.20 * abs(dy) / width +
                 0.10 * math.exp(-0.5 * ((width - nominal_width) / width_tolerance) ** 2))
        if score < min_pair_confidence:
            continue  # Weak nearby hypotheses must not consume output slots.
        left, right = (a, b) if a.position.y >= b.position.y else (b, a)
        result.append(dict(key=tuple(sorted((a.id, b.id))), left=left.position,
                           right=right.position, center_x=cx, center_y=cy,
                           width=width, confidence=score))
    result.sort(key=lambda p: (math.hypot(p['center_x'], p['center_y']),
                               -p['confidence'], p['key']))
    return result[:max_candidates]
