"""Infer beacon state from explicit, tracked color observations over time.

Unknown/missing observations are not OFF. Black must be an explicit model label.
Provided temporal model states take precedence in the tracker. Sampling gaps reset
history instead of fabricating on/off transitions from lost detections.
"""
import math


class BeaconHistory:
    def __init__(self, max_gap=.75, steady_seconds=2.5, period_min=.5, period_max=1.5):
        self.max_gap, self.steady_seconds = max_gap, steady_seconds
        self.period_min, self.period_max = period_min, period_max
        self.runs = []
        self.last_stamp = None

    def update(self, color, stamp, confidence):
        if (color not in ('red', 'green', 'blue', 'black')
                or not all(math.isfinite(v) for v in (stamp, confidence))
                or stamp <= 0 or not 0 <= confidence <= 1):
            return None
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None
        if self.last_stamp is not None and stamp-self.last_stamp > self.max_gap:
            self.runs = []
        self.last_stamp = stamp
        if self.runs and self.runs[-1][0] == color:
            self.runs[-1][2] = stamp
            self.runs[-1][3] = min(self.runs[-1][3], confidence)
        else:
            self.runs.append([color, stamp, stamp, confidence])
        self.runs = self.runs[-8:]
        current = self.runs[-1]
        if stamp-current[1] >= self.steady_seconds:
            return color, 'inactive' if color == 'black' else 'steady', current[3]
        if len(self.runs) < 5 or stamp-current[1] > self.period_max:
            return None
        recent = self.runs[-5:]
        active = {r[0] for r in recent if r[0] != 'black'}
        if len(active) != 1 or any((a[0] == 'black') == (b[0] == 'black')
                                   for a, b in zip(recent, recent[1:])):
            return None
        durations = [b[1]-a[1] for a, b in zip(recent, recent[1:])]
        if not all(self.period_min <= d <= self.period_max for d in durations):
            return None
        return next(iter(active)), 'flashing', min(r[3] for r in recent)
