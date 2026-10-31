"""Collect confirmed beacon states across one bounded survey, not one image."""
from dataclasses import replace
import math


class Survey:
    def __init__(self, epoch, max_age=90., aerial=False, min_confidence=.6):
        if not math.isfinite(epoch) or not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid survey epoch or duration')
        self.epoch, self.max_age = epoch, max_age
        self.aerial, self.min_confidence = aerial, min_confidence
        self.geometry, self.semantics = {}, {}

    def update(self, markers, now, conflicting_ids=()):
        for track_id in conflicting_ids:
            self.semantics.pop(track_id, None)
        for m in markers:
            m.validate()
            self.geometry[m.track_id] = m
            if (m.track_id in conflicting_ids or m.semantic_stamp < self.epoch
                    or not 0 <= now-m.semantic_stamp <= self.max_age
                    or m.confidence < self.min_confidence
                    or (self.aerial and m.color_source != 'uav')):
                continue
            recognized = m.role != 'obstacle' or (m.color == 'black' and m.light_state == 'inactive')
            previous = self.semantics.get(m.track_id)
            if recognized and (previous is None or m.semantic_stamp > previous.semantic_stamp):
                self.semantics[m.track_id] = m

    def snapshot(self, now):
        result = []
        for key, g in self.geometry.items():
            s = self.semantics.get(key)
            if s is not None and 0 <= now-s.semantic_stamp <= self.max_age:
                result.append(replace(g, color=s.color, light_state=s.light_state,
                    confidence=s.confidence, semantic_stamp=s.semantic_stamp, color_source=s.color_source))
            else:
                result.append(replace(g, color='unknown', light_state='unknown', confidence=0.,
                                      semantic_stamp=0., color_source=''))
        return tuple(result)
