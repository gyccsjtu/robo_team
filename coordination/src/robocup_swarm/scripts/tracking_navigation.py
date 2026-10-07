"""Bounded navigation after real visual guidance ages; never visual evidence."""
import math


class TrackingNavigation:
    def __init__(self, window_s=8., fresh_s=1.5, standoff_m=10.):
        if not all(math.isfinite(x) and x > 0 for x in (window_s, fresh_s, standoff_m)):
            raise ValueError('Invalid tracking navigation bounds')
        self.window_s, self.fresh_s, self.standoff_m = window_s, fresh_s, standoff_m
        self.key = None
        self.wait_started_s = None
        self.started_s = None
        self.used = self.failed = False

    def step(self, generation, tid, now, position, point, sample_s,
             eligible=True, allow_scan=True):
        key = (generation, tid)
        if key != self.key:
            self.key, self.started_s = key, None
            self.wait_started_s = now
            self.used = self.failed = False
        if not eligible:
            return 'IDLE', None
        if self.failed:
            return 'FAILED', None
        if sample_s is None or point is None:
            if 0 <= now-self.wait_started_s <= self.window_s:
                return 'WAIT', None
            self.failed = True
            return 'FAILED', None
        if (position is None
                or len(point) != 2 or len(position) != 2
                or not all(math.isfinite(x) for x in (now, sample_s)+tuple(point)+tuple(position))):
            self.failed = True
            return 'FAILED', None
        age = now-sample_s
        if not 0 <= age <= self.window_s:
            self.failed = True
            return 'FAILED', None
        if age <= self.fresh_s:
            self.started_s = None
            return 'CURRENT', tuple(point)
        if self.started_s is not None:
            elapsed = now-self.started_s
            if not 0 <= elapsed < 3.:
                self.failed = True
                return 'FAILED', None
            bearing = math.atan2(point[1]-position[1], point[0]-position[0])
            angle = bearing+(0., .25, -.25)[min(2, int(elapsed))]
            return 'REACQUIRE', (position[0]+8.*math.cos(angle), position[1]+8.*math.sin(angle))
        dx, dy = position[0]-point[0], position[1]-point[1]
        distance = math.hypot(dx, dy)
        if distance > self.standoff_m+.5:
            return 'APPROACH', (point[0]+self.standoff_m*dx/distance,
                                point[1]+self.standoff_m*dy/distance)
        if self.used or not allow_scan:
            self.failed = True
            return 'FAILED', None
        self.used, self.started_s = True, now
        return 'REACQUIRE', tuple(point)
