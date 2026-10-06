"""One bounded yaw-only reacquisition per white tracking authorization."""
import math


class WhiteReacquisition:
    def __init__(self):
        self.reset(None)

    def reset(self, generation, not_before_s=None):
        self.generation = generation
        self.not_before_s = not_before_s
        self.sample_s = None
        self.xy = None
        self.started_s = None
        self.lost_sample_s = None
        self.bearing = None
        self.used = False
        self.failed = False

    def cancel(self):
        self.sample_s = self.xy = self.started_s = None
        self.used = True
        self.failed = False

    def observe(self, generation, stamp, xy, now):
        if (generation != self.generation or self.failed or len(xy) != 2
                or not all(math.isfinite(v) for v in (*xy, stamp, now))
                or not 0 <= now-stamp <= 1.
                or self.not_before_s is not None and stamp < self.not_before_s
                or self.sample_s is not None and stamp <= self.sample_s
                or self.started_s is not None and now-self.started_s >= 3.):
            return False
        self.sample_s, self.xy = stamp, tuple(xy)
        return True

    def step(self, generation, now, position, eligible=True):
        if generation != self.generation:
            self.reset(generation)
        if not eligible:
            self.cancel()
            return 'IDLE', None
        if (len(position) != 2 or not all(math.isfinite(v) for v in (*position, now))):
            self.cancel()
            return 'FAILED', None
        if self.failed:
            return 'FAILED', None
        if self.sample_s is None:
            return 'IDLE', None
        if now < self.sample_s:
            self.failed = True
            return 'FAILED', None
        if self.started_s is not None:
            elapsed = now-self.started_s
            if not 0 <= elapsed < 3.:
                self.failed = True
                return 'FAILED', None
            if self.sample_s > self.lost_sample_s and 0 <= now-self.sample_s <= 1.:
                self.started_s = None
                return 'RESUMED', self.xy
        elif not self.used and now-self.sample_s > 1.:
            self.used = True
            self.started_s = now
            self.lost_sample_s = self.sample_s
            self.bearing = math.atan2(self.xy[1]-position[1], self.xy[0]-position[0])
        else:
            return 'IDLE', None
        offset = (0., .25, -.25)[min(2, int(now-self.started_s))]
        angle = self.bearing+offset
        return 'SCANNING', (position[0]+8.*math.cos(angle), position[1]+8.*math.sin(angle))
