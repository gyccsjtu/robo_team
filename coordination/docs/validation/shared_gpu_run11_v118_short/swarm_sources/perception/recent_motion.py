"""Recent movement evidence from measured image coordinates, never coasting."""
from collections import deque
import math


class RecentMotion:
    def __init__(self, window_s=4.):
        if not math.isfinite(window_s) or window_s < 1.:
            raise ValueError('MOTION_WINDOW')
        self.window_s = window_s
        self.samples = deque()

    def observe(self, stamp, x, y):
        if not all(math.isfinite(v) for v in (stamp,x,y)):
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp,x,y))
        while self.samples and stamp-self.samples[0][0] > self.window_s:
            self.samples.popleft()
        return True

    def speed(self, now):
        samples = [p for p in self.samples if 0 <= now-p[0] <= self.window_s]
        if len(samples) < 2 or now-samples[-1][0] > 1. or samples[-1][0]-samples[0][0] < 1.:
            return 0.
        origin = samples[0][0]
        times = [p[0]-origin for p in samples]
        mean_t = sum(times)/len(times)
        variance = sum((t-mean_t)**2 for t in times)
        vx = sum((t-mean_t)*p[1] for t,p in zip(times,samples))/variance
        vy = sum((t-mean_t)*p[2] for t,p in zip(times,samples))/variance
        return math.hypot(vx,vy)
