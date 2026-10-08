"""Original-image movement qualification; never consumes predicted positions."""
from collections import deque
import math


class MotionIdentity:
    def __init__(self):
        self.samples = deque()
        self.qualified = False

    def observe(self, stamp, x, y):
        if not all(math.isfinite(v) for v in (stamp,x,y)):
            return False
        if self.samples:
            previous = self.samples[-1]
            gap = stamp-previous[0]
            if gap <= 0:
                return False
            # A lost/reassociated track cannot borrow the previous identity.
            if gap > 1.5 or math.hypot(x-previous[1],y-previous[2]) > max(2.,1.+3.*gap):
                self.samples.clear()
                self.qualified = False
        self.samples.append((stamp,x,y))
        while stamp-self.samples[0][0] > 4.:
            self.samples.popleft()
        if not self.qualified and len(self.samples) >= 3:
            first,last = self.samples[0],self.samples[-1]
            if last[0]-first[0] >= 2. and math.hypot(last[1]-first[1],last[2]-first[2]) >= 2.5:
                times = [p[0]-first[0] for p in self.samples]
                mean = sum(times)/len(times)
                variance = sum((t-mean)**2 for t in times)
                residual = 0.
                for axis in (1,2):
                    centre = sum(p[axis] for p in self.samples)/len(times)
                    speed = sum((t-mean)*p[axis] for t,p in zip(times,self.samples))/variance
                    residual += sum((p[axis]-centre-speed*(t-mean))**2 for t,p in zip(times,self.samples))
                self.qualified = math.sqrt(residual/len(times)) <= .6
        return True

    def allowed(self, now):
        return (math.isfinite(now) and self.qualified and bool(self.samples)
                and 0 <= now-self.samples[-1][0] <= 1.)
