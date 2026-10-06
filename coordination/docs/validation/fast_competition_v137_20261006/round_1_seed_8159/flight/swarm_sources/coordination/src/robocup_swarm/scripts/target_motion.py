"""Per-camera target velocity from original observations, without truth input."""
from collections import deque
import math


def current_target_point(targets, stamps, tid, now, maximum_age=1.):
    """Only accepted, fresh camera coordinates may guide a stopped observer."""
    try:
        stamp = stamps[tid]
        xy = tuple(targets[tid][:2])
        if (len(xy) != 2 or not all(math.isfinite(v) for v in (*xy, stamp, now))
                or not 0 <= now-stamp <= maximum_age):
            return None
        return xy
    except (KeyError, TypeError, ValueError):
        return None


def handoff_reacquisition_point(targets, stamps, tid, generation, window, now,
                                authorized, cooldown_until):
    """A bounded yaw hint from actual images; never authorize translation."""
    try:
        target, granted_generation, start, end = window
        if (not authorized or target != tid or granted_generation != generation
                or not all(math.isfinite(v) for v in (start, end, now, cooldown_until))
                or not 0 < end-start <= 5. or not start <= now < end):
            return None
        return current_target_point(targets, stamps, tid, now, maximum_age=5.)
    except (TypeError, ValueError):
        return None


class TargetMotion:
    def __init__(self):
        self.history, self.fast_counts = {}, {}

    def observe(self, tid, uid, stamp, x, y):
        if not all(math.isfinite(v) for v in (stamp,x,y)):
            return None
        key = (tid,uid)
        history = self.history.setdefault(key,deque())
        if history and stamp <= history[-1][0]:
            return None
        if history and stamp-history[-1][0] > 1.:
            history.clear(); self.fast_counts[key] = 0
        history.append((stamp,x,y))
        while history and stamp-history[0][0] > 1.5:
            history.popleft()
        if len(history) < 3 or stamp-history[0][0] < .5:
            return (0.,0.,False)
        origin = history[0][0]
        times = [row[0]-origin for row in history]
        mean = sum(times)/len(times)
        variance = sum((t-mean)**2 for t in times)
        vx,vy = [sum((t-mean)*row[i] for t,row in zip(times,history))/variance for i in (1,2)]
        speed = math.hypot(vx,vy)
        if speed > 3.:
            history.clear(); history.append((stamp,x,y));self.fast_counts[key]=0
            return (0.,0.,False)
        self.fast_counts[key] = self.fast_counts.get(key,0)+1 if speed >= 1.6 else 0
        return (vx,vy,self.fast_counts[key] >= 2)
