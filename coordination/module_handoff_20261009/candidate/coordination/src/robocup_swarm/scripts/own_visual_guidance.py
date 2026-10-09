"""Keep tracking-camera guidance independent of other cameras' arrival order."""
import math


class OwnVisualGuidance:
    def __init__(self, own_id):
        self.own_id = own_id
        self.samples = {}

    def observe(self, tid, uid, stamp, point, velocity, now):
        values = tuple(point)+tuple(velocity)+(stamp, now)
        if (uid != self.own_id or len(point) != 2 or len(velocity) != 2
                or not all(math.isfinite(v) for v in values)
                or not 0 <= now-stamp <= 1.
                or stamp <= self.samples.get(tid, (None, -math.inf))[1]):
            return False
        self.samples[tid] = (tuple(point)+tuple(velocity), stamp)
        return True

    def current(self, tid, now):
        sample = self.samples.get(tid)
        return sample if sample is not None and 0 <= now-sample[1] <= 1. else None

    def clear(self, tid):
        self.samples.pop(tid, None)


def tracking_input(agent, tid, now):
    own = getattr(agent, '_own_visual_guidance', None)
    sample = own.current(tid, now) if own is not None else None
    if sample is not None:
        return sample
    return agent.targets.get(tid), agent._t_seen.get(tid)
