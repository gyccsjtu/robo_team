"""Connected short exit from the real pose to a normally admissible cell.

The caller must build both grids from current observations and apply peer
exclusions to both. This selects a proposal, never an execution authorization.
"""
from collections import deque
import math


class RestEvidence:
    """A continuous stopped interval measured by both existing motion sources.

    Repeated control ticks cannot advance measurement time. Missing evidence,
    source gaps, clock rollback, or a changed authority invalidate the interval.
    This is eligibility for a proposal; it does not authorize motion.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self._authority = None
        self._start = None
        self._last = None
        self._clock = None

    def update(self, evidence, now_s, authority):
        try:
            raw = evidence['velocity_local_sample']
            pose = evidence['pose_rate_sample']
            if (len(raw) != 4 or len(pose) != 4
                    or not all(math.isfinite(v) for v in (*raw, *pose, now_s))
                    or any(not 0 <= now_s-v[3] <= .5 for v in (raw, pose))
                    or any(math.sqrt(sum(x*x for x in v[:3])) > .15
                           for v in (raw, pose))
                    or authority is None):
                self.reset()
                return False
        except (TypeError, KeyError, ValueError, OverflowError):
            self.reset()
            return False
        stamps = (raw[3], pose[3])
        if (authority != self._authority or self._last is None
                or self._clock is None or now_s < self._clock
                or any(s < old or s-old > .5
                       for s, old in zip(stamps, self._last))):
            self._authority = authority
            self._start = stamps
        self._last = stamps
        self._clock = now_s
        return all(s-start >= 1.-1e-9
                   for s, start in zip(stamps, self._start))


def choose_exit(normal, recovery, start, blocking_hits, max_distance=2.):
    from online_radar_planner import OnlinePlanner
    if ((normal.width,normal.height,normal.resolution,normal.origin) !=
            (recovery.width,recovery.height,recovery.resolution,recovery.origin)):
        return None
    if (len(start) != 2 or not all(math.isfinite(v) for v in start)
            or not math.isfinite(max_distance) or not 0 < max_distance <= 2.
            or not blocking_hits
            or any(len(p) != 2 or not all(math.isfinite(v) for v in p)
                   for p in blocking_hits)):
        return None
    initial = recovery.world_to_cell(start)
    if (initial is None or not recovery.is_free(initial)
            or normal.is_free(initial)):
        return None
    visited = {initial}
    pending = deque([(initial, (tuple(start),), 0.)])
    while pending:
        cell, path, length = pending.popleft()
        for dx,dy in ((-1,0),(1,0),(0,-1),(0,1)):
            other = (cell[0]+dx,cell[1]+dy)
            if other in visited or not recovery.is_free(other):
                continue
            point = recovery.cell_to_world(other)
            before = path[-1]
            new_length = length+math.dist(before,point)
            if new_length > max_distance:
                continue
            # Squared distance is convex along a segment. Nonnegative initial
            # derivative proves the entire segment moves away from every hit
            # responsible for the blocked start, not merely the endpoint.
            step = (point[0]-before[0], point[1]-before[1])
            if any(sum((before[i]-hit[i])*step[i] for i in (0,1)) < -1e-9
                   for hit in blocking_hits):
                continue
            if not OnlinePlanner.connector_clear(recovery,before,point):
                continue
            proposal = path+(tuple(point),)
            if normal.is_free(other):
                return proposal
            visited.add(other)
            pending.append((other,proposal,new_length))
    return None
