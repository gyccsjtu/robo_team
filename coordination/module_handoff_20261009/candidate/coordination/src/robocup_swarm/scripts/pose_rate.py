"""Own accepted ENU pose differences; no ROS, truth, or coordinate reanchoring."""
from collections import deque
import math
import threading


class PoseRate:
    def __init__(self, window_s=.3, minimum_span_s=.2, maximum_age_s=.5):
        if (not all(math.isfinite(x) for x in (window_s, minimum_span_s, maximum_age_s))
                or not 0 < minimum_span_s <= window_s <= maximum_age_s):
            raise ValueError('INVALID_POSE_RATE_WINDOW')
        self.window_s = window_s
        self.minimum_span_s = minimum_span_s
        self.maximum_age_s = maximum_age_s
        self._history = deque()
        self._epoch = None
        self._lock = threading.RLock()

    def add(self, xyz, stamp, epoch):
        try:
            if len(xyz) != 3 or not all(math.isfinite(x) for x in (*xyz, stamp)):
                return False
        except (TypeError, ValueError):
            return False
        with self._lock:
            if epoch != self._epoch:
                self._history.clear()
                self._epoch = epoch
            if self._history:
                dt = stamp-self._history[-1][1]
                if dt == 0:
                    return False  # Duplicate headers never renew an estimate.
                if dt < 0:
                    self._history.clear()
                    return False  # No old rate survives a backwards clock.
                if dt > self.maximum_age_s:
                    self._history.clear()
            self._history.append((tuple(xyz), stamp))
            while self._history and stamp-self._history[0][1] > self.window_s+1e-9:
                self._history.popleft()
            return True

    def estimate(self, now_s, epoch):
        with self._lock:
            if len(self._history) < 2 or epoch != self._epoch or not math.isfinite(now_s):
                return None
            first, start_s = self._history[0]
            last, end_s = self._history[-1]
            span = end_s-start_s
            if (span+1e-9 < self.minimum_span_s
                    or not 0 <= now_s-end_s <= self.maximum_age_s):
                return None
            velocity = tuple((b-a)/span for a, b in zip(first, last))
            return dict(velocity_xyz=velocity, sample_s=end_s, window_start_s=start_s)


def motion_evidence(twist_sample, twist_z, pose_estimate, now_s, require_pose=True):
    """Check both directions, never average opposing velocities into a false stop.

    Transport schema1 has one XY vector: choose the larger horizontal magnitude.
    Execution checks keep BOTH candidates. STOP uses the larger 3-D magnitude.
    """
    try:
        if (twist_sample is None or len(twist_sample) != 3
                or not all(math.isfinite(v) for v in (*twist_sample, twist_z, now_s))
                or not 0 <= now_s-twist_sample[2] <= .5):
            return None
        raw_xyz = (*twist_sample[:2], twist_z)
        candidates = [raw_xyz[:2]]
        selected, source = raw_xyz[:2], 'velocity_local'
        speed = math.sqrt(sum(v*v for v in raw_xyz))
        stamp, pose_sample, pose_window = twist_sample[2], None, None
        if pose_estimate is None:
            if require_pose:
                return None
        else:
            xyz = pose_estimate['velocity_xyz']
            ps, start = pose_estimate['sample_s'], pose_estimate['window_start_s']
            if (len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, ps, start))
                    or not .2-1e-9 <= ps-start <= .3+1e-9
                    or not 0 <= now_s-ps <= .5):
                return None
            if tuple(xyz[:2]) not in candidates:
                candidates.append(tuple(xyz[:2]))
            if math.hypot(*xyz[:2]) > math.hypot(*selected):
                selected, source = tuple(xyz[:2]), 'pose_rate'
            speed = max(speed, math.sqrt(sum(v*v for v in xyz)))
            stamp = min(stamp, ps)
            pose_sample, pose_window = [*xyz, ps], [start, ps]
        return dict(velocity_candidates=candidates, selected_xy=selected,
            selected_source=source, sample_s=stamp, speed_mps=speed,
            speed_xy=max(math.hypot(*v) for v in candidates),
            velocity_local_sample=[*raw_xyz, twist_sample[2]],
            pose_rate_sample=pose_sample, pose_window_s=pose_window)
    except (TypeError, KeyError, ValueError, OverflowError):
        return None
