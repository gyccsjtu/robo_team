"""Reject impossible estimator motion without inventing a new world transform."""
import math
from collections import deque


class PoseQuality:
    def __init__(self, maximum_speed=3., jump_floor=3., speed_margin=2.):
        self.maximum_speed = maximum_speed
        self.jump_floor = jump_floor
        self.speed_margin = speed_margin
        self.sample = None
        self.fault = None
        self.history = deque()

    def accept(self, xyz, stamp, enforce=True):
        if self.fault is not None:
            return False
        if len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, stamp)):
            self.fault = 'NONFINITE_POSE'
            return False
        if self.sample is not None:
            old, old_stamp = self.sample
            dt = stamp - old_stamp
            if dt == 0:
                return False  # Duplicate data must not refresh freshness.
            if dt < 0:
                self.fault = 'POSE_CLOCK_REVERSED'
                return False
            if enforce and math.dist(xyz, old) > max(
                    self.jump_floor, self.maximum_speed * dt * self.speed_margin):
                self.fault = 'IMPLAUSIBLE_POSE_JUMP'
                return False
        if not enforce:
            self.history.clear()
        while self.history and stamp - self.history[0][1] > .3:
            self.history.popleft()
        if enforce and self.history:
            old, old_stamp = self.history[0]
            dt = stamp - old_stamp
            if dt >= .2 and math.dist(xyz, old) > self.maximum_speed * self.speed_margin * dt + .3:
                self.fault = 'IMPLAUSIBLE_POSE_SPEED'
                return False
        self.sample = (tuple(xyz), stamp)
        self.history.append(self.sample)
        return True

    def usable(self, now, maximum_age=.5):
        return (self.fault is None and self.sample is not None
                and math.isfinite(now) and 0 <= now - self.sample[1] <= maximum_age)

    # A large reset requires independent frame evidence or a new run. A few
    # stable samples around the new origin do not prove its world position.
