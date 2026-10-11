"""Ground-only MAVROS height datum from the known aircraft landing geometry.

No world services or judge input. Never calibrate on an armed aircraft and
never follow in-flight estimator drift by silently changing the reference.
"""
import math
import statistics


class GroundHeightDatum:
    def __init__(self, body_height):
        if not math.isfinite(body_height) or not 0. < body_height < 1.:
            raise ValueError('invalid known landing body height')
        self.body_height = body_height
        self.samples = []
        self.offset = None
        self.frozen = False

    def observe(self, stamp, z, armed, tilt):
        if armed is True:
            self.frozen = True
        if self.frozen or armed is not False:
            return self.offset
        if not all(math.isfinite(v) for v in (stamp, z, tilt)) or abs(tilt) > math.radians(3.):
            self.samples = []
            self.offset = None
            return None
        if self.samples and stamp <= self.samples[-1][0]:
            return self.offset
        self.samples = [p for p in self.samples if stamp-p[0] <= 2.]
        self.samples.append((stamp, z))
        if len(self.samples) < 15 or stamp-self.samples[0][0] < 1.:
            return self.offset
        zs = [p[1] for p in self.samples]
        if max(zs)-min(zs) > .05:
            self.offset = None
            return None
        self.offset = self.body_height-statistics.median(zs)
        return self.offset
