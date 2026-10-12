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
        self.last_valid_offset = None
        self.last_valid_sample = None
        self._last_stamp = None
        self._last_clock = None

    def observe(self, stamp, z, armed, tilt, now=None):
        clock = stamp if now is None else now
        if not all(math.isfinite(v) for v in (stamp, z, tilt, clock)) or stamp <= 0.:
            if not self.frozen:
                self.samples = []
                self.offset = None
            return None
        if self._last_clock is not None and clock < self._last_clock:
            # An old ground reference has no freshness across a clock reset.
            self.samples = []
            self.offset = self.last_valid_offset = self.last_valid_sample = None
            self._last_stamp = None
        self._last_clock = clock
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return None  # duplicate/reordered poses cannot renew qualification
        self._last_stamp = stamp
        if self.frozen:
            return self.offset
        if armed is True:
            # Freeze only a recent qualified *ground* sample. Do not infer a
            # new offset from the armed pose, even if its height looks plausible.
            at, value = self.last_valid_sample, self.last_valid_offset
            self.offset = (value if at is not None and value is not None and
                math.isfinite(value) and at <= stamp and 0. <= clock-at <= .5 else None)
            self.frozen = True
            return self.offset
        if armed is not False:
            return None
        if abs(tilt) > math.radians(3.):
            self.samples = []
            self.offset = None
            return None
        self.samples = [p for p in self.samples if stamp-p[0] <= 2.]
        self.samples.append((stamp, z))
        if len(self.samples) < 15 or stamp-self.samples[0][0] < 1.:
            return self.offset
        zs = [p[1] for p in self.samples]
        if max(zs)-min(zs) > .05:
            self.offset = None
            return None
        self.offset = self.body_height-statistics.median(zs)
        self.last_valid_offset = self.offset
        self.last_valid_sample = stamp
        return self.offset
