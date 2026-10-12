"""Ground-only MAVROS height datum from the known aircraft landing geometry.

No world services or judge input. Never calibrate on an armed aircraft and
never follow in-flight estimator drift by silently changing the reference.
"""
import math
import statistics
import threading


class GroundHeightDatum:
    def __init__(self, body_height):
        if not math.isfinite(body_height) or not 0. < body_height < 1.:
            raise ValueError('invalid known landing body height')
        self.body_height = body_height
        self.samples = []
        self.offset = None
        self.frozen = False
        self.last_ground_stamp = -math.inf
        self.reason = 'awaiting_stable_ground'
        self._lock = threading.RLock()

    def set_armed(self, armed):
        with self._lock:
            if armed is True:
                self.frozen = True
                self.reason = ('frozen_valid' if self.offset is not None
                               else 'armed_without_ground_reference')

    def observe(self, stamp, z, armed, tilt):
        # State and pose arrive on different ROS callback threads. Never let
        # an old unarmed callback overwrite a reference after arming freezes it.
        with self._lock:
            self.set_armed(armed)
            return self._observe_ground(stamp, z, armed, tilt)

    def _observe_ground(self, stamp, z, armed, tilt):
        if self.frozen or armed is not False:
            return self.offset
        if not math.isfinite(stamp) or stamp <= 0. or stamp <= self.last_ground_stamp:
            return self.offset
        self.last_ground_stamp = stamp
        # A real change of the local height reference must still invalidate
        # calibration. Use the existing 5 cm ground stability tolerance.
        if (self.offset is not None and math.isfinite(z) and
                abs(z + self.offset - self.body_height) > .05 + 1e-9):
            self.offset = None
            self.samples = []
            self.reason = 'ground_reference_changed'
        if not all(math.isfinite(v) for v in (z, tilt)) or abs(tilt) > math.radians(3.):
            self.samples = []
            # Reject the candidate sample, not a previously validated datum.
            # Clearing offset here followed by arming used to freeze None
            # forever while the aircraft continued flying.
            return self.offset
        self.samples = [p for p in self.samples if stamp-p[0] <= 2.]
        self.samples.append((stamp, z))
        if len(self.samples) < 15 or stamp-self.samples[0][0] < 1.:
            return self.offset
        zs = [p[1] for p in self.samples]
        if max(zs)-min(zs) > .05:
            return self.offset  # no replacement from an unstable window
        self.offset = self.body_height-statistics.median(zs)
        self.reason = 'ground_valid'
        return self.offset
