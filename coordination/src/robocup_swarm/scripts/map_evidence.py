"""Fuse dated laser evidence; lost publishers cannot renew old obstacles.

Silence expires a source's vote, not our memory of a building. Only newer,
repeated clear rays can remove that memory. Unknown snapshot cells are not
free-space evidence. This module never reads simulation geometry.
"""
import math
from array import array


class MapEvidenceFusion:
    def __init__(self, size):
        self.cells = bytearray(size)
        self.hit = array('d', [-float('inf')]) * size
        self.clear = array('d', [-float('inf')]) * size
        self.sources = {}
        self.retired = set()

    def apply(self, uid, epoch, sequence, sample, snapshot, observations, now):
        if (not uid or not epoch or sequence < 1 or
                not math.isfinite(sample) or not 0. <= now-sample <= 2. or
                len(snapshot) != len(self.cells)):
            return False
        old = self.sources.get(uid)
        if (uid, epoch) in self.retired:
            return False
        if old and (sample <= old[2] or (epoch == old[0] and sequence <= old[1])):
            return False
        # Validate the entire update before mutating any cell.
        records = {}
        for row in observations:
            if len(row) != 4:
                return False
            idx, at, occupied, count = row
            if (not isinstance(idx, int) or not 0 <= idx < len(self.cells) or
                    not isinstance(count, int) or not 1 <= count <= 255 or
                    occupied not in (0, 1) or not math.isfinite(at) or
                    not 0. < at <= sample or idx in records or
                    occupied != snapshot[idx]):
                return False
            records[idx] = (at, occupied, count)
        if old and epoch != old[0]:
            self.retired.add((uid, old[0]))
        self.sources[uid] = (epoch, sequence, sample)
        for idx, occupied in enumerate(snapshot):
            if occupied and idx not in records and self.hit[idx] == -float('inf'):
                # Missing dating is conservative, never proof of clearance.
                self.hit[idx] = sample
                self.cells[idx] = 1
        for idx, (at, occupied, count) in records.items():
            if occupied:
                self.hit[idx] = max(self.hit[idx], at)
            elif count >= 3 and sample-at <= 2.:
                self.clear[idx] = max(self.clear[idx], at)
            # A late stale snapshot cannot resurrect an already cleared hit.
            # A new physical hit always restores its obstacle immediately.
            self.cells[idx] = int(self.hit[idx] > -float('inf') and
                                  self.clear[idx] < self.hit[idx] + .2)
        return True

    def active_sources(self, now):
        return {uid for uid, (_, _, sample) in self.sources.items()
                if 0. <= now-sample <= 6.}
