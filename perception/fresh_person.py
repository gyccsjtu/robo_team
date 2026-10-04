"""Three distinct same-image person/color matches support green/white tracks."""
import math


class FreshPerson:
    def __init__(self):
        self.stamp = None
        self.hits = 0

    def observe(self, stamp, verified):
        if not math.isfinite(stamp) or stamp <= 0 or verified is not True:
            self.hits = 0
            return
        if self.stamp is not None and stamp <= self.stamp:
            return
        self.hits = self.hits+1 if self.stamp is not None and 0 < stamp-self.stamp <= 1. else 1
        self.stamp = stamp

    def allowed(self, color, miss, now):
        return (color in ('green','white') and miss == 0 and self.hits >= 3
                and self.stamp is not None and math.isfinite(now) and 0 <= now-self.stamp <= 1.)
