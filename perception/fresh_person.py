"""Three distinct same-image person/color matches support green/white tracks."""
import math


class FreshPerson:
    def __init__(self, resume_after_miss=False):
        self.stamp = None
        self.hits = 0
        self.resume_after_miss = resume_after_miss
        self.established = False
        self.current_verified = False
        self.seen_s = None

    def missed(self, stamp):
        """A missing box is no observation; retain only short-track identity."""
        if not math.isfinite(stamp) or stamp <= 0:
            self.observe(stamp, False)
            return
        if self.seen_s is not None and stamp <= self.seen_s:
            return
        self.seen_s = stamp
        self.current_verified = False
        self.hits = 0
        if (not self.resume_after_miss or self.stamp is None
                or not 0 < stamp-self.stamp <= 1.):
            self.established = False

    def observe(self, stamp, verified):
        if not math.isfinite(stamp) or stamp <= 0:
            self.hits = 0
            self.established = self.current_verified = False
            return
        if self.seen_s is not None and stamp <= self.seen_s:
            return
        self.seen_s = stamp
        if verified is not True:
            self.hits = 0
            self.established = self.current_verified = False
            return
        if self.stamp is None or stamp-self.stamp > 1.:
            self.established = False
        self.hits = self.hits+1 if self.stamp is not None and 0 < stamp-self.stamp <= 1. else 1
        self.stamp = stamp
        self.current_verified = True
        if self.hits >= 3:
            self.established = True

    def allowed(self, color, miss, now, allow_red=False, allow_blue=False):
        qualified = self.hits >= 3 or self.resume_after_miss and color == 'white' and self.established
        return ((color in ('green','white') or color == 'red' and allow_red
                 or color == 'blue' and allow_blue) and miss == 0
                and self.current_verified and qualified
                and self.stamp is not None and math.isfinite(now) and 0 <= now-self.stamp <= 1.)
