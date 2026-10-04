"""Two geometric red track slots, independent of official actor numbering."""
import math


def actor_slot_remaining(actor_id, remaining):
    return any(i in remaining for i in (4,5)) if actor_id in (4,5) else actor_id in remaining


class RedObservations:
    def __init__(self):
        self.slots = {}

    def observe(self, stamp, xy):
        if not all(math.isfinite(v) for v in (stamp,*xy)):
            return None
        candidates=[]
        for tag,(previous,point) in self.slots.items():
            age=stamp-previous
            distance=math.dist(point,xy)
            if 0 <= age <= 3. and distance <= min(8.,1.+3.*age):
                candidates.append((distance,tag))
        if candidates:
            tag=min(candidates)[1]
        else:
            available=[tag for tag in ('red1','red2')
                       if tag not in self.slots or stamp-self.slots[tag][0]>6.]
            if not available:
                return None
            tag=available[0]
        self.slots[tag]=(stamp,tuple(xy))
        return tag
