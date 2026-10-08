"""Original-point motion fitting within each camera, without target truth."""
import math


class GreenSourceMotion:
    def __init__(self):
        self.sources = {}

    def observe(self, source, stamp, xy, confidence, observation_id):
        if (not isinstance(source,str) or not source or len(xy)!=2
                or not all(math.isfinite(v) for v in (*xy,stamp,confidence))):
            raise ValueError('INVALID_GREEN_ORIGINAL')
        rows = self.sources.get(source, [])
        if rows and (stamp <= rows[-1][0] or observation_id == rows[-1][4]):
            return None
        rows = [r for r in rows if 0 <= stamp-r[0] <= 1.]
        rows.append((stamp,xy[0],xy[1],max(1e-6,confidence**2),observation_id))
        self.sources[source] = rows
        velocity = (0.,0.)
        if len(rows) >= 2 and stamp-rows[0][0] >= .2:
            total = sum(r[3] for r in rows)
            mean = sum((r[0]-stamp)*r[3] for r in rows)/total
            variance = sum(r[3]*(r[0]-stamp-mean)**2 for r in rows)
            if variance > 1e-12:
                velocity = tuple(sum(r[3]*(r[0]-stamp-mean)*r[k] for r in rows)/variance
                                 for k in (1,2))
                speed = math.hypot(*velocity)
                if speed > 3.:
                    velocity = tuple(v*3./speed for v in velocity)
        return dict(xy=tuple(xy),velocity=velocity,position_s=stamp,
                    reason='GREEN_ORIGINAL_SOURCE',sources=[source])
