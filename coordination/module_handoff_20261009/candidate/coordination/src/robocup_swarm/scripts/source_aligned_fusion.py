"""Align independent camera estimates in time without differencing cameras."""
import math


class SourceAlignedFusion:
    def __init__(self):
        self.sources = {}
        self.primary = None

    def observe(self, source, stamp, xy, confidence, observation_id=None):
        if (not isinstance(source, str) or not source or len(xy) != 2
                or not all(math.isfinite(v) for v in (*xy, stamp, confidence))):
            raise ValueError('Invalid source observation')
        old = self.sources.get(source)
        if old and (stamp <= old['stamp'] or observation_id is not None
                    and observation_id == old['observation_id']):
            return None
        if old is None or stamp-old['stamp'] > 1.5:
            velocity, anchor, anchor_s = (0., 0.), tuple(xy), stamp
        else:
            velocity, anchor, anchor_s = old['velocity'], old['anchor'], old['anchor_s']
            dt = stamp-anchor_s
            if dt >= .1:
                velocity = tuple(max(-4., min(4., .35*(xy[k]-anchor[k])/dt+.65*velocity[k]))
                                 for k in (0, 1))
                anchor, anchor_s = tuple(xy), stamp
        self.sources[source] = dict(stamp=stamp, xy=tuple(xy), confidence=confidence,
            velocity=velocity, anchor=anchor, anchor_s=anchor_s, observation_id=observation_id)
        newest = max(r['stamp'] for r in self.sources.values())
        self.sources = {uid:r for uid,r in self.sources.items() if newest-r['stamp'] <= 1.5}
        rows = [(uid, r, tuple(r['xy'][k]+r['velocity'][k]*(newest-r['stamp']) for k in (0, 1)))
                for uid,r in sorted(self.sources.items()) if newest-r['stamp'] <= .6]
        disagreement = any(math.dist(a[2], b[2]) > 1. for i,a in enumerate(rows) for b in rows[i+1:])
        if disagreement:
            rows = [min(rows, key=lambda row: (-row[1]['stamp'], row[0] != self.primary,
                                               -row[1]['confidence'], row[0]))]
        self.primary = min(rows, key=lambda row: (-row[1]['stamp'], row[0] != self.primary,
                                                  -row[1]['confidence'], row[0]))[0]
        weights = [max(1e-6, row[1]['confidence']**2) for row in rows]
        total = sum(weights)
        point = tuple(sum(w*row[2][k] for w,row in zip(weights,rows))/total for k in (0,1))
        speed = tuple(sum(w*row[1]['velocity'][k] for w,row in zip(weights,rows))/total for k in (0,1))
        return dict(xy=point, velocity=speed, position_s=newest,
            reason='SOURCE_DISAGREEMENT' if disagreement else 'TIME_ALIGNED',
            sources=[dict(uav_id=uid, original_s=r['stamp'], original_xy=r['xy'],
                          velocity=r['velocity'], aligned_xy=aligned, observation_id=r['observation_id'])
                     for uid,r,aligned in rows])
