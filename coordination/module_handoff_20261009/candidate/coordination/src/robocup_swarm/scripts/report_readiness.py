"""Approach before uploading; consumes accepted originals, never target truth."""
import math


class ReportReadiness:
    def __init__(self):
        self.sources = {}
        self.last_now = None

    def observe(self, observation, now, qualified):
        if self.last_now is not None and now < self.last_now:
            self.sources.clear()
        self.last_now = now
        key = (observation['target_id'], observation['uav_id'])
        stamp = observation['sample_s']
        previous = self.sources.get(key)
        if previous and stamp <= previous['stamp']:
            return False
        camera = observation.get('camera_xyz')
        valid = (qualified and observation.get('schema_version') == 3
                 and isinstance(camera,list) and len(camera) == 3
                 and all(type(v) in (int,float) and math.isfinite(v) for v in camera)
                 and 0 <= now-stamp <= 1.
                 and (observation['target_id'] != 'white'
                      or observation.get('person_frame_verified') is True))
        distance = math.hypot(observation['xyz'][0]-camera[0],
                              observation['xyz'][1]-camera[1]) if valid else float('inf')
        continuing = bool(previous and previous['ready'] and 0 < stamp-previous['stamp'] <= 1.)
        if not valid or distance > (22. if continuing else 12.):
            self.sources.pop(key,None)
            return False
        if previous and 0 < stamp-previous['stamp'] <= 1.:
            first, count = previous['first'], previous['count']+1
        else:
            first, count = stamp, 1
        ready = continuing or count >= 3 and stamp-first >= .6-1e-9
        self.sources[key] = dict(stamp=stamp,first=first,count=count,ready=ready,
                                 observation_id=observation['observation_id'],distance_m=distance)
        return ready

    def allowed(self, tag, now, newest_original_s):
        if self.last_now is not None and now < self.last_now:
            self.sources.clear()
            self.last_now = now
            return False
        fresh = [v for (color,_),v in self.sources.items() if color == tag and v['ready']
                 and 0 <= now-v['stamp'] <= 1.]
        # The fused track cannot borrow permission from an older original.
        return any(v['stamp'] >= newest_original_s for v in fresh)

    def clear(self, tag):
        self.sources = {k:v for k,v in self.sources.items() if k[0] != tag}
