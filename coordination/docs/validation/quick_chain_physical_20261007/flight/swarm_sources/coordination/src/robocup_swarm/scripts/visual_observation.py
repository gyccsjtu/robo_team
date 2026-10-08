"""Pure schema-2/3 evidence fence; estimates cannot refresh observations."""
import math

TAG_TO_TID = dict(green='t0', blue='t1', brown='t2', white='t3', red1='t5', red2='t4')


class VisualEvidence:
    def __init__(self, run_id, fleet):
        if not run_id or not fleet:
            raise ValueError('Explicit run and fleet required')
        self.run_id, self.fleet = run_id, tuple(fleet)
        self.sequences, self.stamps, self.last_s = {}, {}, 0.

    def receive(self, message, now):
        required = {'schema_version', 'run_id', 'uav_id', 'seq', 'sample_s',
                    'target_id', 'frame_id', 'xyz', 'confidence', 'observation_id'}
        try:
            version = message['schema_version']
            if type(version) is not int or version not in (2,3):
                return None
            if version == 3:
                required |= {'camera_xyz', 'person_frame_verified'}
                camera = message.get('camera_xyz')
                if (not isinstance(camera,list) or len(camera) != 3
                        or any(type(v) not in (int,float) or not math.isfinite(v) for v in camera)
                        or type(message.get('person_frame_verified')) is not bool):
                    return None
            uid, tag = message['uav_id'], message['target_id']
            seq, stamp, confidence, xyz = message['seq'], message['sample_s'], message['confidence'], message['xyz']
            if (set(message) != required
                    or message['run_id'] != self.run_id or uid not in self.fleet or tag not in TAG_TO_TID
                    or message['frame_id'] != 'world_enu' or type(seq) is not int or seq <= self.sequences.get(uid, 0)
                    or not isinstance(xyz, list) or len(xyz) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in (*xyz, confidence, stamp, now))
                    or not 0 <= confidence <= 1 or not 0 <= now-stamp <= 1 or now < self.last_s
                    or stamp <= self.stamps.get((uid, tag), -1.)
                    or message['observation_id'] != '%s:%s:%d' % (self.run_id, uid, seq)):
                return None
        except (TypeError, KeyError, ValueError):
            return None
        self.sequences[uid], self.stamps[(uid, tag)], self.last_s = seq, stamp, now
        return dict(message)
