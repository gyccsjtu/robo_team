"""Own flight execution feedback; never target observations or judge evidence."""
import math
import os


def relief_enabled():
    return os.environ.get('BLOCKED_TRACKING_RELIEF', '0') == '1'


def motion_feedback(uid, assignment, now, intent_speed, output_speed):
    if assignment is None or getattr(assignment, 'task_type', None) != 1:
        return None
    stamp = assignment.header.stamp.to_sec()
    tid = assignment.target_id
    if (not tid or not all(math.isfinite(v) for v in
            (now, stamp, intent_speed, output_speed)) or
            stamp > now or min(intent_speed, output_speed) < 0.):
        return None
    return dict(uav_id=uid, target_id=tid, sample_s=now, assignment_s=stamp,
                intent_speed=intent_speed, output_speed=output_speed,
                blocked=intent_speed > .3 and output_speed < .2)


class TrackingMotionWindow:
    """Require consecutive fresh feedback for the current responsibility."""
    def __init__(self):
        self.rows = {}

    def offer(self, data, now, owner, assigned_s):
        try:
            uid, tid = data['uav_id'], data['target_id']
            sample, assignment = float(data['sample_s']), float(data['assignment_s'])
            intent, output = float(data['intent_speed']), float(data['output_speed'])
            if (uid != owner or not tid or
                    not all(math.isfinite(v) for v in (sample, assignment, intent, output, assigned_s)) or
                    not 0. <= now-sample <= .6 or not assigned_s <= assignment <= sample or
                    min(intent, output) < 0.):
                return False
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        key = (uid, tid)
        old = self.rows.get(key)
        if old is not None and now < old['sample']:
            self.rows.clear()
            old = None
        if old is not None and sample <= old['sample']:
            return False
        blocked = intent > .3 and output < .2
        continuous = (old is not None and old['generation'] == assigned_s and
                      0. < sample-old['sample'] <= .6 and old['since'] is not None)
        since = (old['since'] if continuous else sample) if blocked else None
        # Old responsibilities cannot consume memory indefinitely.
        self.rows = {k:r for k,r in self.rows.items() if 0. <= now-r['sample'] <= 2.}
        self.rows[key] = dict(sample=sample, since=since, generation=assigned_s)
        return True

    def blocked(self, uid, tid, now, assigned_s):
        row = self.rows.get((uid, tid))
        return bool(row and row['generation'] == assigned_s and
                    row['since'] is not None and 0. <= now-row['sample'] <= .6 and
                    row['sample']-row['since'] >= 1.-1e-6)
