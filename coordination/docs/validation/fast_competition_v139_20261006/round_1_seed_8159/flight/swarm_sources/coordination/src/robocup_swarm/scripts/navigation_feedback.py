"""Advisory blocked-plan feedback; never releases execution/occupancy locks."""
import math


def task_key(task):
    return (('search',task['cell_ix'],task['cell_iy']) if task['task_type'] == 0
            else ('target',task['target_id']))


class RejectedTasks:
    """Do not return a failed task to the same stationary aircraft."""
    def __init__(self):
        self.positions = {}

    def reject(self, uid, key, position):
        self.positions[uid,tuple(key)] = tuple(position)

    def allowed(self, uid, key, samples, now):
        identity = uid,tuple(key)
        if identity not in self.positions:
            return True
        sample = samples.get(uid)
        # Time alone does not prove recovery; actual fresh motion does.
        if (sample and 0 <= now-sample['sample_s'] <= .5
                and math.dist(sample['position_xy'],self.positions[identity]) > .5):
            del self.positions[identity]
            return True
        return False


def accept(message, run_id, fleet, active, motion, now, previous_seq):
    try:
        uid = message['uav_id']
        state = motion.get(uid)
        task = active.get(uid)
        if (set(message) != {'schema_version','run_id','uav_id','generation','seq','sample_s',
                'blocked_since_s','reason','position_xy'} or type(message['schema_version']) is not int or message['schema_version'] != 1
                or message['run_id'] != run_id or uid not in fleet or not task or task['stopping']
                or type(message['generation']) is not int or message['generation'] != task['generation'] or type(message['seq']) is not int
                or message['seq'] <= previous_seq.get(uid,0) or not state
                or message['reason'] not in ('START_CLEARANCE_UNKNOWN','NO_REACHABLE_PROGRESS','TARGET_VISUAL_LOST')
                or (message['reason'] == 'TARGET_VISUAL_LOST' and task.get('task', {}).get('task_type') != 1)
                or len(message['position_xy']) != 2
                or not all(math.isfinite(x) for x in (*message['position_xy'],message['sample_s'],message['blocked_since_s'],now))
                or not 0 <= now-message['sample_s'] <= .5
                or not 0 <= now-state['sample_s'] <= .5
                or message['sample_s']-message['blocked_since_s'] < 6.
                or math.dist(message['position_xy'],state['position_xy']) > .75
                or math.hypot(*state['velocity_xy']) > .15):
            return False
        previous_seq[uid] = message['seq']
        return True
    except (KeyError,TypeError,ValueError):
        return False


def refresh_due(active, now, fresh_tracking=False):
    return now-active['started_s'] >= (50. if fresh_tracking else 30.)
