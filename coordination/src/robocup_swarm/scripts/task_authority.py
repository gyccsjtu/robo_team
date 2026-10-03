"""Transport-free single-writer task locks and executor fencing, schema 2."""
import copy
import math


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def task_key(task):
    if set(task) != {'cell_ix', 'cell_iy', 'target_x', 'target_y', 'task_type', 'target_id'}:
        raise ValueError('TASK_FIELDS')
    if any(not finite(task[k]) for k in ('target_x', 'target_y')):
        raise ValueError('TASK_POSITION')
    if any(type(task[k]) is not int for k in ('cell_ix', 'cell_iy', 'task_type')):
        raise ValueError('TASK_ENUM')
    if not isinstance(task['target_id'], str):
        raise ValueError('TASK_TARGET')
    typ = task['task_type']
    if typ == 0:
        return ('search', task['cell_ix'], task['cell_iy'])
    if typ == 1 and task['target_id']:
        return ('target', task['target_id'])
    if typ in (2, 3):
        return ('terminal', typ)
    raise ValueError('TASK_ENUM')


class TaskAuthority:
    def __init__(self, run_id, fleet, lease_s=3., exit_radius=4.5):
        if not isinstance(run_id, str) or not run_id or not fleet or len(set(fleet)) != len(fleet):
            raise ValueError('RUN_CONFIG')
        if not finite(lease_s) or lease_s <= 0 or not finite(exit_radius) or exit_radius <= 0:
            raise ValueError('LIMITS')
        self.run_id, self.fleet = run_id, tuple(fleet)
        self.lease_s, self.exit_radius = lease_s, exit_radius
        self.active, self.pending, self.locks = {}, {}, {}
        self.seq, self.generation, self.ack_seq, self.last_state = {}, {}, {}, {}
        self._stop_samples = {}
        self._last_ack_status = {}
        self.events, self.last_s = [], 0.
        self.closed = False

    def close(self, now):
        """Fence the run permanently; retain occupancy until measured stop/exit."""
        self._time(now)
        self.closed = True
        self.pending.clear()
        outputs = []
        for uid, active in self.active.items():
            if not active['stopping']:
                self._event('STOP_REQUESTED', uid, now, generation=active['generation'], reason='RUN_CLOSED')
            active['stopping'] = True
            outputs.append(self._message(uid, active, 'STOP', now))
        return outputs

    def refresh(self, uid, now):
        """Stop and fence an old route generation before retrying its task."""
        self._time(now)
        active = self.active.get(uid)
        if self.closed or active is None or active['stopping']:
            return []
        self.pending[uid] = copy.deepcopy(active['task'])
        active['stopping'] = True
        self._event('STOP_REQUESTED', uid, now, generation=active['generation'], reason='ROUTE_REFRESH')
        return [self._message(uid, active, 'STOP', now)]

    def _time(self, now):
        if not finite(now) or now < self.last_s:
            raise ValueError('TIME_ROLLBACK')
        self.last_s = now

    def _event(self, event, uid, now, **details):
        self.events.append(dict(schema_version=2, run_id=self.run_id, sim_s=now,
                                event=event, uav_id=uid, details=details))

    def _key(self, uid, task):
        key = task_key(task)
        return key + (uid,) if key[0] == 'terminal' else key

    def _message(self, uid, active, action, now):
        self.seq[uid] = self.seq.get(uid, 0) + 1
        return dict(schema_version=2, run_id=self.run_id, uav_id=uid,
                    seq=self.seq[uid], generation=active['generation'], action=action,
                    expires_s=active['expires_s'] if action == 'GRANT' else now,
                    task=copy.deepcopy(active['task']))

    def _grant_pending(self, uid, now):
        task = self.pending.get(uid)
        if task is None:
            return []
        key = self._key(uid, task)
        locked = self.locks.get(key)
        if locked is not None and not (locked['owner'] == uid and locked['retired']):
            self._event('TASK_BLOCKED', uid, now, holder=locked['owner'], key=list(key))
            return []
        generation = self.generation.get(uid, 0) + 1
        self.generation[uid] = generation
        active = dict(task=copy.deepcopy(task), key=key, generation=generation,
                      expires_s=now + self.lease_s, stopping=False, started_s=now)
        self.active[uid] = active
        self._stop_samples.pop(uid, None)
        self.locks[key] = dict(owner=uid, generation=generation,
                               point=(task['target_x'], task['target_y']), retired=False)
        del self.pending[uid]
        self._event('TASK_GRANTED', uid, now, generation=generation, key=list(key))
        return [self._message(uid, active, 'GRANT', now)]

    def offer(self, uid, task, now):
        self._time(now)
        if self.closed:
            return []
        if uid not in self.fleet:
            raise ValueError('UNKNOWN_UAV')
        key = self._key(uid, task)
        active = self.active.get(uid)
        if active and not active['stopping'] and active['key'] == key and now < active['expires_s']:
            active['task'] = copy.deepcopy(task)
            if (self._last_ack_status.get(uid) == 'STATE'
                    and 0 <= now - self.last_state.get(uid, -100.) <= .5):
                active['expires_s'] = now + self.lease_s
            self.locks[key]['point'] = (task['target_x'], task['target_y'])
            return [self._message(uid, active, 'GRANT', now)]
        self.pending[uid] = copy.deepcopy(task)
        if active:
            if not active['stopping']:
                active['stopping'] = True
                self._event('STOP_REQUESTED', uid, now, generation=active['generation'])
            return [self._message(uid, active, 'STOP', now)]
        return self._grant_pending(uid, now)

    def ack(self, message, now):
        self._time(now)
        required = {'schema_version', 'run_id', 'uav_id', 'seq', 'generation',
                    'sample_s', 'xyz', 'speed_mps', 'stopped_s', 'status'}
        uid = message.get('uav_id')
        if (set(message) != required or message.get('schema_version') != 2
                or message.get('run_id') != self.run_id or uid not in self.fleet):
            return []
        if (type(message['seq']) is not int or message['seq'] <= self.ack_seq.get(uid, 0)
                or type(message['generation']) is not int
                or message['generation'] != self.generation.get(uid, 0)
                or not finite(message['sample_s']) or not 0 <= now - message['sample_s'] <= .5
                or message['sample_s'] <= self.last_state.get(uid, -100.)
                or not isinstance(message['xyz'], (list, tuple)) or len(message['xyz']) != 3
                or not all(finite(x) for x in message['xyz'])
                or not finite(message['speed_mps']) or message['speed_mps'] < 0
                or not finite(message['stopped_s']) or message['stopped_s'] < 0
                or message['status'] not in ('STATE', 'STOPPED')):
            self._event('ACK_REJECTED', uid, now)
            return []
        self.ack_seq[uid] = message['seq']
        self.last_state[uid] = message['sample_s']
        self._last_ack_status[uid] = message['status']
        for key, lock in list(self.locks.items()):
            if lock['owner'] == uid and lock['retired']:
                d = math.hypot(message['xyz'][0] - lock['point'][0], message['xyz'][1] - lock['point'][1])
                if d > self.exit_radius:
                    del self.locks[key]
                    self._event('TASK_RELEASED', uid, now, key=list(key))
        active = self.active.get(uid)
        observed_stopped_s = 0.
        if active and active['stopping'] and message['speed_mps'] <= .15:
            previous = self._stop_samples.get(uid)
            if previous is None or message['sample_s'] - previous[1] > .5:
                previous = (message['sample_s'], message['sample_s'])
            self._stop_samples[uid] = (previous[0], message['sample_s'])
            observed_stopped_s = message['sample_s'] - previous[0]
        else:
            self._stop_samples.pop(uid, None)
        if (message['status'] == 'STOPPED' and active and active['stopping']
                and message['speed_mps'] <= .15 and message['stopped_s'] >= 1.
                and observed_stopped_s >= 1. - 1e-9):
            self.locks[active['key']]['retired'] = True
            self._event('STOPPED_ACKED', uid, now, generation=active['generation'])
            self._event('TASK_RETIRED', uid, now, key=list(active['key']))
            del self.active[uid]
            return self._grant_pending(uid, now)
        return []

    def tick(self, now):
        self._time(now)
        outputs = []
        for uid, active in list(self.active.items()):
            if active['stopping'] or now >= active['expires_s']:
                if not active['stopping']:
                    active['stopping'] = True
                    self.pending.setdefault(uid, copy.deepcopy(active['task']))
                    self._event('STOP_REQUESTED', uid, now, generation=active['generation'])
                outputs.append(self._message(uid, active, 'STOP', now))
            elif (self._last_ack_status.get(uid) == 'STATE'
                  and 0 <= now - self.last_state.get(uid, -100.) <= .5):
                active['expires_s'] = now + self.lease_s
                outputs.append(self._message(uid, active, 'GRANT', now))
        for uid in self.fleet:
            if uid not in self.active:
                outputs.extend(self._grant_pending(uid, now))
        return outputs


class TaskGate:
    def __init__(self, run_id, uid):
        if not run_id or not uid:
            raise ValueError('RUN_CONFIG')
        self.run_id, self.uid = run_id, uid
        self.seq, self.generation = 0, 0
        self.expires_s, self.task, self.stopping, self.last_s = 0., None, True, 0.

    def receive(self, message, now):
        required = {'schema_version', 'run_id', 'uav_id', 'seq', 'generation',
                    'action', 'expires_s', 'task'}
        try:
            if (set(message) != required or message['schema_version'] != 2
                    or message['run_id'] != self.run_id or message['uav_id'] != self.uid
                    or not finite(now) or now < self.last_s
                    or type(message['seq']) is not int or message['seq'] <= self.seq
                    or type(message['generation']) is not int
                    or message['generation'] < self.generation
                    or not finite(message['expires_s'])
                    or message['action'] not in ('GRANT', 'STOP')):
                return False
            task_key(message['task'])
        except (TypeError, ValueError, KeyError):
            return False
        gen = message['generation']
        if message['action'] == 'GRANT':
            if ((self.generation == 0 and gen != 1)
                    or (self.generation > 0 and gen > self.generation and not self.stopping)):
                return False
            if message['expires_s'] <= now or gen < 1 or (gen == self.generation and self.stopping):
                return False
            if gen == self.generation and self.task and task_key(self.task) != task_key(message['task']):
                return False
            self.task = copy.deepcopy(message['task'])
            self.expires_s, self.stopping = message['expires_s'], False
        elif gen != self.generation:
            return False
        else:
            self.stopping = True
        self.seq, self.generation, self.last_s = message['seq'], gen, now
        return True

    def can_move(self, now):
        if not finite(now) or now < self.last_s or now >= self.expires_s:
            self.stopping = True
        else:
            self.last_s = now
        return self.task is not None and not self.stopping
