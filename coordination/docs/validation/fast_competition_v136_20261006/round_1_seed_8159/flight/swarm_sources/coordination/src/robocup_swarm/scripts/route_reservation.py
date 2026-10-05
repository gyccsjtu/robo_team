"""Single-writer actual polyline reservation and final horizontal executor gate."""
import copy
import math
import os
from allocation_geometry import point_segment_distance, segment_distance

ROUTE_CLEARANCE = float(os.environ.get('SWARM_ROUTE_CLEARANCE_M', '6.0'))
if not math.isfinite(ROUTE_CLEARANCE) or ROUTE_CLEARANCE < 2.5:
    raise ValueError('Invalid shared route clearance')


def valid_points(points):
    return (isinstance(points, (list, tuple)) and 2 <= len(points) <= 2048
            and all(isinstance(p, (list, tuple)) and len(p) == 2
                    and all(type(v) in (int, float) and math.isfinite(v) for v in p) for p in points))


def segments(points):
    return list(zip(points, points[1:]))


def exclude_peers(grid, uid, motion, reservations, now):
    """Inflate measured peer points and retained segments before actual A*."""
    from robocup_navigation.astar import GridMap
    if any(peer not in motion.samples or not 0 <= now-motion.samples[peer]['sample_s'] <= .5 for peer in motion.fleet):
        raise ValueError('FLEET_EVIDENCE_MISSING')
    obstacles = [(tuple(motion.samples[peer]['position_xy']),)*2 for peer in motion.fleet if peer != uid]
    obstacles += [pair for peer, record in reservations.items() if peer != uid for pair in record['segments']]
    cells = bytearray(grid.cells)
    # Every point inside a grid cell must clear the reservation envelope.
    margin = ROUTE_CLEARANCE+math.sqrt(2)*grid.resolution/2
    for index, value in enumerate(cells):
        if value:
            continue
        point = grid.cell_to_world((index % grid.width, index // grid.width))
        if any(point_segment_distance(point, a, b) <= margin for a, b in obstacles):
            cells[index] = 1
    return GridMap(grid.width, grid.height, grid.resolution, grid.origin, bytes(cells), 'map')


def compact_pairs(pairs):
    """Exact union of overlapping collinear axis-aligned intervals; never shrinks."""
    axes, other = {}, []
    for a, b in pairs:
        if a[0] == b[0]:
            key, interval = ('x', a[0]), sorted((a[1], b[1]))
        elif a[1] == b[1]:
            key, interval = ('y', a[1]), sorted((a[0], b[0]))
        else:
            other.append((a, b))
            continue
        axes.setdefault(key, []).append(interval)
    result = list(other)
    for (axis, fixed), intervals in axes.items():
        merged = []
        for lo, hi in sorted(intervals):
            if merged and lo <= merged[-1][1]:
                merged[-1][1] = max(hi, merged[-1][1])
            else:
                merged.append([lo, hi])
        for lo, hi in merged:
            result.append(((fixed, lo), (fixed, hi)) if axis == 'x' else ((lo, fixed), (hi, fixed)))
    return result


class RouteAuthority:
    def __init__(self, run_id, fleet):
        if not isinstance(run_id, str) or not run_id or not fleet or len(set(fleet)) != len(fleet):
            raise ValueError('RUN_CONFIG')
        self.run_id, self.fleet = run_id, tuple(fleet)
        self.reserved, self.current, self.seq, self.offers = {}, {}, {}, {}
        self.snapshot_seq = 0
        self.events = []
        self.last_s = 0.

    def _time(self, now):
        if type(now) not in (int, float) or not math.isfinite(now) or now < self.last_s:
            return False
        self.last_s = now
        return True

    def retire_stopped(self, uid, generation):
        # Called only following the task core's verified STOPPED_ACKED event.
        record = self.current.get(uid)
        if record is not None and record['generation'] == generation:
            self.current.pop(uid, None)
            self.reserved.pop(uid, None)

    def offer(self, message, now, tasks, motion):
        if not self._time(now):
            return []
        uid = message.get('uav_id')
        if (set(message) != {'schema_version', 'run_id', 'uav_id', 'generation', 'offer_id', 'points'}
                or type(message.get('schema_version')) is not int or message.get('schema_version') != 1 or message.get('run_id') != self.run_id
                or uid not in self.fleet or type(message['generation']) is not int
                or type(message['offer_id']) is not int or message['offer_id'] <= self.offers.get(uid, 0)
                or not valid_points(message['points'])):
            return []
        task = tasks.get(uid)
        if not task or task['stopping'] or task['generation'] != message['generation'] or now >= task['expires_s']:
            return []
        self.offers[uid] = message['offer_id']
        states = motion.samples
        if any(peer not in states or not 0 <= now-states[peer]['sample_s'] <= .5 for peer in self.fleet):
            reason = 'FLEET_EVIDENCE_MISSING'
        elif math.dist(message['points'][0], states[uid]['position_xy']) > .75:
            reason = 'START_MISMATCH'
        else:
            reason = None
            for peer in self.fleet:
                if peer == uid:
                    continue
                pos = states[peer]['position_xy']
                if (any(point_segment_distance(pos, a, b) <= ROUTE_CLEARANCE for a, b in segments(message['points']))
                        or any(segment_distance(a, b, c, d) <= ROUTE_CLEARANCE
                               for a, b in segments(message['points'])
                               for c, d in self.reserved.get(peer, []))):
                    reason = 'ROUTE_CONFLICT'
                    break
        old = self.current.get(uid)
        if old is not None and old['generation'] != message['generation']:
            reason = 'UNRETIRED_GENERATION'
        self.events.append(dict(schema_version=1, run_id=self.run_id, sim_s=now,
            event='ROUTE_BLOCKED' if reason else 'ROUTE_GRANTED', uav_id=uid,
            generation=message['generation'], offer_id=message['offer_id'], reason=reason))
        if reason:
            return []
        accumulated = self.reserved.setdefault(uid, [])
        for pair in segments(message['points']):
            pair = tuple(tuple(p) for p in pair)
            if pair not in accumulated:
                accumulated.append(pair)
        self.current[uid] = copy.deepcopy(message)
        self.reserved[uid] = compact_pairs(accumulated)
        return [self._grant(uid, now)]

    def snapshot(self):
        self.snapshot_seq += 1
        return dict(schema_version=1, run_id=self.run_id, seq=self.snapshot_seq,
            reservations={uid: dict(generation=self.current[uid]['generation'], segments=copy.deepcopy(pairs))
                          for uid, pairs in self.reserved.items()})

    def _grant(self, uid, now):
        self.seq[uid] = self.seq.get(uid, 0)+1
        return dict(copy.deepcopy(self.current[uid]), seq=self.seq[uid], expires_s=now+1.2)

    def tick(self, now, tasks, motion):
        outputs = []
        if not self._time(now):
            return outputs
        if any(uid not in motion.samples or not 0 <= now-motion.samples[uid]['sample_s'] <= .5 for uid in self.fleet):
            return outputs
        for uid, record in self.current.items():
            task = tasks.get(uid)
            if task and not task['stopping'] and task['generation'] == record['generation'] and now < task['expires_s']:
                outputs.append(self._grant(uid, now))
        return outputs


class RouteGate:
    def __init__(self, run_id, uid):
        self.run_id, self.uid = run_id, uid
        self.seq, self.record, self.last_s = 0, None, 0.
        self.last_check = None
        self._segments = ()

    def receive(self, message, now, generation, offer_id):
        if (set(message) != {'schema_version', 'run_id', 'uav_id', 'generation', 'offer_id', 'points', 'seq', 'expires_s'}
                or type(now) not in (int, float) or not math.isfinite(now)
                or type(message['schema_version']) is not int or message['schema_version'] != 1 or message['run_id'] != self.run_id
                or type(message['generation']) is not int or type(message['offer_id']) is not int
                or message['uav_id'] != self.uid or message['generation'] != generation
                or message['offer_id'] != offer_id or type(message['seq']) is not int
                or message['seq'] <= self.seq or not valid_points(message['points'])
                or type(message['expires_s']) not in (float, int) or not math.isfinite(message['expires_s'])
                or message['expires_s'] <= now or now < self.last_s):
            return False
        if self.record is None or message['points'] != self.record['points']:
            # Exact union only, preserving the current route geometry. Repeated
            # A* cell vertices need not be revisited for every stopping sample.
            self._segments = tuple(compact_pairs(segments(message['points'])))
        self.seq, self.record, self.last_s = message['seq'], copy.deepcopy(message), now
        return True

    def clear(self):
        self.record = None
        self._segments = ()

    def command_clear(self, position, requested, measured, now, generation):
        if (type(now) not in (int, float) or not math.isfinite(now)
                or self.record is None or self.record['generation'] != generation or now < self.last_s
                or now >= self.record['expires_s'] or position is None
                or not all(math.isfinite(v) for p in (position, requested, measured) for v in p)):
            reason = ('ROUTE_MISSING' if self.record is None else
                      'ROUTE_GENERATION_MISMATCH' if self.record['generation'] != generation else
                      'ROUTE_EXPIRED' if type(now) in (int, float) and now >= self.record['expires_s'] else
                      'ROUTE_TIME_OR_INPUT_INVALID')
            self.last_check = dict(ok=False, reason=reason)
            self.clear()
            return False
        self.last_s = now
        for kind, velocity in (('requested', requested), ('measured', measured)):
            speed = math.hypot(*velocity)
            distance = speed*.5+speed*speed  # latency .5, braking .5 m/s^2
            endpoint = (position[0]+velocity[0]*distance/max(speed, 1e-9),
                        position[1]+velocity[1]*distance/max(speed, 1e-9))
            failure = self._line_failure(position, endpoint)
            if failure is not None:
                self.last_check = dict(ok=False, reason='ROUTE_'+kind.upper()+'_ENVELOPE',
                    velocity_xy=list(velocity), stopping_distance_m=distance, **failure)
                return False
        self.last_check = dict(ok=True, reason='ROUTE_CLEAR')
        return True

    def _line_failure(self, start, end):
        route = self._segments
        steps = max(1, math.ceil(math.dist(start, end)/.1))
        for i in range(steps+1):
            point = tuple(start[k]+(end[k]-start[k])*i/steps for k in (0, 1))
            offset = min(point_segment_distance(point, a, b) for a, b in route)
            # Distance is 1-Lipschitz: .05m allowance covers between samples.
            if offset > .70:
                return dict(first_rejected_xy=list(point), distance_from_route_m=offset)
        return None

    def connector_clear(self, start, end, now, generation):
        """A guide shortcut must remain inside the current authorized polyline."""
        if not self.command_clear(start, (0., 0.), (0., 0.), now, generation):
            return False
        if end is None or not all(math.isfinite(v) for v in end):
            return False
        return self._line_failure(start, end) is None

    def limited_command(self, position, requested, measured_candidates, now, generation):
        """Shorten only the requested stop segment; never discount measured drift."""
        def result(velocity, reason, scale=0.):
            return dict(velocity_xy=tuple(velocity), reason=reason, scale=scale,
                        requested_xy=[v if math.isfinite(v) else None for v in requested],
                        last_check=copy.deepcopy(self.last_check))

        if not measured_candidates:
            return result((0., 0.), 'ROUTE_MOTION_EVIDENCE_MISSING')
        for measured in measured_candidates:
            if not self.command_clear(position, (0., 0.), measured, now, generation):
                return result((0., 0.), self.last_check['reason'])
        if self.command_clear(position, requested, (0., 0.), now, generation):
            return result(requested, 'ROUTE_CLEAR', 1.)
        if self.record is None:
            return result((0., 0.), self.last_check['reason'])
        # Each accepted candidate is checked with the same final envelope test.
        # Bisection avoids a fixed 1m/s cap and keeps long straight routes fast.
        lo, hi = 0., 1.
        for _ in range(10):
            scale = (lo+hi)/2
            candidate = tuple(v*scale for v in requested)
            if self.command_clear(position, candidate, (0., 0.), now, generation):
                lo = scale
            else:
                hi = scale
        candidate = tuple(v*lo for v in requested)
        if math.hypot(*candidate) < .05:
            return result((0., 0.), 'ROUTE_NO_REQUEST_PROGRESS')
        if not all(self.command_clear(position, candidate, v, now, generation)
                   for v in measured_candidates):
            return result((0., 0.), self.last_check['reason'])
        return result(candidate, 'ROUTE_REQUEST_SCALED', lo)
