"""Camera-backed bounded search; ROS-free. Contract: search_observation_v1.md."""
from collections import OrderedDict, Counter, deque
import copy
import math

FRAME_FIELDS = {'schema_version', 'run_id', 'uav_id', 'generation', 'cell', 'seq',
                'image_s', 'sample_s', 'inference_complete', 'size', 'intrinsics',
                'camera_xyz', 'camera_rotation'}
FEEDBACK_FIELDS = {'schema_version', 'run_id', 'uav_id', 'generation', 'cell', 'seq',
                   'sample_s', 'position_xy', 'view_idx', 'view_xy', 'phase', 'outcome',
                   'finished', 'window_start_s', 'blocked_since_s', 'frames'}
OUTCOMES = {'OBSERVED', 'NO_FRESH_FRAMES', 'VIEW_OCCLUDED', 'NO_CONNECTED_VIEW',
            'NO_ROUTE_COMMIT', 'NO_ACTUAL_PROGRESS', 'EXECUTION_GATE_BLOCKED'}
BLOCKED = {'NO_ROUTE_COMMIT', 'NO_ACTUAL_PROGRESS', 'EXECUTION_GATE_BLOCKED'}
REVISIT_S = 30.
MIN_FRAMES = 3


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value, size):
    return isinstance(value, (list, tuple)) and len(value) == size and all(finite(v) for v in value)


def cell_key(value):
    return isinstance(value, (list, tuple)) and len(value) == 2 and all(type(v) is int for v in value)


def valid_frame(message, now):
    """Validate a fresh, actually inferred original image, including geometry."""
    try:
        if (set(message) != FRAME_FIELDS or type(message['schema_version']) is not int
                or message['schema_version'] != 1 or message['inference_complete'] is not True
                or not isinstance(message['run_id'], str) or not message['run_id']
                or not isinstance(message['uav_id'], str) or not message['uav_id']
                or type(message['generation']) is not int or message['generation'] < 1
                or type(message['seq']) is not int or message['seq'] < 1 or not cell_key(message['cell'])
                or not all(finite(v) for v in (now, message['image_s'], message['sample_s']))
                or not 0 <= now-message['image_s'] <= 1.
                or not message['image_s'] <= message['sample_s'] <= now
                or not 0 <= now-message['sample_s'] <= .5
                or not vector(message['size'], 2) or any(type(v) is not int or not 8 <= v <= 8192 for v in message['size'])
                or not vector(message['intrinsics'], 4) or min(message['intrinsics'][:2]) <= 0
                or not 0 <= message['intrinsics'][2] < message['size'][0]
                or not 0 <= message['intrinsics'][3] < message['size'][1]
                or not vector(message['camera_xyz'], 3) or message['camera_xyz'][2] <= 0
                or not vector(message['camera_rotation'], 9)):
            return False
        r = message['camera_rotation']
        rows = [r[i:i+3] for i in (0, 3, 6)]
        if any(abs(sum(a*b for a, b in zip(rows[i], rows[j]))-(1. if i == j else 0.)) > .02
               for i in range(3) for j in range(3)):
            return False
        det = (r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])
               +r[2]*(r[3]*r[7]-r[4]*r[6]))
        return abs(det-1.) <= .02
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


class FrameCache:
    def __init__(self, run_id, fleet):
        self.run_id, self.fleet = run_id, set(fleet)
        self.frames, self.seq, self.image_s = {}, {}, {}

    def receive(self, message, active, now):
        if not valid_frame(message, now):
            return False
        uid = message['uav_id']
        if (message['run_id'] != self.run_id or uid not in self.fleet or active is None
                or active['stopping'] or now >= active['expires_s']
                or active['task']['task_type'] != 0 or message['generation'] != active['generation']
                or tuple(message['cell']) != (active['task']['cell_ix'], active['task']['cell_iy'])
                or message['image_s'] < active['started_s']
                or message['seq'] <= self.seq.get(uid, 0)
                or message['image_s'] <= self.image_s.get(uid, -1.)):
            return False
        self.seq[uid], self.image_s[uid] = message['seq'], message['image_s']
        cached = self.frames.setdefault(uid, OrderedDict())
        cached[message['seq']] = (copy.deepcopy(message), now)
        while len(cached) > 64:
            cached.popitem(last=False)
        return True


def probes(grid, key):
    """Five sampled ground locations, not a claim of whole-cell visibility."""
    ix, iy = key
    x0, y0 = grid.x_min+ix*grid.cell_m, grid.y_min+iy*grid.cell_m
    x1, y1 = min(x0+grid.cell_m, grid.x_max), min(y0+grid.cell_m, grid.y_max)
    cx, cy = (x0+x1)/2., (y0+y1)/2.
    dx, dy = (x1-x0)/4., (y1-y0)/4.
    return ((cx, cy), (cx-dx, cy-dy), (cx-dx, cy+dy), (cx+dx, cy-dy), (cx+dx, cy+dy))


def in_image(frame, point):
    delta = (point[0]-frame['camera_xyz'][0], point[1]-frame['camera_xyz'][1], -frame['camera_xyz'][2])
    if not .5 <= math.hypot(delta[0], delta[1]) <= 20.:
        return False
    r = frame['camera_rotation']
    # R maps link to world; optical=( -link.y, -link.z, link.x ).
    link = [sum(r[3*j+i]*delta[j] for j in range(3)) for i in range(3)]
    if link[0] <= .1:
        return False
    fx, fy, cx, cy = frame['intrinsics']
    u, v = cx-fx*link[1]/link[0], cy-fy*link[2]/link[0]
    return 1 <= u < frame['size'][0]-1 and 1 <= v < frame['size'][1]-1


def planar_clear(observed, origin, point, now):
    """A fresh planar line beyond the lidar/body blind region. No filled disk."""
    if observed.last_scan_s is None or not 0 <= now-observed.last_scan_s <= .5:
        return False
    blind = observed.range_min
    if blind is None or not finite(blind) or blind < 0:
        return False
    distance = math.dist(origin, point)
    if distance <= blind:
        return False
    count = max(1, math.ceil((distance-blind)/(observed.resolution/2)))
    for i in range(count+1):
        length = blind+(distance-blind)*i/count
        p = tuple(origin[j]+(point[j]-origin[j])*length/distance for j in range(2))
        index = observed.index(*p)
        if index is None or observed.cells[index] != 0:
            return False
        stamp = observed.observed_s[index]
        if stamp is None or not 0 <= now-stamp <= 3.:
            return False
    return True


def visible_samples(frame, grid, observed, now):
    xy = frame['camera_xyz'][:2]
    nearby = sorted((math.dist(xy, (c.cx, c.cy)), key) for key, c in grid.cells.items()
                    if math.dist(xy, (c.cx, c.cy)) <= 22.)[:64]
    result = []
    for _, key in nearby:
        mask = sum(1 << i for i, p in enumerate(probes(grid, key))
                   if in_image(frame, p) and planar_clear(observed, xy, p, now))
        if mask:
            result.append(dict(cell=list(key), mask=mask))
    return result


def proven_points(frames):
    counts, latest = Counter(), {}
    for proof in frames:
        for entry in proof['visible']:
            for i in range(5):
                if entry['mask'] & (1 << i):
                    identity = tuple(entry['cell']), i
                    counts[identity] += 1
                    latest[identity] = max(latest.get(identity, -1.), proof['image_s'])
    return {identity: latest[identity] for identity, n in counts.items() if n >= MIN_FRAMES}


class ObservationLedger:
    """Actual image timestamps survive task preemption, without changing locks."""
    def __init__(self):
        self.points, self.review = {}, set()
        self.feedback_seq, self.windows = {}, set()
        self.last_view = {}

    def mask(self, key, now):
        return sum(1 << i for i in range(5) if (tuple(key), i) in self.points
                   and 0 <= now-self.points[tuple(key), i] < REVISIT_S)

    def complete(self, key, now):
        return self.mask(key, now) == 31

    def latest(self, key):
        return max((self.points.get((tuple(key), i), -1.) for i in range(5)), default=-1.)

    def record(self, frames):
        for identity, stamp in proven_points(frames).items():
            self.points[identity] = max(self.points.get(identity, -1.), stamp)

    def accept(self, message, run_id, fleet, active, motion, cache, grid, now):
        """Manager-side evidence validation. No mutation on rejected messages."""
        try:
            uid = message['uav_id']
            state, task = motion.get(uid), active.get(uid)
            if (set(message) != FEEDBACK_FIELDS or type(message['schema_version']) is not int
                    or message['schema_version'] != 1 or message['run_id'] != run_id or uid not in fleet
                    or not task or task['stopping'] or now >= task['expires_s']
                    or task['task']['task_type'] != 0 or type(message['generation']) is not int
                    or message['generation'] != task['generation'] or not cell_key(message['cell'])
                    or tuple(message['cell']) != (task['task']['cell_ix'], task['task']['cell_iy'])
                    or type(message['seq']) is not int or message['seq'] <= self.feedback_seq.get(uid, 0)
                    or type(message['view_idx']) is not int or not 0 <= message['view_idx'] <= 2
                    or type(message['finished']) is not bool or message['outcome'] not in OUTCOMES
                    or message['phase'] not in ('NEXT_VIEW', 'REVIEW_PENDING')
                    or not all(finite(v) for v in (now, message['sample_s'], message['window_start_s'], message['blocked_since_s']))
                    or not 0 <= now-message['sample_s'] <= .5 or not state
                    or not 0 <= now-state['sample_s'] <= .5 or not vector(message['position_xy'], 2)
                    or math.dist(message['position_xy'], state['position_xy']) > .75
                    or math.hypot(*state['velocity_xy']) > (.15 if message['outcome'] in BLOCKED else .25)
                    or not vector(message['view_xy'], 2)
                    or math.dist(message['view_xy'], (task['task']['target_x'], task['task']['target_y'])) > 8.+1e-6
                    or not isinstance(message['frames'], list) or len(message['frames']) > 32):
                return False
            window = (uid, message['generation'], message['view_idx'])
            if (window in self.windows or message['view_idx'] != self.last_view.get(window[:2], -1)+1
                    or (not message['finished'] and message['phase'] != 'NEXT_VIEW')):
                return False
            if message['outcome'] in BLOCKED:
                if (message['frames'] or not message['finished']
                        or not task['started_s'] <= message['blocked_since_s'] <= message['sample_s']-6.):
                    return False
            elif (not task['started_s'] <= message['window_start_s'] <= message['sample_s']
                    or math.dist(message['position_xy'], message['view_xy']) > 1.):
                return False
            seen_seq, seen_stamps = set(), set()
            for proof in message['frames']:
                if set(proof) != {'seq', 'image_s', 'visible'} or type(proof['seq']) is not int:
                    return False
                stored = cache.frames.get(uid, {}).get(proof['seq'])
                if stored is None or proof['seq'] in seen_seq or proof['image_s'] in seen_stamps:
                    return False
                frame, received = stored
                if (frame['image_s'] != proof['image_s'] or frame['generation'] != message['generation']
                        or frame['cell'] != message['cell']
                        or not message['window_start_s'] <= frame['image_s'] <= received <= message['sample_s']
                        or math.dist(frame['camera_xyz'][:2], message['position_xy']) > 1.5
                        or not isinstance(proof['visible'], list) or len(proof['visible']) > 64):
                    return False
                seen_cells = set()
                for entry in proof['visible']:
                    if (set(entry) != {'cell', 'mask'} or not cell_key(entry['cell'])
                            or type(entry['mask']) is not int or not 1 <= entry['mask'] <= 31):
                        return False
                    key = tuple(entry['cell'])
                    if key not in grid.cells or key in seen_cells:
                        return False
                    seen_cells.add(key)
                    if any(entry['mask'] & (1 << i) and not in_image(frame, p)
                           for i, p in enumerate(probes(grid, key))):
                        return False
                seen_seq.add(proof['seq'])
                seen_stamps.add(proof['image_s'])
            if message['frames'] and not 0 <= now-max(p['image_s'] for p in message['frames']) <= 1.:
                return False
            if message['outcome'] in ('OBSERVED', 'VIEW_OCCLUDED', 'NO_CONNECTED_VIEW') and len(seen_seq) < MIN_FRAMES:
                return False
            if message['outcome'] in ('OBSERVED', 'VIEW_OCCLUDED', 'NO_CONNECTED_VIEW') and message['sample_s']-message['window_start_s'] < 2.5:
                return False
            if message['outcome'] == 'NO_FRESH_FRAMES' and (message['frames'] or message['sample_s']-message['window_start_s'] < 3.):
                return False
            self.feedback_seq[uid] = message['seq']
            self.windows.add(window)
            self.last_view[window[:2]] = message['view_idx']
            self.record(message['frames'])
            if message['finished'] and not self.complete(message['cell'], now):
                self.review.add(tuple(message['cell']))
            return True
        except (KeyError, TypeError, ValueError, OverflowError):
            return False

    def sync_grid(self, grid, locks, pending, now):
        """Never overwrite an executing, pending or retired physical claim."""
        from swarm_task import STATE_ASSIGNED, STATE_COVERED, STATE_FREE, STATE_REVIEW
        pending_cells = {(t['cell_ix'], t['cell_iy']) for t in pending.values() if t['task_type'] == 0}
        for key, c in grid.cells.items():
            if c.state == STATE_ASSIGNED or ('search', *key) in locks or key in pending_cells:
                continue
            if self.complete(key, now):
                c.state = STATE_COVERED
            elif c.state == STATE_COVERED or key in self.review or self.latest(key) >= 0:
                c.state = STATE_REVIEW
            else:
                c.state = STATE_FREE


class SearchSweep:
    """At most three camera views. Progress clocks are independent of planning."""
    def __init__(self, generation, key, goal, granted_s, remaining=()):
        self.generation, self.key, self.granted_s = generation, tuple(key), granted_s
        self.origin = tuple(goal)
        self.views = [tuple(p) for p in remaining][:3] or [self.origin]
        self.view_idx, self.phase = 0, 'GO_TO_VIEW'
        self.window_s, self.yaw_start, self.frames = None, 0., []
        self.progress_s, self.progress_xy = None, None
        self.finished, self.report = False, None
        self.pending_view, self.sent_sequences, self.report_ack = None, set(), False
        self.report_started_s = None
        self._progress_window = deque()
        self._bounded_stop_s = None
        self._bounded_rest = None

    @property
    def goal(self):
        return self.views[self.view_idx]

    def remaining(self):
        if self.pending_view is not None and not self.finished:
            return (self.pending_view,)+tuple(p for p in self.views[self.view_idx+1:] if p != self.pending_view)
        return tuple(self.views[self.view_idx:]) if not self.finished else ()

    def start_observing(self, now, position, speed, yaw):
        if self.phase != 'GO_TO_VIEW' or math.dist(position, self.goal) > .8 or speed > .25:
            return False
        self.phase, self.window_s, self.yaw_start = 'OBSERVE', now, yaw
        self.frames = []
        return True

    def add_frame(self, frame, visible, now):
        if (self.phase != 'OBSERVE' or frame['generation'] != self.generation
                or tuple(frame['cell']) != self.key or not self.window_s <= frame['image_s'] <= now
                or not 0 <= now-frame['image_s'] <= 1.
                or any(p['seq'] == frame['seq'] or p['image_s'] == frame['image_s'] for p in self.frames)):
            return False
        self.frames.append(dict(seq=frame['seq'], image_s=frame['image_s'], visible=visible))
        self.frames = self.frames[-32:]
        return True

    def view_ready(self, now):
        if self.phase != 'OBSERVE':
            return False
        fresh = self.frames and 0 <= now-self.frames[-1]['image_s'] <= 1.
        return now-self.window_s >= (2.5 if len(self.frames) >= MIN_FRAMES and fresh else 3.)

    def next_view(self, point, now):
        if len(self.views) <= self.view_idx+1:
            self.views.append(tuple(point))
        else:
            self.views[self.view_idx+1] = tuple(point)
        self.view_idx += 1
        self.phase, self.window_s, self.frames = 'GO_TO_VIEW', None, []
        self.progress_s, self.progress_xy = now, None
        self.report = None
        self.pending_view, self.sent_sequences, self.report_ack = None, set(), False
        self.report_started_s = None
        self._progress_window.clear()
        self._bounded_stop_s, self._bounded_rest = None, None

    def blocked_reason(self, now, position, speed, has_route, stop_reason):
        if self.phase != 'GO_TO_VIEW':
            return None
        if self.progress_xy is None or math.dist(position, self.progress_xy) > .5:
            self.progress_s, self.progress_xy = now, tuple(position)
        if now-self.progress_s < 6. or speed > .15:
            return None
        if not has_route:
            return 'NO_ROUTE_COMMIT'
        if stop_reason not in ('CLEAR', 'MOVING', 'REQUESTED_STOP'):
            return 'EXECUTION_GATE_BLOCKED'
        return 'NO_ACTUAL_PROGRESS'

    def bounded_stall(self, now, position, fresh=True):
        """A small oscillation is not progress toward the current search view.

        Only accepted, fresh own-position samples may enter this method.
        Large detours and slow but genuine approach remain eligible to fly.
        A latched request does not itself prove that the aircraft has stopped.
        """
        if self.phase != 'GO_TO_VIEW':
            return False
        if self._bounded_stop_s is not None:
            return True
        window = self._progress_window
        if not fresh:
            window.clear()
            return False
        if window and now < window[-1][0]:
            window.clear()
        elif window and now == window[-1][0]:
            return False
        if window and now-window[-1][0] > 1.:
            window.clear()
        window.append((now, tuple(position), math.dist(position, self.goal)))
        while len(window) > 1 and window[1][0] <= now-20.:
            window.popleft()
        if now-window[0][0] < 20. or window[-1][2] <= .8:
            return False
        # The diagonal of the enclosing box is a conservative diameter bound.
        span = math.hypot(max(p[1][0] for p in window)-min(p[1][0] for p in window),
                          max(p[1][1] for p in window)-min(p[1][1] for p in window))
        improvement = window[0][2]-min(p[2] for p in window)
        if span <= 4. and improvement < .5:
            self._bounded_stop_s = now
            self.progress_s = now  # Feedback belongs to this active generation.
            return True
        return False

    def bounded_rest_ready(self, now, position, speed, fresh):
        if (not fresh or not math.isfinite(speed) or speed > .15):
            self._bounded_rest = None
            if self._bounded_stop_s is None:
                self._progress_window.clear()
            return False
        if self._bounded_stop_s is None:
            return False
        if (self._bounded_rest is None or math.dist(position, self._bounded_rest[1]) > .5
                or not 0 <= now-self._bounded_rest[2] <= .5):
            self._bounded_rest = (now, tuple(position), now)
        else:
            self._bounded_rest = (self._bounded_rest[0], self._bounded_rest[1], now)
        return now-self._bounded_rest[0] >= 6.

    def yaw(self, now):
        return self.yaw_start+min(3., max(0., now-self.window_s))*.65
