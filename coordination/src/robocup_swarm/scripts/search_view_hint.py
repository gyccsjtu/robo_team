"""Bounded search yaw hints. No person position, observation or report credit."""
import math

IMAGE_TTL_S = 35.0
LOOK_S = 4.0
OBSERVE_S = 1.0
ARRIVAL_M = 2.0
COLORS = ('red', 'green', 'blue', 'brown', 'white')


class ViewRevisitBudget:
    """One supplementary revisit per camera place/direction in sixty seconds.

    Only failed, unlocalized look requests use this budget. New detections,
    localized person tasks and judge reports never consult it. A renamed or
    recoloured cue cannot move the same request into an adjacent free cell.
    """
    def __init__(self):
        self.views = []

    def allows(self, view, now):
        self.views = [v for v in self.views if 0. <= now-v['queued_s'] < 60.]
        for old in self.views:
            angle = math.atan2(math.sin(view['bearing']-old['bearing']),
                               math.cos(view['bearing']-old['bearing']))
            if (math.hypot(view['x']-old['x'], view['y']-old['y']) <= 2. and
                    abs(angle) <= math.radians(15.)):
                return False
        return True

    def remember(self, view, now):
        self.views.append(dict(x=view['x'], y=view['y'], bearing=view['bearing'],
                               queued_s=now))
        self.views = self.views[-128:]


def finite(*values):
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError, OverflowError):
        return False


class CameraOrigins:
    """Exact image-time camera positions; never replace with receipt-time pose."""
    def __init__(self):
        self.frames = {}

    def remember(self, sample, x, y, now):
        if not finite(sample, x, y, now) or not -.05 <= now-sample <= .5:
            return
        self.frames[sample] = (x, y)
        self.frames = dict(sorted((s, xy) for s, xy in self.frames.items()
                                  if 0. <= now-s <= 3.)[-32:])

    def attach(self, cue):
        sample = cue.get('sample_s')
        xy = self.frames.get(sample)
        # A newer bearing must not silently retain an older camera origin.
        cue['view_origin'] = (dict(x=xy[0], y=xy[1], sample_s=sample)
                              if xy is not None else None)


def failed_view(cue, source, now):
    """Only an exact-time origin/bearing pair can create a direction revisit."""
    origin = cue.get('view_origin')
    if not origin or cue.get('color') not in COLORS:
        return None
    sample = cue.get('sample_s')
    bearing = cue.get('bearing')
    if (not finite(sample, bearing, origin['x'], origin['y'], origin['sample_s'], now)
            or sample != origin['sample_s'] or not 0. <= now-sample < IMAGE_TTL_S):
        return None
    return dict(schema=1, source_uav=source, x=origin['x'], y=origin['y'],
                sample_s=now, image_sample_s=sample, origin_sample_s=sample,
                bearing=bearing, color=cue['color'], candidate_id=cue['candidate_id'],
                real_detection=False, report_eligible=False,
                purpose='failed_camera_view')


def parse_failed_view(data, now):
    try:
        sample = float(data['image_sample_s'])
        origin_s = float(data['origin_sample_s'])
        x, y, bearing = (float(data[k]) for k in ('x', 'y', 'bearing'))
        color, candidate = data['color'], str(data['candidate_id'])
        if (data.get('schema') != 1 or data.get('purpose') != 'failed_camera_view'
                or data.get('real_detection') is not False
                or data.get('report_eligible') is not False
                or color not in COLORS or not candidate.startswith(color+':')
                or not finite(sample, origin_s, x, y, bearing, now)
                or sample != origin_s or not 0. <= now-sample < IMAGE_TTL_S):
            return None
        return dict(x=x, y=y, bearing=bearing, color=color, candidate_id=candidate,
                    image_sample_s=sample, origin_sample_s=sample,
                    expires_s=sample+IMAGE_TTL_S, source_uav=data['source_uav'],
                    job_id='%s:%s:%.9f' % (data['source_uav'], candidate, sample))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def assignment_token(message):
    try:
        stamp = float(message.header.stamp.to_sec())
        if message.task_type != 0 or not finite(stamp) or stamp <= 0.:
            return None
        return (message.uav_id, int(message.cell_ix), int(message.cell_iy), stamp)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def make_hint(view, assignment):
    token = assignment_token(assignment)
    if token is None:
        return None
    uid, ix, iy, stamp = token
    return dict(view, schema=1, purpose='search_view_hint', uav_id=uid,
                cell_ix=ix, cell_iy=iy, assignment_s=stamp,
                real_detection=False, report_eligible=False)


def parse_hint(data, uid, now):
    try:
        ix, iy = int(data['cell_ix']), int(data['cell_iy'])
        stamp, sample, origin, expires = (float(data[k]) for k in
            ('assignment_s', 'image_sample_s', 'origin_sample_s', 'expires_s'))
        x, y, bearing = (float(data[k]) for k in ('x', 'y', 'bearing'))
        if (data.get('schema') != 1 or data.get('purpose') != 'search_view_hint'
                or data.get('uav_id') != uid or ix != data['cell_ix'] or iy != data['cell_iy']
                or data.get('real_detection') is not False
                or data.get('report_eligible') is not False
                or data.get('color') not in COLORS
                or not isinstance(data.get('job_id'), str) or not data['job_id']
                or not str(data['candidate_id']).startswith(data['color']+':')
                or not finite(stamp, sample, origin, expires, x, y, bearing, now)
                or sample != origin or not sample <= stamp <= now+.05
                or not -.05 <= now-stamp <= 2. or not 0. <= now-sample < IMAGE_TTL_S
                or not now < expires <= sample+IMAGE_TTL_S):
            return None
        return dict(data, token=(uid, ix, iy, stamp), received_s=now)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


class SearchViewInbox:
    """Handle ROS topic reordering without resurrecting canceled search tasks."""
    def __init__(self):
        self.pending = {}
        self.seen = {}
        self.token = None
        self.active = None
        self.floor = -float('inf')
        self.revoked = -float('inf')
        self.started = {}
        self.last_result = None

    def cancel(self, now):
        self.floor = max(self.floor, now)
        self.revoked = max(self.revoked, now)
        self.pending.clear()
        self.active = None
        self.token = None

    def assign(self, assignment, now):
        token = assignment_token(assignment)
        if token is None:
            try:
                stamp = float(assignment.header.stamp.to_sec())
            except (AttributeError, TypeError, ValueError):
                stamp = now
            self.cancel(stamp if finite(stamp) and stamp > 0. else now)
            return
        if token[3] < self.floor or token[3] <= self.revoked:
            self.active = None
            self.token = None
            return
        self.floor = max(self.floor, token[3])
        if token != self.token:
            self.active = None
            self.token = token
        self.pending = {k: v for k, v in self.pending.items()
                        if k[3] >= self.floor and now-v['received_s'] <= 2.
                        and now < v['expires_s']}
        waiting = self.pending.pop(token, None)
        if waiting is not None:
            self.active = waiting

    def offer(self, data, uid, now):
        hint = parse_hint(data, uid, now)
        if (hint is None or hint['token'][3] < self.floor or
                hint['token'][3] <= self.revoked):
            return False
        token = hint['token']
        self.seen = {k: expiry for k, expiry in self.seen.items() if now < expiry}
        self.started = {k: value for k, value in self.started.items()
                        if now < value[1]}
        if token in self.seen:
            return False  # retransmissions never restart the bounded look
        self.seen[token] = hint['expires_s']
        if token == self.token:
            self.active = hint
        else:
            self.pending[token] = hint
            self.pending = dict(sorted(self.pending.items(), key=lambda p:p[0][3])[-8:])
        return True

    def step(self, now, xy, image_sample, yaw, image_yaw, trusted=True):
        hint = self.active
        if hint is None:
            return None
        if now >= hint['expires_s'] or not trusted:
            self.last_result = dict(job_id=hint['job_id'], reason='expired_or_untrusted')
            self.active = None
            return None
        started = self.started.get(hint['job_id'])
        if started is not None and now-started[0] >= LOOK_S:
            self.last_result = dict(job_id=hint['job_id'], reason='look_timeout')
            self.active = None
            return None
        if (xy is None or not finite(now, *xy, image_sample, yaw, image_yaw)
                or not -1e-6 <= now-image_sample <= .5
                or math.hypot(xy[0]-hint['x'], xy[1]-hint['y']) > ARRIVAL_M):
            hint.pop('aligned_s', None)
            return None
        self.started.setdefault(hint['job_id'], (now, hint['expires_s']))
        hint['started_s'] = self.started[hint['job_id']][0]
        def aligned(angle):
            error = math.atan2(math.sin(angle-hint['bearing']),
                               math.cos(angle-hint['bearing']))
            return abs(error) <= math.radians(12.)
        if aligned(yaw) and aligned(image_yaw) and image_sample >= hint['started_s']:
            hint.setdefault('aligned_s', now)
            hint['phase'] = 'observing'
            if now-hint['aligned_s'] >= OBSERVE_S:
                self.last_result = dict(job_id=hint['job_id'], reason='direction_observed')
                self.active = None
                return None
        else:
            hint.pop('aligned_s', None)
            hint['phase'] = 'turning'
        return hint
