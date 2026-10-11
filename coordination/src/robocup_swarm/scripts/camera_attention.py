"""Bounded image-only approach. Never supplies an actor/world coordinate."""
import math


def approach_image(data):
    """Only a full, modest-size, upright view permits an exploratory approach."""
    try:
        box = tuple(float(v) for v in data['box'])
        w, h = float(data['width']), float(data['height'])
        tilt = float(data['camera_tilt'])
        if len(box) != 4 or not all(math.isfinite(v) for v in (*box, w, h, tilt)):
            return None
        x1, y1, x2, y2 = box
        if (w <= 0 or h <= 0 or not 8 <= x1 < x2 <= w-8 or
                not 8 <= y1 < y2 <= .83*h or
                not 12 <= y2-y1 <= .32*h or
                abs(tilt) > math.radians(8)):
            return None
        result = dict(box=box, width=w, height=h, camera_tilt=tilt)
        result['height_reimage_ok'] = (data.get('footpoint_valid') is not False and
            not data.get('small_box_continuation') and
            data.get('appearance_status', 'actor') == 'actor')
        for key in ('foot_sin_down', 'camera_height_m', 'confidence'):
            if key in data:
                value = float(data[key])
                if math.isfinite(value):
                    result[key] = value
        return result
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def candidate_decision(cue, now, yaw, xy, scan, scan_stamp):
    """Approach at <=.95m/s for <=6s/3m, with fresh view and clear raw rays.

    The returned request STILL goes through all flight safety guards. Unknown
    range is never converted into a target position. Missing/invalid scan or
    image means stop, not optimistic free space.
    """
    image = cue.get('image')
    origin = cue.get('approach_origin')
    if image is None or origin is None:
        return 0., 0., 'missing_image_geometry'
    if cue.get('xy') is not None:
        return 0., 0., 'localized_candidate'
    if not 0 <= now-cue['sample_s'] <= .35:
        return 0., 0., 'image_expired'
    if not 0 <= now-cue['started'] < 6.:
        return 0., 0., 'time_budget'
    if math.hypot(xy[0]-origin[0], xy[1]-origin[1]) >= 3.:
        return 0., 0., 'travel_budget'
    if scan is None or not 0 <= now-scan_stamp <= .3:
        return 0., 0., 'scan_expired'
    error = math.atan2(math.sin(cue['bearing']-yaw), math.cos(cue['bearing']-yaw))
    if abs(error) > math.radians(12):
        return 0., 0., 'turn_to_candidate'
    clear = 0
    try:
        if not math.isfinite(scan.range_max) or scan.range_max < 4.:
            return 0., 0., 'invalid_scan'
        for i, raw in enumerate(scan.ranges):
            angle = scan.angle_min+i*scan.angle_increment
            delta = math.atan2(math.sin(angle-error), math.cos(angle-error))
            if abs(delta) > math.radians(30):
                continue
            r = float(raw)
            if math.isinf(r) and r > 0:  # valid no-return, not NaN/negative inf
                clear += 1
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                if r < 4.:
                    return 0., 0., 'obstacle'
                clear += 1
            else:
                return 0., 0., 'invalid_scan'
    except (AttributeError, TypeError, ValueError):
        return 0., 0., 'invalid_scan'
    if clear < 3:
        return 0., 0., 'insufficient_scan'
    # Slow down before the box grows out of the admissible approach band.
    height = image['box'][3]-image['box'][1]
    speed = min(.95, max(0., (.32-height/image['height'])*5.))
    return speed*math.cos(cue['bearing']), speed*math.sin(cue['bearing']), ('approach' if speed > 0. else 'box_too_large')


def candidate_velocity(cue, now, yaw, xy, scan, scan_stamp):
    return candidate_decision(cue, now, yaw, xy, scan, scan_stamp)[:2]


def candidate_observation_height(cue, now, local_z, ceiling):
    """Bound one upward reimage attempt; never relax coordinate qualification.

    Only a fresh, full-body cue with a measured shallow foot ray can start it.
    The caller must retain obstacle-height and final flight safety gates.
    A new frame never extends the attention deadline or raises the stored goal.
    """
    if (not all(math.isfinite(v) for v in (now, local_z, ceiling)) or
            not 0. <= now-cue.get('sample_s', -1e18) <= .35 or
            not 0. <= now-cue.get('started', now) < 6. or
            now >= cue.get('until', now)):
        return None
    image = cue.get('image')
    if (image is None or not image.get('height_reimage_ok') or
            image.get('confidence', 0.) < .65):
        return None
    goal = cue.get('observation_height')
    if goal is not None:
        return min(ceiling, goal)
    sin_down, height = image.get('foot_sin_down', 0.), image.get('camera_height_m', 0.)
    if (cue.get('xy') is not None or not .18 <= sin_down < .30 or
            not 1.5 <= height <= 5.5 or local_z >= ceiling-.1):
        return None
    # Use the measured ray only to improve the viewpoint, never as actor XY.
    delta = min(.8, max(.3, height*(.32/sin_down-1.)))
    goal = min(ceiling, local_z+delta)
    cue['observation_height'] = goal
    return goal


def candidate_reposition(cue, now, yaw, xy, scan, scan_stamp, grid):
    """A short side look when forward approach is blocked, using local sensors.

    Never infer a person's depth. Both the side laser sector and one metre of
    inflated-map path must be clear. Retain the chosen side for at most 1.5 s;
    new images cannot renew it. Final flight safety still owns the output.
    """
    if candidate_decision(cue, now, yaw, xy, scan, scan_stamp)[2] != 'obstacle':
        return 0., 0., 'reposition_unavailable'
    if grid is None:
        return 0., 0., 'reposition_no_map'
    episode = cue.get('reposition')
    if episode is not None and now-episode[1] >= 1.5:
        return 0., 0., 'reposition_budget'
    choices = []
    for sign in ((episode[0],) if episode is not None else (1, -1)):
        heading = cue['bearing']+sign*math.pi/2.
        distances = []
        try:
            for i, raw in enumerate(scan.ranges):
                angle = yaw+scan.angle_min+i*scan.angle_increment
                delta = math.atan2(math.sin(angle-heading), math.cos(angle-heading))
                if abs(delta) > math.radians(20.):
                    continue
                value = float(raw)
                if math.isinf(value) and value > 0.:
                    value = scan.range_max
                if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
                    distances = []
                    break
                distances.append(value)
            if len(distances) < 3 or min(distances) < 3.:
                continue
            if any(not grid.is_free(grid.world_to_cell(
                    (xy[0]+d*math.cos(heading), xy[1]+d*math.sin(heading))))
                   for d in (.2, .4, .6, .8, 1.)):
                continue
        except (AttributeError, TypeError, ValueError):
            continue
        choices.append((min(distances), sign, heading))
    if not choices:
        return 0., 0., 'reposition_blocked'
    _, sign, heading = max(choices)
    if episode is None:
        cue['reposition'] = (sign, now)
    return .35*math.cos(heading), .35*math.sin(heading), 'side_reobserve'


class ImagePublishClock:
    """Image-time cadence without phase drift or repeat/coast publications."""
    def __init__(self, hz):
        self.period = 1. / max(.1, float(hz))
        self.next_due = None
        self.last_sample = -float('inf')

    def ready(self, sample):
        if not math.isfinite(sample) or sample <= self.last_sample:
            return False
        self.last_sample = sample
        if self.next_due is None:
            self.next_due = sample
        if sample+1e-7 < self.next_due:
            return False
        steps = max(1, int(math.floor((sample-self.next_due+1e-7)/self.period))+1)
        self.next_due += steps*self.period
        return True
