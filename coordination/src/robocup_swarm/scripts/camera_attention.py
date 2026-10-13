"""Bounded image-only approach. Never supplies an actor/world coordinate."""
import math


def attention_expired(cue, now):
    """An extended pixel-only approach requires continuing image evidence.

    Two seconds without the bound candidate ends this exploratory look;
    fresh frames cannot extend the original absolute deadline. Localized
    dispatch holds retain their separate budget. This is no identity verdict.
    """
    return (now >= cue['until'] or
            (cue.get('approach_started', False) and cue.get('xy') is None and
             now - cue['sample_s'] >= 2.0))


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
        return dict(box=box, width=w, height=h, camera_tilt=tilt)
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


def pixel_backoff(view, now, yaw, xy, scan, scan_stamp):
    """Bound cropped-foot recovery to <=1m/6s using a fresh rear laser sector.

    The existing, uniquely bound image recovery supplies the direction only.
    No depth, target position, report permission or renewed deadline is made.
    Final flight guards remain responsible for executing this preference.
    """
    try:
        stamp, anchor, bearing, tilt = (float(view[k]) for k in
            ('sample_s', 'position_sample_s', 'bearing', 'camera_tilt'))
        x1, y1, x2, y2 = map(float, view['box'])
        width, height = float(view['width']), float(view['height'])
        origin = view['recovery_origin']
        distance = math.hypot(xy[0]-origin[0], xy[1]-origin[1])
        values = (now, yaw, scan_stamp, stamp, anchor, bearing, tilt,
                  x1, y1, x2, y2, width, height, *xy, *origin, distance)
        if (not all(math.isfinite(v) for v in values) or
                not view.get('image_recovery') or not view.get('geometry_anchor_ok') or
                view.get('geometry_ok') or view.get('report_ok') or
                not 0. <= now-stamp <= .35 or not .5 <= now-anchor < 6. or
                abs(tilt) > math.radians(8.) or
                not 6. <= x1 < x2 <= width-6. or
                not 0. <= y1 < y2 <= height+2. or y2-y1 < 12. or
                y2 < .94*height or width <= 0. or height <= 0.):
            return 0., 0., 'backoff_ineligible_image'
        if distance >= .95:
            return 0., 0., 'backoff_travel_budget'
        error = math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))
        if abs(error) > math.radians(12.):
            return 0., 0., 'backoff_turn_first'
        if scan is None or not 0. <= now-scan_stamp <= .3:
            return 0., 0., 'backoff_stale_scan'
        heading = bearing+math.pi
        clearance, angles = [], []
        if (not all(math.isfinite(v) for v in
                    (scan.range_max, scan.angle_min, scan.angle_increment)) or
                scan.range_max < 3. or abs(scan.angle_increment) < 1e-8):
            return 0., 0., 'backoff_invalid_scan'
        for index, raw in enumerate(scan.ranges):
            ray = yaw+scan.angle_min+index*scan.angle_increment
            delta = math.atan2(math.sin(ray-heading), math.cos(ray-heading))
            if abs(delta) > math.radians(25.):
                continue
            value = float(raw)
            if math.isinf(value) and value > 0.:
                value = scan.range_max
            if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
                return 0., 0., 'backoff_invalid_scan'
            clearance.append(value)
            angles.append(delta)
        if (len(clearance) < 3 or min(angles) > -math.radians(20.) or
                max(angles) < math.radians(20.)):
            return 0., 0., 'backoff_insufficient_scan'
        if min(clearance) < 3.:
            return 0., 0., 'backoff_rear_obstacle'
        speed = min(.6, max(0., 1.-distance))
        return speed*math.cos(heading), speed*math.sin(heading), 'cropped_backoff'
    except (KeyError, AttributeError, TypeError, ValueError, IndexError, OverflowError):
        return 0., 0., 'backoff_invalid_input'


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
