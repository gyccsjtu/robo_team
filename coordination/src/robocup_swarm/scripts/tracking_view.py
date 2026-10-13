"""Image-space tracking feedback. No judge state and no target-coordinate output."""
import math


class ReportViewWindow:
    """Own-camera quality over capture time; never grants reporting eligibility."""
    def __init__(self):
        self.identity=None
        self.samples=[]

    def observe(self, view):
        identity=(view['target_id'],view.get('world_candidate_id') or view.get('track_id'))
        sample=view['sample_s']
        if identity != self.identity:
            self.identity=identity
            self.samples=[]
        if self.samples and sample <= self.samples[-1][0]:
            return
        good=bool(view.get('geometry_ok') and view.get('report_ok') and
                  view.get('person_verdict_ok', True))
        self.samples=[p for p in self.samples if sample-p[0] <= 4.]+[(sample,good)]
        self.samples=self.samples[-160:]

    def degraded(self, tid, now):
        points=self.samples
        if (not points or self.identity[0]!=tid or
                not 0. <= now-points[-1][0] <= .5 or
                points[-1][0]-points[0][0] < 3.-1e-6):
            return False
        duration=good_duration=0.
        for a,b in zip(points,points[1:]):
            dt=b[0]-a[0]
            if dt > .7:
                return False  # Actual image loss uses the existing lost-view watchdog.
            duration+=dt
            good_duration+=dt*int(a[1])
        return duration>0. and good_duration/duration < .60

    def recovered(self, tid, now):
        """End reimaging after sustained good images, never one lucky frame."""
        if (not self.samples or self.identity[0] != tid or
                not 0. <= now-self.samples[-1][0] <= .5):
            return False
        recent = []
        for point in reversed(self.samples):
            if not point[1] or (recent and recent[-1][0]-point[0] > .7):
                break
            recent.append(point)
        return len(recent) >= 3 and recent[0][0]-recent[-1][0] >= .6-1e-6


def box_from_foot(uv, wh):
    """Undo perception's 5 percent foot inset; uv[1] is NOT box centre."""
    u, v = uv
    w, h = wh
    return [u-w/2., v-.95*h, u+w/2., v+.05*h]


def parse_tracking_view(data, source, tid, now):
    if (data.get('source_uav') != source or data.get('target_id') != tid or
            data.get('real_detection') is not True):
        raise ValueError('view is not from the assigned own-camera track')
    sample = float(data['sample_s'])
    width, height, fy = (float(data[k]) for k in ('width', 'height', 'fy'))
    x1, y1, x2, y2 = map(float, data['box'])
    bearing, confidence = float(data['bearing']), float(data['confidence'])
    tilt = float(data.get('camera_tilt', math.pi))
    x, y = float(data['x']), float(data['y'])
    if (not all(math.isfinite(v) for v in
                (sample, width, height, fy, x1, y1, x2, y2, bearing, confidence, x, y, tilt)) or
            not 0.0 <= now-sample <= .5 or sample <= 0. or
            width <= 0 or height <= 0 or fy <= 0 or
            not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height) or
            confidence < (.30 if tid == 't3' else .40) or
            confidence > 1. or int(data['hits']) < 3):
        raise ValueError('stale or invalid tracking image')
    return dict(sample_s=sample, target_id=tid, track_id=int(data['track_id']),
                width=width, height=height, fy=fy, box=(x1, y1, x2, y2),
                bearing=bearing, x=x, y=y, camera_tilt=tilt,
                candidate_id=data.get('candidate_id'), position_sample_s=sample,
                geometry_ok=data.get('geometry_ok') is True,
                report_ok=data.get('report_ok') is True,
                person_verdict_ok=data.get('person_verdict_ok', True) is True,
                person_verdict_reason=data.get('person_verdict_reason', ''))


def bound_pixel_candidate(cues, cls, box):
    """Bind a world track to the same-frame image box, never by colour alone."""
    matches = []
    x1,y1,x2,y2 = box
    area = max(0.,x2-x1)*max(0.,y2-y1)
    for cue in cues:
        if not cue['candidate_id'].startswith(cls+':') or 'box' not in cue:
            continue
        a,b,c,d = cue['box']
        inter = max(0.,min(x2,c)-max(x1,a))*max(0.,min(y2,d)-max(y1,b))
        union = area+max(0.,c-a)*max(0.,d-b)-inter
        if union > 0. and inter/union >= .8:
            matches.append(cue['candidate_id'])
    return matches[0] if len(matches) == 1 else None


def bind_red_view(view, evidence, assigned_tid):
    """Use the bridge's own-camera, same-image mapping, never red slot names.

    The caller validates source and freshness before supplying evidence.
    Coordinate tolerance accounts only for the report's centimetre rounding;
    it is not a proximity rule for merging two nearby red people.
    """
    if (evidence is None or assigned_tid not in ('t4', 't5') or
            view['target_id'] not in ('t4', 't5') or
            evidence['target_id'] != assigned_tid or
            evidence['local_target_id'] != view['target_id'] or
            abs(evidence['sample_s']-view['sample_s']) > 1e-6 or
            math.hypot(evidence['x']-view['x'], evidence['y']-view['y']) > .02):
        return None
    return dict(view, local_target_id=view['target_id'], target_id=assigned_tid)


def bind_red_instance_view(view, instance, binding, assigned_tid, now):
    """Continue an already mapped camera track between coordination reports.

    The immutable session/track ID, not its red slot or spatial proximity,
    supplies identity. This only controls the camera and never refreshes the
    bridge binding, target position cache, or official observation evidence.
    """
    if (not instance or binding is None or assigned_tid not in ('t4', 't5') or
            view['target_id'] not in ('t4', 't5') or
            instance != binding.get('candidate_id') or
            not 0. <= now-binding['sample_s'] <= 1. or
            not 0. < view['sample_s']-binding['sample_s'] <= .5):
        return None
    return dict(view, local_target_id=view['target_id'], target_id=assigned_tid)


def retain_pixel_view(previous, data, source, tid, now, max_position_age=2.):
    """At most 2s of yaw-only continuity for an explicitly bound image track.

    Position time never advances. No new association, localization, report or
    height correction is obtained from this fallback.
    """
    try:
        if (previous is None or previous['target_id'] != tid or
                not previous.get('candidate_id') or
                data.get('candidate_id') != previous['candidate_id'] or
                data.get('source_uav') != source or data.get('real_detection') is not True):
            return None
        sample = float(data['sample_s'])
        anchor = previous.get('position_sample_s',previous['sample_s'])
        if (not 0. < sample-previous['sample_s'] <= .5 or
                not 0. <= now-anchor <= max_position_age):
            return None
        payload = dict(data, target_id=tid, track_id=previous['track_id'],
                       x=previous['x'], y=previous['y'], fy=previous['fy'],
                       geometry_ok=False, report_ok=False)
        view = parse_tracking_view(payload,source,tid,now)
        if (view['width'] != previous['width'] or view['height'] != previous['height'] or
                abs(math.atan2(math.sin(view['bearing']-previous['bearing']),
                               math.cos(view['bearing']-previous['bearing']))) > .25):
            return None
        view.update(position_sample_s=anchor,pixel_only=True,
                    geometry_anchor_ok=previous.get('geometry_anchor_ok',
                                                    previous.get('geometry_ok',False)),
                    world_candidate_id=previous.get('world_candidate_id'),
                    altitude_goal=previous.get('altitude_goal'),
                    association_error=previous.get('association_error',float('inf')))
        return view
    except (KeyError,ValueError,TypeError,OverflowError):
        return None


def pixel_recovery_view(previous, data, source, tid, now):
    """Six seconds of unambiguous same-instance images, never new positions.

    The longer budget is solely for restoring observation geometry. An
    ambiguous association, changed pixel ID, image gap or appearance conflict
    cannot acquire it. Its deadline stays anchored to the localized image.
    """
    try:
        if (previous is None or
                not (previous.get('geometry_ok') or previous.get('geometry_anchor_ok') or
                     previous.get('image_recovery')) or
                data.get('association_unique') is not True or
                data.get('appearance_status') != 'actor'):
            return None
        down, fy = float(data['foot_down_sin']), float(data['fy'])
        if not math.isfinite(down) or not -1. <= down <= 1. or fy != previous['fy']:
            return None
        if not previous.get('image_recovery'):
            anchor=previous.get('position_sample_s',previous['sample_s'])
            # Pixel cues precede the same image's localized feedback. That
            # normal callback ordering must never interrupt healthy following.
            # Start only after localization has actually been absent, and only
            # for the shallow/full or cropped-foot problem this control solves.
            _,top,_,bottom=map(float,data['box'])
            height=float(data['height'])
            shallow_full=(0. < down < .30 and top >= .04*height and
                          bottom <= .88*height and bottom-top <= .8*height)
            if now-anchor < .5 or not (shallow_full or bottom >= .94*height):
                return None
        view = retain_pixel_view(previous, data, source, tid, now, max_position_age=6.)
        if view is None:
            return None
        view.update(image_recovery=True, foot_down_sin=down,
                    recovery_until=view['position_sample_s']+6.,
                    recovery_origin=(previous.get('recovery_origin')
                                     if previous.get('image_recovery') else None))
        return view
    except (KeyError, ValueError, TypeError, OverflowError):
        return None


def update_pixel_altitude_hold(previous, view, altitude, minimum, maximum):
    """Improve a bound image's vertical view without estimating actor depth.

    A shallow, complete foot ray can request a slow climb; a bottom crop can
    request a small descent. Pitch/roll transients cannot drive either. The
    absolute episode budget, flight floor and ceiling remain independent of
    how many camera messages arrive.
    """
    sample = view['sample_s']
    anchor = view['position_sample_s']
    if previous is None or previous['target_id'] != view['target_id']:
        previous = dict(target_id=view['target_id'], sample_s=anchor,
                        goal=altitude, direction=0, pressure_since=sample)
    if sample <= previous['sample_s']:
        return previous
    dt = sample-previous['sample_s']
    baseline = previous.get('recovery_altitude', altitude)
    goal = previous['goal']
    _, top, _, bottom = view['box']
    height = view['height']
    if bottom >= .92*height:
        direction = -1
    elif (bottom <= .88*height and top >= .04*height and
          bottom-top <= .8*height and 0. < view['foot_down_sin'] < VIEW_MIN_DOWN_SIN):
        direction = 1
    else:
        direction = 0
    since = previous['pressure_since']
    if abs(view['camera_tilt']) > math.radians(8.):
        since += dt  # tilted frames add no sustained-pressure time
        direction = previous['direction']
    else:
        if direction != previous['direction'] or dt > .5:
            since = sample
        if direction and sample-since >= .6-1e-9:
            goal += direction*.15*min(dt, .25)
    goal = min(maximum, baseline+.6, altitude+.25,
               max(minimum, baseline-.3, altitude-.25, goal))
    return dict(target_id=view['target_id'], sample_s=sample, goal=goal,
                direction=direction, pressure_since=since,
                needs_space=bottom >= .92*height, range_floor=0.,
                recovery_altitude=baseline, recovery_until=anchor+6.)


VIEW_MIN_DOWN_SIN = .33
VIEW_PITCH_RESERVE = math.radians(10.)


def view_height_window(view, range_m, minimum, maximum):
    """Height interval for reliable ground rays and a 10 degree pitch reserve.

    range_m is horizontal distance, not the camera's slant-range estimate.
    An empty interval is a request to regain viewing distance, never permission
    to descend below the flight floor or to relax the perception geometry gate.
    """
    angle = math.atan2(.48*view['height'], view['fy'])-VIEW_PITCH_RESERVE
    upper_ratio = max(0., math.tan(max(0., angle)))
    lower_ratio = VIEW_MIN_DOWN_SIN/math.sqrt(1.-VIEW_MIN_DOWN_SIN**2)
    return (max(minimum, range_m*lower_ratio),
            min(maximum, range_m*upper_ratio))


def view_distance_floor(view, altitude):
    angle = math.atan2(.48*view['height'], view['fy'])-VIEW_PITCH_RESERVE
    return altitude/max(1e-6, math.tan(max(0., angle)))


def image_altitude_goal(view, image_local_z, range_m, minimum, maximum):
    """Small altitude correction toward a full-body image with border margin.

    Compute once per fresh image using its captured/received altitude. The
    control loop must not integrate the same image error twenty times/sec.
    A fixed forward camera cannot put a ground person at exact image centre
    at every legal altitude; use an interior band and preserve both ends.
    """
    _, top, _, bottom = view['box']
    h, fy = view['height'], view['fy']
    lower, upper = view_height_window(view, range_m, minimum, maximum)
    # Incompatible height limits or an oversized box need more space. Keep
    # the current legal height while the existing safe observation planner acts.
    if lower > upper or bottom-top > .8*h:
        return min(maximum, max(minimum, image_local_z))
    bottom_angle = math.atan2(.48*h, fy)-VIEW_PITCH_RESERVE
    bottom_limit = .5*h+fy*math.tan(max(0., bottom_angle))
    error = max(bottom-bottom_limit, 0.)
    if error == 0.:
        error = min(top-.10*h, 0.)
    correction = max(-.35, min(.35, -.6*min(12., max(2., range_m))*error/fy))
    desired = min(upper, max(lower, image_local_z+correction))
    correction = max(-.35, min(.35, desired-image_local_z))
    return min(maximum, max(minimum, image_local_z+correction))


def update_altitude_hold(previous, view, altitude, range_m, minimum, maximum):
    """Persistent absolute height; slow changes only after sustained edge pressure.

    Tilted transient images cannot drive height, and reacquiring a frame does
    not reset the goal to a different geometric/range altitude controller.
    """
    sample = view['sample_s']
    if previous is None or previous['target_id'] != view['target_id']:
        previous = dict(target_id=view['target_id'], sample_s=sample,
                        goal=min(maximum, max(minimum, altitude)),
                        direction=0, pressure_since=sample)
    dt = sample-previous['sample_s']
    if dt < 0.:
        return previous
    goal = previous['goal'] if dt <= 2. else min(maximum, max(minimum, altitude))
    lower, upper = view_height_window(view, range_m, minimum, maximum)
    _, top, _, bottom = view['box']
    height = view['height']
    edge_pressure = view_needs_space(view)
    # In 212319, bottom=480 while needs_space=False: a mathematically feasible
    # height interval was mistaken for an actually complete camera image.
    # Keep some hysteresis so one image just inside the border does not draw
    # the UAV back into the same crop. This affects observation motion only.
    hold_space = (previous.get('needs_space', False) and
                  (bottom > .88*height or top < .08*height))
    needs_space = lower > upper or edge_pressure or hold_space
    range_floor = 0.
    if needs_space and lower <= maximum:
        range_floor = max(view_distance_floor(view, altitude),
                          range_m+.5 if edge_pressure or lower > upper
                          else previous.get('range_floor', range_m))
    proposed = image_altitude_goal(view, goal, range_m, minimum, maximum)
    delta = proposed-goal
    # The image dead band must not leave the goal just outside a hard geometry
    # boundary (e.g. 3.16 m at 8 m still consumes the reserved 2% foot border).
    deadband = .02 if lower <= goal <= upper else 1e-9
    direction = (1 if delta > deadband else -1 if delta < -deadband else 0)
    if abs(view.get('camera_tilt', math.pi)) > math.radians(8.):
        # Pause accumulation on transient pitch/roll; do not erase a
        # persistent correction direction. Bad frames add no pressure time.
        return dict(previous, sample_s=sample, goal=goal,
                    pressure_since=previous['pressure_since']+max(0., dt),
                    needs_space=needs_space,
                    range_floor=range_floor)
    since = previous['pressure_since']
    if direction != previous['direction'] or dt > .5:
        since = sample
    if direction and sample-since >= .6:
        step = min(abs(delta), .15*min(max(dt, 0.), .25))
        goal += direction*step
        goal = min(altitude+.25, max(altitude-.25, goal))
    return dict(target_id=view['target_id'], sample_s=sample,
                goal=min(maximum, max(minimum, goal)),
                direction=direction, pressure_since=since,
                needs_space=needs_space,
                range_floor=range_floor)


def view_needs_space(view):
    _, top, _, bottom = view['box']
    h = view['height']
    return bottom >= .94*h or top <= .04*h or bottom-top > .8*h
