"""Image-space tracking feedback. No judge state and no target-coordinate output."""
import math


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
            confidence < .45 or confidence > 1. or int(data['hits']) < 3):
        raise ValueError('stale or invalid tracking image')
    return dict(sample_s=sample, target_id=tid, track_id=int(data['track_id']),
                width=width, height=height, fy=fy, box=(x1, y1, x2, y2),
                bearing=bearing, x=x, y=y, camera_tilt=tilt,
                candidate_id=data.get('candidate_id'), position_sample_s=sample,
                geometry_ok=data.get('geometry_ok') is True)


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


def retain_pixel_view(previous, data, source, tid, now):
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
                not 0. <= now-anchor <= 2.):
            return None
        payload = dict(data, target_id=tid, track_id=previous['track_id'],
                       x=previous['x'], y=previous['y'], fy=previous['fy'],
                       geometry_ok=False)
        view = parse_tracking_view(payload,source,tid,now)
        if (view['width'] != previous['width'] or view['height'] != previous['height'] or
                abs(math.atan2(math.sin(view['bearing']-previous['bearing']),
                               math.cos(view['bearing']-previous['bearing']))) > .25):
            return None
        view.update(position_sample_s=anchor,pixel_only=True,
                    altitude_goal=previous.get('altitude_goal'),
                    association_error=previous.get('association_error',float('inf')))
        return view
    except (KeyError,ValueError,TypeError,OverflowError):
        return None


def image_altitude_goal(view, image_local_z, range_m, minimum, maximum):
    """Small altitude correction toward a full-body image with border margin.

    Compute once per fresh image using its captured/received altitude. The
    control loop must not integrate the same image error twenty times/sec.
    A fixed forward camera cannot put a ground person at exact image centre
    at every legal altitude; use an interior band and preserve both ends.
    """
    _, top, _, bottom = view['box']
    h, fy = view['height'], view['fy']
    # Only protect head/foot margins. A full body anywhere in the interior
    # is preferable to climbing just because its centre passed a narrow band.
    # In conflicting cases a
    # box taller than the usable frame needs more distance, not altitude.
    if bottom-top > .8*h:
        return min(maximum, max(minimum, image_local_z))
    error = max(bottom-.90*h, 0.)
    if error == 0.:
        error = min(top-.10*h, 0.)
    correction = max(-.35, min(.35, -.6*min(12., max(2., range_m))*error/fy))
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
    proposed = image_altitude_goal(view, goal, range_m, minimum, maximum)
    delta = proposed-goal
    direction = (1 if delta > .02 else -1 if delta < -.02 else 0)
    if abs(view.get('camera_tilt', math.pi)) > math.radians(8.):
        direction = 0  # pitch/roll during acceleration is not a height error
    since = previous['pressure_since']
    if direction != previous['direction'] or dt > .5:
        since = sample
    if direction and sample-since >= .6:
        step = min(abs(delta), .15*min(max(dt, 0.), .25))
        goal += direction*step
        goal = min(altitude+.25, max(altitude-.25, goal))
    return dict(target_id=view['target_id'], sample_s=sample,
                goal=min(maximum, max(minimum, goal)),
                direction=direction, pressure_since=since)


def view_needs_space(view):
    _, top, _, bottom = view['box']
    h = view['height']
    return bottom >= .94*h or top <= .04*h or bottom-top > .8*h
