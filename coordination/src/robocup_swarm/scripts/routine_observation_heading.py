"""Admit a search-only camera heading using actual pose and measured rays.

This helper changes no motion, coverage or person state. The proposed sector
check is a conservative admission test on the current scan, not an assertion
that the future scan will be clear. The flight executor must still run its
ordinary radar and final direction/braking checks throughout the yaw change.
"""
import math


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _fresh(now, sample, maximum=.25):
    return math.isfinite(sample) and 0. <= now-sample <= maximum


def _sector(scan, direction, required):
    """Require angular support, three spread finite returns and no hidden gap."""
    half, spread = math.radians(20.), math.radians(15.)
    amin, step = float(scan.angle_min), float(scan.angle_increment)
    rmin, rmax = float(scan.range_min), float(scan.range_max)
    beams = scan.ranges
    amax = amin+step*(len(beams)-1)
    if not amin+half <= direction <= amax-half:
        return False
    angles, finite = [], []
    nearest = rmax
    for i, raw in enumerate(beams):
        beam = amin+i*step
        delta = beam-direction
        if abs(delta) > half+1e-9:
            continue
        value = float(raw)
        if value == math.inf:
            angles.append(delta)  # Known no return, never a finite proof ray.
        elif math.isfinite(value) and rmin <= value <= rmax:
            angles.append(delta)
            finite.append(delta)
            nearest = min(nearest, value*math.cos(delta))
        # NaN, negative infinity and invalid short/long returns leave a hole.
    if (len(finite) < 3 or min(finite) > -spread+1e-9 or max(finite) < spread-1e-9 or
            not any(abs(a) <= math.radians(5.)+1e-9 for a in finite)):
        return False
    angles = [-half]+sorted(angles)+[half]
    if max(b-a for a,b in zip(angles,angles[1:])) > max(math.radians(2.5), 2.1*step):
        return False
    return nearest >= required


def routine_observation_heading(route_heading, own_xy, look_xy, actor_bounds, *,
        purpose, task_type, own_yaw, own_sample_s, camera_pose,
        scan, scan_sample_s, now, velocity, measured_velocity,
        attention_active=False, brake_delay=.25, decel=.8, margin=2.,
        yaw_rate_limit=1.5):
    """Return ``(heading, reason)``; rejected evidence keeps the route heading.

    camera_pose is the existing calibrated fixed-camera processed tuple
    (sample, x, y, yaw, height, down_pitch). measured_velocity is the existing
    MAVROS (sample, (vx, vy)) tuple. Commands never substitute for these data.
    At most 90 degrees from actual requested translation is admitted.
    """
    fallback = route_heading
    if (task_type != 0 or purpose not in ('south_patrol', 'coverage_view') or
            attention_active):
        return fallback, 'person_or_nonroutine'
    try:
        x,y = map(float, own_xy)
        lx,ly = map(float, look_xy)
        xmin,xmax,ymin,ymax = map(float, actor_bounds)
        sample,cx,cy,cyaw,height,pitch = map(float, camera_pose)
        measured_s, measured = measured_velocity
        mvx,mvy = map(float, measured)
        vx,vy = map(float, velocity)
        values = (route_heading,x,y,lx,ly,xmin,xmax,ymin,ymax,own_yaw,
                  own_sample_s,sample,cx,cy,cyaw,height,pitch,scan_sample_s,
                  now,measured_s,mvx,mvy,vx,vy,brake_delay,decel,margin,yaw_rate_limit)
        if not all(math.isfinite(v) for v in values):
            return fallback, 'invalid_evidence'
        if not (xmin < xmax and ymin < ymax and xmin <= lx <= xmax and ymin <= ly <= ymax):
            return fallback, 'look_outside_actor_area'
        if (not all(_fresh(now,s) for s in (own_sample_s,scan_sample_s,measured_s)) or
                not _fresh(now,sample,.5)):
            return fallback, 'stale_actual_evidence'
        image_age = now-sample
        proof_speed = max(math.hypot(vx,vy),math.hypot(mvx,mvy))
        # Calibrated image pose is old by construction; lawful translation
        # and yaw during inference must not alternate route/look each frame.
        # The desired heading below always uses current own XY + public look.
        if (height <= 0. or yaw_rate_limit <= 0. or
                math.hypot(cx-x,cy-y) > proof_speed*image_age+.25 or
                abs(_angle(cyaw-own_yaw)) > yaw_rate_limit*image_age+.05):
            return fallback, 'camera_pose_mismatch'
        amin,step,rmin,rmax = (float(getattr(scan,k)) for k in
                              ('angle_min','angle_increment','range_min','range_max'))
        count = len(scan.ranges)
        if (not all(math.isfinite(v) for v in (amin,step,rmin,rmax)) or
                count < 3 or not 0. < step <= math.radians(2.5) or
                not 0. <= rmin < rmax or step*(count-1) > 2.*math.pi+.02):
            return fallback, 'invalid_scan_geometry'
        if brake_delay < 0. or not 0. < decel <= .8 or margin < 0.:
            return fallback, 'invalid_braking_proof'
        speed = math.hypot(vx,vy)
        if speed < .05 or math.hypot(lx-x,ly-y) < .05:
            return fallback, 'not_travelling_or_no_look_direction'
        movement = math.atan2(vy,vx)
        desired = math.atan2(ly-y,lx-x)
        offset = max(-math.pi/2., min(math.pi/2., _angle(desired-movement)))
        heading = _angle(movement+offset)
        required = max(2.,margin)+proof_speed*(brake_delay+.25)+proof_speed**2/(2.*decel)
        current_direction = _angle(movement-own_yaw)
        proposed_direction = _angle(movement-heading)
        if not (_sector(scan,current_direction,required) and
                _sector(scan,proposed_direction,required)):
            return fallback, 'motion_or_proposed_sector_unproven'
        return heading, 'measured_routine_look'
    except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
        return fallback, 'invalid_evidence'
