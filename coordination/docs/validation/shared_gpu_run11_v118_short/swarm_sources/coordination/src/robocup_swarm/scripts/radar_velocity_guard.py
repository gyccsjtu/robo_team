"""ROS-free final planar velocity guard; contract radar_velocity_contract_v1.md."""
import math


def _result(velocity, reason, clearance=None):
    return dict(schema_version=1, velocity_xy=tuple(velocity),
                reason=reason, clearance_m=clearance)


def guard_velocity(request, measured, yaw, ranges, angle_min, angle_increment,
                   range_min, range_max, scan_s, now_s, max_age=.5,
                   radius=1.2, latency=.5, brake_accel=.5):
    """Only reduce request along its direction; stop on missing/unsafe evidence.

    ENU measured velocity covers momentum even when the requested direction
    changes. Laser angles are in the body frame. This is a planar constraint.
    """
    stop = lambda reason: _result((0., 0.), reason)
    try:
        if len(request) != 2 or len(measured) != 2:
            return stop('INVALID_INPUT')
        values = (*request, *measured, yaw, angle_min, angle_increment,
                  range_min, range_max, scan_s, now_s, max_age,
                  radius, latency, brake_accel)
        if not all(math.isfinite(x) for x in values):
            return stop('INVALID_INPUT')
        if (radius <= 0 or brake_accel <= 0 or latency < 0 or max_age <= 0
                or range_min < 0 or range_max <= range_min
                or angle_increment <= 0 or angle_increment > math.pi / 2):
            return stop('INVALID_INPUT')
        if now_s < scan_s or now_s - scan_s > max_age:
            return stop('STALE_SCAN')
        if ranges is None or len(ranges) < 4:
            return stop('MISSING_SCAN')
        span = (len(ranges) - 1) * angle_increment
        if span < 2 * math.pi - 1.5 * angle_increment or span > 2 * math.pi + 2 * angle_increment:
            return stop('INCOMPLETE_SCAN')
        hits = []
        for i, raw in enumerate(ranges):
            r = float(raw)
            if math.isnan(r) or r == -math.inf or r < range_min:
                return stop('UNKNOWN_RANGE')
            if r == math.inf or r > range_max:
                continue
            angle = yaw + angle_min + i * angle_increment
            hits.append((r * math.cos(angle), r * math.sin(angle),
                         r * math.sin(angle_increment / 2)))
    except (TypeError, ValueError, OverflowError):
        return stop('INVALID_INPUT')

    def corridor_clearance(velocity):
        speed = math.hypot(*velocity)
        if speed <= 1e-9:
            return range_max - radius
        ux, uy = velocity[0] / speed, velocity[1] / speed
        clear = max(0., range_max - radius)
        for x, y, beam_width in hits:
            along, across = x * ux + y * uy, abs(x * uy - y * ux)
            inflated = radius + beam_width
            if across <= inflated:
                half_chord = math.sqrt(max(0., inflated ** 2 - across ** 2))
                if along + half_chord >= 0:
                    clear = min(clear, max(0., along - half_chord))
        return clear

    actual_speed = math.hypot(*measured)
    actual_clear = corridor_clearance(measured)
    if actual_speed * latency + actual_speed ** 2 / (2 * brake_accel) > actual_clear:
        return _result((0., 0.), 'BRAKING_REQUIRED', actual_clear)
    speed = math.hypot(*request)
    if speed <= 1e-9:
        return _result((0., 0.), 'REQUESTED_STOP', actual_clear)
    clear = corridor_clearance(request)
    limit = max(0., math.sqrt((brake_accel * latency) ** 2 + 2 * brake_accel * clear)
                - brake_accel * latency)
    scale = min(1., limit / speed)
    return _result((request[0] * scale, request[1] * scale),
                   'CLEAR' if scale == 1. else 'SLOW' if scale > 0 else 'BLOCKED', clear)
