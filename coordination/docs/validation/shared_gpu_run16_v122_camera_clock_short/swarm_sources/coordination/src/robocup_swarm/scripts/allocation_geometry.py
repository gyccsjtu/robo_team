"""Pure conservative straight-leg screening for auction candidates.

This rejects obvious conflicts; it does not authorize an actual planned route.
"""
import math


def point_segment_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    t = max(0., min(1., ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2)) if length2 else 0.
    return math.hypot(point[0]-start[0]-t*dx, point[1]-start[1]-t*dy)


def segment_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if ab_c*ab_d < 0 and cd_a*cd_b < 0:
        return 0.
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def screen_leg(start, end, peer_positions, peer_legs, separation=4.5):
    try:
        points = [start, end] + list(peer_positions) + [p for leg in peer_legs for p in leg]
        if separation <= 0 or not math.isfinite(separation) or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            return False
        if any(point_segment_distance(p, start, end) <= separation for p in peer_positions):
            return False
        if any(segment_distance(start, end, leg[0], leg[1]) <= separation for leg in peer_legs):
            return False
        return True
    except (TypeError, ValueError, IndexError):
        return False
