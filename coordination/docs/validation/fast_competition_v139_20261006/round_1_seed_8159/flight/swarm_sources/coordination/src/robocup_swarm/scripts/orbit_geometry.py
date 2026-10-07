"""Choose an orbit planning endpoint; this never grants a flight segment."""
import math


def orbit_goal(position, center, radius, chord_m=0., velocity_candidates=(), target_velocity=None):
    if (len(position) != 2 or len(center) != 2
            or not all(math.isfinite(v) for v in (*position, *center, radius, chord_m))
            or radius <= 0. or not 0. <= chord_m <= 2.*radius):
        raise ValueError('Invalid orbit geometry')
    dx, dy = position[0]-center[0], position[1]-center[1]
    distance, bearing = math.hypot(dx, dy), math.atan2(dy, dx)
    if chord_m == 0.:
        step = .2  # Preserve the existing general-purpose endpoint exactly.
    else:
        cosine = ((distance*distance+radius*radius-chord_m*chord_m)/(2.*distance*radius)
                  if distance > 1e-9 else -1.)
        step = math.acos(max(-1., min(1., cosine)))
    points = [(center[0]+radius*math.cos(bearing+sign*step),
               center[1]+radius*math.sin(bearing+sign*step)) for sign in (1., -1.)]
    if chord_m == 0.:
        return points[0]
    directions = []
    for velocity in velocity_candidates:
        if len(velocity) != 2 or not all(math.isfinite(v) for v in velocity):
            raise ValueError('Invalid measured orbit direction')
        speed = math.hypot(*velocity)
        if speed > .15:
            directions.append(tuple(v/speed for v in velocity))
    if not directions and target_velocity is not None:
        if len(target_velocity) != 2 or not all(math.isfinite(v) for v in target_velocity):
            raise ValueError('Invalid target orbit direction')
        speed = math.hypot(*target_velocity)
        if speed > .15:
            directions.append(tuple(v/speed for v in target_velocity))
    if not directions:
        return points[0]

    def alignment(point):
        delta = (point[0]-position[0], point[1]-position[1])
        length = max(math.hypot(*delta), 1e-9)
        return min(sum(a*b for a, b in zip(delta, direction))/length for direction in directions)
    return max(points, key=alignment)
