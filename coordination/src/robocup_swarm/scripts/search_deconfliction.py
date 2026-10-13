"""Search workload separation, using leased views rather than actor positions.

This ranks search proposals only. It neither reserves cells, credits images,
excludes a region, nor replaces the aircraft's collision avoidance.
"""
import math


def finite_point(point):
    return (isinstance(point, (tuple, list)) and len(point) == 2 and
            all(isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) for value in point))


def crowding_cost(point, look, layouts, uid, flight_radius=14., look_radius=10.5):
    """Worst nearby active search commitment; zero means no local conflict.

    Aircraft position and every uncompleted flight/look pair matter, even
    when a camera has already covered its reserved cell. Person tasks are
    deliberately absent from this manager-provided search-only snapshot.
    """
    cost = 0.
    for other, layout in layouts.items():
        if other == uid:
            continue
        positions = list(layout.get('points', ()))
        position = layout.get('position')
        if finite_point(position):
            positions.append(position)
        if finite_point(point) and flight_radius > 0.:
            for planned in positions:
                if finite_point(planned):
                    cost = max(cost, 1.-math.dist(point, planned)/flight_radius)
        if finite_point(look) and look_radius > 0.:
            for planned in layout.get('look_points', ()):
                if finite_point(planned):
                    cost = max(cost, 1.-math.dist(look, planned)/look_radius)
    return max(0., cost)
