"""Routine aircraft viewpoints stay inside the public actor activity area.

Observed person positions and official reports are deliberately not accepted
as inputs to this module. They must never be clipped to aircraft boundaries.
"""
import math
from swarm_task import ACTOR_SEARCH_BOUNDS

ROUTINE_INSET = 1.


def routine_point_allowed(point, look=None):
    xmin, xmax, ymin, ymax = ACTOR_SEARCH_BOUNDS
    try:
        x, y = point
        if (not all(math.isfinite(v) for v in point) or
                not xmin+ROUTINE_INSET <= x <= xmax-ROUTINE_INSET or
                not ymin+ROUTINE_INSET <= y <= ymax-ROUTINE_INSET):
            return False
        if look is not None:
            lx, ly = look
            return (all(math.isfinite(v) for v in look) and
                    xmin <= lx <= xmax and ymin <= ly <= ymax)
        return True
    except (ValueError, TypeError):
        return False


def routine_flight_bounds(flight):
    ax0, ax1, ay0, ay1 = ACTOR_SEARCH_BOUNDS
    fx0, fx1, fy0, fy1 = flight
    return max(ax0, fx0), min(ax1, fx1), max(ay0, fy0), min(ay1, fy1)
