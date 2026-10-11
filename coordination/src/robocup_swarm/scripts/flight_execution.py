"""Execution constraints that only shorten an already chosen horizontal step."""
import math


def bounded_step(position, step, bounds, inset=0.0):
    """Clip along the requested ray; never turn a stop into boundary recovery.

    A position outside the operational inset may hold or move inward. Recovery
    is a separately planned, sensor-checked command, never a position clamp.
    """
    x, y = position
    dx, dy = step
    if not all(math.isfinite(v) for v in (x, y, dx, dy, *bounds, inset)):
        return 0.0, 0.0
    xmin, xmax, ymin, ymax = bounds
    limits = ((min(x, xmin+inset), max(x, xmax-inset)),
              (min(y, ymin+inset), max(y, ymax-inset)))
    scale = 1.0
    for p, d, (lo, hi) in zip((x, y), (dx, dy), limits):
        if d > 0.0:
            scale = min(scale, max(0.0, (hi-p)/d))
        elif d < 0.0:
            scale = min(scale, max(0.0, (lo-p)/d))
    return dx*scale, dy*scale
