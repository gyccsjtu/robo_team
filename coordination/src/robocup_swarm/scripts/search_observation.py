"""Shared camera footprint used by search motion and coverage accounting."""
import math


def cell_points(grid, cell):
    half = grid.cell_m * .5
    return [(max(grid.x_min+.01, min(grid.x_max-.01, cell.cx+(ix-1)*(half-.01))),
             max(grid.y_min+.01, min(grid.y_max-.01, cell.cy+(iy-1)*(half-.01))))
            for iy in range(3) for ix in range(3)]


def in_view(ux, uy, yaw, px, py, altitude=None, down_pitch=None):
    distance = math.hypot(px-ux, py-uy)
    if not .5 <= distance <= 9.0:
        return False
    if altitude is not None and down_pitch is not None:
        if not math.isfinite(altitude) or not math.isfinite(down_pitch) or altitude <= 0.:
            return False
        if abs(math.atan2(altitude, distance)-down_pitch) > math.radians(27.):
            return False
    error = math.atan2(py-uy, px-ux)-yaw
    return abs(math.atan2(math.sin(error), math.cos(error))) <= math.radians(45.)


def observable_points(points, ux, uy, altitude, down_pitch, visible):
    """Points reachable by yaw alone at this position, with sensed LOS."""
    return {i: p for i, p in enumerate(points)
            if in_view(ux, uy, math.atan2(p[1]-uy, p[0]-ux),
                       *p, altitude, down_pitch) and visible(ux, uy, *p)}


def next_heading(points, ux, uy, yaw):
    """Choose one view covering most remaining points, then the smallest turn."""
    if not points:
        return None
    candidates = []
    for px, py in points.values():
        desired = math.atan2(py-uy, px-ux)
        count = sum(in_view(ux, uy, desired, *p) for p in points.values())
        turn = abs(math.atan2(math.sin(desired-yaw), math.cos(desired-yaw)))
        candidates.append((-count, turn, desired))
    return min(candidates)[2]
