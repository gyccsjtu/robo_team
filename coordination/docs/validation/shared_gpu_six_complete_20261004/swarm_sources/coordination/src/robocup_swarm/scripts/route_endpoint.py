"""Preserve the exact task endpoint only through a checked free connector."""
import math


def connect_exact_goal(points, goal, grid):
    result = list(points)
    if not result or len(goal) != 2 or not all(math.isfinite(v) for v in goal):
        return result
    start_cell, end_cell = grid.world_to_cell(result[-1]), grid.world_to_cell(goal)
    # A* returns the centre of its final cell. The requested point can be
    # anywhere inside that cell. Do not bridge to an occupied alternate goal.
    if start_cell is None or start_cell != end_cell or not grid.is_free(end_cell):
        return result
    if math.hypot(result[-1][0] - goal[0], result[-1][1] - goal[1]) > 1e-9:
        result.append(tuple(goal))
    return result
