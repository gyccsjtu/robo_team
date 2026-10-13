"""Constrain complete search paths, while permitting a checked inward return.

This adapter masks only public flight limits over the existing sensed grid.
It neither changes occupancy evidence nor handles person/report coordinates.
"""
import math
from robocup_navigation.astar import PlanResult, plan
from routine_search import routine_point_allowed, ROUTINE_INSET
from swarm_task import ACTOR_SEARCH_BOUNDS
from search_segments import clear_line


class RoutinePlanningGrid:
    def __init__(self, grid):
        self.grid = grid

    def __getattr__(self, name):
        return getattr(self.grid, name)

    def world_to_cell(self, point):
        return self.grid.world_to_cell(point) if routine_point_allowed(point) else None

    def is_free(self, cell):
        if cell is None or not self.grid.is_free(cell):
            return False
        point = self.cell_to_world(cell)
        return routine_point_allowed(point) and self.grid.world_to_cell(point) == cell

    def cell_to_world(self, cell):
        # An exact legal edge can lie in a grid cell whose centre is outside.
        # Keep only that cell's legal portion; never move evidence to a
        # neighbouring cell or turn an occupied cell into a free one.
        x, y = self.grid.cell_to_world(cell)
        xmin, xmax, ymin, ymax = ACTOR_SEARCH_BOUNDS
        return (min(xmax-ROUTINE_INSET, max(xmin+ROUTINE_INSET, x)),
                min(ymax-ROUTINE_INSET, max(ymin+ROUTINE_INSET, y)))


def plan_routine(grid, start, goal, **options):
    """No outside detour. An outside start can only return by a checked line.

    A prior person task may finish beyond the routine search box. Such a
    start does not authorize exploration there: select a nearby free inward
    entry, check the entire connection, then use bounded A* for the remainder.
    If no checked connection exists, normal task failure/replanning applies.
    """
    try:
        valid = all(len(point) == 2 and all(not isinstance(value, bool) and
                    math.isfinite(value) for value in point) for point in (start, goal))
    except (TypeError, ValueError):
        valid = False
    if not valid:
        return PlanResult(False, 'INVALID_POINT', (), (), 0., 0, None, None)
    bounded = RoutinePlanningGrid(grid)
    if routine_point_allowed(start):
        return plan(bounded, start, goal, **options)
    start_cell, goal_cell = grid.world_to_cell(start), bounded.world_to_cell(goal)
    def failure(reason):
        return PlanResult(False, reason, (), (), 0., 0, start_cell, goal_cell)
    if goal_cell is None or not bounded.is_free(goal_cell):
        return failure('GOAL_OUT_OF_BOUNDS' if goal_cell is None else 'GOAL_OCCUPIED')
    if start_cell is None or not grid.is_free(start_cell):
        return failure('START_OUT_OF_BOUNDS' if start_cell is None else 'START_OCCUPIED')
    radius = int(math.ceil(20./grid.resolution))
    entries = []
    for ix in range(max(0, start_cell[0]-radius), min(grid.width, start_cell[0]+radius+1)):
        for iy in range(max(0, start_cell[1]-radius), min(grid.height, start_cell[1]+radius+1)):
            cell = (ix, iy)
            if bounded.is_free(cell):
                point = bounded.cell_to_world(cell)
                distance = math.dist(start, point)
                if distance <= 20.:
                    entries.append((distance, point))
    for distance, entry in sorted(entries)[:32]:
        if not clear_line(grid, start, entry):
            continue
        route = plan(bounded, entry, goal, **options)
        if route.success:
            return PlanResult(True, route.reason, (start_cell,)+route.cells,
                              (tuple(start),)+route.points,
                              distance+route.path_length_m, route.expanded_nodes,
                              start_cell, route.goal_cell)
    return failure('NO_CHECKED_INWARD_ENTRY')
