"""One bounded south-search itinerary inside the normal task allocator.

No actor positions, independent controller, reservations or coverage writes.
ENU negative Y identifies the south side of the allowed search region.
"""
import math
from robocup_navigation.astar import plan
from search_segments import clear_line
from swarm_task import STATE_ASSIGNED, STATE_COVERED


class SouthPatrol:
    def __init__(self, flight_bounds, max_hold=90., max_segments=3):
        self.bounds = tuple(flight_bounds)
        self.max_hold = max_hold
        self.max_segments = max_segments
        self.uid = None
        self.deadline = 0.
        self.task_id = None
        self.completed = 0
        self.next_s = 0.
        self.empty_rounds = 0
        self.direction = 0

    def finish(self, now, productive=False):
        self.empty_rounds = 0 if productive else min(3, self.empty_rounds+1)
        self.next_s = now+60.*max(1, self.empty_rounds)
        self.uid = self.task_id = None
        self.completed = self.direction = 0

    def ended(self, task, reason, now):
        if task.task_id != self.task_id:
            return
        self.task_id = None
        if reason == 'complete':
            self.completed += 1
            if self.completed < self.max_segments and now < self.deadline:
                return
        self.finish(now, productive=reason in ('person_task', 'released_for_new_task'))

    def choose(self, manager, idle, now):
        """Return one checked proposal. Manager holds its task lock throughout."""
        if self.task_id is not None or now < self.next_s:
            return None
        if self.uid is not None and (self.uid not in idle or now >= self.deadline):
            self.finish(now)
            return None
        grid = getattr(manager, '_g_infl', None)
        if grid is None:
            return None
        keys = []
        viewpoints = {}
        ymin = self.bounds[2]
        for key, cell in manager.grid.cells.items():
            if (abs(cell.cy-ymin) > manager.grid.cell_m*.5 or
                    key in manager._blocked_cells or key in manager._segments.reserved or
                    cell.state in (STATE_ASSIGNED, STATE_COVERED) or cell.owner is not None or
                    now-manager.allocator.visit_time.get(key, -1e18) < 60. or
                    now-manager.allocator.failed_visit.get(key, -1e18) <
                        manager.allocator.failed_novelty_ttl):
                continue
            point = manager._segment_point(key)
            # A clipped last/first coverage row may be narrower than 7m.
            # Keep an observable standoff instead of flying over its centre.
            point=(point[0],max(point[1],cell.cy+8.))
            # Preserve an interior standoff and observe the cell from there.
            # If the live map cannot supply this viewpoint, leave a blind job
            # to the ordinary allocator rather than camping on a wall.
            if (not point[1] > cell.cy or math.dist(point, (cell.cx, cell.cy)) > 14. or
                    not manager._search_point_free(point)):
                continue
            keys.append(key)
            viewpoints[key]=point
        pairs = []
        for uid in idle:
            if self.uid is not None and uid != self.uid:
                continue
            s = manager.status[uid]
            received = manager.last_report.get(uid)
            if (received is None or not 0. <= now-received.to_sec() < 1. or
                    not all(math.isfinite(v) for v in (s.x, s.y))):
                continue
            for key in keys:
                point = viewpoints[key]
                if self.direction and self.direction*(point[0]-s.x) < -1.:
                    continue
                pairs.append((math.dist((s.x, s.y), point), uid, key, point))
        # Bounded planning work. A straight distance is only a lower bound;
        # select by checked path length amongst these nearest candidates.
        checked = []
        for distance, uid, key, point in sorted(pairs)[:8]:
            if checked and distance > min(row[0] for row in checked)+1.:
                break
            s = manager.status[uid]
            route = plan(grid, (s.x, s.y), point, connectivity=8,
                         prevent_corner_cutting=True, max_expansions=2500)
            if not route.success:
                continue
            route_points = [(s.x, s.y)]+list(route.points)+[point]
            if not all(clear_line(grid, a, b) for a, b in zip(route_points, route_points[1:])):
                continue
            checked.append((route.path_length_m, uid, key, point))
        if not checked:
            if self.uid is not None:
                self.finish(now)
            else:
                self.next_s = now+5.
            return None
        _, uid, key, point = min(checked)
        cells, points = [key], [point]
        directions = [self.direction] if self.direction else [1, -1]
        extensions = []
        for direction in directions:
            ks, ps = list(cells), list(points)
            for _ in range(2):
                nxt = (ks[-1][0]+direction, ks[-1][1])
                if nxt not in keys:
                    break
                p = viewpoints[nxt]
                if not .6*manager.grid.cell_m <= math.dist(ps[-1], p) <= 1.6*manager.grid.cell_m:
                    break
                if not clear_line(grid, ps[-1], p):
                    break
                ks.append(nxt); ps.append(p)
            extensions.append((len(ks), direction, ks, ps))
        _, direction, cells, points = max(extensions)
        return uid, cells, points, direction

    def started(self, task, direction, now):
        if self.uid is None:
            self.uid = task.uid
            self.deadline = now+self.max_hold
            self.completed = 0
        self.task_id = task.task_id
        self.direction = direction
        task.deadline = min(task.deadline, self.deadline)
        task.expires = min(task.expires, task.deadline)
