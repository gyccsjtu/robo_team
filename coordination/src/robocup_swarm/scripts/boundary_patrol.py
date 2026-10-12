"""One bounded directed-search itinerary inside the normal task allocator.

No actor positions, independent controller, reservations or coverage writes.
ENU negative Y identifies south and positive X/Y identifies northeast.
"""
import math
import os
from robocup_navigation.astar import plan
from search_segments import clear_line
from swarm_task import STATE_ASSIGNED, STATE_COVERED


class SouthPatrol:
    def __init__(self, flight_bounds, max_hold=90., max_segments=3,
                 southeast=None, northeast=None):
        self.bounds = tuple(flight_bounds)
        self.max_hold = max_hold
        self.max_segments = max_segments
        # Southeast begins with one inner leg, then visits the south edge.
        # Northeast retains one aircraft for a bounded, productive visit to
        # nearby cells; it never sends that aircraft onward to the south edge.
        self.southeast = (os.environ.get('SOUTH_PATROL_SOUTHEAST', '0') == '1'
                          if southeast is None else bool(southeast))
        self.northeast = (os.environ.get('SOUTH_PATROL_NORTHEAST', '0') == '1'
                          if northeast is None else bool(northeast))
        if self.southeast and self.northeast:
            raise ValueError('choose only one initial patrol region')
        self._first_leg_started = False
        self.uid = None
        self.deadline = 0.
        self.task_id = None
        self.attempts = 0
        self.next_s = 0.
        self.empty_rounds = 0
        self.direction = 0
        self._manager = None
        self.proposal_region = None

    def finish(self, now, productive=False):
        self.empty_rounds = 0 if productive else min(3, self.empty_rounds+1)
        self.next_s = now+60.*max(1, self.empty_rounds)
        self.uid = self.task_id = None
        self.attempts = self.direction = 0
        self._manager = None
        self.proposal_region = None

    def _fresh_image_gain(self, task):
        """Require actual processed image coverage before another NE leg."""
        manager = self._manager
        if manager is None:
            return False
        # A whole-cell credit comes only from the processed-frame callback;
        # retain it even if its short-lived subpoint evidence was pruned.
        if any((cell := manager.grid.cell(key)) is not None and
               cell.state == STATE_COVERED and cell.covered_t >= task.created
               for key in task.cells):
            return True
        seen = getattr(manager, '_camera_point_seen', {}) or {}
        return any(key in task.cells and sample >= task.created
                   for (key, _subpoint), sample in seen.items())

    def ended(self, task, reason, now):
        if task.task_id != self.task_id:
            return
        self.task_id = None
        obstacle_retry = (self.northeast and reason in
                          ('local_GOAL_OCCUPIED', 'blocked_viewpoint'))
        if reason == 'complete' or obstacle_retry:
            if reason == 'complete' and self.northeast and not self._fresh_image_gain(task):
                self.finish(now)
                return
            if self.attempts < self.max_segments and now < self.deadline:
                return
        self.finish(now, productive=reason in ('person_task', 'released_for_new_task'))

    def choose(self, manager, idle, now):
        """Return one checked proposal. Manager holds its task lock throughout."""
        if self.task_id is not None or now < self.next_s:
            return None
        if self.uid is not None:
            if now >= self.deadline:
                self.finish(now)
                return None
            if self.uid not in idle:
                # The ordinary obstacle release imposes a 2 s cooldown. It
                # does not transfer the regional responsibility to a peer or
                # buy another time budget. A real person task takes priority.
                status = manager.status.get(self.uid)
                received = manager.last_report.get(self.uid)
                cooldown = getattr(manager, '_release_cooldown', {}).get(self.uid)
                cooling = (self.northeast and cooldown is not None and
                           now < cooldown.to_sec() and status is not None and
                           status.connected and status.z >= 1. and
                           received is not None and 0. <= now-received.to_sec() < 3. and
                           self.uid not in manager._tracking.values() and
                           self.uid not in manager._backup.values() and
                           self.uid not in manager._segments.tasks)
                if not cooling:
                    self.finish(now)
                return None
        grid = getattr(manager, '_g_infl', None)
        if grid is None:
            return None
        self._manager = manager
        if self.northeast:
            if self._first_leg_started and self.uid is None:
                return None
            proposal = self._choose_band(manager, idle, now, grid, 'northeast')
            if proposal is None:
                if self.uid is not None:
                    self.finish(now)
                else:
                    self.next_s = now+5.
            return proposal
        if self.southeast and not self._first_leg_started:
            proposal = self._choose_band(manager, idle, now, grid, 'southeast')
            if proposal is not None:
                return proposal
        proposal = self._choose_band(manager, idle, now, grid, 'south')
        if proposal is None:
            if self.uid is not None:
                self.finish(now)
            else:
                self.next_s = now+5.
        return proposal

    def _choose_band(self, manager, idle, now, grid, region):
        """Check one camera band with safe flight points and real image geometry."""
        keys = []
        viewpoints = {}
        ymin = self.bounds[2]
        for key, cell in manager.grid.cells.items():
            if region == 'southeast':
                in_band = (key[1] == 2 and 50. <= cell.cx <= 110. and
                           abs(cell.cy-(ymin+2.5*manager.grid.cell_m)) <=
                           manager.grid.cell_m)
            elif region == 'northeast':
                # The northeast actor corridor is inside the legal map, not
                # along the outer wall. Use contiguous east-west cells.
                in_band = 105. <= cell.cx <= 133. and 20. <= cell.cy <= 42.
            else:
                in_band = abs(cell.cy-ymin) <= manager.grid.cell_m*.5
            if (not in_band or key in manager._blocked_cells or
                    key in manager._segments.reserved or
                    cell.state in (STATE_ASSIGNED, STATE_COVERED) or cell.owner is not None or
                    now-manager.allocator.visit_time.get(key, -1e18) < 60. or
                    now-manager.allocator.failed_visit.get(key, -1e18) <
                        manager.allocator.failed_novelty_ttl):
                continue
            if region in ('southeast', 'northeast'):
                # The front camera looks north (southeast leg) or east
                # (northeast leg) from an inner, checked viewpoint. At 4.45 m,
                # 8.95 m keeps the centre inside the 27-degree vertical limit
                # and leaves 5 cm within the 9 m horizontal footprint. Other
                # cells are never credited merely because this route flew by.
                point = ((cell.cx, cell.cy-8.95) if region == 'southeast'
                         else (cell.cx-8.95, cell.cy))
                viewpoint_valid = getattr(manager, '_blind_viewpoint_valid', None)
                if (not manager._search_point_free(point) or
                        viewpoint_valid is None or not viewpoint_valid(cell, point) or
                        not clear_line(grid, point, (cell.cx, cell.cy))):
                    continue
            else:
                point = manager._segment_point(key)
                # A clipped last/first coverage row may be narrower than 7m.
                # Keep an observable standoff instead of flying over its centre.
                point=(point[0],max(point[1],cell.cy+8.95))
                # Preserve an interior standoff and observe the cell from there.
                # If the live map cannot supply this viewpoint, leave a blind job
                # to the ordinary allocator rather than camping on a wall.
                viewpoint_valid = getattr(manager, '_blind_viewpoint_valid', None)
                if (not point[1] > cell.cy or
                        not manager._search_point_free(point) or
                        viewpoint_valid is None or not viewpoint_valid(cell, point) or
                        not clear_line(grid, point, (cell.cx, cell.cy))):
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
                # All three bands need the same processed-image geometry; a
                # safe waypoint alone cannot credit a cell or find a person.
                cell = manager.grid.cell(key)
                height = getattr(s, 'z', float('nan'))
                yaw = math.atan2(cell.cy-point[1], cell.cx-point[0])
                if (not math.isfinite(height) or
                        not manager._in_processed_view(
                            point[0], point[1], yaw, cell.cx,
                            cell.cy, altitude=max(4.5, height),
                            down_pitch=0.)):
                    continue
                # A northeast sweep may need to reverse along the next row;
                # recent-visit and reservation checks prevent duplication.
                if (region != 'northeast' and self.direction and
                        self.direction*(point[0]-s.x) < -1.):
                    continue
                pairs.append((math.dist((s.x, s.y), point), uid, key, point))
        # Bounded planning work. A nearby building may make the nearest
        # straight-line candidate unreachable; spread the inner-band probes
        # across aircraft and subregions before filling the candidate limit.
        ordered = sorted(pairs)
        if region in ('southeast', 'northeast'):
            spread = {}
            for candidate in ordered:
                _, uid, _, point = candidate
                if region == 'southeast':
                    bucket = (uid, min(2, max(0, int((point[0]-50.)/20.))))
                else:
                    bucket = (uid, min(3, max(0, int((point[1]-20.)/7.))))
                spread.setdefault(bucket, candidate)
            selected = sorted(spread.values())
            selected_set = {(uid, key) for _, uid, key, _ in selected}
            selected.extend(candidate for candidate in ordered
                            if (candidate[1], candidate[2]) not in selected_set)
            selected = sorted(selected[:24])
        else:
            selected = ordered[:8]
        # A straight distance is only a lower bound; select by checked path
        # length amongst the candidates actually reached by safe A* legs.
        checked = []
        for distance, uid, key, point in selected:
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
            return None
        _, uid, key, point = min(checked)
        cells, points = [key], [point]
        directions = ([self.direction] if self.direction and region != 'northeast'
                      else [1, -1])
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
        self.proposal_region = region
        return uid, cells, points, direction

    def started(self, task, direction, now):
        if self.uid is None:
            self.uid = task.uid
            self.deadline = now+self.max_hold
            self.attempts = 0
        # A failed/person-preempted first task cannot regrant the special
        # priority in a later patrol episode.
        self._first_leg_started = True
        self.attempts += 1
        self.task_id = task.task_id
        self.direction = direction
        task.deadline = min(task.deadline, self.deadline)
        task.expires = min(task.expires, task.deadline)
