"""Directed regional search inside the normal task allocator.

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
                 southeast=None, northeast=None, east=None):
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
        self.east = (os.environ.get('EAST_PATROL', '0') == '1'
                     if east is None else bool(east))
        if sum((self.southeast, self.northeast, self.east)) > 1:
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
        self.proposal_budget = max_hold
        self.unreachable_since = None
        self.owner_retry = {}

    def accepts_target(self, uid, x, y, has_camera=False):
        """Real local images preempt patrol; blind western errands do not."""
        return (not self.east or uid != self.uid or has_camera or
                x >= self.bounds[1]-55.)

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
        if self.east:
            # A route expires, not the regional responsibility. Failed cells
            # have a finite retry delay; reservations/coverage stay in manager.
            if reason not in ('person_task', 'released_for_new_task', 'camera_unavailable') and (
                    reason != 'complete' or not self._fresh_image_gain(task)):
                if self._manager is not None:
                    for key in task.cells[task.index:]:
                        self._manager.allocator.failed_visit[key] = now
            if reason in ('person_task', 'released_for_new_task'):
                self.uid = None  # allow another idle camera to cover the region
            self.next_s = now+(0. if reason == 'complete' or self.uid is None else 2.)
            self.unreachable_since = None
            return
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
        if self.east:
            return self._choose_east(manager, idle, now)
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

    def _choose_east(self, manager, idle, now):
        self._manager = manager
        if self.uid is not None:
            st = manager.status.get(self.uid)
            received = manager.last_report.get(self.uid)
            healthy = (st is not None and st.connected and st.z >= 1. and
                       received is not None and 0. <= now-received.to_sec() < 3. and
                       all(math.isfinite(v) for v in (st.x, st.y, st.z)) and
                       manager._tracking_uav_available(self.uid, now))
            if (not healthy or self.uid in manager._tracking.values() or
                    self.uid in manager._backup.values()):
                self.uid = None
                self.unreachable_since = None
                self.next_s = now
        if self.task_id is not None or now < self.next_s:
            return None
        if self.uid is not None and self.uid not in idle:
            return None  # retain ownership across the real release cooldown
        candidates = [uid for uid in idle if now >= self.owner_retry.get(uid, 0.)]
        grid = getattr(manager, '_g_infl', None)
        proposal = (self._choose_band(manager, candidates, now, grid, 'east')
                    if grid is not None else None)
        if proposal is not None:
            self.unreachable_since = None
            return proposal
        self.next_s = now+2.
        if self.uid is not None:
            if self.unreachable_since is None:
                self.unreachable_since = now
            elif now-self.unreachable_since >= 10.:
                # Do not pin a stranded aircraft forever. A healthy peer may
                # have an accessible entry on a different side of a building.
                self.owner_retry[self.uid] = now+30.
                self.uid = None
                self.unreachable_since = None
        return None

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
            elif region == 'east':
                in_band = (self.bounds[1]-45. <= cell.cx <= self.bounds[1]-17. and
                           self.bounds[2]+8. <= cell.cy <= self.bounds[3]-8.)
            elif region == 'northeast':
                # The northeast actor corridor is inside the legal map, not
                # along the outer wall. Use contiguous east-west cells.
                in_band = 105. <= cell.cx <= 133. and 20. <= cell.cy <= 42.
            else:
                in_band = abs(cell.cy-ymin) <= manager.grid.cell_m*.5
            if (not in_band or key in manager._blocked_cells or
                    key in manager._segments.reserved or
                    cell.state == STATE_ASSIGNED or cell.owner is not None or
                    (cell.state == STATE_COVERED and
                     (region != 'east' or now-cell.covered_t < 60.)) or
                    now-manager.allocator.visit_time.get(key, -1e18) < 60. or
                    now-manager.allocator.failed_visit.get(key, -1e18) <
                        manager.allocator.failed_novelty_ttl):
                continue
            if region in ('southeast', 'northeast', 'east'):
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
                if (region not in ('northeast', 'east') and self.direction and
                        self.direction*(point[0]-s.x) < -1.):
                    continue
                pairs.append((math.dist((s.x, s.y), point), uid, key, point))
        # Bounded planning work. A nearby building may make the nearest
        # straight-line candidate unreachable; spread the inner-band probes
        # across aircraft and subregions before filling the candidate limit.
        def revisited(key):
            return (region == 'east' and
                    (manager.allocator.visit_time.get(key, 0.) > 0. or
                     manager.grid.cell(key).covered_t > 0.))

        def priority(candidate):
            return (revisited(candidate[2]),) + candidate

        # A 60-second cooldown alone favors the nearest old strip again
        # while distant corners still have no visit. Finish reachable new
        # viewpoints first, keeping old views as a fallback, not deleting them.
        ordered = sorted(pairs, key=priority)
        if region in ('southeast', 'northeast', 'east'):
            spread = {}
            for candidate in ordered:
                _, uid, _, point = candidate
                if region == 'southeast':
                    bucket = (uid, min(2, max(0, int((point[0]-50.)/20.))))
                elif region == 'east':
                    bucket = (uid, min(3, max(0, int((point[1]-ymin)/30.))))
                else:
                    bucket = (uid, min(3, max(0, int((point[1]-20.)/7.))))
                spread.setdefault(bucket, candidate)
            selected = sorted(spread.values(), key=priority)
            selected_set = {(uid, key) for _, uid, key, _ in selected}
            selected.extend(candidate for candidate in ordered
                            if (candidate[1], candidate[2]) not in selected_set)
            selected = sorted(selected[:24], key=priority)
        else:
            selected = ordered[:8]
        # A straight distance is only a lower bound; select by checked path
        # length amongst the candidates actually reached by safe A* legs.
        checked = []
        checked_revisit = None
        for distance, uid, key, point in selected:
            if checked and (revisited(key) != checked_revisit or
                            distance > min(row[0] for row in checked)+1.):
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
            checked_revisit = revisited(key)
        if not checked:
            return None
        length, uid, key, point = min(checked)
        cells, points = [key], [point]
        directions = ([self.direction] if self.direction and region not in ('northeast', 'east')
                      else [1, -1])
        extensions = []
        for direction in directions:
            ks, ps = list(cells), list(points)
            for _ in range(2):
                nxt = ((ks[-1][0], ks[-1][1]+direction) if region == 'east'
                       else (ks[-1][0]+direction, ks[-1][1]))
                if nxt not in keys:
                    break
                if region == 'east' and not revisited(key) and revisited(nxt):
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
        # Conservative measured transit speed, plus turning/observation time.
        # The ordinary heartbeat and no-progress watchdog still apply.
        travel = length+sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.proposal_budget = (min(180., max(45., travel/1.4+12.*len(cells)+15.))
                                if region == 'east' else self.max_hold)
        return uid, cells, points, direction

    def started(self, task, direction, now):
        if self.east:
            self.uid, self.task_id = task.uid, task.task_id
            self.direction = direction
            self.deadline = task.deadline  # diagnostic, never extended by ACKs
            return
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
