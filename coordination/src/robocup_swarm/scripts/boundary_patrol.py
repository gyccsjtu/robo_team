"""Directed regional search inside the normal task allocator.

No actor positions, independent controller, reservations or coverage writes.
ENU negative Y identifies south and positive X/Y identifies northeast.
"""
import math
import os
from types import SimpleNamespace
from robocup_navigation.astar import plan
from search_segments import clear_line
from search_observation import cell_points
from swarm_task import STATE_ASSIGNED, STATE_COVERED


class SouthPatrol:
    def __init__(self, search_bounds, max_hold=90., max_segments=3,
                 southeast=None, northeast=None, east=None, east_half=None):
        self.bounds = tuple(search_bounds)
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
        if east_half not in (None, 'north', 'south') or (east_half is not None and not self.east):
            raise ValueError('east_half requires an eastern north/south patrol')
        # Opt-in complementary regional jobs; the ordinary single east
        # patrol retains its original band and policy.
        self.east_half = east_half
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
        self.proposal_look_points = None
        self.proposal_budget = max_hold
        self.unreachable_since = None
        self.owner_retry = {}
        self._edge_observed = set()
        self._corner_observed = set()
        self._corner_observed_at = {}

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
        self.proposal_look_points = None

    def _corner_target(self, manager, key):
        """The actual eastern corner subpoint, independent of its cell centre."""
        if self.east_half is None:
            return None
        grid = manager.grid
        cell = grid.cell(key)
        if cell is None or not hasattr(grid, 'cell_bounds'):
            return None
        _, x1, y0, y1 = grid.cell_bounds(key)
        outer_y = y1 if self.east_half == 'north' else y0
        boundary_y = self.bounds[3] if self.east_half == 'north' else self.bounds[2]
        if (not math.isclose(x1, self.bounds[1], abs_tol=1e-6) or
                not math.isclose(outer_y, boundary_y, abs_tol=1e-6)):
            return None
        points = cell_points(grid, cell)
        index = 8 if self.east_half == 'north' else 2
        return (points[index], index) if len(points) == 9 else None

    def _corner_needs_first_view(self, manager, key):
        corner = self._corner_target(manager, key)
        return bool(corner is not None and key not in self._corner_observed and
                    manager.grid.cell(key).state != STATE_COVERED)

    def _corner_review_due(self, manager, key, now):
        """Recheck a previously imaged corner after the existing 60 s cycle.

        First-visit memory survives cache pruning, but is not permanent
        evidence that a moving person cannot have entered that corner.
        Route completion never refreshes the image time.
        """
        if (self._corner_target(manager, key) is None or
                self._corner_needs_first_view(manager, key)):
            return False
        samples = [self._corner_observed_at.get(key, float('nan'))]
        cell = manager.grid.cell(key)
        if cell.state == STATE_COVERED:
            samples.append(cell.covered_t)
        valid = [sample for sample in samples if math.isfinite(sample) and
                 0. <= sample <= now+.05]
        return not valid or now-max(valid) >= 60.

    def _fresh_corner_times(self, task, now):
        """Only task-bound corner images or full coverage remove this priority."""
        manager = self._manager
        if manager is None:
            return {}
        seen = getattr(manager, '_camera_point_seen', {}) or {}
        observed = {}
        for key in task.cells:
            corner = self._corner_target(manager, key)
            if corner is None:
                continue
            cell = manager.grid.cell(key)
            sample = seen.get((key, corner[1]))
            if (sample is not None and math.isfinite(sample) and
                    task.created <= sample <= now+.05):
                observed[key] = sample
            if (cell.state == STATE_COVERED and math.isfinite(cell.covered_t) and
                    task.created <= cell.covered_t <= now+.05):
                observed[key] = max(observed.get(key, -float('inf')), cell.covered_t)
        return observed

    def _fresh_corner_keys(self, task, now):
        return set(self._fresh_corner_times(task, now))

    def _fresh_image_keys(self, task, now=None):
        """Task-bound processed-image evidence, never a passed waypoint.

        The manager already rejects stale images when receiving them. A
        successful first visit remains evidence when an earlier leg's image
        ages while the aircraft completes later legs of the same task.
        """
        manager = self._manager
        if manager is None:
            return set()
        # A whole-cell credit comes only from the processed-frame callback;
        # retain it even if its short-lived subpoint evidence was pruned.
        seen_keys = {key for key in task.cells
                     if (cell := manager.grid.cell(key)) is not None and
                     cell.state == STATE_COVERED and cell.covered_t >= task.created and
                     (now is None or (math.isfinite(cell.covered_t) and
                                      cell.covered_t <= now+.05))}
        seen = getattr(manager, '_camera_point_seen', {}) or {}
        seen_keys.update(key for (key, _subpoint), sample in seen.items()
                         if key in task.cells and sample >= task.created and
                         (now is None or (math.isfinite(sample) and
                                          sample <= now+.05)))
        return seen_keys

    def _fresh_image_gain(self, task):
        """Require actual processed image coverage before another NE leg."""
        return bool(self._fresh_image_keys(task))

    def _edge_needs_first_view(self, manager, key):
        """Prioritize unseen outer lane rows using lawful map bounds."""
        if self.east_half is None:
            return False
        if self._corner_needs_first_view(manager, key):
            return True
        cell = manager.grid.cell(key)
        width = 2.*manager.grid.cell_m
        at_edge = (cell.cy >= self.bounds[3]-width if self.east_half == 'north'
                   else cell.cy <= self.bounds[2]+width)
        if not at_edge or key in self._edge_observed or cell.state == STATE_COVERED:
            return False
        return True  # an old/unbound subpoint cannot waive the first directed look

    def ended(self, task, reason, now):
        if task.task_id != self.task_id:
            return
        self.task_id = None
        if self.east:
            if self.east_half is not None:
                # Preserve first actual image visits after the coverage
                # callback prunes short-lived subpoint history or hands the
                # regional responsibility to another aircraft.
                self._edge_observed.update(self._fresh_image_keys(task, now))
                corner_times = self._fresh_corner_times(task, now)
                self._corner_observed.update(corner_times)
                self._corner_observed_at.update(corner_times)
                if reason == 'complete' and self._manager is not None:
                    # Completion can mean every point timed out without a
                    # usable frame. Apply the existing finite failed-view
                    # cooldown to unseen edge cells even after index==len.
                    for key in task.cells:
                        if (self._edge_needs_first_view(self._manager, key) or
                                self._corner_review_due(self._manager, key, now)):
                            self._manager.allocator.failed_visit[key] = now
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
        look_points = {}
        self.proposal_look_points = None
        ymin = self.bounds[2]
        for key, cell in manager.grid.cells.items():
            if region == 'southeast':
                in_band = (key[1] == 2 and 50. <= cell.cx <= 110. and
                           abs(cell.cy-(ymin+2.5*manager.grid.cell_m)) <=
                           manager.grid.cell_m)
            elif region == 'east':
                if self.east_half is None:
                    in_band = (self.bounds[1]-45. <= cell.cx <= self.bounds[1]-17. and
                               self.bounds[2]+8. <= cell.cy <= self.bounds[3]-8.)
                else:
                    middle = .5*(self.bounds[2]+self.bounds[3])
                    in_band = (self.bounds[1]-45. <= cell.cx <= self.bounds[1] and
                               self.bounds[2] <= cell.cy <= self.bounds[3] and
                               (cell.cy >= middle if self.east_half == 'north'
                                else cell.cy < middle))
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
                    (now-manager.allocator.visit_time.get(key, -1e18) < 60. and
                     not self._edge_needs_first_view(manager, key) and
                     not self._corner_review_due(manager, key, now)) or
                    now-manager.allocator.failed_visit.get(key, -1e18) <
                        manager.allocator.failed_novelty_ttl):
                continue
            if region in ('southeast', 'northeast', 'east'):
                # The front camera looks north (southeast leg) or east
                # (northeast leg) from an inner, checked viewpoint. At 4.45 m,
                # 8.95 m keeps the centre inside the 27-degree vertical limit
                # and leaves 5 cm within the 9 m horizontal footprint. Other
                # cells are never credited merely because this route flew by.
                corner = self._corner_target(manager, key) if region == 'east' else None
                look = corner[0] if corner is not None else (cell.cx, cell.cy)
                point = ((look[0], look[1]-8.95) if region == 'southeast'
                         else (look[0]-8.95, look[1]))
                if region == 'east' and self.east_half is not None:
                    # A clipped outer row has centre y=59.5 while the legal
                    # aircraft inset ends at 58/58.5. Keep the observed cell
                    # unchanged and move the UAV inward in Y, preserving the
                    # 8.95 m horizontal standoff rather than moving the cell
                    # or weakening the 9 m image/range gate.
                    _, safe_y = manager._segment_flight_waypoint(*point)
                    dy = look[1]-safe_y
                    if not math.isfinite(dy) or abs(dy) >= 8.95:
                        continue
                    point = (look[0]-math.sqrt(8.95**2-dy**2), safe_y)
                viewpoint_valid = getattr(manager, '_blind_viewpoint_valid', None)
                # Reuse the exact flight/clearance/image checks for the region
                # being observed, without changing the coverage cell itself.
                observation = (SimpleNamespace(cx=look[0], cy=look[1])
                               if corner is not None else cell)
                if (not manager._search_point_free(point) or
                        viewpoint_valid is None or not viewpoint_valid(observation, point)):
                    continue
            else:
                look = (cell.cx, cell.cy)
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
                        viewpoint_valid is None or not viewpoint_valid(cell, point)):
                    continue
            keys.append(key)
            viewpoints[key]=point
            look_points[key]=look
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
                look = look_points[key]
                yaw = math.atan2(look[1]-point[1], look[0]-point[0])
                if (not math.isfinite(height) or
                        not manager._in_processed_view(
                            point[0], point[1], yaw, look[0],
                            look[1], altitude=max(4.5, height),
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
            return tier(candidate[2]) + candidate

        # Reach the east with useful images before making a long diagonal
        # trip to its corner. Once an aircraft has entered the eastern band,
        # the existing unseen-corner/edge priority applies unchanged.
        entering_east = (region == 'east' and self.east_half is not None and
                         os.environ.get('EAST_FIRST_ENTRY','1') == '1' and
                         bool(idle) and all(manager.status[u].x < self.bounds[1]-55.
                                            for u in idle
                                            if self.uid is None or u == self.uid))

        def tier(key):
            if entering_east:
                point = viewpoints[key]
                entry = (self.bounds[1]-35. <= point[0] <= self.bounds[1]-15. and
                         abs(point[1]-.5*(self.bounds[2]+self.bounds[3])) <= 25.)
                return (not entry,revisited(key))
            return ((not self._corner_needs_first_view(manager, key),
                     not self._corner_review_due(manager, key, now),
                     not self._edge_needs_first_view(manager, key), revisited(key))
                    if region == 'east' and self.east_half is not None
                    else (revisited(key),))

        # A 60-second cooldown alone favors the nearest old strip again
        # while distant corners still have no visit. Finish reachable new
        # viewpoints first, keeping old views as a fallback, not deleting them.
        ordered = sorted(pairs, key=priority)
        if region in ('southeast', 'northeast', 'east'):
            def spread_candidates(candidates, limit):
                spread = {}
                for candidate in candidates:
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
                selected.extend(candidate for candidate in candidates
                                if (candidate[1], candidate[2]) not in selected_set)
                return sorted(selected[:limit], key=priority)

            if region == 'east' and self.east_half is not None:
                edge = [c for c in ordered if self._edge_needs_first_view(manager, c[2])]
                other = [c for c in ordered if not self._edge_needs_first_view(manager, c[2])]
                # A blocked/unreachable outer strip cannot consume every
                # bounded planning attempt and pin this aircraft forever.
                edge_limit = 12 if other else 24
                selected = spread_candidates(edge, edge_limit)
                selected += spread_candidates(other, 24-len(selected))
                selected.sort(key=priority)
            else:
                selected = spread_candidates(ordered, 24)
        else:
            selected = ordered[:8]
        # A straight distance is only a lower bound; select by checked path
        # length amongst the candidates actually reached by safe A* legs.
        checked = []
        checked_tier = None
        failed_edges, reachable_edges = set(), set()
        for distance, uid, key, point in selected:
            if checked and (tier(key) != checked_tier or
                            distance > min(row[0] for row in checked)+1.):
                break
            s = manager.status[uid]
            route = plan(grid, (s.x, s.y), point, connectivity=8,
                         prevent_corner_cutting=True, max_expansions=2500)
            if not route.success:
                if (self._edge_needs_first_view(manager, key) or
                        self._corner_review_due(manager, key, now)):
                    failed_edges.add(key)
                continue
            route_points = [(s.x, s.y)]+list(route.points)+[point]
            if not all(clear_line(grid, a, b) for a, b in zip(route_points, route_points[1:])):
                continue
            checked.append((route.path_length_m, uid, key, point))
            reachable_edges.add(key)
            checked_tier = tier(key)
        for key in failed_edges-reachable_edges:
            manager.allocator.failed_visit[key] = now
        if not checked:
            return None
        length, uid, key, point = min(checked)
        cells, points = [key], [point]
        # An expired corner view must not wait behind the entire first edge
        # sweep again. Re-image it as one bounded job, then resume new strips.
        corner_review = (region == 'east' and
                         self._corner_review_due(manager, key, now))
        directions = ([self.direction] if self.direction and region not in ('northeast', 'east')
                      else [1, -1])
        extensions = []
        for direction in directions:
            ks, ps = list(cells), list(points)
            for _ in range(0 if corner_review else 2):
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
        if region == 'east' and self.east_half is not None:
            outward = 1 if self.east_half == 'north' else -1
            extension = next(e for e in extensions if e[1] == outward)
            # An unseen edge must continue toward its corner when a safe
            # outward link exists. Filling the three-point maximum inward
            # would defer that corner again. A blocked/one-point outward
            # chain still falls back to the normal checked longest segment.
            if not (self._edge_needs_first_view(manager, key) and extension[0] >= 2):
                extension = max(extensions, key=lambda e: (e[0], e[1] == outward))
        else:
            extension = max(extensions)
        _, direction, cells, points = extension
        self.proposal_region = region
        self.proposal_look_points = [look_points[key] for key in cells]
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
