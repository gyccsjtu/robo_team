"""Manager integration for bounded, versioned search segments.

Methods run under SwarmManager's existing serialized task lock. All search
sources use the same allocator and SegmentBook; coverage never releases a task.
"""
import math
import rospy
from geometry_msgs.msg import Point
from robocup_swarm.msg import SearchAssignment
from search_segments import TRAVEL, LOOK, ATTEND, YIELD, DONE, FAILED, aligned, clear_line
from swarm_task import STATE_FREE, STATE_REVIEW, STATE_COVERED
from building_viewpoints import boundary_corner, valid_view, view_options, measured_view_options
from search_observation import cell_points, ground_distance_band
from swarm_task import ACTOR_SEARCH_BOUNDS
from robocup_navigation.astar import plan
from routine_planning import plan_routine

# The search yaw executor limits the commanded heading to 25 degrees/sec.
# Deadline estimates must use that limit, not the faster general yaw ceiling.
SEARCH_LOOK_TURN_RATE = math.radians(25.)


class SearchSegmentManager:
    def _regional_search_patrols(self):
        """One default patrol or complementary jobs using the same task book."""
        patrols = getattr(self, '_regional_patrols', None)
        if patrols is not None:
            return tuple(patrols)
        patrol = getattr(self, '_south_patrol', None)
        return (patrol,) if patrol is not None else ()

    def _patrol_accepts_target(self, uid, x, y, has_camera=False):
        return all(patrol.accepts_target(uid, x, y, has_camera)
                   for patrol in self._regional_search_patrols())

    def _segment_point(self, key):
        return self._segment_flight_waypoint(*self._search_waypoint(key,self.grid.cell(key)))

    def _subpoint_views(self, key):
        review = getattr(self, '_review_subpoint_views', {}).get(key)
        now = rospy.Time.now().to_sec()
        if review is not None:
            if now >= review['expires_s']:
                self._review_subpoint_views.pop(key, None)
            else:
                from search_observation import cell_points
                samples = cell_points(self.grid, self.grid.cell(key))
                seen = getattr(self, '_camera_point_seen', {})
                return tuple((point, look) for point, look in review['options']
                             if now-seen.get((key, samples.index(look)), -1e18) > 30.)
        return getattr(self, '_building_viewpoints', {}).get(key, ())

    def _reachable_subpoint_view(self, key, origin, options, max_path_m=None,
                                 altitude=4.5, preferred_heading=None, max_candidates=24,
                                 with_path=False, avoid_search_uid=None):
        """A free ground sample is useful only with a checked safe flight path."""
        raw = self._merged_grid() if options else None
        grid = getattr(self, '_g_infl', None)
        routine_bounds = getattr(self, '_routine_search_bounds', False)
        if routine_bounds:
            from routine_search import routine_point_allowed
            # Filter before the A* candidate cap: an outer option may not
            # crowd a lawful inner observation out of the bounded attempts.
            options = tuple((point, look) for point, look in options
                            if routine_point_allowed(point, look))
        layouts = (self._active_search_layout() if avoid_search_uid is not None and
                   getattr(getattr(self, 'allocator', None), 'search_deconflict', False) else {})
        def priority(option):
            point, look = option
            from search_deconfliction import crowding_cost
            crowded = crowding_cost(point, look, layouts, avoid_search_uid)
            if preferred_heading is None:
                return (not boundary_corner(self.grid, key, look), crowded, math.dist(origin, point))
            bearing = math.atan2(look[1]-point[1], look[0]-point[0])
            error = abs(math.atan2(math.sin(bearing-preferred_heading),
                                   math.cos(bearing-preferred_heading)))
            return (round(error, 6), math.dist(origin, point))
        for point, look in sorted(options, key=priority)[:max_candidates]:
            if max_path_m is not None and math.dist(origin,point) > max_path_m:
                continue
            if not valid_view(raw, grid, look, point, self._segment_flight_waypoint,
                              altitude, routine_bounds=routine_bounds):
                continue
            route = (plan_routine if routine_bounds else plan)(grid, origin, point, connectivity=8,
                         prevent_corner_cutting=True, max_expansions=2500)
            if (not route.success or
                    (max_path_m is not None and route.path_length_m > max_path_m)):
                continue
            points = [origin]+list(route.points)+[point]
            if all(clear_line(grid, a, b) for a, b in zip(points, points[1:])):
                return (point, look, tuple(points)) if with_path else (point, look)
        return None

    def _directed_camera_height(self, uid, now):
        """Only a recent processed own-camera pose can design a neutral view."""
        camera = getattr(self, '_processed_camera_poses', {}).get(uid)
        status = self.status.get(uid)
        if not isinstance(camera, dict) or status is None:
            return None
        try:
            sample, altitude, aircraft_z = (float(camera['sample_s']),
                                           float(camera['z']), float(status.z))
        except (KeyError, TypeError, ValueError):
            return None
        if (not all(math.isfinite(v) for v in (sample, altitude, aircraft_z)) or
                not -.05 <= now-sample <= .75 or altitude < 2. or
                abs(altitude-aircraft_z) > .5 or
                ground_distance_band(altitude, 0.) is None):
            return None
        return altitude

    def _directed_segment(self, uid, candidates, now, heading=None, budget_s=None):
        """Build at most three image viewpoints, preserving safe route order.

        A point observes one of the reserved cell's nine samples. Neither a
        proposal nor reaching that point grants the other eight samples.
        """
        altitude = self._directed_camera_height(uid, now)
        if budget_s is not None and (not math.isfinite(budget_s) or budget_s <= .4):
            return None
        raw, inflated = self._merged_grid(), getattr(self, '_g_infl', None)
        if altitude is None or raw is None or inflated is None:
            return None
        status = self.status[uid]
        origin = (status.x, status.y)
        cells, points, looks = [], [], []
        camera = getattr(self, '_processed_camera_poses', {}).get(uid, {})
        yaw = camera.get('yaw')
        if budget_s is not None and (isinstance(yaw, bool) or
                not isinstance(yaw, (int, float)) or not math.isfinite(yaw)):
            return None  # no invented orientation for a deadline estimate
        path_m, turn_rad = 0., 0.
        travel_heading = heading
        for key in candidates[:6]:
            if len(cells) >= 3:
                break
            cell = self.grid.cell(key)
            if cell is None or self._search_view_job(key) is not None:
                break  # retain existing assignment-bound candidate views
            if travel_heading is None:
                travel_heading = math.atan2(cell.cy-origin[1], cell.cx-origin[0])
            seen = getattr(self, '_camera_point_seen', {})
            samples = [point for index, point in enumerate(cell_points(self.grid, cell))
                       if now-seen.get((key, index), -1e18) > 30.]
            # Prefer the centre, then nearby still-unobserved ground. Links
            # retain one travel direction instead of zigzagging between views.
            samples.sort(key=lambda p: math.dist(p, (cell.cx, cell.cy)))
            options = measured_view_options(self.grid, key, raw, inflated,
                self._segment_flight_waypoint, altitude, travel_heading, samples)
            previous = points[-1] if points else origin
            # A lower-bound rejection avoids spending A* work on a journey
            # which cannot fit even in a straight, obstacle-free corridor.
            path_budget = (None if budget_s is None else max(0.,
                (budget_s-path_m/1.4-turn_rad/SEARCH_LOOK_TURN_RATE-(len(cells)+1)*.4)*1.4))
            view = self._reachable_subpoint_view(key, previous, options,
                altitude=altitude, preferred_heading=travel_heading,
                max_candidates=4, with_path=True, max_path_m=path_budget)
            if view is None:
                break
            point, look, route = view
            if points:
                forward = ((point[0]-points[-1][0])*math.cos(travel_heading)+
                           (point[1]-points[-1][1])*math.sin(travel_heading))
                if forward < .6*self.grid.cell_m or math.dist(look, looks[-1]) < 2.:
                    continue  # leave the degenerate cell free for actual review
            if points and (not clear_line(inflated, points[-1], point) or
                    (len(points) >= 2 and not aligned(points[-2], points[-1], point))):
                break
            # Avoid a checked but backward first flight leg. A* can detour
            # around a building; the endpoint must still advance the lane.
            dx, dy = point[0]-previous[0], point[1]-previous[1]
            if heading is not None and dx*math.cos(travel_heading)+dy*math.sin(travel_heading) < -.5:
                break
            leg_m = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            look_heading = math.atan2(look[1]-point[1], look[0]-point[0])
            prior_heading = (math.atan2(looks[-1][1]-points[-1][1],
                                       looks[-1][0]-points[-1][0]) if points else yaw)
            turn = (abs(math.atan2(math.sin(look_heading-prior_heading),
                                   math.cos(look_heading-prior_heading)))
                    if prior_heading is not None else 0.)
            # The checked path includes endpoint offsets omitted by the A*
            # cell-centre path_length. Use measured own yaw and a conservative
            # 1.4 m/s estimate; each not-yet-imaged look needs its real frames.
            eta = ((path_m+leg_m)/1.4+(turn_rad+turn)/SEARCH_LOOK_TURN_RATE+
                   (len(cells)+1)*.4)
            if budget_s is not None and eta > budget_s:
                break  # retain an affordable prefix, never renew the episode
            path_m += leg_m
            turn_rad += turn
            cells.append(key)
            points.append(point)
            looks.append(look)
        return (cells, points, looks) if cells else None

    def _road_height_stable(self, uid, stamp):
        """Do not design a road view during the still-changing takeoff climb.

        Only new processed frames buy stability credit. A missing window
        leaves this aircraft available to the ordinary existing allocator.
        """
        window = getattr(self, '_processed_camera_height_window', {}).get(uid, ())
        if not isinstance(window, (list, tuple)) or len(window) < 3:
            return False
        try:
            samples, heights = zip(*window)
            if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in (*samples, *heights)) or
                    not -.05 <= stamp-samples[-1] <= .5 or
                    samples[-1]-samples[0] < .4-1e-6 or
                    any(not 0. < b-a <= .5 for a, b in zip(samples, samples[1:])) or
                    max(heights)-min(heights) > .12+1e-6):
                return False
        except (TypeError, ValueError):
            return False
        return True

    def _allocate_road_first_pass(self, idle, stamp):
        road = getattr(self, '_road_first_pass', None)
        if road is None:
            return
        def eligible(key):
            cell = self.grid.cell(key)
            return (cell is not None and cell.state == STATE_FREE and cell.owner is None and
                    key not in self._segments.reserved and key not in self._blocked_cells and
                    not self._subpoint_views(key) and self._search_view_job(key) is None and
                    stamp-self.allocator.failed_visit.get(key, -1e18) >= 30.)
        for _ in range(4):
            positions = {uid: (self.status[uid].x, self.status[uid].y) for uid in idle
                         if self._directed_camera_height(uid, stamp) is not None and
                         self._road_height_stable(uid, stamp)}
            proposal = road.choose(self.grid, positions, stamp, eligible,
                lambda uid, cells, heading: self._directed_segment(uid, cells, stamp, heading,
                    budget_s=min(self._segment_max_hold, road.remaining(stamp))))
            if proposal is None:
                break
            lane, uid, cells, points, looks = proposal
            task = self._start_search_segment(uid, cells, points, stamp,
                purpose='coverage_view', look_points=looks, publish=False,
                max_hold=min(self._segment_max_hold, road.remaining(stamp)))
            if task is None:
                break  # only SegmentBook may acquire the whole route
            road.started(task, lane)
            self._send_segment(task)
            idle.remove(uid)
            rospy.loginfo('[ROAD_FIRST_PASS] uav=%s task=%s lane=%s cells=%s points=%s looks=%s '
                          'deadline=%.3f; sensed path, no image credit',
                          uid, task.task_id, lane, cells, points, looks, task.deadline)

    def _select_building_view(self, uid, key):
        status = self.status[uid]
        return self._reachable_subpoint_view(key, (status.x, status.y),
                                             self._subpoint_views(key), avoid_search_uid=uid)

    def _active_search_layout(self, stamp=None):
        """Uncompleted task views under the same allocation/reservation lock."""
        from search_deconfliction import finite_point
        stamp = rospy.Time.now().to_sec() if stamp is None else stamp
        layouts = {}
        for uid, task in self._segments.tasks.items():
            if (task.index >= len(task.cells) or
                    not task.created <= stamp < min(task.expires, task.deadline)):
                continue
            layout = dict(points=task.points[task.index:],
                          look_points=task.look_points[task.index:])
            status = self.status.get(uid)
            sample = getattr(getattr(status, 'header', None), 'stamp', None)
            sample_s = sample.to_sec() if sample is not None else -float('inf')
            if (status is not None and finite_point((status.x, status.y)) and
                    math.isfinite(sample_s) and 0. <= stamp-sample_s <= .5):
                layout['position'] = (status.x, status.y)
            layouts[uid] = layout
        return layouts

    def _west_corner_subpoints(self):
        """The two public actor-area corners, using the existing nine samples."""
        xmin, _, ymin, ymax = ACTOR_SEARCH_BOUNDS
        result = []
        for name, look in (('southwest', (xmin+.01, ymin+.01)),
                           ('northwest', (xmin+.01, ymax-.01))):
            key = self.grid.world_to_cell(*look)
            cell = self.grid.cell(key) if key is not None else None
            samples = cell_points(self.grid, cell) if cell is not None else ()
            if look in samples and boundary_corner(self.grid, key, look):
                result.append((name, key, samples.index(look), look))
        return tuple(result)

    def _west_corner_image_time(self, name, now):
        book = self.__dict__.setdefault('_west_corner_seen', {})
        sample = book.get(name)
        if sample is not None and (isinstance(sample, bool) or not isinstance(sample, (int, float)) or
                not math.isfinite(sample) or not 0. < sample <= now+.05):
            book.pop(name, None)  # a clock reset cannot permanently hide a corner
            sample = None
        return sample

    def _west_corner_has_person(self, uid, stamp):
        views = [view for view in getattr(self, '_discovery_pool', {}).values()
                 if isinstance(view, dict) and view.get('appearance_status') == 'actor']
        views += [view for view in getattr(self, '_eligible_observations', {}).values()]
        views += [view for view in getattr(self, '_visual_observations', {}).values()
                  if isinstance(view, dict) and view.get('eligible')]
        for view in views:
            if not isinstance(view, dict) or view.get('source_uav') != uid:
                continue
            sample = view.get('sample_s')
            if (not isinstance(sample, bool) and isinstance(sample, (int, float)) and
                    math.isfinite(sample) and 0. <= stamp-sample <= 1.):
                return True
        return False

    def _west_corner_borrowable(self, task, stamp, busy):
        """Only a healthy current ordinary transit may yield to an overdue view."""
        uid = task.uid
        status = self.status.get(uid)
        frames = getattr(self, '_processed_frame_times', {})
        frame = frames.get(uid)
        road = getattr(self, '_road_first_pass', None)
        if (task.purpose not in ('coverage', 'coverage_view') or
                (road is not None and task.task_id in road.bindings) or uid in busy or status is None or
                status.search_state != TRAVEL or
                not self._segments.matches(uid, status.search_task_id, status.search_revision) or
                status.search_index != task.index or task.index >= len(task.cells) or
                stamp >= task.deadline or stamp >= task.expires or
                not all(math.isfinite(v) for v in (status.x, status.y, status.z)) or
                status.z < 1. or not self._tracking_uav_available(uid, stamp) or
                self._west_corner_has_person(uid, stamp)):
            return False
        pose = status.header.stamp.to_sec()
        cooldown = getattr(self, '_release_cooldown', {}).get(uid)
        return (math.isfinite(pose) and task.created <= pose <= stamp and stamp-pose < 1. and
                not isinstance(frame, bool) and isinstance(frame, (int, float)) and
                math.isfinite(frame) and 0. <= stamp-frame <= 2. and
                (cooldown is None or stamp >= cooldown.to_sec()))

    def _allocate_west_corner_review(self, idle, stamp):
        """One additional checked corner look, leaving ordinary search capacity."""
        corners = self._west_corner_subpoints()
        if any(task.purpose == 'building_review' and any(
                key in task.cells and look in task.look_points
                for _, key, _, look in corners)
                for task in self._segments.tasks.values()):
            return
        busy = set(self._tracking.values()) | set(self._backup.values())
        busy.update(p.uid for p in self._regional_search_patrols()
                    if p.east and p.uid is not None)
        available = [uid for uid in idle if uid not in busy and
            uid not in self._segments.tasks and
            getattr(self.status[uid], 'search_state', None) != ATTEND and
            self._tracking_uav_available(uid, stamp) and
            not self._west_corner_has_person(uid, stamp)]
        ordinary = sum(task.purpose in ('coverage', 'building_review', 'coverage_view') and
            task.uid not in busy and self._tracking_uav_available(task.uid, stamp) and
            getattr(self.status[task.uid], 'search_state', None) != ATTEND and
            not self._west_corner_has_person(task.uid, stamp)
            for task in self._segments.tasks.values())
        started = self.__dict__.get('_west_corner_allocation_started_s')
        if (isinstance(started, bool) or not isinstance(started, (int, float)) or
                not math.isfinite(started) or stamp < started):
            started = None
        if started is None and (available or ordinary):
            started = stamp
            self._west_corner_allocation_started_s = stamp
        borrowers = [task for task in self._segments.tasks.values()
                     if self._west_corner_borrowable(task, stamp, busy)]
        if (not (available or borrowers) or len(available)+ordinary < 2 or
                getattr(self, '_g_infl', None) is None):
            return
        raw = self._merged_grid()
        if raw is None:
            return
        ordered = []
        for name, key, _, look in corners:
            last = self._west_corner_image_time(name, stamp)
            blind = self.allocator.revisit_jobs.get(('blind', key))
            if (key in self._blocked_cells or key in self._segments.reserved or
                    (blind is not None and stamp < blind[0]) or
                    (last is not None and stamp-last < 60.) or
                    stamp-self.allocator.failed_visit.get(key, -1e18) <
                    self.allocator.failed_novelty_ttl):
                continue
            ordered.append((last is not None, last or -1e18, name, key, look))
        for _, last, name, key, look in sorted(ordered):
            options = tuple((point, sample) for point, sample in view_options(
                self.grid, key, raw, self._g_infl, self._segment_flight_waypoint)
                if sample == look)
            # A never-seen corner may borrow only after the first healthy
            # allocation has run for 60 s. Normal idle first visits stay intact.
            borrow_due = last != -1e18 or (started is not None and stamp-started >= 60.)
            borrowing = {task.uid: task for task in borrowers} if borrow_due else {}
            for uid in sorted(available+list(borrowing), key=lambda u:
                    (math.dist((self.status[u].x, self.status[u].y), look), u)):
                st = self.status[uid]
                # A corner revisit is a local supplement, never a 143 m
                # cross-map errand taking a scarce aircraft out of the east.
                view = self._reachable_subpoint_view(key, (st.x, st.y), options,
                                                    max_path_m=60.)
                if view is None:
                    continue
                point, sample = view
                previous = borrowing.get(uid)
                if previous is not None:
                    if (self._segments.tasks.get(uid) is not previous or
                            not self._west_corner_borrowable(previous, stamp, busy)):
                        continue
                    # Caller holds the same task lock through safe route check,
                    # cancellation, release of all remaining cells and reserve.
                    self._drop_segment(uid, 'yield_reallocate')
                task = self._start_search_segment(uid, [key], [point], stamp,
                    purpose='building_review', look_points=[sample])
                if task is not None:
                    if uid in idle:
                        idle.remove(uid)
                    rospy.loginfo('[WEST_CORNER_REVIEW] uav=%s task=%s corner=%s '
                        'cell=%s point=%s look=%s last_image=%s borrowed=%s; checked path, no coverage credit',
                        uid, task.task_id, name, key, point, sample,
                        None if last == -1e18 else last,
                        previous.task_id if previous is not None else None)
                    return
                if previous is not None and uid not in idle:
                    idle.append(uid)  # released work stays open if reserve fails
                    return  # never cancel a second ordinary task in this tick
            # A blocked start or bounded A* failure belongs to this idle batch,
            # not to the corner. Let ordinary search use these aircraft while
            # a later idle aircraft can still reach an already safe viewpoint.
            # Only absent safe views justify the existing cell-wide cooldown.
            if not options:
                self.allocator.failed_visit[key] = stamp

    def _segment_cells(self, uid, first, now):
        cells = [first]
        points = [self._segment_point(first)]
        revisit_cells = {job[1] for job in self.allocator.revisit_jobs}
        # A blind-view/candidate revisit is an observation job, not a through leg.
        if (first in getattr(self, '_blind_viewpoints', {}) or
                self._search_view_job(first) is not None or
                first in revisit_cells):
            return cells, points
        grid = getattr(self, '_g_infl', None)
        st = self.status[uid]
        previous = (st.x,st.y)
        blocked = set(self._blocked_cells) | set(self._segments.reserved)
        for _ in range(2):
            options = []
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                key = (cells[-1][0]+dx,cells[-1][1]+dy)
                cell = self.grid.cell(key)
                if (cell is None or key in blocked or key in cells or
                        cell.state != STATE_FREE or cell.owner is not None or
                        self._subpoint_views(key) or
                        self.grid.get_zone_id(key) != self.grid.get_zone_id(first) or
                        key in getattr(self, '_blind_viewpoints', {}) or
                        key in revisit_cells or
                        now-self.allocator.failed_visit.get(key,-1e18) < self.allocator.failed_novelty_ttl):
                    continue
                point = self._segment_point(key)
                if (math.dist(points[-1],point) < self.grid.cell_m*.6 or
                        not aligned(previous,points[-1],point) or
                        not clear_line(grid,points[-1],point)):
                    continue
                options.append((math.dist(previous,point),key,point))
            if not options:
                break
            _,key,point = max(options)
            previous = points[-1]
            cells.append(key)
            points.append(point)
        return cells,points

    def _send_segment(self, task):
        msg = SearchAssignment()
        msg.header.stamp = rospy.Time.from_sec(task.created)
        msg.uav_id = task.uid
        msg.task_type = 0
        # Repeated messages are lease heartbeats, not new tasks. Whole arrays
        # stay immutable within a revision; the agent keeps its local index.
        msg.cell_ix,msg.cell_iy = task.cells[0]
        msg.target_x,msg.target_y = task.points[0]
        msg.search_task_id,msg.search_revision = task.task_id,task.revision
        msg.search_cell_ix = [k[0] for k in task.cells]
        msg.search_cell_iy = [k[1] for k in task.cells]
        msg.search_waypoints = [Point(x=x,y=y,z=0.) for x,y in task.points]
        msg.search_valid_until = rospy.Time.from_sec(task.expires)
        msg.search_purpose = task.purpose
        msg.search_look_points = [Point(x=x,y=y,z=0.) for x,y in task.look_points]
        self.assign_pub.publish(msg)

    def _drop_segment(self, uid, reason, cancel=True, failed=False):
        task = self._segments.release(uid)
        if task is None:
            return
        now = rospy.Time.now().to_sec()
        if task.purpose == 'building_review':
            for name, key, _, look in self._west_corner_subpoints():
                if key in task.cells and look in task.look_points:
                    image = self._west_corner_image_time(name, now)
                    if image is None or image < task.created:
                        index = task.cells.index(key)
                        safe_interruption = (
                            not failed and reason in ('yield_reallocate', 'person_task',
                                                      'released_for_new_task', 'observation_timeout') and
                            task.look_points[index] == look and
                            valid_view(self._merged_grid(), getattr(self, '_g_infl', None),
                                       look, task.points[index], self._segment_flight_waypoint))
                        # A safe view survives a source aircraft yielding or
                        # taking a person task; it is not a failed destination.
                        if not safe_interruption:
                            self.allocator.failed_visit[key] = now
        for patrol in self._regional_search_patrols():
            patrol.ended(task, reason, now)
        road = getattr(self, '_road_first_pass', None)
        if road is not None:
            road.ended(task, reason, now)
        for key in task.cells[task.index:]:
            self.lease.force_expire(uid,key)
            if failed:
                self.allocator.failed_visit[key] = now
        self._active_leases.pop(uid,None)
        self._lease_motion.pop(uid,None)
        getattr(self,'_search_dispatched',{}).pop(uid,None)
        for key in list(self._lease_hold_since):
            if key[0] == uid:
                self._lease_hold_since.pop(key,None)
        if cancel:
            msg = SearchAssignment()
            msg.header.stamp = rospy.Time.from_sec(now)
            msg.uav_id,msg.task_type = uid,255
            msg.search_task_id,msg.search_revision = task.task_id,task.revision
            self.assign_pub.publish(msg)
        rospy.loginfo('[SEARCH_SEGMENT_END] uav=%s task=%s v=%d visited=%d/%d reason=%s',
                      uid,task.task_id,task.revision,task.index,len(task.cells),reason)

    def _allocate_segments(self, now, west_corner_review=False):
        stamp = now.to_sec()
        self._renew_segments(now)
        idle = self._idle_uavs()
        if not idle and west_corner_review:
            self._allocate_west_corner_review(idle, stamp)
        if not idle:
            self._last_alloc_t = now
            return
        # Choose and reserve BOTH complementary lanes before ordinary jobs
        # consume the remaining idle aircraft. _allocate's existing task lock
        # covers this entire loop; SegmentBook supplies cross-source mutexes.
        patrols = self._regional_search_patrols()
        for patrol in patrols:
            # A completed leg releases its cells, not its regional owner.
            # The preceding lane must not borrow that idle aircraft before
            # its own lane can issue the next leg. Health/person/failed-entry
            # cleanup remains in choose; it can release ownership this tick.
            peer_owners = {peer.uid for peer in patrols
                           if peer is not patrol and peer.east and
                           peer.uid is not None}
            lane_idle = [uid for uid in idle if uid not in peer_owners]
            proposed = patrol.choose(self, lane_idle, stamp)
            if proposed is not None:
                uid, cells, points, direction = proposed
                task = self._start_search_segment(uid, cells, points, stamp,
                    purpose='south_patrol', publish=False,
                    max_hold=patrol.proposal_budget if patrol.east else None,
                    look_points=patrol.proposal_look_points)
                if task is not None:
                    patrol.started(task, direction, stamp)
                    self._send_segment(task)
                    idle.remove(uid)
                    rospy.loginfo('[SOUTH_PATROL] uav=%s task=%s region=%s cells=%s points=%s '
                        'look_points=%s deadline=%.3f '
                        'ENU_south=-Y; short transit then directed image observation lane=%s',
                        uid, task.task_id, patrol.proposal_region,
                        cells, task.points, task.look_points, task.deadline,
                        patrol.east_half or 'full')
            # A cooldown/replan is still an eastern responsibility, even
            # when no route could be issued on this particular scheduler tick.
            if patrol.east and patrol.uid in idle:
                idle.remove(patrol.uid)
        # Keep the two eastern owners first. Other idle aircraft take at most
        # four complementary public-road preferences through this same book.
        self._allocate_road_first_pass(idle, stamp)
        if west_corner_review:
            self._allocate_west_corner_review(idle, stamp)
        positions = {uid:(self.status[uid].x,self.status[uid].y) for uid in idle}
        # Outer task lock covers allocator selection, segment extension and
        # whole-segment reservation. No callback can see a half-reserved route.
        kwargs = {}
        if getattr(self.allocator, 'search_deconflict', False):
            kwargs = dict(active_search=self._active_search_layout(stamp),
                          search_points={key:self._segment_point(key)
                                         for key in self.grid.uncovered_cells()
                                         if key not in self._blocked_cells})
        seeds = self.allocator.allocate(positions,
            blocked=set(self._blocked_cells)|set(self._segments.reserved), **kwargs)
        self._auction_cycle += 1
        for uid,key in seeds.items():
            if key is None:
                continue
            if self._subpoint_views(key) and self._search_view_job(key) is None:
                view = self._select_building_view(uid, key)
                if view is None:
                    self.lease.force_expire(uid, key)
                    self.allocator.failed_visit[key] = stamp
                    continue
                point, look = view
                review = getattr(self, '_review_subpoint_views', {}).get(key)
                max_hold = (min(self._segment_max_hold, review['expires_s']-stamp)
                            if review is not None else None)
                task = self._start_search_segment(uid, [key], [point], stamp,
                    purpose='building_review', look_points=[look], max_hold=max_hold)
                if task is None:
                    self.lease.force_expire(uid, key)
                    continue
                rospy.loginfo('[BUILDING_REVIEW] uav=%s task=%s cell=%s point=%s look=%s; '
                              'free ground, checked path, no coverage credit',
                              uid, task.task_id, key, point, look)
                continue
            cells,points = self._segment_cells(uid,key,stamp)
            if getattr(self, '_directed_coverage', False) and self._search_view_job(key) is None:
                proposal = self._directed_segment(uid, cells, stamp)
                if proposal is None:
                    # Missing/stale height or impossible view remains search
                    # debt; do not fallback to a supposedly visible centre.
                    self.lease.force_expire(uid, key)
                    continue
                cells, points, looks = proposal
                task = self._start_search_segment(uid, cells, points, stamp,
                    purpose='coverage_view', look_points=looks)
                if task is None:
                    self.lease.force_expire(uid, key)
                    continue
                rospy.loginfo('[DIRECTED_COVERAGE] uav=%s task=%s cells=%s points=%s looks=%s; '
                              'reserved samples, actual images required',
                              uid, task.task_id, cells, points, looks)
                continue
            task = self._start_search_segment(uid,cells,points,stamp)
            if task is None:
                self.lease.force_expire(uid,key)
                continue
            # Keep the existing assignment-bound missed-view hints for a
            # one-cell observation task. They never count as fresh detection.
            self._segment_view_hint(task)
            rospy.loginfo('[SEARCH_SEGMENT_START] uav=%s task=%s v=%d cells=%s points=%s',
                          uid,task.task_id,task.revision,cells,points)
        self._last_alloc_t = now
        if any(key is None for key in seeds.values()):
            self._reopen_covered_cells(force=True)

    def _start_search_segment(self, uid, cells, points, stamp, purpose='coverage', publish=True,
                              max_hold=None, look_points=None):
        """Single reservation/lease entry for internal and boundary search."""
        from routine_search import routine_point_allowed
        look = ([(self.grid.cell(key).cx, self.grid.cell(key).cy) for key in cells]
                if look_points is None else look_points)
        if getattr(self, '_routine_search_bounds', False) and (len(points) != len(look) or
                any(not routine_point_allowed(point, ground)
                    for point, ground in zip(points, look))):
            return None  # change neither coordinates, coverage nor leases
        evidence_deadlines=[expires for (kind,key),(ready,expires) in
            self.allocator.revisit_jobs.items() if key in cells and
            kind in ('lost','unconfirmed') and ready<=stamp<expires]
        max_hold=self._segment_max_hold if max_hold is None else max_hold
        if evidence_deadlines and purpose in ('coverage', 'coverage_view'):
            max_hold=min(max_hold,min(evidence_deadlines)-stamp)
        if purpose == 'building_review':
            blind_deadlines = [expires for (kind, key), (_, expires) in
                self.allocator.revisit_jobs.items()
                if kind == 'blind' and key in cells and expires > stamp]
            if blind_deadlines:
                max_hold = min(max_hold, min(blind_deadlines)-stamp)
        task = self._segments.reserve(uid,cells,points,stamp,min(20.,max_hold),max_hold,
                                      purpose=purpose, look_points=look)
        if task is None:
            return None
        for cell in cells:
            # SegmentBook already excludes other routes. A revisit must not
            # erase historical image coverage merely by reserving a cell.
            if self.grid.cell(cell).state != STATE_COVERED:
                self.lease.grant(uid,cell,stamp,task.expires-stamp)
        self._active_leases[uid] = cells[0]
        if publish:
            self._send_segment(task)
        return task

    def _segment_view_hint(self, task):
        from search_view_hint import make_hint
        from std_msgs.msg import String
        import json
        view = self._search_view_job(task.cells[0])
        publisher = getattr(self,'_search_view_hint_pub',None)
        if view is None or publisher is None or math.dist(task.points[0],(view['x'],view['y'])) > 1e-6:
            return
        msg = SearchAssignment()
        msg.uav_id = task.uid
        msg.header.stamp = rospy.Time.from_sec(task.created)
        msg.cell_ix,msg.cell_iy = task.cells[0]
        msg.target_x,msg.target_y = task.points[0]
        msg.task_type = 0
        hint = make_hint(view,msg)
        if hint is not None:
            publisher.publish(String(data=json.dumps(hint)))

    def _segment_visit(self, uid, key, now):
        cell = self.grid.cell(key)
        observed = cell.state == STATE_COVERED
        self.lease.force_expire(uid,key)
        self.allocator.complete_waypoint(uid,key,observed=observed,now=now)
        if not observed:
            self.grid.mark_review(key,0.)
            st = self.status[uid]
            side = self._choose_blind_viewpoint(key,cell,(st.x,st.y))
            if side is not None:
                self._blind_viewpoints[key] = side
            self.allocator.queue_revisit('blind',cell.cx,cell.cy,now=now,
                delay_s=8.,ttl_s=90.,blocked=self._blocked_cells)

    def _renew_segments(self, now):
        stamp = now.to_sec()
        for uid,task in list(self._segments.tasks.items()):
            if uid in self._tracking.values() or uid in self._backup.values():
                self._drop_segment(uid,'person_task')
                continue
            st = self.status.get(uid)
            received = self.last_report.get(uid)
            age = stamp-received.to_sec() if received is not None else float('inf')
            if stamp >= task.deadline or stamp >= task.expires or age >= 3. or st is None or not st.connected:
                self._drop_segment(uid,'timeout_or_unavailable')
                continue
            if not self._tracking_uav_available(uid, stamp):
                self._drop_segment(uid, 'camera_unavailable')
                continue
            sample = st.header.stamp.to_sec()
            valid = (self._segments.matches(uid,st.search_task_id,st.search_revision) and
                     task.created <= sample <= stamp+.05 and sample > task.feedback_s and
                     st.search_state in (TRAVEL,LOOK,ATTEND,YIELD,DONE,FAILED) and
                     all(math.isfinite(v) for v in (st.x,st.y,st.z,st.search_path_remaining)))
            if valid:
                if (task.purpose in ('building_review', 'coverage_view') and st.search_state == FAILED and
                        st.search_blocked_reason == 'observation_timeout'):
                    if st.search_index != task.index or task.index >= len(task.cells):
                        continue  # this failure cannot advance or complete a leg
                    task.feedback_s = sample
                    self._observation_timeout(task, stamp)
                    continue
                visited = self._segments.advance(uid,task.task_id,task.revision,st.search_index)
                if visited is None:
                    continue
                task.feedback_s = sample
                road = getattr(self, '_road_first_pass', None)
                if road is not None:
                    road.progress(task)
                for key in visited:
                    self._segment_visit(uid,key,stamp)
                if st.search_state == FAILED:
                    self._drop_segment(uid,'agent_failed',failed=True)
                    continue
                if st.search_state == DONE and task.index == len(task.cells):
                    self._drop_segment(uid,'complete')
                    continue
                if task.index == len(task.cells):
                    # A completed prefix alone never manufactures DONE.
                    continue
                blocked = st.search_blocked_waypoint-1
                if st.search_blocked_waypoint and task.index <= blocked < len(task.cells):
                    if st.search_blocked_reason == 'START_OCCUPIED' and blocked == task.index:
                        if task.blocked_since is None:
                            task.blocked_since = stamp
                        if stamp-task.blocked_since < 8.:
                            # Bounded local exit uses measured lidar and raw
                            # cells. Changing task/version cannot buy more time.
                            self._send_segment(task)
                            continue
                    if blocked > task.index:
                        # Keep the reachable prefix, but remember the exact
                        # failed destination. Releasing it without this record
                        # let the very next auction issue the same blocked goal.
                        self.allocator.failed_visit[task.cells[blocked]] = stamp
                        self._revise_segment(task,task.cells[task.index:blocked],
                            task.points[task.index:blocked],stamp,'local_'+st.search_blocked_reason)
                    else:
                        self._drop_segment(uid,'local_'+st.search_blocked_reason,failed=True)
                        self._release_cooldown[uid] = now+rospy.Duration(2.)
                    continue
                task.blocked_since = None
                self._active_leases[uid] = task.cells[task.index]
                current = (st.search_path_revision,st.search_path_remaining,st.x,st.y)
                if visited:
                    task.progress_s,task.progress = stamp,current
                elif task.progress is None:
                    task.progress = current
                elif (current[0] == task.progress[0] and current[1] >= 0 and
                      task.progress[1]-current[1] >= .8 and
                      math.dist(current[2:],task.progress[2:]) >= .5):
                    task.progress_s,task.progress = stamp,current
                elif current[0] != task.progress[0]:
                    # Replans change the arc baseline, not the stall deadline.
                    task.progress = current
                waiting = st.search_state in (LOOK,ATTEND,YIELD)
                task.wait_since = ((task.wait_since if task.wait_since is not None else stamp)
                                   if waiting else None)
                if st.search_state == YIELD and stamp-task.wait_since >= 8.:
                    # Deterministically make only the larger-id participant
                    # leave its old route; the other participant keeps its task.
                    neighbours = [other for other,s in self.status.items() if other != uid and
                                  s.connected and math.hypot(s.x-st.x,s.y-st.y) < 12.]
                    if neighbours and uid > min(neighbours) and not task.yield_replanned:
                        task.yield_replanned = True
                        key = task.cells[task.index]
                        point = self._choose_blind_viewpoint(key,self.grid.cell(key),(st.x,st.y))
                        if point is not None and math.dist(point,task.points[task.index]) >= 2.:
                            self._revise_segment(task,[key],[point],stamp,'yield_replan')
                            continue
                        self._drop_segment(uid,'yield_reallocate')
                        self._release_cooldown[uid] = now+rospy.Duration(3.)
                        continue
                grace = 18. if waiting else 0.
                if stamp-task.progress_s > self._segment_stall_s+grace:
                    self._drop_segment(uid,'no_path_progress',failed=not waiting)
                    continue
                task.expires = min(stamp+20.,task.deadline)
            # A fresh disconnected/old-task status cannot renew a new task.
            # Re-send until its first ACK, retaining the original finite lease.
            for key in task.cells[task.index:]:
                cell = self.grid.cell(key)
                if cell.state != STATE_COVERED:
                    self.lease.grant(uid,key,stamp,max(0.,task.expires-stamp))
            self._send_segment(task)
        self._last_lease_t = now

    def _observation_timeout(self, task, stamp):
        """Recover a safe missed image using the original finite blind job."""
        key = task.cells[task.index]
        look, point = task.look_points[task.index], task.points[task.index]
        job = self.allocator.revisit_jobs.get(('blind', key))
        review = getattr(self, '_review_subpoint_views', {}).get(key)
        deadlines = [stamp+90.]
        if job is not None:
            deadlines.append(job[1])
        if review is not None:
            deadlines.append(review['expires_s'])
        expires = min(deadlines)
        altitude = (self._directed_camera_height(task.uid, stamp)
                    if task.purpose == 'coverage_view' else 4.5)
        safe = (expires > stamp and key not in self._blocked_cells and altitude is not None and
                valid_view(self._merged_grid(), getattr(self, '_g_infl', None),
                           look, point, self._segment_flight_waypoint, altitude))
        # Unavailable height is not evidence that this physical point failed.
        self._drop_segment(task.uid, 'observation_timeout', failed=not safe and altitude is not None)
        cell = self.grid.cell(key)
        if not safe or cell.state == STATE_COVERED:
            return
        # Unlike _segment_visit, a timeout records no arrival, image or search
        # progress. Preserve historical credit and reopen only the missed view.
        self.grid.mark_review(key, 0.)
        status = self.status[task.uid]
        side = self._choose_blind_viewpoint(key, cell, (status.x, status.y))
        if side is not None:
            self._blind_viewpoints[key] = side
        review = getattr(self, '_review_subpoint_views', {}).get(key)
        if review is not None:
            review['expires_s'] = min(review['expires_s'], expires)
        if job is None:
            self.allocator.queue_revisit('blind', cell.cx, cell.cy, now=stamp,
                delay_s=8., ttl_s=expires-stamp, blocked=self._blocked_cells)
        else:
            # Retrying may postpone readiness but never renew the episode TTL.
            self.allocator.revisit_jobs[('blind', key)] = (max(job[0], stamp+8.), expires)

    def _revise_segment(self, task, cells, points, now, reason):
        if task.purpose == 'coverage_view':
            altitude = self._directed_camera_height(task.uid, now)
            if altitude is None:
                return  # stale geometry may not authorize a new viewpoint
            looks = dict(zip(task.cells, task.look_points))
            raw = self._merged_grid()
            if any(key not in looks or not valid_view(raw, getattr(self, '_g_infl', None),
                    looks[key], point, self._segment_flight_waypoint, altitude)
                    for key, point in zip(cells, points)):
                self._drop_segment(task.uid, reason+'_invalid_view', failed=False)
                return
        old = set(task.cells[task.index:])
        changed = self._segments.revise(task.uid,cells,points,now)
        if changed is None:
            return
        for key in old-set(cells):
            self.lease.force_expire(task.uid,key)
        self._active_leases[task.uid] = cells[0]
        self._send_segment(task)
        rospy.loginfo('[SEARCH_SEGMENT_REVISE] uav=%s task=%s v=%d reason=%s cells=%s',
                      task.uid,task.task_id,task.revision,reason,cells)

    def _reconcile_segments(self):
        grid = getattr(self,'_g_infl',None)
        if grid is None:
            return
        now = rospy.Time.now().to_sec()
        for uid,task in list(self._segments.tasks.items()):
            if task.index >= len(task.cells):
                continue
            cells = list(task.cells[task.index:])
            points = list(task.points[task.index:])
            if not self._search_point_free(points[0]):
                if task.purpose in ('south_patrol', 'building_review', 'coverage_view'):
                    # A normal cell-centre replacement loses the checked
                    # camera standoff. Release it through the same regional
                    # allocator, preserving its original attempt/time budget.
                    self._drop_segment(uid,'blocked_viewpoint',failed=True)
                    continue
                replacement = self._segment_point(cells[0])
                if not self._search_point_free(replacement):
                    self._drop_segment(uid,'blocked_viewpoint',failed=True)
                    continue
                self._revise_segment(task,cells[:1],[replacement],now,'new_obstacle')
                continue
            if task.purpose in ('building_review', 'coverage_view'):
                look = task.look_points[task.index]
                altitude = (self._directed_camera_height(uid, now)
                            if task.purpose == 'coverage_view' else 4.5)
                if altitude is not None and not valid_view(self._merged_grid(), grid, look, points[0],
                                  self._segment_flight_waypoint, altitude):
                    self._drop_segment(uid, 'blocked_ground_view',
                                       failed=task.purpose == 'building_review')
                    continue
            for i in range(1,len(points)):
                if not clear_line(grid,points[i-1],points[i]):
                    self._revise_segment(task,cells[:i],points[:i],now,'blocked_connection')
                    break
