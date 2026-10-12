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
        for patrol in self._regional_search_patrols():
            patrol.ended(task, reason, now)
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

    def _allocate_segments(self, now):
        stamp = now.to_sec()
        self._renew_segments(now)
        idle = self._idle_uavs()
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
        positions = {uid:(self.status[uid].x,self.status[uid].y) for uid in idle}
        # Outer task lock covers allocator selection, segment extension and
        # whole-segment reservation. No callback can see a half-reserved route.
        seeds = self.allocator.allocate(positions, blocked=set(self._blocked_cells)|set(self._segments.reserved))
        self._auction_cycle += 1
        for uid,key in seeds.items():
            if key is None:
                continue
            cells,points = self._segment_cells(uid,key,stamp)
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
        evidence_deadlines=[expires for (kind,key),(ready,expires) in
            self.allocator.revisit_jobs.items() if key in cells and
            kind in ('lost','unconfirmed') and ready<=stamp<expires]
        max_hold=self._segment_max_hold if max_hold is None else max_hold
        if evidence_deadlines and purpose == 'coverage':
            max_hold=min(max_hold,min(evidence_deadlines)-stamp)
        look = ([(self.grid.cell(key).cx, self.grid.cell(key).cy) for key in cells]
                if look_points is None else look_points)
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
                visited = self._segments.advance(uid,task.task_id,task.revision,st.search_index)
                if visited is None:
                    continue
                task.feedback_s = sample
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

    def _revise_segment(self, task, cells, points, now, reason):
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
                if task.purpose == 'south_patrol':
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
            for i in range(1,len(points)):
                if not clear_line(grid,points[i-1],points[i]):
                    self._revise_segment(task,cells[:i],points[:i],now,'blocked_connection')
                    break
