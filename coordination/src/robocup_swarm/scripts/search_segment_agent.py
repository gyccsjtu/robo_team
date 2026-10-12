"""Aircraft execution of versioned short search segments; no gimbal coupling."""
import math
import rospy
from search_segments import (SegmentBook, TRAVEL, LOOK, ATTEND, YIELD, DONE, FAILED,
                             clear_line, path_remaining)
from robocup_navigation.astar import plan
from search_observation import cell_points, ground_distance_band, in_view
from swarm_task import ACTOR_SEARCH_BOUNDS
from building_viewpoints import DIRECTED_PURPOSES, reserved_subpoint


def _reserved_patrol_corner(coverage, key, point):
    """Admit only the two exact eastern corner subpoints of a legal grid.

    A corner look differs from its cell centre, but remains bound to that
    same reserved cell. Other points inside that cell are not new protocol
    permissions; normal coverage observations continue to use the centre.
    """
    cell = coverage.cell(key)
    if cell is None or not hasattr(coverage, 'cell_bounds'):
        return False
    xmin, xmax, ymin, ymax = ACTOR_SEARCH_BOUNDS
    if not xmin <= point[0] <= xmax or not ymin <= point[1] <= ymax:
        return False
    if coverage.world_to_cell(*point) != key:
        return False
    x0, x1, y0, y1 = coverage.cell_bounds(key)
    if (not all(math.isfinite(v) for v in (x0, x1, y0, y1)) or
            not math.isclose(x1, coverage.x_max, abs_tol=1e-6)):
        return False
    points = cell_points(coverage, cell)
    if len(points) != 9:
        return False
    return ((math.isclose(y0, coverage.y_min, abs_tol=1e-6) and
             math.dist(point, points[2]) <= 1e-6) or
            (math.isclose(y1, coverage.y_max, abs_tol=1e-6) and
             math.dist(point, points[8]) <= 1e-6))


class SearchSegmentAgent:
    def _accept_segment_message(self, msg):
        if msg.uav_id != self.uav_id:
            return True
        current = getattr(self,'_search_segment',None)
        tid = getattr(msg,'search_task_id','')
        revision = getattr(msg,'search_revision',0)
        stamp = getattr(getattr(msg,'header',None),'stamp',None)
        if stamp is None:
            # Legacy in-process callers cannot touch an active v2 task.
            return not tid and current is None
        sample = stamp.to_sec()
        closed = self.__dict__.setdefault('_closed_search_segments',{})
        if msg.task_type == 255 and tid:
            if current is None or (current['id'],current['revision']) != (tid,revision):
                return False  # delayed search cancellation cannot clear pursuit
        elif msg.task_type == 255 and msg.target_id and current is not None:
            return False  # delayed cancellation of the previous person task
        elif msg.task_type == 0 and tid:
            now = rospy.Time.now().to_sec()
            if tid in closed or not math.isfinite(sample) or not 0 <= sample <= now+.05:
                return False
            cells = tuple(zip(msg.search_cell_ix,msg.search_cell_iy))
            points = tuple((p.x,p.y) for p in msg.search_waypoints)
            purpose = getattr(msg, 'search_purpose', '') or 'coverage'
            looks = tuple((p.x,p.y) for p in getattr(msg, 'search_look_points', ()))
            try:
                SegmentBook.validate(cells,points)
                if looks:
                    SegmentBook.validate(cells, looks)
            except (ValueError,TypeError):
                return False
            if (purpose not in ('coverage', *DIRECTED_PURPOSES) or
                    (purpose in DIRECTED_PURPOSES and not looks) or
                    (purpose == 'building_review' and len(cells) != 1)):
                return False
            if looks:
                coverage = getattr(self, 'cov_grid', None)
                if coverage is None:
                    return False
                for key, point in zip(cells, looks):
                    cell = coverage.cell(key)
                    if purpose == 'building_review' and not reserved_subpoint(coverage,key,point):
                        return False
                    if cell is None or (math.dist(point, (cell.cx, cell.cy)) > 1e-6 and
                            not (purpose == 'south_patrol' and
                                 _reserved_patrol_corner(coverage, key, point)) and
                            not (purpose == 'building_review' and
                                 reserved_subpoint(coverage, key, point))):
                        return False  # observation region must match its reservation
            if len(msg.search_cell_ix) != len(msg.search_cell_iy) or revision < 1:
                return False
            if any(self._segment_flight_waypoint(*point) != point for point in points):
                return False
            until = msg.search_valid_until.to_sec()
            if not now < until or until > now+120.:
                return False
            if current is not None and current['id'] == tid:
                if revision < current['revision']:
                    return False
                if revision == current['revision']:
                    if (cells == current['cells'] and points == current['points'] and
                            looks == current.get('look_points', ()) and
                            purpose == current.get('purpose', 'coverage')):
                        current['until'] = max(current['until'],until)
                    return False  # heartbeat cannot reset index, scan or attention
            elif sample < getattr(self,'_last_task_stamp',-1.):
                return False
            old_look = None
            if current is not None and current['index'] < len(current.get('look_points', ())):
                old_look = current['look_points'][current['index']]
            same_view = (current is not None and current['id'] == tid and
                         current['index'] < len(current['cells']) and
                         cells[0] == current['cells'][current['index']] and
                         points[0] == current['points'][current['index']] and
                         purpose == current.get('purpose', 'coverage') and
                         old_look == (looks[0] if looks else None))
            self._close_segment(retire=current is None or current['id'] != tid)
            self._discard_old_plan()
            if not same_view:
                self._search_scan_key = None
            self._search_segment = dict(id=tid,revision=revision,cells=cells,points=points,
                index=0,until=until,state=TRAVEL,created=sample,blocked=0,reason='',
                purpose=purpose,look_points=looks)
            self._segment_path_key = None
            self._last_task_stamp = max(sample,getattr(self,'_last_task_stamp',-1.))
            return True
        elif sample < getattr(self,'_last_task_stamp',-1.):
            return False
        elif msg.task_type == 0 and current is not None:
            return False  # no silent protocol downgrade during an active segment
        self._last_task_stamp = max(sample,getattr(self,'_last_task_stamp',-1.))
        self._close_segment()
        return True

    def _close_segment(self, retire=True):
        task = getattr(self,'_search_segment',None)
        if task is not None and retire:
            closed = self.__dict__.setdefault('_closed_search_segments',{})
            closed[task['id']] = rospy.Time.now().to_sec()
            while len(closed)>128:
                closed.pop(next(iter(closed)))
        self._search_segment = None
        self._segment_path_key = None

    def _segment_identity(self):
        task = getattr(self,'_search_segment',None)
        return (task['id'],task['revision'],task['index']) if task is not None else None

    def _compute_segment_plan(self, points):
        grid = self.grid
        start = self.world_xy
        if start is None or not points:
            return [],points[-1] if points else (0.,0.),False,1,'NO_START'
        path = [tuple(start)]
        for index,point in enumerate(points):
            route = plan(grid,start,point,connectivity=8,prevent_corner_cutting=True)
            if not route.success:
                reason = route.reason
                leg = None
            else:
                reason = 'UNSAFE_CONNECTION'
                leg = [tuple(start)]+list(route.points)+[tuple(point)]
            # Keep every obstacle-required turn. No blind straight-line fallback
            # and no spline that cuts across an inflated building corner.
            if leg is None or any(not clear_line(grid,a,b) for a,b in zip(leg,leg[1:])):
                rospy.logwarn_throttle(5.,
                    '[%s] search segment blocked leg=%d/%d point=(%.2f,%.2f) reason=%s',
                    self.uav_id,index+1,len(points),point[0],point[1],reason)
                # Only this verified prefix may execute. The task itself is
                # shortened by the manager in a new revision, never locally.
                return (path if index else []),start,index>0,index+1,reason
            path.extend(leg[1:])
            start = point
        return path,points[-1],True,0,''

    def _segment_next(self):
        task = self._search_segment
        task['index'] += 1
        with self._plan_lock:
            self._plan_epoch += 1
            self._plan_pending = None
        if task['index'] >= len(task['cells']):
            task['state'] = DONE
            self.path = []
            self._last_flight_v = (0.,0.)
            return
        i = task['index']
        self.assignment.cell_ix,self.assignment.cell_iy = task['cells'][i]
        self.assignment.target_x,self.assignment.target_y = task['points'][i]
        self._search_scan_key = (*task['cells'][i],*map(lambda x:round(x,2),task['points'][i]))
        self._search_scan_yaw0 = None
        self._search_scan_started_t = None
        self._search_scan_settle_t = None
        self._search_scan_directed_first = None
        self._search_scan_directed_last = None
        self._search_scan_directed_done = False
        self._search_scan_phase = 0
        self._search_scan_done = self._search_scan_failed = False
        self._search_scan_seen,self._search_scan_first = set(),{}
        task['state'] = TRAVEL

    def _segment_control(self, cruise, max_speed, kp, decel, arrive):
        task = getattr(self,'_search_segment',None)
        if task is None:
            return False
        now = rospy.Time.now().to_sec()
        if now >= task['until']:
            task['state'] = FAILED
            self._discard_old_plan()
        if task['state'] in (DONE,FAILED):
            self._send_vel(0.,0.)
            return True
        i = task['index']
        if task.get('blocked',0) and i >= task['blocked']-1:
            if task.get('reason') == 'START_OCCUPIED':
                own = self.grid.world_to_cell(self.world_xy)
                if self.grid.is_free(own):
                    self._request_plan(task['points'][-1])
                elif getattr(self.grid,'is_free_raw',lambda _:False)(own):
                    # Strict A* cannot start inside inflation. Reuse the
                    # existing slow, lidar-checked raw-free exit rather than
                    # introducing a straight-through planning fallback.
                    dx = task['points'][i][0]-self.world_xy[0]
                    dy = task['points'][i][1]-self.world_xy[1]
                    vx,vy = self._occupied_cell_exit(dx,dy)
                    vx,vy = self._apply_friend_avoidance(vx,vy)
                    task['state'] = YIELD
                    self._send_vel(vx,vy)
                    return True
            self._send_vel(0.,0.)
            return True  # await explicit revision/cancellation of blocked tail
        point = task['points'][i]
        distance = math.dist(self.world_xy,point)
        directed = (task.get('purpose') in DIRECTED_PURPOSES and
                    i < len(task.get('look_points', ())))
        at_view = distance < arrive
        if at_view and directed and task['state'] != LOOK:
            # A transit tolerance is not proof of an observable viewpoint.
            # Start the bounded scan only when a fresh aligned camera can
            # actually view the reserved region after turning toward it.
            at_view = False
            camera = getattr(self,'_search_camera_pose',None)
            if (self._world_trusted() and
                    0. <= now-getattr(self,'_pose_sample_s',-1e18) <= .3 and
                    camera is not None and 0. <= now-camera[0] <= .5):
                _,x,y,_yaw,height,pitch = camera
                if math.dist((x,y),self.world_xy) <= 1.:
                    look = task['look_points'][i]
                    bearing = math.atan2(look[1]-y,look[0]-x)
                    at_view = in_view(x,y,bearing,*look,height,pitch)
        if at_view or (directed and task['state'] == LOOK):
            camera = getattr(self,'_search_camera_pose',None)
            fresh = camera is not None and 0 <= now-camera[0] <= .5
            missing = None
            if fresh:
                _,x,y,_yaw,height,pitch = camera
                if math.dist((x,y),self.world_xy) <= 1.:
                    observable = self._search_observable_points(x,y,height,pitch)
                    missing = set(observable)-self._search_scan_seen
            if self._search_scan_done or self._search_scan_failed or (
                    not directed and missing == set()):
                # A forward camera can arrive facing the outer wall. Reuse
                # the same bounded yaw executor for a useful inward/edge look
                # before advancing; a person cue preempts this in _control.
                if not self._search_scan_failed and self._hold_edge_review():
                    task['state'] = LOOK
                    return True
                # An ordinary empty observable set is only a visit. A directed
                # patrol must first point its camera at the explicit region
                # and receive processed frames, or end with a bounded failure.
                self._segment_next()
                if task['state'] == DONE:
                    self._send_vel(0.,0.)
                    return True
                if task.get('blocked',0) and task['index'] >= task['blocked']-1:
                    self._send_vel(0.,0.)
                    return True
                point = task['points'][task['index']]
            else:
                task['state'] = LOOK
                self._search_scan_step()
                return True
        target = task['points'][-1]
        key = (task['id'],task['revision'])
        if self._segment_path_key != key or not self.path:
            if now-self._plan_fail_t >= 2.:
                self._request_plan(target)
            self._send_vel(0.,0.)
            return True
        route = self.path
        if directed:
            # The full plan contains later observation points. Its ordinary
            # lookahead may skip the current one, so use only its verified leg.
            stop = next((n for n,p in enumerate(route)
                         if math.dist(p,point) <= 1e-6),None)
            if stop is None:
                self._request_plan(target)
                self._send_vel(0.,0.)
                return True
            route = route[:stop+1]
        local = self._pick_local_goal(route) if directed else self._pick_local_goal()
        if local is None:
            self._send_vel(0.,0.)
            return True
        if not clear_line(self.grid,self.world_xy,local):
            nearest = min(range(len(route)),key=lambda n:math.dist(route[n],self.world_xy))
            visible = None
            for candidate in route[nearest+1:]:
                if not clear_line(self.grid,self.world_xy,candidate):
                    break
                visible = candidate
                if math.dist(self.world_xy,candidate) >= 2.:
                    break
            local = visible
        if local is None:
            self._request_plan(target)
            self._send_vel(0.,0.)
            return True
        dx,dy = local[0]-self.world_xy[0],local[1]-self.world_xy[1]
        distance = math.hypot(dx,dy)
        # Lateral sensing has not been qualified for blind continuous patrol.
        # Travel facing the route; at each reserved boundary cell the same
        # bounded scan executor observes its explicit ground region. No second
        # yaw publisher or inferred coverage is introduced.
        heading = math.atan2(dy,dx)
        if task.get('purpose') not in DIRECTED_PURPOSES:
            heading = self._search_sweep_bearing(heading,distance)
        self._set_search_heading(heading)
        self._last_local_goal = local
        speed = min(max_speed,cruise if distance>decel else kp*distance,self._transit_cap())
        vx,vy = dx*speed/max(distance,1e-9),dy*speed/max(distance,1e-9)
        vx,vy = self._search_start_cap(vx,vy)
        desired = math.hypot(vx,vy)
        vx,vy = self._apply_friend_avoidance(vx,vy)
        task['state'] = YIELD if desired>.3 and math.hypot(vx,vy)<.25 else TRAVEL
        self._send_vel(vx,vy)
        return True

    def _segment_status(self, st):
        task = getattr(self,'_search_segment',None)
        st.search_path_remaining = -1.
        if task is None:
            return
        st.search_task_id,st.search_revision = task['id'],task['revision']
        st.search_index,st.search_state = task['index'],task['state']
        st.search_blocked_waypoint = task.get('blocked',0)
        st.search_blocked_reason = task.get('reason','')
        cue = getattr(self,'_camera_cue',None)
        if cue is not None and rospy.Time.now().to_sec() < cue['until']:
            st.search_state = ATTEND
        st.search_path_revision = getattr(self,'_segment_path_sequence',0)
        st.search_path_remaining = path_remaining(self.path,self.world_xy)
