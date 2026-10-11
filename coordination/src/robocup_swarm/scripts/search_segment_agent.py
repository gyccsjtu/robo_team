"""Aircraft execution of versioned short search segments; no gimbal coupling."""
import math
import rospy
from search_segments import (SegmentBook, TRAVEL, LOOK, ATTEND, YIELD, DONE, FAILED,
                             clear_line, path_remaining)
from robocup_navigation.astar import plan


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
            try:
                SegmentBook.validate(cells,points)
            except (ValueError,TypeError):
                return False
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
                    if cells == current['cells'] and points == current['points']:
                        current['until'] = max(current['until'],until)
                    return False  # heartbeat cannot reset index, scan or attention
            elif sample < getattr(self,'_last_task_stamp',-1.):
                return False
            self._close_segment(retire=current is None or current['id'] != tid)
            self._discard_old_plan()
            self._search_scan_key = None
            self._search_segment = dict(id=tid,revision=revision,cells=cells,points=points,
                index=0,until=until,state=TRAVEL,created=sample)
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
            return [],points[-1] if points else (0.,0.),False
        path = [tuple(start)]
        for point in points:
            route = plan(grid,start,point,connectivity=8,prevent_corner_cutting=True)
            if not route.success:
                rospy.logwarn_throttle(5.,'[%s] search segment route failed: %s',self.uav_id,route.reason)
                return [],points[-1],False
            leg = [tuple(start)]+list(route.points)+[tuple(point)]
            # Keep every obstacle-required turn. No blind straight-line fallback
            # and no spline that cuts across an inflated building corner.
            if any(not clear_line(grid,a,b) for a,b in zip(leg,leg[1:])):
                return [],points[-1],False
            path.extend(leg[1:])
            start = point
        return path,points[-1],True

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
        point = task['points'][i]
        distance = math.dist(self.world_xy,point)
        if distance < arrive:
            camera = getattr(self,'_search_camera_pose',None)
            fresh = camera is not None and 0 <= now-camera[0] <= .5
            missing = None
            if fresh:
                _,x,y,_yaw,height,pitch = camera
                if math.dist((x,y),self.world_xy) <= 1.:
                    observable = self._search_observable_points(x,y,height,pitch)
                    missing = set(observable)-self._search_scan_seen
            if missing == set() or self._search_scan_done or self._search_scan_failed:
                # Empty observable set is a visit, never image coverage. The
                # manager explicitly queues the unobserved cell for a side view.
                self._segment_next()
                if task['state'] == DONE:
                    self._send_vel(0.,0.)
                    return True
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
        local = self._pick_local_goal()
        if local is None:
            self._send_vel(0.,0.)
            return True
        if not clear_line(self.grid,self.world_xy,local):
            nearest = min(range(len(self.path)),key=lambda n:math.dist(self.path[n],self.world_xy))
            visible = None
            for candidate in self.path[nearest+1:]:
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
        heading = self._search_sweep_bearing(math.atan2(dy,dx),distance)
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
        cue = getattr(self,'_camera_cue',None)
        if cue is not None and rospy.Time.now().to_sec() < cue['until']:
            st.search_state = ATTEND
        st.search_path_revision = getattr(self,'_segment_path_sequence',0)
        st.search_path_remaining = path_remaining(self.path,self.world_xy)
