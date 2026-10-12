"""Optional first southeast leg stays a safe, short, ordinary search task."""
import math
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import boundary_patrol as bp
import swarm_manager as sm
import swarm_task as st
from search_segments import SegmentBook, clear_line, TRAVEL
from search_observation import cell_points, in_view
from robocup_swarm.msg import UavStatus


class SoutheastPatrolTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        for context in (
                patch.object(sm.rospy.Time, 'now',
                             side_effect=lambda: sm.rospy.Time.from_sec(self.now)),
                patch.object(st.rospy, 'get_time', side_effect=lambda: self.now)):
            context.start()
            self.addCleanup(context.stop)
        manager = self.m = sm.SwarmManager.__new__(sm.SwarmManager)
        manager._task_lock = threading.RLock()
        manager._segments = SegmentBook(manager._task_lock)
        manager._segment_max_hold = 90.
        manager._segment_stall_s = 20.
        # These are the clipped flight bounds used with the official map.
        manager.grid = st.CoverageGrid(-50, 150, -60, 60, 7, 6)
        manager.allocator = st.TaskAllocator(manager.grid)
        manager.lease = st.LeaseManager(manager.grid)
        manager._g_infl = sm.GridMap(420, 260, .5, (-55., -65.),
                                     bytes(420*260))
        manager._cell_waypoint = {}
        manager._blocked_cells = set()
        manager._blind_viewpoints = {}
        manager._search_view_job = Mock(return_value=None)
        manager._tracking = {}
        manager._backup = {}
        manager._active_leases = {}
        manager._lease_hold_since = {}
        manager._lease_motion = {}
        manager._release_cooldown = {}
        manager.assign_pub = NS(publish=Mock())
        manager._auction_cycle = 0
        manager._reopen_covered_cells = Mock()
        manager._south_patrol = bp.SouthPatrol((-50, 150, -60, 60),
                                               southeast=True)
        manager.uav_ids = [f'typhoon_h480_{i}' for i in range(6)]
        manager.status = {}
        manager.last_report = {}
        for i, uid in enumerate(manager.uav_ids):
            status = UavStatus(uav_id=uid, connected=True,
                               x=float(i), y=0., z=2.8)
            status.header.stamp = sm.rospy.Time(100)
            manager.status[uid] = status
            manager.last_report[uid] = sm.rospy.Time(100)

    def test_first_segment_reserves_only_southeast_inner_cells(self):
        self.m._allocate()
        tasks = [t for t in self.m._segments.tasks.values()
                 if t.purpose == 'south_patrol']
        self.assertEqual(len(tasks), 1)
        task = tasks[0]
        self.assertIn(task.uid, self.m.uav_ids)
        self.assertLessEqual(len(task.cells), 3)
        self.assertEqual(len(self.m._segments.tasks), 6)
        self.assertEqual(set(task.cells), set(task.cells) &
                         set(self.m._segments.reserved))
        for key, point, look in zip(task.cells, task.points, task.look_points):
            cell = self.m.grid.cell(key)
            self.assertEqual(key[1], 2)
            self.assertTrue(50. <= cell.cx <= 110.)
            self.assertAlmostEqual(cell.cy, -42.5)
            self.assertEqual(look, (cell.cx, cell.cy))
            self.assertAlmostEqual(point[0], cell.cx)
            self.assertAlmostEqual(point[1], cell.cy-8.95)
            self.assertGreaterEqual(math.dist(point, look), 8.8)
            self.assertLess(math.dist(point, look), 9.)
            self.assertGreater(look[1], point[1])
            self.assertTrue(self.m._search_point_free(point))
            self.assertTrue(self.m._blind_viewpoint_valid(cell, point))
            self.assertTrue(clear_line(self.m._g_infl, point, look))
            self.assertNotEqual(cell.state, st.STATE_COVERED)
        self.assertFalse(any(key[1] == 0 for key in task.cells))
        self.assertAlmostEqual(task.deadline, 190.)

    def test_realistic_heights_keep_centre_visible_without_claiming_edges(self):
        proposal = self.m._south_patrol.choose(self.m, self.m.uav_ids,
                                                self.now)
        self.assertIsNotNone(proposal)
        key, point = proposal[1][0], proposal[2][0]
        cell = self.m.grid.cell(key)
        yaw = math.pi/2.
        for height in (2.6, 4.0, 4.45, 4.5):
            with self.subTest(height=height):
                self.assertTrue(in_view(*point, yaw, cell.cx, cell.cy,
                                        height, 0.))
        self.assertFalse(in_view(*point, yaw, cell.cx, cell.cy, 4.6, 0.))
        footprint = cell_points(self.m.grid, cell)
        self.assertEqual(len(footprint), 9)
        visible = [p for p in footprint
                   if in_view(*point, yaw, *p, 4.45, 0.)]
        self.assertEqual(visible, [(cell.cx, cell.cy)])
        south = min(footprint, key=lambda p: p[1])
        north = max(footprint, key=lambda p: p[1])
        self.assertFalse(in_view(*point, yaw, *south, 4.45, 0.))
        self.assertFalse(in_view(*point, yaw, *north, 4.45, 0.))
        self.assertNotEqual(cell.state, st.STATE_COVERED)

    def test_aircraft_too_high_for_nine_metre_footprint_is_not_pretended_visible(self):
        for uid in self.m.uav_ids:
            self.m.status[uid].z = 4.6
        proposal = self.m._south_patrol.choose(self.m, self.m.uav_ids,
                                                self.now)
        self.assertIsNone(proposal)

    def test_completed_inner_leg_reverts_to_south_edge_with_same_budget(self):
        patrol = self.m._south_patrol
        first = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(first)
        uid, cells, _, direction = first
        self.assertTrue(all(key[1] == 2 for key in cells))
        task = NS(uid=uid, task_id='inner', deadline=190., expires=120.)
        patrol.started(task, direction, self.now)
        patrol.ended(task, 'complete', 105.)
        self.assertEqual(patrol.attempts, 1)
        self.assertAlmostEqual(patrol.deadline, 190.)
        self.now = 105.
        self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        second = patrol.choose(self.m, [uid], self.now)
        self.assertIsNotNone(second)
        self.assertEqual(second[0], uid)
        self.assertTrue(all(key[1] == 0 for key in second[1]))
        edge_task = NS(uid=uid, task_id='edge', deadline=195., expires=125.)
        patrol.started(edge_task, second[3], self.now)
        self.assertAlmostEqual(edge_task.deadline, 190.)
        self.assertFalse(any(key[1] == 0 and
                             self.m.grid.cell(key).state == st.STATE_COVERED
                             for key in self.m.grid.cells))

    def test_failed_initial_leg_cannot_regrant_southeast_priority(self):
        patrol = self.m._south_patrol
        first = patrol.choose(self.m, self.m.uav_ids, 100.)
        self.assertIsNotNone(first)
        uid = first[0]
        task = NS(uid=uid, task_id='inner', deadline=190., expires=120.)
        patrol.started(task, first[3], 100.)
        patrol.ended(task, 'person_task', 101.)
        self.assertIsNone(patrol.uid)
        self.assertAlmostEqual(patrol.deadline, 190.)
        self.now = 162.
        for name in self.m.uav_ids:
            self.m.last_report[name] = sm.rospy.Time.from_sec(self.now)
        second = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(second)
        self.assertTrue(all(key[1] == 0 for key in second[1]))

    def test_inner_paths_unavailable_fall_back_to_original_south_row(self):
        actual_plan = bp.plan

        def block_inner(grid, start, end, **kwargs):
            if 50. <= end[0] <= 110. and end[1] < -50.:
                return NS(success=False)
            return actual_plan(grid, start, end, **kwargs)

        with patch.object(bp, 'plan', side_effect=block_inner):
            proposal = self.m._south_patrol.choose(
                self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(proposal)
        self.assertTrue(all(key[1] == 0 for key in proposal[1]))

    def test_default_flag_off_preserves_original_south_edge(self):
        self.m._south_patrol = bp.SouthPatrol((-50, 150, -60, 60),
                                              southeast=False)
        proposal = self.m._south_patrol.choose(self.m, self.m.uav_ids,
                                                self.now)
        self.assertIsNotNone(proposal)
        self.assertTrue(all(key[1] == 0 for key in proposal[1]))
        for key, point in zip(proposal[1], proposal[2]):
            cell = self.m.grid.cell(key)
            self.assertAlmostEqual(point[1]-cell.cy, 8.95)
            self.assertTrue(self.m._blind_viewpoint_valid(cell, point))
            self.assertTrue(in_view(*point, -math.pi/2., cell.cx,
                                    cell.cy, 4.45, 0.))

    def test_northeast_first_segment_uses_one_airborne_uav_and_eastward_view(self):
        self.m._south_patrol = bp.SouthPatrol((-50, 150, -60, 60),
                                              southeast=False, northeast=True)
        self.m.status[self.m.uav_ids[0]].z = .2
        self.m._allocate()
        tasks = [t for t in self.m._segments.tasks.values()
                 if t.purpose == 'south_patrol']
        self.assertEqual(len(tasks), 1)
        task = tasks[0]
        self.assertNotEqual(task.uid, self.m.uav_ids[0])
        self.assertLessEqual(len(task.cells), 3)
        self.assertAlmostEqual(task.deadline, 190.)
        for key, point, look in zip(task.cells, task.points, task.look_points):
            cell = self.m.grid.cell(key)
            self.assertTrue(105. <= cell.cx <= 133.)
            self.assertTrue(20. <= cell.cy <= 42.)
            self.assertEqual(look, (cell.cx, cell.cy))
            self.assertAlmostEqual(cell.cx-point[0], 8.95)
            self.assertAlmostEqual(cell.cy, point[1])
            self.assertTrue(in_view(*point, 0., *look, 4.45, 0.))
            self.assertTrue(self.m._search_point_free(point))
            self.assertTrue(self.m._blind_viewpoint_valid(cell, point))
            self.assertTrue(clear_line(self.m._g_infl, point, look))
            self.assertNotEqual(cell.state, st.STATE_COVERED)

    def test_northeast_initial_task_is_not_reissued_after_person_preemption(self):
        patrol = self.m._south_patrol = bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=True)
        first = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(first)
        task = NS(uid=first[0], task_id='northeast', deadline=190., expires=120.)
        patrol.started(task, first[3], self.now)
        patrol.ended(task, 'person_task', 101.)
        self.now = 162.
        for uid in self.m.uav_ids:
            self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        next_leg = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNone(next_leg)
        self.assertIsNone(patrol.uid)

    def test_northeast_followup_uses_fresh_images_and_other_ne_cells(self):
        patrol = self.m._south_patrol = bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=True)
        first = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(first)
        self.assertEqual(patrol.proposal_region, 'northeast')
        uid, first_keys, first_points, direction = first
        task = self.m._segments.reserve(uid, first_keys, first_points,
                                        100., 20., 90., purpose='south_patrol')
        self.assertIsNotNone(task)
        patrol.started(task, direction, 100.)
        for key in first_keys:
            self.m.allocator.visit_time[key] = 101.
        self.m._camera_point_seen = {(first_keys[0], 4): 101.}
        self.m._segments.advance(uid, task.task_id, task.revision,
                                 len(first_keys))
        self.m._segments.release(uid)
        patrol.ended(task, 'complete', 102.)
        self.assertEqual(patrol.attempts, 1)
        self.now = 102.
        self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        self.m.status[uid].x, self.m.status[uid].y = first_points[-1]
        second = patrol.choose(self.m, [uid], self.now)
        self.assertIsNotNone(second)
        self.assertEqual(second[0], uid)
        self.assertEqual(patrol.proposal_region, 'northeast')
        self.assertTrue(set(second[1]).isdisjoint(first_keys))
        self.assertTrue(all(105. <= self.m.grid.cell(key).cx <= 133. and
                            20. <= self.m.grid.cell(key).cy <= 42.
                            for key in second[1]))
        next_task = self.m._segments.reserve(uid, second[1], second[2],
                                             102., 20., 90.,
                                             purpose='south_patrol')
        self.assertIsNotNone(next_task)
        patrol.started(next_task, second[3], self.now)
        self.assertAlmostEqual(next_task.deadline, 190.)
        self.assertEqual(set(self.m._segments.reserved), set(second[1]))
        self.assertTrue(set(self.m._segments.reserved).isdisjoint(first_keys))

    def test_northeast_without_processed_image_gain_releases_aircraft(self):
        patrol = self.m._south_patrol = bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=True)
        first = patrol.choose(self.m, self.m.uav_ids, 100.)
        uid = first[0]
        task = NS(uid=uid, task_id='ne-no-image', cells=tuple(first[1]),
                  created=100., deadline=190., expires=120.)
        patrol.started(task, first[3], 100.)
        self.m._camera_point_seen = {(first[1][0], 4): 99.}
        # Waypoint arrival alone must not buy another dedicated NE segment.
        patrol.ended(task, 'complete', 102.)
        self.assertIsNone(patrol.uid)
        self.now = 162.
        for name in self.m.uav_ids:
            self.m.last_report[name] = sm.rospy.Time.from_sec(self.now)
        self.assertIsNone(patrol.choose(self.m, self.m.uav_ids, self.now))

    def test_northeast_timeout_and_exhausted_cells_release_aircraft(self):
        patrol = self.m._south_patrol = bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=True)
        first = patrol.choose(self.m, self.m.uav_ids, 100.)
        uid = first[0]
        task = NS(uid=uid, task_id='ne-timeout', cells=tuple(first[1]),
                  created=100., deadline=190., expires=120.)
        patrol.started(task, first[3], 100.)
        self.m._camera_point_seen = {(first[1][0], 4): 101.}
        patrol.ended(task, 'complete', 190.)
        self.assertIsNone(patrol.uid)
        self.now = 191.
        self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        self.assertIsNone(patrol.choose(self.m, [uid], self.now))

        fresh = self.m._south_patrol = bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=True)
        self.now = 200.
        for name in self.m.uav_ids:
            self.m.last_report[name] = sm.rospy.Time.from_sec(self.now)
        first = fresh.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(first)
        uid = first[0]
        task = NS(uid=uid, task_id='ne-exhausted', cells=tuple(first[1]),
                  created=200., deadline=290., expires=220.)
        fresh.started(task, first[3], self.now)
        self.m._camera_point_seen = {(first[1][0], 4): 201.}
        fresh.ended(task, 'complete', 202.)
        for key, cell in self.m.grid.cells.items():
            if 105. <= cell.cx <= 133. and 20. <= cell.cy <= 42.:
                self.m._blocked_cells.add(key)
        self.now = 202.
        self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        self.assertIsNone(fresh.choose(self.m, [uid], self.now))
        self.assertIsNone(fresh.uid)

    def test_initial_regions_are_mutually_exclusive(self):
        with self.assertRaises(ValueError):
            bp.SouthPatrol((-50, 150, -60, 60), southeast=True, northeast=True)

    def nearby_northeast_task(self):
        m=self.m
        m._south_patrol=bp.SouthPatrol((-50,150,-60,60),
                                       southeast=False,northeast=True)
        uid=m.uav_ids[4]
        m.status[uid].x,m.status[uid].y=98.55,20.5
        m._allocate()
        task=m._segments.tasks[uid]
        self.assertEqual(task.purpose,'south_patrol')
        return task

    def refresh_statuses(self, now):
        self.now=now
        for uid,status in self.m.status.items():
            status.header.stamp=sm.rospy.Time.from_sec(now)
            self.m.last_report[uid]=sm.rospy.Time.from_sec(now)

    def block_current_ne_task(self, task, now):
        self.refresh_statuses(now)
        status=self.m.status[task.uid]
        status.search_task_id=task.task_id
        status.search_revision=task.revision
        status.search_state=TRAVEL
        status.search_index=0
        status.search_path_remaining=12.
        status.search_blocked_waypoint=1
        status.search_blocked_reason='GOAL_OCCUPIED'
        self.m._renew_segments(sm.rospy.Time.from_sec(now))

    def test_new_obstacle_retries_same_owner_after_real_release_cooldown(self):
        first=self.nearby_northeast_task()
        uid=first.uid
        self.block_current_ne_task(first,101.)
        patrol=self.m._south_patrol
        self.assertEqual(patrol.uid,uid)
        self.assertEqual(patrol.attempts,1)
        self.assertNotIn(uid,self.m._segments.tasks)
        self.assertTrue(set(first.cells).isdisjoint(self.m._segments.reserved))
        self.assertTrue(all(self.m.allocator.failed_visit[k]==101. for k in first.cells))
        # Ensure choose runs while the owner is excluded by its real cooldown.
        other=next(name for name in self.m._segments.tasks if name!=uid)
        self.m._drop_segment(other,'test_release')
        self.refresh_statuses(102.)
        self.m._allocate()
        self.assertEqual(patrol.uid,uid)
        self.assertNotIn(uid,self.m._segments.tasks)
        self.assertAlmostEqual(patrol.deadline,190.)
        self.refresh_statuses(103.1)
        self.m._allocate()
        second=self.m._segments.tasks[uid]
        self.assertEqual(second.purpose,'south_patrol')
        self.assertEqual(patrol.attempts,2)
        self.assertEqual(second.uid,uid)
        self.assertTrue(set(second.cells).isdisjoint(first.cells))
        self.assertAlmostEqual(second.deadline,190.)
        self.assertTrue(all(self.m.grid.cell(k).state!=st.STATE_COVERED
                            for k in first.cells))

    def test_northeast_three_obstacle_attempts_end_without_restart(self):
        task=self.nearby_northeast_task()
        uid=task.uid
        attempts=[]
        for n in range(3):
            attempts.append(task)
            self.assertEqual(self.m._south_patrol.attempts,n+1)
            self.assertAlmostEqual(task.deadline,190.)
            self.block_current_ne_task(task,101.+n*3.1)
            self.refresh_statuses(103.1+n*3.1)
            self.m._allocate()
            if n<2:
                task=self.m._segments.tasks[uid]
                self.assertEqual(task.purpose,'south_patrol')
                self.assertTrue(all(set(task.cells).isdisjoint(old.cells)
                                    for old in attempts))
        self.assertIsNone(self.m._south_patrol.uid)
        self.assertTrue(all(t.purpose!='south_patrol'
                            for t in self.m._segments.tasks.values()))
        self.refresh_statuses(180.)
        self.assertIsNone(self.m._south_patrol.choose(self.m,[uid],self.now))

    def test_northeast_cooldown_does_not_block_person_owner(self):
        first=self.nearby_northeast_task()
        uid=first.uid
        self.block_current_ne_task(first,101.)
        self.m._tracking['white-person']=uid
        self.refresh_statuses(102.)
        self.assertIsNone(self.m._south_patrol.choose(self.m,[],self.now))
        self.assertIsNone(self.m._south_patrol.uid)
        self.assertEqual(self.m._tracking['white-person'],uid)

    def test_northeast_cooldown_cannot_extend_original_deadline(self):
        first=self.nearby_northeast_task()
        uid=first.uid
        first.expires=190.
        self.block_current_ne_task(first,189.)
        self.assertEqual(self.m._south_patrol.uid,uid)
        self.refresh_statuses(190.1)
        self.assertIsNone(self.m._south_patrol.choose(self.m,[],self.now))
        self.assertIsNone(self.m._south_patrol.uid)

    def test_disconnected_northeast_owner_is_released_during_cooldown(self):
        first=self.nearby_northeast_task()
        self.block_current_ne_task(first,101.)
        self.refresh_statuses(102.)
        self.m.status[first.uid].connected=False
        self.assertIsNone(self.m._south_patrol.choose(self.m,[],self.now))
        self.assertIsNone(self.m._south_patrol.uid)

    def test_reconcile_blocked_directed_view_uses_original_selector_not_grid_centre(self):
        first=self.nearby_northeast_task()
        original=self.m._search_point_free
        with patch.object(self.m,'_search_point_free',
                          side_effect=lambda p:p!=first.points[0] and original(p)), \
             patch.object(self.m,'_segment_point',
                          side_effect=AssertionError('ordinary replacement for directed view')):
            self.refresh_statuses(101.)
            self.m._reconcile_segments()
        self.assertEqual(self.m._south_patrol.uid,first.uid)
        self.assertNotIn(first.uid,self.m._segments.tasks)
        self.m._allocate()
        second=self.m._segments.tasks[first.uid]
        self.assertEqual(second.purpose,'south_patrol')
        self.assertTrue(set(first.cells).isdisjoint(second.cells))
        self.assertAlmostEqual(second.deadline,190.)


if __name__ == '__main__':
    unittest.main()
