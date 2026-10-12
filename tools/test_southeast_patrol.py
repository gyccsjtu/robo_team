"""Optional first southeast leg stays a safe, short, ordinary search task."""
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
from search_segments import SegmentBook, clear_line
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
            self.assertAlmostEqual(point[1], cell.cy-8.5)
            self.assertGreater(look[1], point[1])
            self.assertTrue(self.m._search_point_free(point))
            self.assertTrue(self.m._blind_viewpoint_valid(cell, point))
            self.assertTrue(clear_line(self.m._g_infl, point, look))
            self.assertNotEqual(cell.state, st.STATE_COVERED)
        self.assertFalse(any(key[1] == 0 for key in task.cells))
        self.assertAlmostEqual(task.deadline, 190.)

    def test_completed_inner_leg_reverts_to_south_edge_with_same_budget(self):
        patrol = self.m._south_patrol
        first = patrol.choose(self.m, self.m.uav_ids, self.now)
        self.assertIsNotNone(first)
        uid, cells, _, direction = first
        self.assertTrue(all(key[1] == 2 for key in cells))
        task = NS(uid=uid, task_id='inner', deadline=190., expires=120.)
        patrol.started(task, direction, self.now)
        patrol.ended(task, 'complete', 105.)
        self.assertEqual(patrol.completed, 1)
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
        self.assertAlmostEqual(patrol._first_deadline, 190.)
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


if __name__ == '__main__':
    unittest.main()
