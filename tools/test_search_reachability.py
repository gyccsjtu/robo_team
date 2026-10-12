"""A sensed obstruction must not erase unobserved search space."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
import swarm_manager as sm
import swarm_task as st


class SearchReachabilityTests(unittest.TestCase):
    def setUp(self):
        clock = patch.object(sm.rospy.Time, 'now',
                             return_value=sm.rospy.Time.from_sec(100.))
        clock.start()
        self.addCleanup(clock.stop)

    def manager(self, occupied=False, outer=False):
        m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.grid = st.CoverageGrid(-55, -48, -65, -58, cell_m=7, num_uavs=1) \
            if outer else st.CoverageGrid(0, 7, 0, 7, cell_m=7, num_uavs=1)
        origin = (-70., -80.) if outer else (-20., -20.)
        raw = sm.GridMap(80, 80, 1., origin,
                         bytes([int(occupied)]) * (80*80))
        m._merged_grid = Mock(return_value=raw)
        return m, raw, next(iter(m.grid.cells))

    def block_centre(self, m, raw, key):
        cell = m.grid.cell(key)
        index = raw.world_to_cell(sm.search_viewpoint(cell.cx, cell.cy))
        cells = bytearray(raw.cells)
        cells[index[1]*raw.width+index[0]] = 1
        updated = sm.GridMap(raw.width, raw.height, raw.resolution,
                             raw.origin, cells)
        m._merged_grid.return_value = updated

    def test_centre_obstacle_keeps_safe_alternative_with_bias_disabled(self):
        m, raw, key = self.manager()
        self.block_centre(m, raw, key)
        with patch.object(sm, 'EDGE_ENABLE', False):
            blocked = m._find_blocked_cells()
        self.assertNotIn(key, blocked)
        point = m._cell_waypoint[key]
        self.assertNotEqual(point, (3.5, 3.5))
        self.assertTrue(m._g_infl.is_free(m._g_infl.world_to_cell(point)))
        self.assertEqual(point, sm.flight_waypoint(*point))
        self.assertEqual(m._cell_edge, {})
        self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)

    def test_bias_switch_does_not_change_reachable_waypoint(self):
        m, raw, key = self.manager()
        self.block_centre(m, raw, key)
        with patch.object(sm, 'EDGE_ENABLE', True):
            expected = m._find_blocked_cells(), dict(m._cell_waypoint)
        self.assertIn(key, m._cell_edge)
        with patch.object(sm, 'EDGE_ENABLE', False):
            actual = m._find_blocked_cells(), dict(m._cell_waypoint)
        self.assertEqual(expected, actual)
        self.assertEqual(m._cell_edge, {})

    def test_all_blocked_stays_excluded_without_coverage_credit(self):
        m, _, key = self.manager(occupied=True)
        with patch.object(sm, 'EDGE_ENABLE', False):
            self.assertEqual(m._find_blocked_cells(), {key})
        self.assertEqual(m._cell_waypoint, {})
        self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)

    def test_outer_alternative_stays_inside_flight_box_and_camera_range(self):
        m, raw, key = self.manager(outer=True)
        self.block_centre(m, raw, key)
        with patch.object(sm, 'EDGE_ENABLE', False):
            self.assertNotIn(key, m._find_blocked_cells())
        point = m._cell_waypoint[key]
        cell = m.grid.cell(key)
        self.assertEqual(point, sm.flight_waypoint(*point))
        distance = math.hypot(point[0]-cell.cx, point[1]-cell.cy)
        self.assertGreaterEqual(distance, 7.8)
        self.assertLessEqual(distance, 9.)
        self.assertTrue(m._g_infl.is_free(m._g_infl.world_to_cell(point)))

    def test_no_map_keeps_unknown_cells_unobserved(self):
        m, _, key = self.manager()
        m._merged_grid.return_value = None
        self.assertEqual(m._find_blocked_cells(), set())
        self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)

    def test_blocked_then_reachable_enters_next_auction_without_fake_coverage(self):
        m, _, key = self.manager()
        uid = 'typhoon_h480_0'
        m.allocator = st.TaskAllocator(m.grid)
        m.lease = st.LeaseManager(m.grid)
        m._blocked_cells = {key}
        m._active_leases, m._lease_hold_since, m._lease_motion = {}, {}, {}
        m._auction_cycle = 0
        m._csv = NS(write=Mock())
        m.status = {uid: NS(x=0., y=0., z=4.)}
        m._idle_uavs = lambda: [uid]
        m._reopen_covered_cells = Mock()
        m._publish_assignment = Mock()
        m._cell_waypoint, m._blind_viewpoints = {}, {}
        with patch.object(sm.rospy.Time, 'now',
                          return_value=sm.rospy.Time.from_sec(100.)), \
                patch.object(st, '_now', return_value=100.):
            m._allocate()
            self.assertEqual(m.grid.cell(key).state, st.STATE_FREE)
            self.assertFalse(m._publish_assignment.called)
            m._blocked_cells.clear()  # New legal lidar evidence permits it.
            m._allocate()
        self.assertEqual(m._active_leases[uid], key)
        self.assertEqual(m.grid.cell(key).state, st.STATE_ASSIGNED)
        self.assertEqual(m.allocator.last_seen, {})
        m._publish_assignment.assert_called_once()

    def test_observed_cell_is_not_reopened_when_block_clears(self):
        m, _, key = self.manager()
        m.grid.cell(key).state = st.STATE_COVERED
        allocator = st.TaskAllocator(m.grid)
        with patch.object(st, '_now', return_value=100.):
            self.assertIsNone(allocator.allocate({'u0': (0., 0.)},
                                                blocked={key})['u0'])
            self.assertIsNone(allocator.allocate({'u0': (0., 0.)})['u0'])
        self.assertEqual(m.grid.cell(key).state, st.STATE_COVERED)


if __name__ == '__main__':
    unittest.main()
