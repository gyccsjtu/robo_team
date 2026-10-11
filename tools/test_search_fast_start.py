"""Flight-loop regressions for early search, boundary views and task priority."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st


class SearchFastStartTests(unittest.TestCase):
    def setUp(self):
        a = self.a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'u0'
        a.local_xy, a.offset, a.local_z = (0., 0.), (0., 0.), 3.
        a._landing, a._takeoff_done = False, True
        a.assignment = NS(task_type=0, target_id='', target_x=30., target_y=0.)
        a._hold_camera_candidate = lambda: False
        a._update_orbit = lambda: None
        a._orbit_target = None
        a.path_target = (30., 0.)
        a._pick_local_goal = lambda: (30., 0.)
        a._search_sweep_bearing = lambda bearing, distance: bearing
        a._set_search_heading = lambda bearing: None
        a._transit_cap = lambda: 6.
        a._apply_friend_avoidance = lambda vx, vy: (vx, vy)
        a._world_trusted = lambda: True
        self.commands = []
        a._send_vel = lambda *args: self.commands.append(args)
        self.now = 100.
        self.clock = patch.object(sa.rospy.Time, 'now',
            side_effect=lambda: sa.rospy.Time.from_sec(self.now))
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_actual_search_loop_caps_first_metres_then_restores_cruise(self):
        self.a._control()
        self.assertAlmostEqual(self.commands[-1][0], 1.5)
        # Real motion at the capped speed; each control tick must keep it.
        for i in range(1, 67):
            self.now += .1
            self.a.local_xy = (.15*i, 0.)
            self.a._control()
            self.assertAlmostEqual(self.commands[-1][0], 1.5)
        self.now += .1
        self.a.local_xy = (10.05, 0.)
        self.a._control()
        self.assertAlmostEqual(self.commands[-1][0], sa.SEARCH_CRUISE_SPEED)

    def test_stationary_commands_and_pose_jumps_cannot_finish_start_phase(self):
        for _ in range(100):
            self.now += .1
            self.a._control()
        self.assertEqual(self.a._search_start_progress['metres'], 0.)
        for dt, x in ((.1, 20.), (5., 21.), (-2., 22.)):
            self.now += dt
            self.a.local_xy = (x, 0.)
            self.assertEqual(self.a._search_start_cap(3., 0.), (1.5, 0.))
        self.assertEqual(self.a._search_start_progress['metres'], 0.)

    def test_safety_stop_is_after_cap_and_does_not_consume_motion_budget(self):
        self.a._apply_friend_avoidance = lambda vx, vy: (0., 0.)
        self.a._control()
        self.assertEqual(self.commands[-1], (0., 0.))
        self.assertEqual(self.a._search_start_progress['metres'], 0.)

    def test_candidate_preempts_route_and_cap(self):
        self.a._hold_camera_candidate = lambda: True
        self.a._search_start_cap = lambda *args: self.fail('search cap on candidate')
        self.a._pick_local_goal = lambda: self.fail('candidate still follows route')
        self.a._control()
        self.assertEqual(self.commands, [])

    def test_search_resume_preserves_budget_and_does_not_count_tracking_travel(self):
        self.a._search_start_cap(3., 0.)
        self.now += .1
        self.a.local_xy = (.15, 0.)
        self.a._search_start_cap(3., 0.)
        before = self.a._search_start_progress['metres']
        self.now += 20.
        self.a.local_xy = (20., 0.)
        self.a.assignment = NS(task_type=0, target_id='', target_x=40., target_y=0.)
        self.a._search_start_cap(3., 0.)
        self.assertEqual(self.a._search_start_progress['metres'], before)

    def test_boundary_search_views_avoid_slow_band_without_changing_flight_box(self):
        for x, y in ((20., 54.5), (20., -54.5), (-44.5, 0.), (144.5, 0.)):
            view = sm.search_viewpoint(x, y)
            edge = min(view[0]-sm.FLIGHT_X_MIN, sm.FLIGHT_X_MAX-view[0],
                       view[1]-sm.FLIGHT_Y_MIN, sm.FLIGHT_Y_MAX-view[1])
            self.assertGreaterEqual(edge, 8.5)
            self.assertAlmostEqual(math.hypot(view[0]-x, view[1]-y), 8.5)
            self.assertTrue(sm.SwarmManager._in_processed_view(
                *view, math.atan2(y-view[1], x-view[0]), x, y, 4., 0.))
        self.assertEqual(sm.flight_waypoint(155., 65.), (148., 58.))

    def test_outer_cells_remain_visible_and_no_grid_cell_is_marked_observed(self):
        grid = st.CoverageGrid(-55, 155, -65, 65, cell_m=7, num_uavs=6)
        for cell in grid.cells.values():
            view = sm.search_viewpoint(cell.cx, cell.cy)
            distance = math.hypot(view[0]-cell.cx, view[1]-cell.cy)
            if distance > .01:
                self.assertGreaterEqual(distance, 8.5-1e-6)
                self.assertLessEqual(distance, 9.)
                self.assertTrue(sm.SwarmManager._in_processed_view(
                    *view, math.atan2(cell.cy-view[1], cell.cx-view[0]),
                    cell.cx, cell.cy, 4., 0.))
            self.assertEqual(cell.state, st.STATE_FREE)

    def test_inward_view_inside_sensed_obstacle_uses_existing_free_alternative(self):
        m = sm.SwarmManager.__new__(sm.SwarmManager)
        m.grid = st.CoverageGrid(16.5, 23.5, 51., 58., cell_m=7, num_uavs=1)
        cell = next(iter(m.grid.cells.values()))
        nominal = sm.search_viewpoint(cell.cx, cell.cy)
        raw = bytearray(210 * 130)
        raw[int(nominal[1]+65)*210+int(nominal[0]+55)-2] = 1
        g = sm.GridMap(210, 130, 1., (-55., -65.), raw)
        m._merged_grid = lambda: g
        with patch.object(sm, 'EDGE_ENABLE', True):
            blocked = m._find_blocked_cells()
        self.assertEqual(blocked, set())
        alternative = next(iter(m._cell_waypoint.values()))
        self.assertNotEqual(alternative, nominal)
        self.assertTrue(m._g_infl.is_free(m._g_infl.world_to_cell(alternative)))
        self.assertEqual(cell.state, st.STATE_FREE)


if __name__ == '__main__':
    unittest.main()
