"""Complete bounded paths, forbidden shortcuts, and explicit inward returns."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'coordination/src/robocup_swarm/scripts'),
               str(ROOT/'coordination/src/robocup_navigation/src')]
import test_search_segments  # ROS message/fixture environment shared by integrations
from robocup_navigation.astar import GridMap, plan
from routine_planning import plan_routine
from routine_search import routine_point_allowed
from search_segment_agent import SearchSegmentAgent
from search_segment_manager import SearchSegmentManager


class RoutinePlanningTests(unittest.TestCase):
    def grid(self, wall=False):
        grid = GridMap(40, 30, 1., (110., -15.), bytearray(40*30))
        if wall:
            for iy in range(grid.height):
                for ix in range(grid.width):
                    x, y = grid.cell_to_world((ix, iy))
                    if 123. <= x <= 130. and abs(y) <= 5.:
                        grid.cells[iy*grid.width+ix] = 1
        return grid

    def test_interior_endpoints_cannot_buy_an_outside_obstacle_shortcut(self):
        grid = self.grid(wall=True)
        start, goal = (127.5, -8.5), (127.5, 8.5)
        old = plan(grid, start, goal)
        self.assertTrue(old.success)
        self.assertTrue(any(not routine_point_allowed(point) for point in old.points))
        route = plan_routine(grid, start, goal)
        self.assertTrue(route.success)
        self.assertTrue(all(routine_point_allowed(point) for point in route.points))
        self.assertGreater(route.path_length_m, old.path_length_m)
        self.assertTrue(all(grid.is_free(cell) for cell in route.cells))

    def test_only_outside_connection_does_not_become_a_fallback_flight(self):
        grid = self.grid(wall=True)
        for iy in range(grid.height):
            for ix in range(grid.width):
                x, y = grid.cell_to_world((ix, iy))
                if x < 123. and abs(y) <= 5.:
                    grid.cells[iy*grid.width+ix] = 1
        self.assertTrue(plan(grid, (127.5, -8.5), (127.5, 8.5)).success)
        route = plan_routine(grid, (127.5, -8.5), (127.5, 8.5))
        self.assertFalse(route.success)
        self.assertEqual(route.points, ())

    def test_outside_previous_person_task_may_return_then_stay_inside(self):
        grid = self.grid()
        route = plan_routine(grid, (138.5, 0.5), (121.5, 8.5))
        self.assertTrue(route.success)
        self.assertEqual(route.points[0], (138.5, 0.5))
        self.assertTrue(all(routine_point_allowed(point) for point in route.points[1:]))
        self.assertLess(route.points[1][0], route.points[0][0])

    def test_inward_return_does_not_cross_sensed_wall(self):
        grid = self.grid()
        for iy in range(grid.height):
            grid.cells[iy*grid.width+21] = 1  # x131.5, separating outside from inside
        route = plan_routine(grid, (138.5, 0.5), (121.5, 8.5))
        self.assertFalse(route.success)
        self.assertEqual(route.reason, 'NO_CHECKED_INWARD_ENTRY')

    def test_exact_legal_edges_are_not_false_occupied_due_to_cell_centre(self):
        grid = GridMap(400, 240, .5, (-50., -60.), bytearray(400*240))
        for edge in ((129., 0.), (-49., 0.), (0., 59.), (0., -59.)):
            for start, goal in ((edge, (0., 0.)), ((0., 0.), edge)):
                route = plan_routine(grid, start, goal)
                self.assertTrue(route.success, (start, goal, route.reason))
                self.assertTrue(all(routine_point_allowed(point) for point in route.points))
        grid.cells[grid.world_to_cell((129., 0.))[1]*grid.width +
                   grid.world_to_cell((129., 0.))[0]] = 1
        self.assertEqual(plan_routine(grid, (0., 0.), (129., 0.)).reason, 'GOAL_OCCUPIED')

    def test_blocked_nearest_entry_rays_do_not_hide_safe_lateral_return(self):
        grid = self.grid()
        for iy in range(grid.height):
            _, y = grid.cell_to_world((22, iy))
            if abs(y) <= 3.5:
                grid.cells[iy*grid.width+22] = 1  # short x132.5 wall
        route = plan_routine(grid, (138.5, .5), (121.5, 8.5))
        self.assertTrue(route.success)
        self.assertTrue(all(routine_point_allowed(point) for point in route.points[1:]))
        self.assertGreater(abs(route.points[1][1]), 3.5)

    def test_invalid_or_remote_outside_start_and_outside_goal_fail(self):
        grid = self.grid()
        for start, goal in (((float('nan'), 0.), (121.5, 0.5)),
                            ((138.5, 0.5), (140.5, 0.5)),
                            ((149.5, 0.5), (121.5, 0.5))):
            route = plan_routine(grid, start, goal)
            self.assertFalse(route.success)
            self.assertEqual(route.points, ())

    def test_actual_segment_agent_uses_bounded_route_and_default_preserves_old(self):
        agent = NS(grid=self.grid(wall=True), world_xy=(127.5, -8.5),
                   uav_id='fixture', _routine_search_bounds=True)
        path, target, ok, blocked, _ = SearchSegmentAgent._compute_segment_plan(agent, [(127.5, 8.5)])
        self.assertTrue(ok)
        self.assertEqual(blocked, 0)
        self.assertEqual(target, (127.5, 8.5))
        self.assertTrue(all(routine_point_allowed(point) for point in path))
        agent._routine_search_bounds = False
        old, _, ok, _, _ = SearchSegmentAgent._compute_segment_plan(agent, [(127.5, 8.5)])
        self.assertTrue(ok)
        self.assertTrue(any(not routine_point_allowed(point) for point in old))

    def test_manager_and_agent_retain_the_same_bounded_astar_entry(self):
        # Exercise the actual manager candidate selector. View geometry is
        # separately tested; this case isolates its checked route contract.
        from unittest.mock import patch
        grid = self.grid(wall=True)
        manager = NS(_g_infl=grid, _routine_search_bounds=True,
                     _merged_grid=lambda: grid,
                     _segment_flight_waypoint=lambda x, y: (x, y))
        with patch('search_segment_manager.valid_view', return_value=True):
            result = SearchSegmentManager._reachable_subpoint_view(
                manager, (0, 0), (127.5, -8.5), [((127.5, 8.5), (127.5, 1.5))],
                preferred_heading=0., with_path=True)
        self.assertIsNotNone(result)
        self.assertTrue(all(routine_point_allowed(point) for point in result[2]))


if __name__ == '__main__':
    unittest.main()
