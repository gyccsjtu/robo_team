"""Search-route and image-evidence regressions for the six-UAV manager.

Run after sourcing ROS and the coordination workspace, with the navigation
package on PYTHONPATH (the same environment used by run_match.sh).
"""

import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import swarm_task
import swarm_manager


class SearchPlanTests(unittest.TestCase):
    def test_unimaged_inner_cell_revisit_uses_safe_side_view(self):
        grid = swarm_task.CoverageGrid(0, 35, 0, 35, cell_m=7,
                                       num_uavs=1)
        key = grid.world_to_cell(17.5, 17.5)
        cell = grid.cell(key)
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.grid = grid
        manager._cell_waypoint = {key: (cell.cx, cell.cy)}
        manager._blind_viewpoints = {}
        manager._blocked_cells = set()
        # The east side is traversable, but a sensed wall blocks its view of
        # the target cell. The alternative must have lidar-map line of sight.
        raw = [0] * (120 * 120)
        raw[57 * 120 + 61] = 1  # world (21, 17), on the east camera ray
        manager._g_infl = swarm_manager.GridMap(
            120, 120, 1.0, (-40.0, -40.0), raw)
        manager.allocator = swarm_task.TaskAllocator(grid)
        manager.lease = swarm_task.LeaseManager(grid)
        manager.lease.grant('u0', key, 99.0, 20.0)
        manager.uav_ids = ['u0']
        manager.last_report = {'u0': swarm_manager.rospy.Time.from_sec(99.5)}
        manager._active_leases = {'u0': key}
        manager._lease_hold_since = {('u0', key): 99.0}
        manager._lease_motion = {}
        manager.status = {'u0': SimpleNamespace(connected=True, x=cell.cx,
            y=cell.cy, assigned_cell_ix=key[0], assigned_cell_iy=key[1],
            confidence=1.0)}
        published = []
        manager.assign_pub = SimpleNamespace(publish=published.append)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.0)):
            manager._renew_leases()
            manager._publish_assignment('u0', key, cell)

        self.assertEqual(cell.state, swarm_task.STATE_REVIEW)
        self.assertIn(('blind', key), manager.allocator.revisit_jobs)
        side = manager._blind_viewpoints[key]
        self.assertEqual((published[-1].target_x, published[-1].target_y),
                         side)
        self.assertAlmostEqual(math.hypot(side[0] - cell.cx,
                                          side[1] - cell.cy), 8.5)
        self.assertNotAlmostEqual(side[0], cell.cx + 8.5)
        yaw = math.atan2(cell.cy - side[1], cell.cx - side[0])
        self.assertTrue(manager._in_processed_view(
            *side, yaw, cell.cx, cell.cy, 4.0, 0.0))
        self.assertTrue(manager._blind_viewpoint_valid(cell, side))
        self.assertTrue(swarm_manager.plan(
            manager._g_infl, (cell.cx, cell.cy), side).success)
        self.assertEqual(manager.allocator.last_seen, {})

    def test_outer_cell_assignment_uses_camera_visible_interior_viewpoint(self):
        cell_x, cell_y = -51.5, -51.5
        nearest = swarm_manager.flight_waypoint(cell_x, cell_y)
        yaw_nearest = math.atan2(cell_y - nearest[1], cell_x - nearest[0])
        self.assertFalse(swarm_manager.SwarmManager._in_processed_view(
            *nearest, yaw_nearest, cell_x, cell_y, 4.0, 0.0))

        view = swarm_manager.search_viewpoint(cell_x, cell_y)
        self.assertAlmostEqual(math.hypot(view[0] - cell_x,
                                          view[1] - cell_y), 8.5)
        self.assertGreaterEqual(view[0], swarm_manager.FLIGHT_X_MIN + 2.0)
        self.assertGreaterEqual(view[1], swarm_manager.FLIGHT_Y_MIN + 2.0)
        yaw = math.atan2(cell_y - view[1], cell_x - view[0])
        self.assertTrue(swarm_manager.SwarmManager._in_processed_view(
            *view, yaw, cell_x, cell_y, 4.0, 0.0))
        self.assertEqual(swarm_manager.search_viewpoint(20.0, 10.0),
                         (20.0, 10.0))

        grid = swarm_task.CoverageGrid(-55, 155, -55, 55,
                                       cell_m=7, num_uavs=6)
        key = grid.world_to_cell(cell_x, cell_y)
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._cell_waypoint = {}
        sent = []
        manager.assign_pub = SimpleNamespace(publish=sent.append)
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.0)):
            manager._publish_assignment('typhoon_h480_0', key, grid.cell(key))
        self.assertAlmostEqual(sent[0].target_x, view[0])
        self.assertAlmostEqual(sent[0].target_y, view[1])

    @staticmethod
    def _east_orphan_grid():
        grid = swarm_task.CoverageGrid(-55, 155, -65, 65,
                                       cell_m=7, num_uavs=6)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        for zone in (1, 3, 5):
            for key in grid.zone_cells[zone]:
                cell = grid.cell(key)
                if (zone == 3 and cell.cx >= 95 or
                        zone == 1 and cell.cx >= 130 and cell.cy <= -40 or
                        zone == 5 and cell.cx >= 130 and cell.cy >= 40):
                    cell.state = swarm_task.STATE_FREE
        return grid

    def test_unattended_east_middle_is_taken_over_before_edge_patrol(self):
        # UAV 3 had the middle-east sector but lost its search lease. UAVs
        # 1 and 5 are still searching the southeast/northeast edge strips.
        grid = self._east_orphan_grid()
        allocator = swarm_task.TaskAllocator(grid)
        allocator.mission_start = 100.0
        with patch.object(swarm_task, '_now', return_value=300.0):
            assigned = allocator.allocate({
                'typhoon_h480_1': (130.0, -32.0),
                'typhoon_h480_5': (130.0, 32.0),
            })
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_1']), 3)
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_5']), 5)
        self.assertGreater(grid.cell(assigned['typhoon_h480_1']).cx, 90.0)
        first = assigned['typhoon_h480_1']
        first_cell = grid.cell(first)
        first_cell.state = swarm_task.STATE_COVERED
        first_cell.owner = None
        allocator.complete_waypoint('typhoon_h480_1', first, observed=True,
                                    now=301.0)
        with patch.object(swarm_task, '_now', return_value=304.0):
            second = allocator.allocate({
                'typhoon_h480_1': (first_cell.cx, first_cell.cy),
            })['typhoon_h480_1']
        self.assertEqual(grid.get_zone_id(second), 3)
        self.assertLessEqual(
            ((grid.cell(second).cx - first_cell.cx) ** 2 +
             (grid.cell(second).cy - first_cell.cy) ** 2) ** 0.5, 14.0)

    def test_active_middle_search_lease_prevents_duplicate_takeover(self):
        grid = self._east_orphan_grid()
        middle = next(key for key in grid.zone_cells[3]
                      if grid.cell(key).state == swarm_task.STATE_FREE)
        grid.cell(middle).state = swarm_task.STATE_ASSIGNED
        grid.cell(middle).owner = 'typhoon_h480_3'
        allocator = swarm_task.TaskAllocator(grid)
        allocator.mission_start = 100.0
        with patch.object(swarm_task, '_now', return_value=300.0):
            assigned = allocator.allocate({'typhoon_h480_1': (130.0, -32.0)})
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_1']), 1)

    def test_initial_home_spread_remains_in_force(self):
        grid = self._east_orphan_grid()
        allocator = swarm_task.TaskAllocator(grid)
        allocator.mission_start = 100.0
        with patch.object(swarm_task, '_now', return_value=129.9):
            assigned = allocator.allocate({'typhoon_h480_1': (130.0, -32.0)})
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_1']), 1)

    def test_orphan_takeover_opens_after_thirty_seconds(self):
        grid = self._east_orphan_grid()
        allocator = swarm_task.TaskAllocator(grid)
        allocator.mission_start = 100.0
        with patch.object(swarm_task, '_now', return_value=130.1):
            assigned = allocator.allocate({'typhoon_h480_1': (130.0, -32.0)})
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_1']), 3)

    def test_neighbor_keeps_equally_unseen_home_sector(self):
        grid = swarm_task.CoverageGrid(-55, 155, -65, 65,
                                       cell_m=7, num_uavs=6)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        for zone in (1, 3):
            for key in grid.zone_cells[zone]:
                if grid.cell(key).cx >= 95:
                    grid.cell(key).state = swarm_task.STATE_FREE
        allocator = swarm_task.TaskAllocator(grid)
        allocator.mission_start = 100.0
        with patch.object(swarm_task, '_now', return_value=300.0):
            assigned = allocator.allocate({'typhoon_h480_1': (130.0, -32.0)})
        self.assertEqual(grid.get_zone_id(assigned['typhoon_h480_1']), 1)

    def test_assignment_does_not_advance_route_or_claim_camera_view(self):
        grid = swarm_task.CoverageGrid(0, 28, 0, 21, num_uavs=1)
        allocator = swarm_task.TaskAllocator(grid)
        with patch.object(swarm_task, '_now', return_value=100.0):
            key = allocator.allocate({'uav_0': (3.5, 10.5)})['uav_0']
        self.assertIsNotNone(key)
        self.assertEqual(allocator.route_cursor, {})
        self.assertEqual(allocator.visit_time, {})
        self.assertEqual(allocator.last_seen, {})
        # Pursuit interrupts before a camera scan: the unfinished waypoint
        # returns to the pool and remains ahead of the completed-route cursor.
        swarm_task.LeaseManager(grid).force_expire('uav_0', key)
        self.assertEqual(grid.cell(key).state, swarm_task.STATE_FREE)
        with patch.object(swarm_task, '_now', return_value=102.0):
            allocator.complete_waypoint('uav_0', key, observed=False)
        zone = grid.get_zone_id(key)
        self.assertEqual(allocator.route_cursor[zone], allocator.route_rank[zone][key])
        self.assertEqual(allocator.last_seen, {})
        allocator.observed_cell(key, 102.2)
        self.assertEqual(allocator.last_seen[key], 102.2)

    def test_unseen_boundary_can_be_assigned_during_first_pass(self):
        grid = swarm_task.CoverageGrid(0, 14, 0, 21, cell_m=7, num_uavs=1)
        outer = grid.world_to_cell(3.5, 3.5)
        inner = grid.world_to_cell(3.5, 10.5)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        grid.cell(outer).state = swarm_task.STATE_FREE
        grid.cell(inner).state = swarm_task.STATE_FREE
        allocator = swarm_task.TaskAllocator(grid, w_flight=1.0,
                                              w_zone=0.0, w_novelty=0.0,
                                              cruise_speed=1.0)
        with patch.object(swarm_task, '_now', return_value=100.0):
            first = allocator.allocate({'uav_0': (3.5, 3.5)})['uav_0']
            second = allocator.allocate({'uav_0': (3.5, 3.5)})['uav_0']
        self.assertEqual((first, second), (inner, outer))

    def test_completed_route_uses_nearby_next_viewpoint(self):
        grid = swarm_task.CoverageGrid(0, 35, 0, 35, cell_m=7,
                                       num_uavs=1)
        allocator = swarm_task.TaskAllocator(grid)
        position = (3.5, 17.5)
        assigned = []
        for step in range(4):
            with patch.object(swarm_task, '_now', return_value=100.0 + step):
                key = allocator.allocate({'uav_0': position})['uav_0']
            self.assertIsNotNone(key)
            cell = grid.cell(key)
            next_position = (cell.cx, cell.cy)
            if assigned:
                self.assertLessEqual(
                    ((position[0] - next_position[0]) ** 2 +
                     (position[1] - next_position[1]) ** 2) ** 0.5, 14.0)
            assigned.append(key)
            cell.state = swarm_task.STATE_COVERED
            cell.owner = None
            allocator.complete_waypoint('uav_0', key, observed=True,
                                        now=100.0 + step)
            position = next_position
        self.assertEqual(len(set(assigned)), len(assigned))

    def test_lost_target_revisit_uses_one_cell_and_has_cooldown(self):
        grid = swarm_task.CoverageGrid(0, 28, 0, 14, cell_m=7,
                                       num_uavs=1)
        lost = grid.world_to_cell(17.5, 3.5)
        novel = grid.world_to_cell(3.5, 3.5)
        for cell in grid.cells.values():
            cell.state = swarm_task.STATE_COVERED
        grid.cell(novel).state = swarm_task.STATE_FREE
        allocator = swarm_task.TaskAllocator(grid)
        with patch.object(swarm_task, '_now', return_value=100.0):
            self.assertEqual(allocator.queue_revisit(
                'lost', 17.5, 3.5, now=100.0), lost)
        self.assertEqual(grid.cell(lost).state, swarm_task.STATE_REVIEW)
        with patch.object(swarm_task, '_now', return_value=100.0):
            self.assertEqual(allocator.allocate({'uav_0': (17.5, 3.5)})['uav_0'],
                             lost)
        allocator.complete_waypoint('uav_0', lost, observed=False, now=102.0)
        grid.cell(lost).state = swarm_task.STATE_COVERED
        self.assertIsNone(allocator.queue_revisit(
            'lost', 17.5, 3.5, now=105.0))
        self.assertEqual(grid.cell(lost).state, swarm_task.STATE_COVERED)
        with patch.object(swarm_task, '_now', return_value=118.0):
            self.assertEqual(allocator.queue_revisit(
                'lost', 17.5, 3.5, now=118.0), lost)

    def test_blind_request_cannot_reopen_a_real_camera_covered_cell(self):
        grid = swarm_task.CoverageGrid(0, 7, 0, 7, num_uavs=1)
        key = next(iter(grid.cells))
        grid.cell(key).state = swarm_task.STATE_COVERED
        allocator = swarm_task.TaskAllocator(grid)
        self.assertIsNone(allocator.queue_revisit(
            'blind', 3.5, 3.5, now=100.0))
        self.assertEqual(grid.cell(key).state, swarm_task.STATE_COVERED)

    def test_cross_color_conflict_spares_fresh_eligible_camera_track(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager._visual_observations = {('u1', 't1'):
            {'sample_s': 100.0, 'eligible': True}}
        manager._visual_conflict_sites = []
        manager._visual_quarantine = {}
        manager._tracking = {'t1': 'u1', 't3': 'u3'}
        manager._backup = {}
        manager._truth_cache = {'t1': (1.0, 2.0, 100.0),
                                't3': (1.0, 2.0, 100.0)}
        manager._cur_targets = {}
        manager._visual_candidate = {}
        manager._visual_source = {}
        manager._target_hist = {}
        manager.tracker = SimpleNamespace(targets={'t1': object(), 't3': object()})
        manager._get_target_pos = lambda tid, now: (1.0, 2.0)
        released = []
        manager._release_tracker = lambda tid, reason='': released.append((tid, reason))
        event = {'x': 1.0, 'y': 2.0, 'sample_s': 100.0,
                 'target_ids': ['t1', 't3'], 'radius_m': 1.0, 'ttl_s': 12.0}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.5)):
            manager._visual_conflict_cb(SimpleNamespace(data=json.dumps(event)))
        self.assertEqual(released, [('t3', 'visual_conflict')])
        self.assertIn('t1', manager.tracker.targets)
        self.assertNotIn('t3', manager.tracker.targets)
        self.assertFalse(manager._quarantined_visual('t1', 1.0, 2.0, 101.0))
        self.assertTrue(manager._quarantined_visual('t3', 1.0, 2.0, 101.0))
        self.assertFalse(manager._quarantined_visual('t3', 5.0, 2.0, 101.0))

    def test_failed_pixel_cue_revisits_camera_viewpoint_only(self):
        manager = swarm_manager.SwarmManager.__new__(swarm_manager.SwarmManager)
        manager.uav_ids = ['u0']
        manager.status = {'u0': SimpleNamespace(x=-3.5, y=-3.5,
                                                 connected=True)}
        manager._blocked_cells = set()
        manager.grid = swarm_task.CoverageGrid(-7, 7, -7, 7,
                                               num_uavs=1)
        manager.allocator = swarm_task.TaskAllocator(manager.grid)
        key = manager.grid.world_to_cell(-3.5, -3.5)
        manager.grid.cell(key).state = swarm_task.STATE_COVERED
        message = {'source_uav': 'u0', 'x': -3.5, 'y': -3.5,
                   'sample_s': 100.0, 'yaw': 1.0}
        with patch.object(swarm_manager.rospy.Time, 'now',
                          return_value=swarm_manager.rospy.Time.from_sec(100.2)), \
             patch.object(swarm_task, '_now', return_value=100.2):
            manager._visual_candidate_missed_cb(
                SimpleNamespace(data=json.dumps(message)))
        self.assertEqual(manager.grid.cell(key).state,
                         swarm_task.STATE_REVIEW)
        self.assertIn(('unconfirmed', key), manager.allocator.revisit_jobs)
        self.assertFalse(hasattr(manager, 'tracker'))


if __name__ == '__main__':
    unittest.main()
