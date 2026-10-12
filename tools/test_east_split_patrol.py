"""Two opt-in east lanes share reservations, lawful viewpoints and preemption."""
import math
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import test_southeast_patrol as patrol_fixture
import test_joint_dispatch as dispatch_fixture
import boundary_patrol as bp
import swarm_manager as sm
import swarm_agent as sa
import swarm_task as st


class EastSplitPatrolTests(unittest.TestCase):
    def setUp(self):
        patrol_fixture.SoutheastPatrolTests.setUp(self)
        self.m._regional_patrols = tuple(bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=False,
            east=True, east_half=half) for half in ('north', 'south'))
        self.m._south_patrol = self.m._regional_patrols[0]

    def refresh(self, now):
        self.now = now
        for uid, status in self.m.status.items():
            status.header.stamp = sm.rospy.Time.from_sec(now)
            self.m.last_report[uid] = sm.rospy.Time.from_sec(now)

    def allocate(self):
        self.m._allocate()
        return [task for task in self.m._segments.tasks.values()
                if task.purpose == 'south_patrol']

    def test_both_lanes_are_reserved_before_ordinary_routes_under_one_lock(self):
        original = self.m.allocator.allocate
        def ordinary(positions, **kwargs):
            regional = [t for t in self.m._segments.tasks.values()
                        if t.purpose == 'south_patrol']
            self.assertEqual(len(regional), 2)
            self.assertTrue(self.m._task_lock._is_owned())
            self.assertTrue(set(t.uid for t in regional).isdisjoint(positions))
            self.assertTrue(set(self.m._segments.reserved) <= kwargs['blocked'])
            return original(positions, **kwargs)
        with patch.object(self.m.allocator, 'allocate', side_effect=ordinary):
            tasks = self.allocate()
        self.assertEqual(len(tasks), 2)
        self.assertEqual(len(self.m._segments.tasks), 6)
        self.assertEqual(len({t.uid for t in tasks}), 2)
        cells = [key for task in self.m._segments.tasks.values() for key in task.cells]
        self.assertEqual(len(cells), len(set(cells)))
        for patrol in self.m._regional_patrols:
            task = self.m._segments.tasks[patrol.uid]
            for key, point, look in zip(task.cells, task.points, task.look_points):
                self.assertGreaterEqual(look[0], 105.)
                self.assertEqual(look[1] >= 0., patrol.east_half == 'north')
                self.assertAlmostEqual(math.dist(point, look), 8.95)
                self.assertTrue(self.m._search_point_free(point))
                self.assertTrue(self.m._blind_viewpoint_valid(NS(cx=look[0], cy=look[1]), point))
                self.assertNotEqual(self.m.grid.cell(key).state, st.STATE_COVERED)

    def test_concurrent_scheduler_calls_cannot_duplicate_lanes_or_cells(self):
        barrier = threading.Barrier(2)
        errors = []
        def tick():
            try:
                barrier.wait()
                self.m._allocate()
            except Exception as exc:
                errors.append(exc)
        workers = [threading.Thread(target=tick) for _ in range(2)]
        for worker in workers: worker.start()
        for worker in workers: worker.join(5.)
        self.assertFalse(errors)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        tasks = [t for t in self.m._segments.tasks.values() if t.purpose == 'south_patrol']
        self.assertEqual(len(tasks), 2)
        self.assertEqual(len(self.m._segments.tasks), 6)
        cells = [key for t in self.m._segments.tasks.values() for key in t.cells]
        self.assertEqual(len(cells), len(set(cells)))

    def test_outer_northeast_and_southeast_cells_have_safe_inward_viewpoints(self):
        north, south = (28, 17), (28, 0)
        self.m._blocked_cells = set(self.m.grid.cells)-{north, south}
        cell = self.m.grid.cell(north)
        self.assertEqual((cell.cx, cell.cy), (148., 59.5))
        # Test a corner leg from an eastern entry. The unchanged bounded
        # A* work budget may reject a direct 150 m diagonal from spawn;
        # neither this experiment nor its test bypasses that reachability gate.
        for uid, y in zip(self.m.uav_ids[:2], (45., -45.)):
            self.m.status[uid].x, self.m.status[uid].y = 110., y
        original = sm.flight_waypoint
        for margin in (1.5, 2.0):
            with self.subTest(margin=margin):
                legal = lambda x, y: original(x, y, inset=margin)
                self.m._segment_flight_waypoint = legal
                with patch.object(sm, 'flight_waypoint', side_effect=legal):
                    for patrol, key in zip(self.m._regional_patrols, (north, south)):
                        proposal = patrol.choose(self.m, self.m.uav_ids, self.now)
                        self.assertIsNotNone(proposal)
                        self.assertEqual(proposal[1], [key])
                        point = proposal[2][0]
                        look = patrol.proposal_look_points[0]
                        self.assertEqual(legal(*point), point)
                        self.assertAlmostEqual(math.dist(point, look), 8.95)
                        self.assertGreater(point[0], 133.)  # old band's omitted outer strip
                        self.assertTrue(self.m._blind_viewpoint_valid(NS(cx=look[0], cy=look[1]), point))
                        self.assertTrue(self.m._in_processed_view(
                            *point, math.atan2(look[1]-point[1], look[0]-point[0]),
                            *look, 4.5, 0.))
                        self.assertEqual((cell.cx, cell.cy), (148., 59.5))

    def test_empty_map_corner_deployment_can_plan_from_the_initial_aircraft_positions(self):
        # The original float priorities exhausted 2500 nodes even without
        # buildings; canonical move counts must support both initial legs.
        corners={(28,17),(28,0)}
        self.m._blocked_cells=set(self.m.grid.cells)-corners
        tasks=self.allocate()
        self.assertEqual(len(tasks),2)
        self.assertEqual({key for task in tasks for key in task.cells},corners)
        self.assertEqual(len({task.uid for task in tasks}),2)
        for task in tasks:
            self.assertEqual(len(task.points),1)
            self.assertTrue(self.m._search_point_free(task.points[0]))

    def test_person_preemption_replaces_only_its_lane_without_false_coverage(self):
        self.allocate()
        north, south = self.m._regional_patrols
        first = self.m._segments.tasks[north.uid]
        southern = self.m._segments.tasks[south.uid]
        self.m._tracking['real_person'] = first.uid
        self.m._free_search_lease(first.uid)
        self.assertIsNone(north.uid)
        self.assertEqual((south.uid, south.task_id), (southern.uid, southern.task_id))
        self.assertTrue(set(first.cells).isdisjoint(self.m._segments.reserved))
        donor = next(uid for uid in self.m.uav_ids
                     if uid not in (first.uid, southern.uid))
        self.m._drop_segment(donor, 'test_idle')
        self.refresh(103.)
        self.assertEqual(len(self.allocate()), 2)
        self.assertEqual(north.uid, donor)
        self.assertEqual(south.task_id, southern.task_id)
        self.assertNotIn(first.uid, self.m._segments.tasks)
        for key in first.cells:
            self.assertNotIn(key, self.m.allocator.failed_visit)
            self.assertNotEqual(self.m.grid.cell(key).state, st.STATE_COVERED)

    def test_extended_grid_metadata_cannot_add_outside_flight_region_cells(self):
        self.m.grid = st.CoverageGrid(-50, 157, -60, 67, 7, 6)
        self.m.allocator = st.TaskAllocator(self.m.grid)
        self.m.lease = st.LeaseManager(self.m.grid)
        legal, outside_x, outside_y = (28, 16), (29, 16), (28, 17)
        self.m._blocked_cells = set(self.m.grid.cells)-{legal, outside_x, outside_y}
        uid = self.m.uav_ids[0]
        self.m.status[uid].x, self.m.status[uid].y = 145., 58.
        north = self.m._regional_patrols[0]
        proposal = north.choose(self.m, [uid], self.now)
        self.assertIsNotNone(proposal)
        self.assertEqual(proposal[1], [legal])
        self.assertGreater(self.m.grid.cell(outside_x).cx, north.bounds[1])
        self.assertGreater(self.m.grid.cell(outside_y).cy, north.bounds[3])

    def test_failed_lane_keeps_owner_during_cooldown_then_uses_new_task(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m._drop_segment(old.uid, 'blocked_viewpoint', failed=True)
        self.m._release_cooldown[old.uid] = sm.rospy.Time(102.)
        self.refresh(101.)
        self.allocate()
        self.assertEqual(north.uid, old.uid)
        self.assertNotIn(old.uid, self.m._segments.tasks)
        self.assertEqual(south.task_id, south_id)
        self.refresh(102.1)
        self.allocate()
        new = self.m._segments.tasks[old.uid]
        self.assertNotEqual(new.task_id, old.task_id)
        self.assertTrue(set(new.cells).isdisjoint(old.cells))
        self.assertEqual(south.task_id, south_id)

    def test_unhealthy_owner_releases_only_its_lane_and_allows_replacement(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m.status[old.uid].connected = False
        donor = next(uid for uid in self.m.uav_ids if uid not in (old.uid, south.uid))
        self.m._drop_segment(donor, 'test_idle')
        self.refresh(103.)
        self.allocate()
        self.assertEqual(north.uid, donor)
        self.assertEqual(south.task_id, south_id)
        self.assertNotIn(old.uid, self.m._segments.tasks)

    def test_unreachable_lane_releases_idle_owner_after_bounded_retry(self):
        self.allocate()
        north, south = self.m._regional_patrols
        old = self.m._segments.tasks[north.uid]
        south_id = south.task_id
        self.m._drop_segment(old.uid, 'blocked_viewpoint')
        self.m._blocked_cells |= {key for key, cell in self.m.grid.cells.items() if cell.cy >= 0.}
        self.refresh(103.)
        self.assertIsNone(north.choose(self.m, [old.uid], self.now))
        self.assertEqual(north.uid, old.uid)
        self.refresh(114.)
        self.assertIsNone(north.choose(self.m, [old.uid], self.now))
        self.assertIsNone(north.uid)
        self.assertGreater(north.owner_retry[old.uid], self.now)
        self.assertEqual(south.task_id, south_id)

    def test_split_lane_argument_does_not_change_default_east_band(self):
        legacy = bp.SouthPatrol((-50, 150, -60, 60), southeast=False,
                                northeast=False, east=True)
        self.assertIsNone(legacy.east_half)
        for kwargs in (dict(east=False, east_half='north'),
                       dict(east=True, east_half='west')):
            with self.assertRaises(ValueError):
                bp.SouthPatrol((-50, 150, -60, 60), southeast=False, northeast=False, **kwargs)

    def edge_and_inner(self):
        edge, inner = (22, 15), (22, 10)
        self.m._blocked_cells = set(self.m.grid.cells)-{edge, inner}
        uid = self.m.uav_ids[0]
        self.m.status[uid].x, self.m.status[uid].y = 98.55, 13.5
        return self.m._regional_patrols[0], uid, edge, inner

    def test_unobserved_edge_precedes_nearer_middle_even_after_waypoint_visit(self):
        patrol, uid, edge, inner = self.edge_and_inner()
        self.m.allocator.visit_time[edge] = self.now-.1
        self.m._camera_point_seen = {(edge, 4): self.now-1.}  # not a task-bound view
        proposal = patrol.choose(self.m, [uid], self.now)
        self.assertEqual(proposal[1], [edge])
        legacy = bp.SouthPatrol((-50, 150, -60, 60),
            southeast=False, northeast=False, east=True)
        self.assertEqual(legacy.choose(self.m, [uid], self.now)[1], [inner])

    def test_actual_fresh_task_view_removes_first_edge_priority_after_cache_pruning(self):
        patrol, uid, edge, inner = self.edge_and_inner()
        proposal = patrol.choose(self.m, [uid], self.now)
        task = self.m._start_search_segment(uid, proposal[1], proposal[2], self.now,
            purpose='south_patrol', publish=False, max_hold=90.)
        patrol.started(task, proposal[3], self.now)
        self.m._camera_point_seen = {(edge, 4): 100.8}
        self.refresh(101.)
        self.m._drop_segment(uid, 'complete')
        self.assertIn(edge, patrol._edge_observed)
        self.m._camera_point_seen.clear()
        self.assertFalse(patrol._edge_needs_first_view(self.m, edge))
        self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [inner])

    def test_old_missing_or_future_images_cannot_waive_edge_first_view(self):
        for sample in (None, 99.9, 102., math.nan, math.inf):
            with self.subTest(sample=sample):
                self.refresh(100.)
                patrol, uid, edge, _ = self.edge_and_inner()
                patrol.uid = patrol.task_id = None
                patrol.next_s = 0.
                self.m.allocator.failed_visit.clear()
                self.m._camera_point_seen = {} if sample is None else {(edge, 4): sample}
                proposal = patrol.choose(self.m, [uid], 100.)
                task = self.m._start_search_segment(uid, proposal[1], proposal[2], 100.,
                    purpose='south_patrol', publish=False, max_hold=90.)
                patrol.started(task, proposal[3], 100.)
                self.refresh(101.)
                self.m._drop_segment(uid, 'complete')
                self.assertNotIn(edge, patrol._edge_observed)
                self.assertTrue(patrol._edge_needs_first_view(self.m, edge))
                self.assertEqual(self.m.allocator.failed_visit[edge], 101.)

    def test_earlier_leg_processed_image_remains_valid_at_later_task_completion(self):
        patrol, uid, edge, inner = self.edge_and_inner()
        proposal = patrol.choose(self.m, [uid], self.now)
        task = self.m._start_search_segment(uid, proposal[1], proposal[2], self.now,
            purpose='south_patrol', publish=False, max_hold=90.)
        patrol.started(task, proposal[3], self.now)
        # This is manager-accepted image evidence from this task, already
        # fresh when received. Reaching later legs must not erase the first
        # actual view just because the image is now more than .7 seconds old.
        self.m._camera_point_seen = {(edge, 4): 100.01}
        self.refresh(120.)
        self.m._drop_segment(uid, 'complete')
        self.assertIn(edge, patrol._edge_observed)
        self.assertNotIn(edge, self.m.allocator.failed_visit)
        self.m._camera_point_seen.clear()
        self.assertFalse(patrol._edge_needs_first_view(self.m, edge))
        self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [inner])

    def test_unreachable_edge_uses_safe_middle_and_existing_failure_cooldown(self):
        patrol, uid, edge, inner = self.edge_and_inner()
        original = bp.plan
        probes = []
        def reachable(grid, start, goal, **kwargs):
            probes.append(goal)
            if goal[1] > 46.:
                return type('FailedRoute', (), {'success': False})()
            return original(grid, start, goal, **kwargs)
        with patch.object(bp, 'plan', side_effect=reachable):
            self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [inner])
        self.assertEqual(self.m.allocator.failed_visit[edge], self.now)
        probes.clear()
        with patch.object(bp, 'plan', side_effect=reachable):
            self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [inner])
        self.assertTrue(all(point[1] <= 46. for point in probes))
        self.refresh(self.now+self.m.allocator.failed_novelty_ttl+.1)
        self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [edge])

    def test_outer_strip_uses_bounds_and_cell_size_not_fixed_compass_coordinates(self):
        self.m.grid = st.CoverageGrid(-40, 160, -30, 70, 7, 6)
        self.m._camera_point_seen = {}
        north = bp.SouthPatrol((-40, 160, -30, 70), southeast=False,
                              northeast=False, east=True, east_half='north')
        south = bp.SouthPatrol((-40, 160, -30, 70), southeast=False,
                              northeast=False, east=True, east_half='south')
        for key, cell in self.m.grid.cells.items():
            self.assertEqual(north._edge_needs_first_view(self.m, key), cell.cy >= 56.)
            self.assertEqual(south._edge_needs_first_view(self.m, key), cell.cy <= -16.)

    def test_equal_length_extension_points_outward_in_both_lanes(self):
        for half, row, direction in (('north', 15, 1), ('south', 1, -1)):
            with self.subTest(half=half):
                patrol = self.m._regional_patrols[0 if half == 'north' else 1]
                keys = {(22, row-1), (22, row), (22, row+1)}
                self.m._blocked_cells = set(self.m.grid.cells)-keys
                uid = self.m.uav_ids[0]
                self.m.status[uid].x = 98.55
                self.m.status[uid].y = self.m.grid.cell((22, row)).cy
                proposal = patrol.choose(self.m, [uid], self.now)
                self.assertEqual(proposal[1], [(22, row), (22, row+direction)])
                self.assertEqual(proposal[3], direction)

    def short_outward_setup(self, half):
        row, outward = (15, 1) if half == 'north' else (1, -1)
        patrol = self.m._regional_patrols[0 if half == 'north' else 1]
        keys = {(22, row), (22, row+outward),
                (22, row-outward), (22, row-2*outward)}
        self.m._blocked_cells = set(self.m.grid.cells)-keys
        uid = self.m.uav_ids[0]
        self.m.status[uid].x = 98.55
        self.m.status[uid].y = self.m.grid.cell((22, row)).cy
        return patrol, uid, row, outward, keys

    def test_first_edge_uses_two_point_outward_leg_instead_of_three_point_inward_leg(self):
        # Real seven-metre rows from the 004028 run: north 15/16 versus
        # 15/14/13, south 1/0 versus 1/2/3. Three is a maximum, not a quota.
        for half in ('north', 'south'):
            with self.subTest(half=half):
                patrol, uid, row, outward, _ = self.short_outward_setup(half)
                proposal = patrol.choose(self.m, [uid], self.now)
                self.assertEqual(proposal[1], [(22, row), (22, row+outward)])
                self.assertEqual(proposal[3], outward)
                self.assertAlmostEqual(math.dist(*proposal[2]), 7.)
                self.assertTrue(bp.clear_line(self.m._g_infl, *proposal[2]))
                self.assertTrue(all(self.m._search_point_free(point)
                                    for point in proposal[2]))

    def test_blocked_outward_link_falls_back_to_checked_three_point_inward_leg(self):
        original = bp.clear_line
        for half in ('north', 'south'):
            with self.subTest(half=half):
                patrol, uid, row, outward, _ = self.short_outward_setup(half)
                start = (98.55, self.m.grid.cell((22, row)).cy)
                outer = (98.55, self.m.grid.cell((22, row+outward)).cy)
                def clear_except_outer(grid, a, b):
                    if math.dist(a, start) < .01 and math.dist(b, outer) < .01:
                        return False
                    return original(grid, a, b)
                with patch.object(bp, 'clear_line', side_effect=clear_except_outer):
                    proposal = patrol.choose(self.m, [uid], self.now)
                self.assertEqual(proposal[1], [(22, row), (22, row-outward),
                                              (22, row-2*outward)])
                self.assertEqual(proposal[3], -outward)

    def test_previously_observed_edge_keeps_longest_safe_leg_policy(self):
        for half in ('north', 'south'):
            with self.subTest(half=half):
                patrol, uid, row, outward, keys = self.short_outward_setup(half)
                patrol._edge_observed.update(keys)
                proposal = patrol.choose(self.m, [uid], self.now)
                self.assertEqual(proposal[1], [(22, row), (22, row-outward),
                                              (22, row-2*outward)])
                self.assertEqual(proposal[3], -outward)


class DefaultEastSplitPatrolTests(unittest.TestCase):
    use_default_flight_bounds = True
    allocate = EastSplitPatrolTests.allocate
    refresh = EastSplitPatrolTests.refresh

    def setUp(self):
        patrol_fixture.SoutheastPatrolTests.setUp(self)
        bounds = (self.m.grid.x_min, self.m.grid.x_max,
                  self.m.grid.y_min, self.m.grid.y_max)
        self.m._regional_patrols = tuple(bp.SouthPatrol(bounds,
            southeast=False, northeast=False, east=True, east_half=half)
            for half in ('north', 'south'))
        self.m._south_patrol = self.m._regional_patrols[0]

    def test_default_two_lanes_allocate_in_468_actor_cells_with_legal_points(self):
        self.assertEqual(sa.flight_bounds(), (-50., 150., -60., 60.))
        self.assertEqual(len(self.m.grid.cells), 468)
        self.m._allocate()
        tasks = [t for t in self.m._segments.tasks.values() if t.purpose == 'south_patrol']
        self.assertEqual(len(tasks), 2)
        all_keys = [key for t in self.m._segments.tasks.values() for key in t.cells]
        self.assertEqual(len(all_keys), len(set(all_keys)))
        for patrol in self.m._regional_patrols:
            task = self.m._segments.tasks[patrol.uid]
            for key, point, look in zip(task.cells, task.points, task.look_points):
                self.assertTrue(-58. <= point[1] <= 58.)
                self.assertTrue(-60. < look[1] < 60.)
                self.assertLess(look[0], 130.)
                self.assertEqual(look[1] >= 0., patrol.east_half == 'north')
                self.assertTrue(self.m._blind_viewpoint_valid(NS(cx=look[0], cy=look[1]), point))
                self.assertAlmostEqual(math.dist(point, look), 8.95)

    def test_default_true_corner_keeps_cell59_5_and_clamps_aircraft_inside(self):
        north, south = (25, 17), (25, 0)
        self.m._blocked_cells = set(self.m.grid.cells)-{north, south}
        self.assertEqual((self.m.grid.cell(north).cx, self.m.grid.cell(north).cy), (127.5, 59.5))
        original = sm.flight_waypoint
        for patrol, key in zip(self.m._regional_patrols, (north, south)):
            for margin in (1.5, 2.):
                with self.subTest(lane=patrol.east_half, margin=margin):
                    legal = lambda x, y: original(x, y, inset=margin)
                    self.m._segment_flight_waypoint = legal
                    with patch.object(sm, 'flight_waypoint', side_effect=legal):
                        proposal = patrol.choose(self.m, self.m.uav_ids, self.now)
                    self.assertIsNotNone(proposal)
                    self.assertEqual(proposal[1], [key])
                    point = proposal[2][0]
                    look = patrol.proposal_look_points[0]
                    self.assertEqual(legal(*point), point)
                    self.assertLessEqual(abs(point[1]), 60.-margin)
                    self.assertAlmostEqual(math.dist(point, look), 8.95)
                    self.assertEqual(look, (129.99, 59.99 if patrol.east_half == 'north' else -59.99))
                    with patch.object(sm, 'flight_waypoint', side_effect=legal):
                        self.assertTrue(self.m._blind_viewpoint_valid(NS(cx=look[0], cy=look[1]), point))
                    actual = (130., 60. if patrol.east_half == 'north' else -60.)
                    yaw = math.atan2(actual[1]-point[1], actual[0]-point[0])
                    self.assertTrue(self.m._in_processed_view(*point, yaw, *actual, 4.5, 0.))
                    self.assertLess(math.dist(point, actual), 9.)
                    self.assertEqual(self.m.grid.cell(north).cy, 59.5)

    def test_cell_centre_standoff_misses_actual_corners_but_explicit_look_does_not(self):
        corners = {(25, 17), (25, 0)}
        self.m._blocked_cells = set(self.m.grid.cells)-corners
        tasks = [task for task in self.allocate()]
        self.assertEqual({task.cells[0] for task in tasks}, corners)
        for task in tasks:
            cell = self.m.grid.cell(task.cells[0])
            actual = (130., 60. if cell.cy > 0. else -60.)
            _, safe_y = self.m._segment_flight_waypoint(cell.cx-8.95, cell.cy)
            old_point = (cell.cx-math.sqrt(8.95**2-(cell.cy-safe_y)**2), safe_y)
            self.assertGreater(math.dist(old_point, actual), 9.5)
            point, look = task.points[0], task.look_points[0]
            self.assertLess(math.dist(point, actual), 9.)
            self.assertNotEqual(look, (cell.cx, cell.cy))
            self.assertEqual(self.m.grid.world_to_cell(*look), task.cells[0])
            self.assertNotEqual(cell.state, st.STATE_COVERED)
            messages = [call.args[0] for call in self.m.assign_pub.publish.call_args_list
                        if call.args[0].search_task_id == task.task_id]
            self.assertTrue(messages)
            self.assertEqual([(p.x, p.y) for p in messages[-1].search_look_points],
                             list(task.look_points))

    def corner_and_near_edge(self):
        patrol = self.m._regional_patrols[0]
        uid = self.m.uav_ids[0]
        self.m.status[uid].x, self.m.status[uid].y = 84., 55.
        corner, near = (25, 17), (20, 17)
        self.m._blocked_cells = set(self.m.grid.cells)-{corner, near}
        return patrol, uid, corner, near

    def test_unseen_actual_corner_precedes_nearest_outer_strip(self):
        patrol, uid, corner, near = self.corner_and_near_edge()
        proposal = patrol.choose(self.m, [uid], self.now)
        self.assertEqual(proposal[1], [corner])
        self.assertEqual(patrol.proposal_look_points, [(129.99, 59.99)])
        self.assertLess(math.dist((self.m.status[uid].x, self.m.status[uid].y),
                                 self.m._segment_point(near)),
                        math.dist((self.m.status[uid].x, self.m.status[uid].y), proposal[2][0]))

    def test_centre_only_image_does_not_waive_true_corner_priority(self):
        patrol, uid, corner, _ = self.corner_and_near_edge()
        proposal = patrol.choose(self.m, [uid], self.now)
        task = self.m._start_search_segment(uid, proposal[1], proposal[2], self.now,
            purpose='south_patrol', publish=False,
            look_points=patrol.proposal_look_points)
        patrol.started(task, proposal[3], self.now)
        self.m._camera_point_seen = {(corner, 4): 100.4}
        self.refresh(101.)
        self.m._drop_segment(uid, 'complete')
        self.assertIn(corner, patrol._edge_observed)
        self.assertNotIn(corner, patrol._corner_observed)
        self.assertTrue(patrol._corner_needs_first_view(self.m, corner))
        self.assertTrue(patrol._edge_needs_first_view(self.m, corner))
        self.assertEqual(self.m.allocator.failed_visit[corner], 101.)
        self.assertNotEqual(self.m.grid.cell(corner).state, st.STATE_COVERED)

    def test_real_task_corner_image_survives_pruning_and_revision(self):
        patrol, uid, corner, _ = self.corner_and_near_edge()
        proposal = patrol.choose(self.m, [uid], self.now)
        task = self.m._start_search_segment(uid, proposal[1], proposal[2], self.now,
            purpose='south_patrol', publish=False,
            look_points=patrol.proposal_look_points)
        patrol.started(task, proposal[3], self.now)
        look, deadline = task.look_points, task.deadline
        self.m._revise_segment(task, list(task.cells), list(task.points), 100.1,
                               'test_prefix_preserves_corner')
        self.assertEqual(task.look_points, look)
        self.assertEqual(task.deadline, deadline)
        self.m._camera_point_seen = {(corner, 8): 100.4}
        self.refresh(101.)
        self.m._drop_segment(uid, 'complete')
        self.assertIn(corner, patrol._corner_observed)
        self.m._camera_point_seen.clear()
        self.assertFalse(patrol._corner_needs_first_view(self.m, corner))
        self.assertNotIn(corner, self.m.allocator.failed_visit)
        self.assertNotEqual(self.m.grid.cell(corner).state, st.STATE_COVERED)

    def test_both_corners_require_their_exact_current_subpoint(self):
        for patrol, key, index in zip(self.m._regional_patrols,
                                     ((25, 17), (25, 0)), (8, 2)):
            patrol._manager = self.m
            task = NS(cells=(key,), created=100.)
            for point, sample in ((4, 100.4), (10-index, 100.4),
                                  (index, 99.9), (index, 101.1),
                                  (index, math.nan), (index, math.inf)):
                with self.subTest(lane=patrol.east_half, point=point, sample=sample):
                    self.m._camera_point_seen = {(key, point): sample}
                    self.assertEqual(patrol._fresh_corner_keys(task, 101.), set())
            self.m._camera_point_seen = {(key, index): 100.4}
            self.assertEqual(patrol._fresh_corner_keys(task, 101.), {key})

    def test_blocked_actual_corner_falls_back_and_keeps_finite_retry(self):
        patrol, uid, corner, near = self.corner_and_near_edge()
        original = bp.plan
        probes = []
        def reachable(grid, start, goal, **kwargs):
            probes.append(goal)
            if goal[0] > 120.:
                return NS(success=False)
            return original(grid, start, goal, **kwargs)
        with patch.object(bp, 'plan', side_effect=reachable):
            proposal = patrol.choose(self.m, [uid], self.now)
        self.assertEqual(proposal[1], [near])
        self.assertEqual(self.m.allocator.failed_visit[corner], self.now)
        self.assertLessEqual(len(probes), 24)
        probes.clear()
        with patch.object(bp, 'plan', side_effect=reachable):
            self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [near])
        self.assertTrue(all(point[0] < 120. for point in probes))
        self.refresh(self.now+self.m.allocator.failed_novelty_ttl+.1)
        self.assertEqual(patrol.choose(self.m, [uid], self.now)[1], [corner])


class EastSplitPersonDispatchTests(unittest.TestCase):
    def setUp(self):
        dispatch_fixture.JointDispatchTests.setUp(self)
        self.m._regional_patrols = tuple(bp.SouthPatrol(
            (-50, 150, -60, 60), southeast=False, northeast=False,
            east=True, east_half=half) for half in ('north', 'south'))
        for patrol, uid in zip(self.m._regional_patrols, ('u0', 'u1')):
            patrol.uid = uid

    def test_both_lane_owners_reject_blind_west_but_own_camera_preempts(self):
        self.assertFalse(self.m._patrol_accepts_target('u0', 1., 0., False))
        self.assertFalse(self.m._patrol_accepts_target('u1', 1., 0., False))
        self.assertTrue(self.m._patrol_accepts_target('u2', 1., 0., False))
        for uid in ('u0', 'u1'):
            with self.subTest(uid=uid), patch.object(self.m, '_camera_candidates',
                                                    return_value=[(uid, 0.)]):
                self.m._tracking.clear()
                self.m._dispatch_target_batch([('t0', 1., 0.)], 100.)
                self.assertEqual(self.m._tracking, {'t0': uid})


if __name__ == '__main__':
    unittest.main()
