"""Public road preference remains bounded, sensor-checked and image-based."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import test_search_segments as fixture
import swarm_manager as sm
import swarm_task as st
from building_viewpoints import measured_view_options, reserved_subpoint, valid_view
from road_first_pass import RoadFirstPass, PUBLIC_FIRST_LANES
from search_observation import ground_distance_band
from search_segments import SegmentBook, DONE, clear_line


class RoadFirstPassTests(unittest.TestCase):
    def setUp(self):
        self.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7., 6)
        self.road = RoadFirstPass()
        self.book = SegmentBook()
        self.positions = {str(i): (0., -float(i)) for i in range(6)}

    def build(self, uid, keys, heading):
        # Pure-task fixture; integration cases below check actual SLAM paths.
        keys = keys[:3]
        points = [(self.grid.cell(k).cx, self.grid.cell(k).cy) for k in keys]
        return keys, points, points

    def choose(self, positions=None, now=100.):
        return self.road.choose(self.grid, self.positions if positions is None else positions,
            now, lambda k: k not in self.book.reserved, self.build)

    def start(self, proposal, now=100.):
        lane, uid, cells, points, looks = proposal
        task = self.book.reserve(uid, cells, points, now, 20., self.road.remaining(now),
                                 purpose='coverage_view', look_points=looks)
        if task is not None:
            self.road.started(task, lane)
        return task

    def test_two_eastern_owners_leave_four_complementary_finite_lanes(self):
        available = {uid: p for uid, p in self.positions.items() if uid not in ('4', '5')}
        tasks = []
        for _ in range(4):
            proposal = self.choose(available)
            self.assertIsNotNone(proposal)
            task = self.start(proposal)
            tasks.append(task)
            available.pop(task.uid)
        self.assertEqual(set(self.road.bindings.values()), {p.name for p in PUBLIC_FIRST_LANES})
        self.assertEqual(len(set(t.uid for t in tasks)), 4)
        self.assertNotIn('4', self.road.owners.values())
        self.assertNotIn('5', self.road.owners.values())
        self.assertEqual(len(self.book.reserved), sum(len(t.cells) for t in tasks))
        self.assertTrue(all(len(t.cells) <= 3 for t in tasks))
        self.assertTrue(all(look[0] < 95. for t in tasks for look in t.look_points))

    def test_reservation_conflict_cannot_claim_lane_or_partial_route(self):
        proposal = self.choose()
        lane, uid, keys, points, looks = proposal
        self.book.reserve('other', [keys[1]], [points[1]], 100., 20., 90.)
        self.assertIsNone(self.start(proposal))
        self.assertEqual(self.road.owners, {})
        self.assertEqual(self.road.bindings, {})
        self.assertEqual(set(self.book.reserved), {keys[1]})

    def test_unreachable_proposals_do_not_acquire_owner_or_start_budget(self):
        proposal = self.road.choose(self.grid, self.positions, 100., lambda k: True,
                                   lambda *args: None)
        self.assertIsNone(proposal)
        self.assertEqual(self.road.owners, {})
        self.assertIsNone(self.road.started_s)

    def test_person_interruption_retains_progress_releases_owner_and_unfinished_cells(self):
        task = self.start(self.choose())
        first, pending = task.cells[0], task.cells[1:]
        lane = self.road.bindings[task.task_id]
        self.book.advance(task.uid, task.task_id, task.revision, 1)
        self.road.progress(task)
        self.book.release(task.uid)
        self.road.ended(task, 'person_task', 103.)
        self.assertIn(first, self.road.visited[lane])
        self.assertFalse(set(pending) & self.road.visited[lane])
        self.assertNotIn(lane, self.road.owners)
        self.assertEqual(self.book.reserved, {})
        proposal = self.choose({uid: xy for uid, xy in self.positions.items() if uid != task.uid}, 104.)
        self.assertEqual(proposal[0], lane)
        self.assertNotIn(first, proposal[2])
        self.assertEqual(self.grid.cell(first).state, st.STATE_FREE)
        self.assertEqual(self.road.started_s, 100.)

    def test_revise_cannot_forget_prefix_or_extend_first_pass_budget(self):
        task = self.start(self.choose())
        lane = self.road.bindings[task.task_id]
        first = task.cells[0]
        self.book.advance(task.uid, task.task_id, 1, 1)
        self.road.progress(task)
        self.book.revise(task.uid, task.cells[1:], task.points[1:], 110.)
        self.book.release(task.uid)
        self.road.ended(task, 'person_task', 111.)
        self.assertIn(first, self.road.visited[lane])
        self.assertEqual(self.road.remaining(180.), 10.)
        self.assertIsNone(self.choose(now=190.))

    def test_failed_remaining_cells_have_bounded_retry_not_permanent_blacklist(self):
        task = self.start(self.choose())
        lane = self.road.bindings[task.task_id]
        self.book.release(task.uid)
        self.road.ended(task, 'no_path_progress', 105.)
        self.assertTrue(all(self.road.retry_after[(lane, key)] == 113. for key in task.cells))
        self.assertTrue(all(key not in self.road.visited[lane] for key in task.cells))

    def test_first_available_lane_prioritizes_middle_east_without_a_third_outer_east_job(self):
        proposal = self.choose({'healthy_near': (44., 0.), 'healthy_far': (0., -8.)})
        self.assertEqual(proposal[0], 'middle_east')
        self.assertEqual(proposal[1], 'healthy_near')
        self.assertTrue(all(41.5 <= self.grid.cell(key).cx <= 93.5 for key in proposal[2]))
        self.assertEqual({lane.name for lane in self.road.lanes},
                         {'middle_east', 'central_east', 'west_north', 'west_south'})

    def test_completed_owner_missing_height_or_busy_ordinary_can_be_replaced_without_erasing_progress(self):
        for reason in ('unknown_height_window', 'ordinary_search'):
            with self.subTest(reason=reason):
                self.road=RoadFirstPass();self.book=SegmentBook()
                task=self.start(self.choose())
                lane=self.road.bindings[task.task_id]
                visited=task.cells
                self.book.advance(task.uid,task.task_id,task.revision,len(task.cells))
                self.book.release(task.uid)
                self.road.ended(task,'complete',105.)
                self.assertEqual(self.road.owners[lane],task.uid)
                eligible={uid:xy for uid,xy in self.positions.items() if uid!=task.uid}
                proposal=self.choose(eligible,106.)
                self.assertEqual(proposal[0],lane)
                self.assertNotEqual(proposal[1],task.uid)
                self.assertFalse(set(proposal[2]) & set(visited))
                self.assertEqual(self.road.visited[lane],set(visited))
                self.assertEqual(self.road.started_s,100.)
                self.assertEqual(self.road.remaining(106.),84.)

    def test_active_owner_cannot_be_released_even_if_missing_current_eligible_positions(self):
        task=self.start(self.choose())
        lane=self.road.bindings[task.task_id]
        eligible={uid:xy for uid,xy in self.positions.items() if uid!=task.uid}
        proposal=self.choose(eligible,104.)
        self.assertEqual(self.road.owners[lane],task.uid)
        self.assertEqual(self.road.bindings[task.task_id],lane)
        self.assertNotEqual(proposal[0],lane)
        self.assertEqual(self.book.tasks[task.uid],task)
        self.assertEqual(self.road.started_s,100.)


class DirectedViewManagerTests(unittest.TestCase):
    uid = fixture.ManagerSegmentTests.uid

    def setUp(self):
        fixture.ManagerSegmentTests.setUp(self)
        self.m.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7., 6)
        self.m.allocator = st.TaskAllocator(self.m.grid)
        self.m.lease = st.LeaseManager(self.m.grid)
        self.m._g_infl = sm.GridMap(240, 160, 1., (-70., -80.), bytes(240*160))
        self.m._merged_grid = Mock(return_value=self.m._g_infl)
        self.m._processed_camera_poses = {self.uid: {'sample_s': self.now, 'z': 4.4, 'yaw': 0.}}
        self.m._processed_camera_height_window = {
            self.uid: [(self.now-.4,4.4),(self.now-.2,4.4),(self.now,4.4)]}
        self.m.status[self.uid].x = 0.
        self.m.status[self.uid].y = 0.
        self.m.status[self.uid].z = 4.4
        self.m._cell_waypoint = {key: (cell.cx, cell.cy) for key, cell in self.m.grid.cells.items()}
        self.m._directed_coverage = True
        self.m._routine_search_bounds = True
        self.m._road_first_pass = RoadFirstPass()
        self.m._camera_point_seen = {}

    def east_cells(self):
        return [self.m.grid.world_to_cell(x, 0.) for x in (16., 23., 30.)]

    def test_measured_high_hover_has_valid_standoff_instead_of_cell_centre(self):
        proposal = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        self.assertIsNotNone(proposal)
        cells, points, looks = proposal
        self.assertEqual(len(cells), 3)
        near, far = ground_distance_band(4.4, 0.)
        self.assertLess(far-near, .4)
        for key, point, look in zip(cells, points, looks):
            self.assertTrue(reserved_subpoint(self.m.grid, key, look))
            self.assertTrue(valid_view(self.m._g_infl, self.m._g_infl, look, point,
                                       self.m._segment_flight_waypoint, 4.4))
            self.assertGreaterEqual(math.dist(point, look), near)
            self.assertLessEqual(math.dist(point, look), far)
            self.assertNotEqual(point, look)
        self.assertTrue(all(clear_line(self.m._g_infl, a, b) for a, b in zip(points, points[1:])))

    def test_unknown_stale_nonfinite_and_impossible_height_never_fallback_to_4_5(self):
        for camera in (None, {}, {'sample_s': 90., 'z': 4.4},
                       {'sample_s': 101., 'z': 4.4}, {'sample_s': 100., 'z': math.nan},
                       {'sample_s': 100., 'z': 4.7}, {'sample_s': 100., 'z': 1.9}):
            with self.subTest(camera=camera):
                self.m._processed_camera_poses[self.uid] = camera
                self.assertIsNone(self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.))
        self.m._processed_camera_poses[self.uid] = {'sample_s': 100., 'z': 3.}
        # Valid geometry at 3m is insufficient if the aircraft is at 4.4m.
        self.assertIsNone(self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.))

    def test_midpoint_visibility_uses_actual_height_and_preserves_legacy_defaults(self):
        key = self.east_cells()[0]
        self.m.status[self.uid].z = 2.6
        self.m._processed_camera_poses[self.uid]['z'] = 2.6
        proposal = self.m._directed_segment(self.uid, [key], self.now, 0.)
        point, look = proposal[1][0], proposal[2][0]
        self.assertTrue(valid_view(self.m._g_infl, self.m._g_infl, look, point,
                                  self.m._segment_flight_waypoint, 2.6))
        self.assertFalse(valid_view(self.m._g_infl, self.m._g_infl, look, point,
                                   self.m._segment_flight_waypoint))

    def test_occupied_ground_or_unsafe_air_grid_returns_no_view(self):
        key = self.east_cells()[0]
        occupied = sm.GridMap(240, 160, 1., (-70., -80.), bytes([1])*(240*160))
        for raw, air in ((occupied, self.m._g_infl), (self.m._g_infl, occupied), (None, self.m._g_infl)):
            with self.subTest(raw=raw is None, air=air is occupied):
                self.m._merged_grid.return_value = raw
                self.m._g_infl = air
                self.assertIsNone(self.m._directed_segment(self.uid, [key], self.now, 0.))

    def test_obstructed_second_connection_shortens_route(self):
        original = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        self.assertEqual(len(original[0]), 3)
        grid = self.m._g_infl
        data = bytearray(grid.cells)
        # An inflated flight barrier separates first/second viewpoints; raw
        # ground LOS still permits the first image. No alternative A* path or
        # diagonal corner may manufacture a continuous safe flight leg.
        x = grid.world_to_cell(((original[1][0][0]+original[1][1][0])*.5, 0.))[0]
        for y in range(grid.height):
            data[y*grid.width+x] = 1
        self.m._g_infl = sm.GridMap(grid.width, grid.height, grid.resolution, grid.origin, data)
        shortened = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        self.assertIsNotNone(shortened)
        self.assertEqual(len(shortened[0]), 1)

    def test_allocation_does_not_claim_coverage_and_person_release_uses_existing_entry(self):
        idle = [self.uid]
        self.m._allocate_road_first_pass(idle, self.now)
        self.assertEqual(idle, [])
        task = self.m._segments.tasks[self.uid]
        self.assertEqual(task.purpose, 'coverage_view')
        self.assertEqual(len(self.m._segments.reserved), len(task.cells))
        self.assertFalse(any(self.m.grid.cell(k).state == st.STATE_COVERED for k in task.cells))
        self.assertEqual(self.m.allocator.last_seen, {})
        self.m._drop_segment(self.uid, 'person_task')
        self.assertEqual(self.m._segments.reserved, {})
        self.assertEqual(self.m._road_first_pass.bindings, {})
        self.assertFalse(any(self.m.grid.cell(k).state == st.STATE_COVERED for k in task.cells))

    def test_passed_sample_does_not_remove_unobserved_other_samples(self):
        cells, points, looks = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        key = cells[0]
        self.m._camera_point_seen[(key, 4)] = self.now
        self.m._segment_visit(self.uid, key, self.now+.2)
        self.assertNotEqual(self.m.grid.cell(key).state, st.STATE_COVERED)
        self.assertIn(('blind', key), self.m.allocator.revisit_jobs)
        self.assertEqual(self.m.allocator.last_seen, {})

    def test_old_task_revision_cannot_release_new_ground_sample_reservation(self):
        cells, points, looks = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        self.m._processed_camera_poses[self.uid]['sample_s'] = self.now+1.
        self.m._revise_segment(task, cells[:1], points[:1], self.now+1., 'test')
        self.assertEqual(task.look_points, (looks[0],))
        self.assertIsNone(self.m._segments.release(self.uid, task.task_id, 1))
        self.assertEqual(task.deadline, self.now+90.)

    def test_replan_to_cell_centre_cannot_destroy_checked_ground_view(self):
        cells, points, looks = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        self.m._revise_segment(task, cells[:1], looks[:1], self.now, 'yield_replan')
        self.assertNotIn(self.uid, self.m._segments.tasks)
        self.assertEqual(self.m._segments.reserved, {})
        self.assertNotIn(cells[0], self.m.allocator.failed_visit)

    def test_expired_camera_height_cannot_authorize_replanning(self):
        cells, points, looks = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        self.m._revise_segment(task, cells[:1], points[:1], self.now+2., 'new_obstacle')
        self.assertEqual(task.revision, 1)
        self.assertEqual(task.deadline, self.now+90.)

    def test_measured_height_change_releases_old_view_without_marking_false_obstacle(self):
        cells, points, looks = self.m._directed_segment(self.uid, self.east_cells(), self.now, 0.)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        self.m.status[self.uid].z = 4.57
        self.m._processed_camera_poses[self.uid]['z'] = 4.57
        self.m._reconcile_segments()
        self.assertNotIn(self.uid, self.m._segments.tasks)
        self.assertEqual(self.m._segments.reserved, {})
        self.assertFalse(any(k in self.m.allocator.failed_visit for k in task.cells))
        self.assertFalse(any(self.m.grid.cell(k).state == st.STATE_COVERED for k in task.cells))

    def test_requires_samples_and_max_three_even_for_new_purpose(self):
        with self.assertRaises(ValueError):
            SegmentBook().reserve('a', [(0, 0)], [(0., 0.)], 100., 20., 90., purpose='coverage_view')
        with self.assertRaises(ValueError):
            SegmentBook().reserve('a', [(i, 0) for i in range(4)], [(float(i), 0.) for i in range(4)],
                100., 20., 90., purpose='coverage_view', look_points=[(float(i), 0.) for i in range(4)])

    def test_recorded_adjacent_border_views_do_not_issue_point_02m_apart(self):
        # B1 task48bfdbe... used look12.99 and13.01 in adjacent cells,
        # issuing flight points only .02m apart and spending5.6s on them.
        keys = [self.m.grid.world_to_cell(x, 0.) for x in (12., 16., 23., 30.)]
        cells, points, looks = self.m._directed_segment(self.uid, keys, self.now, 0.)
        self.assertEqual(len(cells), 3)
        self.assertNotIn(keys[1], cells)
        self.assertTrue(all(b[0]-a[0] >= .6*self.m.grid.cell_m
                            for a, b in zip(points, points[1:])))
        self.assertTrue(all(math.dist(a,b) >= 2. for a,b in zip(looks,looks[1:])))
        self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks)
        self.assertNotIn(keys[1], self.m._segments.reserved)
        self.assertEqual(self.m.grid.cell(keys[1]).state, st.STATE_FREE)

    def test_recorded_82m_west_errand_cannot_fit_9_28s_remaining(self):
        from unittest.mock import patch
        # Replay the own start of B1 task3caa261...; no actor coordinates.
        self.m.status[self.uid].x, self.m.status[self.uid].y = 46.61066055, .46384525
        self.m.status[self.uid].z = 3.
        self.m._processed_camera_poses[self.uid]['z'] = 3.
        key = self.m.grid.world_to_cell(-43.01, 2.99)
        unbudgeted = self.m._directed_segment(self.uid, [key], self.now, math.pi)
        self.assertIsNotNone(unbudgeted)
        self.assertGreater(math.dist((46.61066055,.46384525), unbudgeted[1][0]), 80.)
        with patch('search_segment_manager.plan',side_effect=AssertionError('unaffordable A*')):
            self.assertIsNone(self.m._directed_segment(self.uid, [key], self.now,
                                                     math.pi, budget_s=9.28))
        self.assertEqual(self.m._segments.reserved, {})

    def test_deadline_shortens_to_affordable_prefix_and_keeps_remaining_cells_free(self):
        proposal = self.m._directed_segment(self.uid, self.east_cells(), self.now,
                                           0., budget_s=8.)
        self.assertIsNotNone(proposal)
        cells, points, looks = proposal
        self.assertEqual(len(cells), 1)
        task = self.m._start_search_segment(self.uid, cells, points, self.now,
            purpose='coverage_view', look_points=looks, max_hold=8.)
        self.assertEqual(task.deadline, self.now+8.)
        self.assertEqual(set(self.m._segments.reserved), {cells[0]})
        self.assertIsNone(self.m._directed_segment(self.uid, self.east_cells(), self.now,
                                                 0., budget_s=2.))

    def test_budget_uses_checked_detour_not_straight_goal_distance(self):
        from unittest.mock import patch
        # Direct travel10/1.4 + one frame budget would fit10s; the checked
        # 30m route cannot. This distinguishes ETA from an Euclidean proxy.
        point, look = (10.,0.), (18.,0.)
        path = ((0.,0.), (0.,10.), (10.,10.), point)
        with patch.object(self.m, '_reachable_subpoint_view', return_value=(point,look,path)):
            self.assertIsNone(self.m._directed_segment(self.uid, self.east_cells()[:1],
                                                      self.now, 0., budget_s=10.))

    def test_unknown_camera_yaw_cannot_invent_deadline_orientation(self):
        self.m._processed_camera_poses[self.uid].pop('yaw')
        self.assertIsNone(self.m._directed_segment(self.uid, self.east_cells(), self.now,
                                                  0., budget_s=90.))

    def test_new_road_height_window_requires_three_actual_stable_images(self):
        self.assertTrue(self.m._road_height_stable(self.uid,self.now))
        for window in ([],[(99.6,4.4),(100.,4.4)],
                       [(99.6,3.8),(99.8,4.1),(100.,4.3)],
                       [(99.6,4.4),(99.6,4.4),(100.,4.4)],
                       [(99.7,4.4),(99.8,4.4),(100.,4.4)],
                       [(99.,4.4),(99.7,4.4),(100.,4.4)],
                       [(98.,4.4),(98.2,4.4),(98.4,4.4)],
                       [(99.6,4.4),(99.8,math.nan),(100.,4.4)]):
            with self.subTest(window=window):
                self.m._processed_camera_height_window[self.uid]=window
                self.assertFalse(self.m._road_height_stable(self.uid,self.now))

    def test_changing_or_unknown_road_height_does_not_pin_idle_aircraft_or_start_budget(self):
        for window in (None,[(99.6,3.8),(99.8,4.1),(100.,4.3)]):
            with self.subTest(window=window):
                self.m._processed_camera_height_window[self.uid]=window
                idle=[self.uid]
                self.m._allocate_road_first_pass(idle,self.now)
                self.assertEqual(idle,[self.uid])
                self.assertEqual(self.m._segments.tasks,{})
                self.assertIsNone(self.m._road_first_pass.started_s)
                # The ordinary path builder itself retains its previous
                # measured-height contract; the gate is road-only.
                self.assertIsNotNone(self.m._directed_segment(self.uid,self.east_cells(),self.now,0.))

    def test_single_entry_rejects_outer_routine_without_clipping_ground_or_leaving_lease(self):
        key = self.m.grid.world_to_cell(129.,20.)
        point, look = (137.43391575825774,20.5), (129.99,20.5)
        self.assertIsNone(self.m._start_search_segment(self.uid,[key],[point],self.now,
            purpose='coverage_view',look_points=[look]))
        self.assertEqual(self.m._segments.tasks, {})
        self.assertEqual(self.m._segments.reserved, {})
        self.assertEqual(self.m.grid.cell(key).state, st.STATE_FREE)
        self.assertEqual(look,(129.99,20.5))
        # The disabled option preserves the previous broad-flight behavior.
        self.m._routine_search_bounds = False
        task = self.m._start_search_segment(self.uid,[key],[point],self.now,
            purpose='coverage_view',look_points=[look])
        self.assertEqual(task.points, (point,))
        self.assertEqual(task.look_points, (look,))

    def test_outside_first_candidate_does_not_crowd_inner_view_out_of_four_tries(self):
        key = self.m.grid.world_to_cell(127.,20.)
        look = (127.5,20.5)
        outer = (136.45,20.5)
        inner = (118.55,20.5)
        options = [(outer,look)]*5+[(inner,look)]
        self.m.status[self.uid].z = 4.5
        self.m._processed_camera_poses[self.uid]['z'] = 4.5
        view = self.m._reachable_subpoint_view(key,(125.,20.5),options,
                                              max_candidates=4)
        self.assertEqual(view,(inner,look))


if __name__ == '__main__':
    unittest.main()
