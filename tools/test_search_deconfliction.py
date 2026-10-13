"""Real allocator counterexamples: different cells can still duplicate work."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
import swarm_task as tasklib
from search_segments import SegmentBook
from search_segment_manager import SearchSegmentManager
from search_deconfliction import crowding_cost


class SearchDeconflictionTests(unittest.TestCase):
    def fixture(self, enabled=True):
        grid = tasklib.CoverageGrid(0., 70., 0., 35., cell_m=7., num_uavs=1)
        allocator = tasklib.TaskAllocator(grid)
        allocator.search_deconflict = enabled
        allocator.mission_start = 1.
        return grid, allocator

    def allocate(self, allocator, positions=None, **kwargs):
        with patch.object(tasklib, '_now', return_value=100.):
            return allocator.allocate(positions or {'typhoon_h480_1': (17.5, 17.5)},
                                      **kwargs)

    def test_nearby_search_lease_has_alternatives_not_only_an_unused_penalty(self):
        points = []
        for enabled in (False, True):
            grid, allocator = self.fixture(enabled)
            key = grid.world_to_cell(24.5, 17.5)
            grid.cell(key).state = tasklib.STATE_ASSIGNED
            grid.cell(key).owner = 'typhoon_h480_0'
            chosen = self.allocate(allocator)['typhoon_h480_1']
            cell = grid.cell(chosen)
            points.append((cell.cx, cell.cy))
            self.assertEqual(allocator.route_cursor, {})
            self.assertEqual(allocator.last_seen, {})
            self.assertEqual(allocator.visit_time, {})
            self.assertEqual(grid.cell(key).owner, 'typhoon_h480_0')
        self.assertEqual(math.dist(points[0], (24.5, 17.5)), 7.)
        self.assertGreaterEqual(math.dist(points[1], (24.5, 17.5)), 14.)
        self.assertLessEqual(math.dist(points[1], (17.5, 17.5)), 14.)

    def test_covered_patrol_cells_do_not_hide_actual_flight_and_look_commitments(self):
        grid, allocator = self.fixture()
        layouts = {'patrol': dict(points=[(17.5, 17.5)],
                                  look_points=[(24.5, 17.5)], position=(17.5, 17.5))}
        reserved = grid.world_to_cell(24.5, 17.5)
        grid.cell(reserved).state = tasklib.STATE_COVERED
        selected = self.allocate(allocator, active_search=layouts,
                                  blocked={reserved})['typhoon_h480_1']
        cell = grid.cell(selected)
        point = (cell.cx, cell.cy)
        self.assertEqual(crowding_cost(point, point, layouts, 'typhoon_h480_1'), 0.)
        self.assertEqual(grid.cell(reserved).state, tasklib.STATE_COVERED)

    def test_mapped_viewpoint_counts_even_when_its_ground_cell_is_elsewhere(self):
        grid, allocator = self.fixture()
        primary = grid.world_to_cell(17.5, 17.5)
        layouts = {'patrol': dict(points=[(60., 17.5)], look_points=[(60., 25.)])}
        selected = self.allocate(allocator, active_search=layouts,
            search_points={primary: (60., 17.5)})['typhoon_h480_1']
        self.assertNotEqual(selected, primary)
        self.assertEqual(grid.cell(primary).state, tasklib.STATE_FREE)

    def test_no_conflict_keeps_exact_baseline_route_and_no_false_coverage(self):
        selections = []
        for enabled in (False, True):
            grid, allocator = self.fixture(enabled)
            allocator.route_cursor[0] = 0
            layouts = {'remote': dict(points=[(130., -50.)], look_points=[(130., -60.)])}
            selections.append(self.allocate(allocator, active_search=layouts)['typhoon_h480_1'])
            self.assertEqual(allocator.route_cursor, {0: 0})
            self.assertEqual(allocator.last_seen, {})
        self.assertEqual(*selections)

    def test_disabled_switch_ignores_new_snapshot(self):
        _, allocator = self.fixture(False)
        chosen = self.allocate(allocator, active_search={
            'other': dict(points=[(17.5, 17.5)], look_points=[(17.5, 17.5)])})
        _, baseline = self.fixture(False)
        self.assertEqual(chosen, self.allocate(baseline))

    def test_fresh_lost_person_hint_preempts_search_separation(self):
        grid, allocator = self.fixture()
        hint = allocator.queue_revisit('lost', 17.5, 17.5, now=100.)
        selected = self.allocate(allocator, active_search={
            'other': dict(points=[(17.5, 17.5)], look_points=[(17.5, 17.5)])})
        self.assertEqual(selected['typhoon_h480_1'], hint)

    def test_batch_assignments_are_distinct_and_account_for_prior_choices(self):
        grid, allocator = self.fixture()
        selections = self.allocate(allocator, positions={
            'typhoon_h480_1': (17.5, 17.5), 'typhoon_h480_2': (17.5, 17.5)})
        self.assertEqual(len(set(selections.values())), 2)
        points = [(grid.cell(key).cx, grid.cell(key).cy) for key in selections.values()]
        self.assertGreaterEqual(math.dist(*points), 14.)

    def test_fully_congested_small_region_does_not_deadlock_or_create_coverage(self):
        grid, allocator = self.fixture()
        only = grid.world_to_cell(17.5, 17.5)
        for key, cell in grid.cells.items():
            if key != only:
                cell.state = tasklib.STATE_COVERED
        selected = self.allocate(allocator, active_search={
            'other': dict(points=[(17.5, 17.5)], look_points=[(17.5, 17.5)])})
        self.assertEqual(selected['typhoon_h480_1'], only)
        self.assertEqual(allocator.last_seen, {})

    def test_blocked_and_reserved_alternatives_remain_unassignable(self):
        grid, allocator = self.fixture()
        blocked = {key for key, cell in grid.cells.items() if cell.cx <= 10.5}
        selected = self.allocate(allocator, active_search={
            'other': dict(points=[(24.5, 17.5)], look_points=[(24.5, 17.5)])}, blocked=blocked)
        self.assertNotIn(selected['typhoon_h480_1'], blocked)

    def test_snapshot_excludes_completed_legs_and_retains_covered_reservations(self):
        manager = SearchSegmentManager()
        manager._segments = SegmentBook()
        manager.status = {'a': NS(x=7., y=0.)}
        task = manager._segments.reserve('a', [(0, 0), (1, 0)], [(0., 0.), (7., 0.)],
                                          100., 20., 90., look_points=[(0., 8.), (7., 8.)])
        manager._segments.advance('a', task.task_id, task.revision, 1)
        layout = manager._active_search_layout(101.)['a']
        self.assertEqual(layout['points'], ((7., 0.),))
        self.assertEqual(layout['look_points'], ((7., 8.),))
        manager._segments.release('a')
        self.assertEqual(manager._active_search_layout(101.), {})

    def test_expired_lease_and_stale_pose_cannot_hold_fresh_search_space(self):
        import rospy
        manager = SearchSegmentManager()
        manager._segments = SegmentBook()
        manager.status = {'a': NS(x=30., y=30.,
            header=NS(stamp=rospy.Time.from_sec(100.)))}
        manager._segments.reserve('a', [(0, 0)], [(0., 0.)], 100., 20., 90.,
                                  look_points=[(0., 8.)])
        layout = manager._active_search_layout(101.)['a']
        self.assertNotIn('position', layout)
        self.assertEqual(layout['points'], ((0., 0.),))
        self.assertEqual(manager._active_search_layout(120.), {})


if __name__ == '__main__':
    unittest.main()
