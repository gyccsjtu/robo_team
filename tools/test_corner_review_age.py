"""Corner first visits expire for review without fabricating image coverage."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import test_east_split_patrol as fixture
import boundary_patrol as bp
import swarm_task as st


class CornerReviewAgeTests(unittest.TestCase):
    use_default_flight_bounds = True
    refresh = fixture.EastSplitPatrolTests.refresh

    def setUp(self):
        fixture.DefaultEastSplitPatrolTests.setUp(self)
        self.p = self.m._regional_patrols[0]
        self.uid = self.m.uav_ids[0]
        self.corner, self.edge, self.inner = (25, 17), (20, 17), (20, 10)
        self.m.status[self.uid].x, self.m.status[self.uid].y = 84., 40.
        self.m._blocked_cells = set(self.m.grid.cells)-{
            self.corner, self.edge, self.inner}

    def image_memory(self, key, sample):
        self.p._corner_observed.add(key)
        self.p._corner_observed_at[key] = sample
        self.p._edge_observed.add(key)
        self.m.allocator.visit_time[key] = sample

    def start_corner(self):
        self.m._blocked_cells = set(self.m.grid.cells)-{self.corner}
        proposal = self.p.choose(self.m, [self.uid], self.now)
        task = self.m._start_search_segment(self.uid, proposal[1], proposal[2], self.now,
            purpose='south_patrol', publish=False,
            look_points=self.p.proposal_look_points)
        self.p.started(task, proposal[3], self.now)
        return task

    def test_sample_time_not_route_end_defines_sixty_second_review(self):
        task = self.start_corner()
        self.m._camera_point_seen = {(self.corner, 8): 100.4}
        self.refresh(109.)
        self.m._drop_segment(self.uid, 'complete')
        self.assertEqual(self.p._corner_observed_at[self.corner], 100.4)
        self.m._camera_point_seen.clear()
        self.assertFalse(self.p._corner_review_due(self.m, self.corner, 160.399))
        self.assertTrue(self.p._corner_review_due(self.m, self.corner, 160.4))
        self.assertFalse(self.p._corner_needs_first_view(self.m, self.corner))
        self.assertNotEqual(self.m.grid.cell(self.corner).state, st.STATE_COVERED)
        self.assertNotIn(self.corner, self.m.allocator.failed_visit)

    def test_recorded_se_gap_is_review_due_without_waiting_for_all_new_cells(self):
        self.p = self.m._regional_patrols[1]
        self.corner = (25, 0)
        self.inner = (20, 7)
        self.m._blocked_cells = set(self.m.grid.cells)-{self.corner, self.inner}
        self.image_memory(self.corner, 2555.060)
        self.m.allocator.visit_time[self.corner] = 2775.  # no new image
        self.refresh(2780.272)
        self.m.status[self.uid].x, self.m.status[self.uid].y = 84., -8.
        self.assertTrue(self.p._corner_review_due(self.m, self.corner, self.now))
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.corner)
        self.assertEqual(self.p.proposal_look_points[0], (129.99, -59.99))

    def test_unseen_corner_precedes_old_strip_and_new_inner_cell(self):
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.corner)

    def test_due_corner_precedes_unseen_outer_strip(self):
        self.image_memory(self.corner, 10.)
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1], [self.corner])
        self.assertEqual(self.p.proposal_look_points, [(129.99, 59.99)])
        self.assertNotEqual(self.m.grid.cell(self.corner).state, st.STATE_COVERED)

    def test_current_corner_does_not_displace_unseen_outer_strip(self):
        self.image_memory(self.corner, 80.)
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.edge)

    def test_recorded_south_corner_review_cannot_wait_for_every_new_edge_row(self):
        # 034453: the healthy south owner kept receiving first-pass edge
        # rows after this real corner-image time. A route visit is no renewal.
        self.p = self.m._regional_patrols[1]
        self.corner, self.edge = (25, 0), (20, 0)
        self.m._blocked_cells = set(self.m.grid.cells)-{self.corner, self.edge}
        self.image_memory(self.corner, 2568.636)
        self.refresh(2651.336)
        self.m.status[self.uid].x, self.m.status[self.uid].y = 88., -49.
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1], [self.corner])
        self.assertEqual(self.p.proposal_look_points, [(129.99, -59.99)])
        self.assertEqual(self.p._corner_observed_at[self.corner], 2568.636)

    def test_corner_review_is_one_point_even_when_adjacent_strip_is_reachable(self):
        adjacent = (25, 16)
        self.m._blocked_cells = set(self.m.grid.cells)-{self.corner, adjacent}
        self.image_memory(self.corner, 10.)
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1], [self.corner])
        self.assertEqual(len(proposal[2]), 1)

    def test_due_corner_precedes_nearer_new_inner_cell(self):
        self.m._blocked_cells.add(self.edge)
        self.image_memory(self.corner, 10.)
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.corner)
        self.assertLessEqual(len(proposal[1]), 3)
        self.assertEqual(self.p.proposal_look_points[0], (129.99, 59.99))
        self.assertTrue(self.m._search_point_free(proposal[2][0]))
        self.assertNotEqual(self.m.grid.cell(self.corner).state, st.STATE_COVERED)

    def test_current_corner_does_not_displace_new_search(self):
        self.m._blocked_cells.add(self.edge)
        self.image_memory(self.corner, 80.)
        proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.inner)

    def test_complete_without_images_does_not_refresh_memory_and_cools_retry(self):
        self.image_memory(self.corner, 10.)
        task = self.start_corner()
        self.refresh(110.)
        self.m._drop_segment(self.uid, 'complete')
        self.assertEqual(self.p._corner_observed_at[self.corner], 10.)
        self.assertEqual(self.m.allocator.failed_visit[self.corner], 110.)
        self.assertIsNone(self.p.choose(self.m, [self.uid], self.now))
        self.refresh(110.+self.m.allocator.failed_novelty_ttl+.1)
        self.assertEqual(self.p.choose(self.m, [self.uid], self.now)[1][0], self.corner)

    def test_stale_corner_astar_failure_uses_existing_finite_cooldown(self):
        self.m._blocked_cells.add(self.edge)
        self.image_memory(self.corner, 10.)
        original = bp.plan
        def no_corner(grid, start, goal, **kwargs):
            if goal[0] > 120.:
                return NS(success=False)
            return original(grid, start, goal, **kwargs)
        with patch.object(bp, 'plan', side_effect=no_corner):
            proposal = self.p.choose(self.m, [self.uid], self.now)
        self.assertEqual(proposal[1][0], self.inner)
        self.assertEqual(self.m.allocator.failed_visit[self.corner], self.now)
        self.refresh(self.now+self.m.allocator.failed_novelty_ttl+.1)
        self.assertEqual(self.p.choose(self.m, [self.uid], self.now)[1][0], self.corner)

    def test_task_before_and_future_and_other_subpoint_images_cannot_refresh(self):
        for point, sample in ((8, 99.9), (8, 110.1), (4, 100.4),
                              (8, math.nan), (8, math.inf)):
            with self.subTest(point=point, sample=sample):
                self.p._manager = self.m
                self.m._camera_point_seen = {(self.corner, point): sample}
                task = NS(cells=(self.corner,), created=100.)
                self.assertEqual(self.p._fresh_corner_times(task, 110.), {})

    def test_revised_task_and_full_coverage_keep_actual_latest_image_time(self):
        task = self.start_corner()
        self.m._revise_segment(task, list(task.cells), list(task.points), 100.1,
                               'test_prefix_retains_creation_and_look')
        self.m._camera_point_seen = {(self.corner, 8): 100.4}
        cell = self.m.grid.cell(self.corner)
        cell.state, cell.covered_t = st.STATE_COVERED, 100.8
        self.refresh(109.)
        self.m._drop_segment(self.uid, 'complete')
        self.assertEqual(self.p._corner_observed_at[self.corner], 100.8)
        self.assertFalse(self.p._corner_review_due(self.m, self.corner, 160.7))
        self.assertTrue(self.p._corner_review_due(self.m, self.corner, 160.8))

    def test_independent_real_full_coverage_refreshes_review_until_reopened(self):
        self.image_memory(self.corner, 10.)
        cell = self.m.grid.cell(self.corner)
        cell.state, cell.covered_t = st.STATE_COVERED, 80.
        self.assertFalse(self.p._corner_review_due(self.m, self.corner, 100.))
        self.assertTrue(self.p._corner_review_due(self.m, self.corner, 140.))
        self.assertNotIn(self.corner, self.m.allocator.failed_visit)

    def test_old_task_completion_cannot_change_current_memory_or_task(self):
        task = self.start_corner()
        self.m._camera_point_seen = {(self.corner, 8): 100.4}
        old = NS(task_id='obsolete', cells=(self.corner,), created=90., index=0)
        self.p.ended(old, 'complete', 101.)
        self.assertEqual(self.p.task_id, task.task_id)
        self.assertNotIn(self.corner, self.p._corner_observed_at)

    def test_person_task_preempts_due_corner_and_releases_without_fake_image(self):
        self.image_memory(self.corner, 10.)
        task = self.start_corner()
        self.m._tracking['real_person'] = self.uid
        self.m._free_search_lease(self.uid)
        self.assertIsNone(self.p.uid)
        self.assertIsNone(self.p.task_id)
        self.assertNotIn(self.corner, self.m._segments.reserved)
        self.assertEqual(self.p._corner_observed_at[self.corner], 10.)
        self.assertNotIn(self.corner, self.m.allocator.failed_visit)
        self.assertFalse(self.p._corner_needs_first_view(self.m, self.corner))
        self.assertNotEqual(self.m.grid.cell(self.corner).state, st.STATE_COVERED)

    def test_single_east_and_non_corner_cells_do_not_gain_review_priority(self):
        plain = bp.SouthPatrol(self.p.bounds, southeast=False,
                               northeast=False, east=True)
        self.assertFalse(plain._corner_review_due(self.m, self.corner, self.now))
        self.assertFalse(self.p._corner_review_due(self.m, self.inner, self.now))

    def test_clock_reset_does_not_preserve_permanent_current_view(self):
        self.image_memory(self.corner, 1000.)
        self.assertTrue(self.p._corner_review_due(self.m, self.corner, self.now))
        self.assertFalse(self.p._corner_needs_first_view(self.m, self.corner))


if __name__ == '__main__':
    unittest.main()
