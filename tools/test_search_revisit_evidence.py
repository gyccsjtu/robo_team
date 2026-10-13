"""A missed search view must not consume a still-live person reacquisition cue."""
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))

import swarm_task as st
import test_search_segments as segment_fixtures


class PersonRevisitEvidenceTests(unittest.TestCase):
    def fixture(self, enabled=True):
        with patch.dict(os.environ, {'SEARCH_REVISIT_EVIDENCE': '1' if enabled else '0'}):
            grid = st.CoverageGrid(0, 35, 0, 7, cell_m=7, num_uavs=1)
            allocator = st.TaskAllocator(grid)
        for cell in grid.cells.values():
            cell.state = st.STATE_COVERED
        key = grid.world_to_cell(17.5, 3.5)
        return grid, allocator, key

    def request(self, allocator, key, kind='lost', now=100., ttl=25.):
        cell = allocator.grid.cell(key)
        with patch.object(st, '_now', return_value=now):
            self.assertEqual(allocator.queue_revisit(kind, cell.cx, cell.cy,
                             now=now, ttl_s=ttl), key)
        return (kind, key)

    def test_disabled_flag_retains_original_waypoint_completion(self):
        grid, allocator, key = self.fixture(enabled=False)
        job = self.request(allocator, key)
        allocator.complete_waypoint('uav_0', key, observed=False, now=101.)
        self.assertNotIn(job, allocator.revisit_jobs)
        self.assertEqual(allocator.revisit_last_served[job], 101.)

    def test_missed_view_preserves_both_person_cues_without_image_credit(self):
        grid, allocator, key = self.fixture()
        lost = self.request(allocator, key)
        candidate = self.request(allocator, key, kind='unconfirmed', ttl=8.)
        allocator.complete_waypoint('uav_0', key, observed=False, now=101.)
        self.assertEqual(allocator.revisit_jobs[lost], (103., 125.))
        self.assertEqual(allocator.revisit_jobs[candidate], (103., 108.))
        self.assertEqual(allocator.visit_time[key], 101.)
        self.assertIn(key, allocator._search_route_first)
        self.assertNotIn(key, allocator.last_seen)
        self.assertNotIn(key, allocator._search_image_first)
        self.assertNotIn(lost, allocator.revisit_last_served)
        self.assertNotIn(candidate, allocator.revisit_last_served)

    def test_duplicate_completion_during_backoff_does_not_postpone_retry(self):
        grid, allocator, key = self.fixture()
        job = self.request(allocator, key)
        allocator.complete_waypoint('uav_0', key, now=101.)
        for now in (101., 101.2, 102.9):
            allocator.complete_waypoint('uav_0', key, observed=False, now=now)
        self.assertEqual(allocator.revisit_jobs[job], (103., 125.))

    def test_repeat_request_cannot_erase_backoff_or_extend_episode(self):
        grid, allocator, key = self.fixture()
        job = self.request(allocator, key)
        allocator.complete_waypoint('uav_0', key, now=101.)
        self.request(allocator, key, now=102., ttl=200.)
        self.assertEqual(allocator.revisit_jobs[job], (103., 125.))
        self.assertEqual(allocator._active_revisit(102., {key}), [])

    def test_backoff_blocks_ordinary_route_reassignment_as_well_as_priority(self):
        grid, allocator, key = self.fixture()
        self.request(allocator, key)
        allocator.complete_waypoint('uav_0', key, now=101.)
        with patch.object(st, '_now', return_value=102.):
            selected = allocator.allocate({'uav_0': (17.5, 3.5)})
        self.assertIsNone(selected['uav_0'])
        with patch.object(st, '_now', return_value=103.):
            selected = allocator.allocate({'uav_0': (17.5, 3.5)})
        self.assertEqual(selected['uav_0'], key)

    def test_retry_deadline_is_never_extended_and_expiry_is_not_success(self):
        grid, allocator, key = self.fixture()
        job = self.request(allocator, key, kind='unconfirmed', ttl=8.)
        for now in (101., 103., 105., 107.):
            allocator.complete_waypoint('uav_0', key, now=now)
            self.assertEqual(allocator.revisit_jobs[job][1], 108.)
            self.assertLessEqual(allocator.revisit_jobs[job][0], 108.)
        self.assertEqual(allocator._active_revisit(108., {key}), [])
        self.assertNotIn(job, allocator.revisit_jobs)
        self.assertNotIn(job, allocator.revisit_last_served)

    def test_already_expired_missed_view_is_removed_without_served_cooldown(self):
        grid, allocator, key = self.fixture()
        job = self.request(allocator, key, ttl=3.)
        allocator.complete_waypoint('uav_0', key, observed=False, now=103.)
        self.assertNotIn(job, allocator.revisit_jobs)
        self.assertNotIn(job, allocator.revisit_last_served)
        self.request(allocator, key, now=104., ttl=5.)
        self.assertEqual(allocator.revisit_jobs[job], (104., 109.))

    def test_successful_imaged_visit_consumes_cues_and_applies_existing_cooldown(self):
        grid, allocator, key = self.fixture()
        jobs = [self.request(allocator, key),
                self.request(allocator, key, kind='unconfirmed', ttl=8.)]
        allocator.complete_waypoint('uav_0', key, observed=True, now=102.)
        self.assertEqual(allocator.last_seen[key], 102.)
        self.assertEqual(allocator._search_image_first[key], 102.)
        for job in jobs:
            self.assertNotIn(job, allocator.revisit_jobs)
            self.assertEqual(allocator.revisit_last_served[job], 102.)
        self.assertIsNone(allocator.queue_revisit('lost', 17.5, 3.5, now=103.))

    def test_background_ground_coverage_does_not_complete_explicit_person_revisit(self):
        grid, allocator, key = self.fixture()
        job = self.request(allocator, key)
        allocator.observed_cell(key, now=101.)
        self.assertIn(job, allocator.revisit_jobs)
        self.assertNotIn(job, allocator.revisit_last_served)

    def test_completion_leaves_other_cells_and_blind_behavior_unchanged(self):
        grid, allocator, key = self.fixture()
        lost = self.request(allocator, key)
        other = grid.world_to_cell(3.5, 3.5)
        second = self.request(allocator, other, kind='unconfirmed', ttl=8.)
        blind = self.request(allocator, key, kind='blind', ttl=90.)
        allocator.complete_waypoint('uav_0', key, observed=False, now=101.)
        self.assertEqual(allocator.revisit_jobs[lost], (103., 125.))
        self.assertEqual(allocator.revisit_jobs[second], (100., 108.))
        self.assertNotIn(blind, allocator.revisit_jobs)
        self.assertEqual(allocator.revisit_last_served[blind], 101.)

    def test_blocked_or_owned_retry_cannot_bypass_existing_safety_or_mutex(self):
        for scenario in ('blocked', 'owned', 'failed'):
            with self.subTest(scenario=scenario):
                grid, allocator, key = self.fixture()
                self.request(allocator, key)
                allocator.complete_waypoint('uav_0', key, now=101.)
                if scenario == 'owned':
                    grid.cell(key).state = st.STATE_ASSIGNED
                    grid.cell(key).owner = 'uav_1'
                if scenario == 'failed':
                    allocator.failed_visit[key] = 102.
                with patch.object(st, '_now', return_value=103.):
                    selected = allocator.allocate({'uav_0': (17.5, 3.5)},
                        blocked={key} if scenario == 'blocked' else ())
                self.assertNotEqual(selected['uav_0'], key)


class SegmentRevisitEvidenceTests(unittest.TestCase):
    uid = segment_fixtures.ManagerSegmentTests.uid
    setUp = segment_fixtures.ManagerSegmentTests.setUp
    reserve = segment_fixtures.ManagerSegmentTests.reserve
    feedback = segment_fixtures.ManagerSegmentTests.feedback

    def test_legacy_single_waypoint_completion_uses_the_same_person_evidence_rule(self):
        manager = self.m
        manager._segments = None
        manager.allocator.search_revisit_evidence = True
        key = (0, 0)
        cell = manager.grid.cell(key)
        manager.allocator.queue_revisit('lost', cell.cx, cell.cy,
                                       now=98., ttl_s=25.)
        manager.lease.grant(self.uid, key, 99., 20.)
        manager._active_leases[self.uid] = key
        manager._lease_hold_since[(self.uid, key)] = 99.
        status = manager.status[self.uid]
        status.x, status.y = cell.cx, cell.cy
        status.assigned_cell_ix, status.assigned_cell_iy = key
        status.confidence = 1.
        self.now = 101.
        manager.last_report[self.uid] = segment_fixtures.sm.rospy.Time.from_sec(self.now)
        side = (cell.cx, cell.cy+8.5)
        manager._choose_blind_viewpoint = Mock(return_value=side)
        manager._renew_leases()
        self.assertEqual(manager.allocator.revisit_jobs[('lost', key)], (103., 123.))
        self.assertEqual(manager._blind_viewpoints[key], side)
        self.assertNotIn(self.uid, manager._active_leases)
        self.assertEqual(manager.grid.cell(key).state, st.STATE_REVIEW)
        self.assertNotIn(key, manager.allocator.last_seen)

    def test_segment_advance_preserves_person_deadline_and_prepares_existing_side_view(self):
        manager = self.m
        manager.allocator.search_revisit_evidence = True
        key = (0, 0)
        cell = manager.grid.cell(key)
        manager.allocator.queue_revisit('lost', cell.cx, cell.cy,
                                       now=98., ttl_s=25.)
        side = (cell.cx, cell.cy+8.5)
        manager._choose_blind_viewpoint = Mock(return_value=side)
        task = self.reserve()
        self.now = 101.
        self.feedback(task, index=1)
        manager._renew_segments(segment_fixtures.sm.rospy.Time.from_sec(self.now))
        self.assertEqual(manager.allocator.revisit_jobs[('lost', key)], (103., 123.))
        self.assertEqual(manager._blind_viewpoints[key], side)
        self.assertEqual(manager.grid.cell(key).state, st.STATE_REVIEW)
        self.assertEqual(manager.allocator.revisit_jobs[('blind', key)], (109., 191.))
        self.assertEqual(task.index, 1)
        self.assertIn(task.uid, manager._segments.tasks)
        self.assertNotIn(key, manager._segments.reserved)
        self.assertNotIn(key, manager.allocator.last_seen)


if __name__ == '__main__':
    unittest.main()
