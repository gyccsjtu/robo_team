"""Overdue west looks may borrow a current ordinary transit, never a person task."""
import math
import threading
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as NS

import test_west_corner_review as fixture
import swarm_manager as sm
import swarm_task as st
from search_segments import ATTEND, DONE, FAILED, LOOK, TRAVEL, YIELD


class WestCornerBorrowTests(unittest.TestCase):
    uid = fixture.WestCornerReviewTests.uid
    refresh = fixture.WestCornerReviewTests.refresh
    corner_tasks = fixture.WestCornerReviewTests.corner_tasks

    def setUp(self):
        fixture.WestCornerReviewTests.setUp(self)
        self.m._processed_frame_times = {uid: self.now for uid in self.m.uav_ids}
        self.m._west_corner_allocation_started_s = self.now-60.
        self.m._west_corner_seen = {'southwest': self.now-61., 'northwest': self.now}
        self.old = {}
        for i, uid in enumerate(self.m.uav_ids):
            keys = [(4+i*3, 8), (5+i*3, 8), (6+i*3, 8)]
            task = self.m._start_search_segment(uid, keys,
                [(self.m.grid.cell(k).cx, self.m.grid.cell(k).cy) for k in keys], self.now)
            self.old[uid] = task
            status = self.m.status[uid]
            status.search_task_id, status.search_revision = task.task_id, task.revision
            status.search_index, status.search_state = 0, TRAVEL
            status.search_path_remaining = 30.
        self.m.assign_pub.publish.reset_mock()

    def allocate(self):
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', True):
            self.m._allocate()

    def test_no_idle_entry_borrows_once_and_reopens_every_uncovered_remaining_cell(self):
        self.assertEqual(self.m._idle_uavs(), [])
        self.allocate()
        task = self.corner_tasks()[0]
        previous = self.old[task.uid]
        self.assertNotEqual(task.task_id, previous.task_id)
        self.assertEqual(task.look_points, ((-49.99, -59.99),))
        self.assertEqual(task.purpose, 'building_review')
        for key in previous.cells:
            self.assertNotIn(key, self.m._segments.reserved)
            self.assertEqual(self.m.grid.cell(key).state, st.STATE_FREE)
            self.assertIsNone(self.m.grid.cell(key).owner)
        self.assertEqual(len(self.m._segments.tasks), 3)
        self.assertEqual(sum(t.purpose == 'coverage' for t in self.m._segments.tasks.values()), 2)
        self.assertEqual(self.m._west_corner_seen['southwest'], self.now-61.)
        self.assertEqual(self.m.grid.cell(task.cells[0]).covered_t, 0.)
        commands = [c.args[0] for c in self.m.assign_pub.publish.call_args_list
                    if c.args[0].uav_id == task.uid]
        self.assertEqual([(c.task_type, c.search_task_id) for c in commands],
                         [(0, previous.task_id), (255, previous.task_id), (0, task.task_id)])

    def test_unseen_corner_waits_sixty_seconds_from_first_healthy_allocation(self):
        self.m._west_corner_seen.pop('southwest')
        self.m.__dict__.pop('_west_corner_allocation_started_s')
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(self.corner_tasks(), [])
        self.assertEqual(self.m._west_corner_allocation_started_s, 100.)
        for stamp, expected in ((159.999, False), (160., True)):
            self.now = stamp
            self.refresh()
            self.m._processed_frame_times = {uid: stamp for uid in self.m.uav_ids}
            for task in self.old.values():
                task.expires = stamp+20.
            self.m._allocate_west_corner_review([], stamp)
            self.assertEqual(bool(self.corner_tasks()), expected)

    def test_current_corner_failure_cooldown_blind_delay_and_reservation_are_preserved(self):
        key = (0, 0)
        cases = ('fresh_image', 'failed', 'blind_delay', 'reserved', 'blocked')
        for case in cases:
            with self.subTest(case=case):
                changes = {}
                if case == 'fresh_image':
                    changes['seen'] = self.m._west_corner_seen.copy()
                    self.m._west_corner_seen['southwest'] = self.now-59.999
                elif case == 'failed': self.m.allocator.failed_visit[key] = self.now-59.
                elif case == 'blind_delay': self.m.allocator.revisit_jobs[('blind', key)] = (self.now+1., self.now+90.)
                elif case == 'reserved': self.m._segments.reserved[key] = 'peer-task'
                else: self.m._blocked_cells.add(key)
                self.m._allocate_west_corner_review([], self.now)
                self.assertEqual(self.corner_tasks(), [])
                self.assertTrue(all(self.m._segments.tasks[uid] is old for uid, old in self.old.items()))
                if 'seen' in changes: self.m._west_corner_seen = changes['seen']
                self.m.allocator.failed_visit.clear()
                self.m.allocator.revisit_jobs.clear()
                self.m._segments.reserved.pop(key, None)
                self.m._blocked_cells.clear()

    def test_look_attend_yield_done_failed_and_building_jobs_are_never_borrowed(self):
        for state in (LOOK, ATTEND, YIELD, DONE, FAILED):
            with self.subTest(state=state):
                for status in self.m.status.values(): status.search_state = state
                self.m._allocate_west_corner_review([], self.now)
                self.assertEqual(self.corner_tasks(), [])
        for status in self.m.status.values(): status.search_state = TRAVEL
        for task in self.old.values(): task.purpose = 'building_review'
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(self.corner_tasks(), [])

    def test_two_east_owners_and_person_owners_leave_no_borrowable_capacity(self):
        protected = self.m.uav_ids[:2]
        self.m._regional_patrols = tuple(NS(east=True, uid=uid) for uid in protected)
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(self.corner_tasks(), [])
        self.m._regional_patrols = ()
        self.m._tracking, self.m._backup = {'a': protected[0]}, {'a': protected[1]}
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(self.corner_tasks(), [])

    def test_fresh_person_evidence_and_stale_camera_pose_mode_or_epoch_are_rejected(self):
        task = self.old[self.m.uav_ids[0]]
        status = self.m.status[task.uid]
        for kind in ('person', 'frame_missing', 'frame_old', 'frame_future',
                     'frame_nan', 'pose_old', 'pose_future', 'mismatch',
                     'index_mismatch', 'disconnected', 'ground', 'nonfinite', 'cooldown'):
            with self.subTest(kind=kind):
                original = (status.x, status.z, status.connected, status.search_task_id,
                            status.search_index, status.header.stamp)
                self.m._processed_frame_times[task.uid] = self.now
                if kind == 'person': self.m._eligible_observations = {'a': dict(source_uav=task.uid, sample_s=self.now)}
                elif kind == 'frame_missing': self.m._processed_frame_times.pop(task.uid)
                elif kind == 'frame_old': self.m._processed_frame_times[task.uid] = self.now-2.001
                elif kind == 'frame_future': self.m._processed_frame_times[task.uid] = self.now+.01
                elif kind == 'frame_nan': self.m._processed_frame_times[task.uid] = math.nan
                elif kind == 'pose_old': status.header.stamp = sm.rospy.Time.from_sec(self.now-1.001)
                elif kind == 'pose_future': status.header.stamp = sm.rospy.Time.from_sec(self.now+.01)
                elif kind == 'mismatch': status.search_task_id = 'old-task'
                elif kind == 'index_mismatch': status.search_index = 1
                elif kind == 'disconnected': status.connected = False
                elif kind == 'ground': status.z = .1
                elif kind == 'nonfinite': status.x = math.nan
                else: self.m._release_cooldown[task.uid] = sm.rospy.Time.from_sec(self.now+1.)
                self.assertFalse(self.m._west_corner_borrowable(task, self.now, set()))
                (status.x, status.z, status.connected, status.search_task_id,
                 status.search_index, status.header.stamp) = original
                self.m._eligible_observations = {}
                self.m._release_cooldown.clear()

    def test_safe_route_must_exist_before_any_old_task_is_cancelled(self):
        with patch.object(self.m, '_reachable_subpoint_view', return_value=None):
            self.m._allocate_west_corner_review([], self.now)
        self.assertTrue(all(self.m._segments.tasks[uid] is old for uid, old in self.old.items()))
        self.m.assign_pub.publish.assert_not_called()
        self.assertNotIn((0, 0), self.m.allocator.failed_visit)

    def test_unexpected_reservation_failure_releases_only_one_old_route(self):
        idle = []
        with patch.object(self.m, '_start_search_segment', return_value=None):
            self.m._allocate_west_corner_review(idle, self.now)
        self.assertEqual(len(idle), 1)
        self.assertEqual(len(self.m._segments.tasks), 2)
        self.assertEqual(self.corner_tasks(), [])
        released = self.old[idle[0]]
        self.assertTrue(all(key not in self.m._segments.reserved for key in released.cells))
        self.assertTrue(all(self.m.grid.cell(key).state == st.STATE_FREE for key in released.cells))

    def test_two_healthy_searchers_keep_one_ordinary_and_expired_tasks_are_not_borrowed(self):
        removed = self.m.uav_ids[2]
        self.m._drop_segment(removed, 'test_cleanup')
        self.m.status[removed].connected = False
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(len(self.corner_tasks()), 1)
        self.assertEqual(sum(t.purpose == 'coverage' for t in self.m._segments.tasks.values()), 1)
        old = self.old[self.m.uav_ids[1]]
        old.expires = self.now
        self.assertFalse(self.m._west_corner_borrowable(old, self.now, set()))

    def test_actual_obstacle_and_new_attention_after_route_check_prevent_cancellation(self):
        cells = bytearray(self.raw.cells)
        ix, iy = self.raw.world_to_cell((-49.99, -59.99))
        cells[iy*self.raw.width+ix] = 1
        self.m._merged_grid.return_value = sm.GridMap(
            self.raw.width, self.raw.height, self.raw.resolution, self.raw.origin, cells)
        self.m._allocate_west_corner_review([], self.now)
        self.assertTrue(all(self.m._segments.tasks[uid] is old for uid, old in self.old.items()))
        self.m._merged_grid.return_value = self.raw
        self.m.allocator.failed_visit.clear()
        def new_attention(key, origin, options):
            for status in self.m.status.values(): status.search_state = ATTEND
            return options[0]
        with patch.object(self.m, '_reachable_subpoint_view', side_effect=new_attention):
            self.m._allocate_west_corner_review([], self.now)
        self.assertTrue(all(self.m._segments.tasks[uid] is old for uid, old in self.old.items()))

    def test_covered_prefix_survives_release_and_old_feedback_cannot_take_new_task(self):
        uid = self.m.uav_ids[0]
        previous = self.old[uid]
        visited = self.m._segments.advance(uid, previous.task_id, previous.revision, 1)
        self.m.grid.cell(visited[0]).state = st.STATE_COVERED
        self.m.grid.cell(visited[0]).covered_t = self.now-1.
        self.m.status[uid].search_index = 1
        self.m._allocate_west_corner_review([], self.now)
        task = self.corner_tasks()[0]
        self.assertEqual(task.uid, uid)
        self.assertEqual(self.m.grid.cell(visited[0]).state, st.STATE_COVERED)
        self.assertEqual(self.m.grid.cell(visited[0]).covered_t, self.now-1.)
        self.assertIsNone(self.m._segments.advance(uid, previous.task_id, previous.revision, 3))
        self.assertIs(self.m._segments.tasks[uid], task)

    def test_clock_reset_unseen_wait_is_restarted_and_simultaneous_allocations_only_borrow_once(self):
        self.m._west_corner_seen = {'northwest': self.now}
        self.m._west_corner_allocation_started_s = 1000.
        self.m._allocate_west_corner_review([], self.now)
        self.assertEqual(self.corner_tasks(), [])
        self.assertEqual(self.m._west_corner_allocation_started_s, self.now)
        self.m._west_corner_seen['southwest'] = self.now-61.
        barrier = threading.Barrier(2)
        def run():
            barrier.wait()
            self.allocate()
        threads = [threading.Thread(target=run) for _ in range(2)]
        for thread in threads: thread.start()
        for thread in threads: thread.join(2.)
        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(len(self.corner_tasks()), 1)
        self.assertEqual(sum(t.purpose == 'coverage' for t in self.m._segments.tasks.values()), 2)

    def test_recorded_2126_nearby_healthy_travel_can_borrow_on_a_checked_free_map(self):
        # Legitimate status/processed-frame samples from 060353 at ROS2126.8.
        # The test map is deliberately free; production still checks live A*.
        self.now = 2126.8
        self.m._west_corner_allocation_started_s = 1932.048
        self.m._west_corner_seen = {'southwest': 2066.784, 'northwest': 2011.184}
        self.m.allocator.failed_visit[(0, 17)] = 2109.048
        uid = self.m.uav_ids[0]
        status = self.m.status[uid]
        status.x, status.y, status.z = -35.29, -51.20, 3.44
        self.refresh()
        status.header.stamp = sm.rospy.Time.from_sec(self.now-.124)
        self.m._processed_frame_times = {u: self.now-.252 for u in self.m.uav_ids}
        for task in self.old.values():
            task.created, task.deadline, task.expires = 2124.548, 2214.548, 2144.548
        self.m._allocate_west_corner_review([], self.now)
        task = self.corner_tasks()[0]
        self.assertEqual(task.uid, uid)
        self.assertEqual(task.look_points, ((-49.99, -59.99),))
        self.assertEqual(self.m.allocator.failed_visit[(0, 17)], 2109.048)


if __name__ == '__main__':
    unittest.main()
