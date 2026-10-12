"""Regional owners stay exclusive between legs, including the 2193s replay."""
import threading
import unittest

import test_east_split_patrol as fixture
import swarm_manager as sm
import swarm_task as st
from search_segments import DONE


class PeerPatrolOwnershipTests(unittest.TestCase):
    use_default_flight_bounds = True
    refresh = fixture.EastSplitPatrolTests.refresh

    def setUp(self):
        fixture.DefaultEastSplitPatrolTests.setUp(self)
        self.north, self.south = self.m._regional_patrols
        self.owner, self.peer = self.m.uav_ids[:2]
        self.corners = {(25, 17), (25, 0)}

    def idle_only(self, ids):
        self.m._tracking = {'busy_'+uid: uid for uid in self.m.uav_ids
                            if uid not in ids}

    def keep_only_corners(self):
        self.m._blocked_cells = set(self.m.grid.cells)-self.corners

    def completed_lane(self, lane):
        # Actual south case: completed72cf... at2193.352; before the next
        # tick the lane keeps uid0, while its old reservation is released.
        self.refresh(2175.260)
        cells = [(20, 1), (20, 2)] if lane.east_half == 'south' else [(20, 15), (20, 16)]
        looks = [(93.5, self.m.grid.cell(key).cy) for key in cells]
        points = [(84.55, look[1]) for look in looks]
        task = self.m._start_search_segment(self.owner, cells, points, self.now,
            purpose='south_patrol', publish=False, look_points=looks)
        lane.started(task, -1 if lane.east_half == 'south' else 1, self.now)
        self.refresh(2193.352)
        status = self.m.status[self.owner]
        status.x, status.y = points[-1]
        status.search_task_id = task.task_id
        status.search_revision = task.revision
        status.search_index = len(task.cells)
        status.search_state = DONE
        status.search_path_remaining = 0.
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertNotIn(self.owner, self.m._segments.tasks)
        self.assertEqual(lane.uid, self.owner)
        self.assertIsNone(lane.task_id)
        self.assertTrue(set(cells).isdisjoint(self.m._segments.reserved))
        return task

    def assert_mutual_owners_and_cells(self):
        owners = [p.uid for p in self.m._regional_patrols if p.uid is not None]
        self.assertEqual(len(owners), len(set(owners)))
        keys = [key for task in self.m._segments.tasks.values() for key in task.cells]
        self.assertEqual(len(keys), len(set(keys)))
        for patrol in self.m._regional_patrols:
            task = self.m._segments.tasks.get(patrol.uid)
            if task is not None:
                self.assertEqual(patrol.task_id, task.task_id)
                self.assertLessEqual(len(task.cells), 3)
                for point, look in zip(task.points, task.look_points):
                    self.assertTrue(self.m._search_point_free(point))
                    self.assertEqual(look[1] >= 0., patrol.east_half == 'north')

    def test_2193_south_completion_north_cannot_borrow_only_idle_owner(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner})
        self.m._allocate()
        self.assertIsNone(self.north.uid)
        self.assertEqual(self.south.uid, self.owner)
        task = self.m._segments.tasks[self.owner]
        self.assertEqual(task.cells[0], (25, 0))
        self.assertEqual(task.look_points[0], (129.99, -59.99))
        self.assertNotEqual(self.m.grid.cell((25, 0)).state, st.STATE_COVERED)
        self.assert_mutual_owners_and_cells()

    def test_north_uses_peer_even_when_south_owner_is_nearer(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        self.m.status[self.owner].x, self.m.status[self.owner].y = 121., 10.
        self.m.status[self.peer].x, self.m.status[self.peer].y = 3., 0.
        self.m._allocate()
        self.assertEqual(self.north.uid, self.peer)
        self.assertEqual(self.south.uid, self.owner)
        self.assertEqual(len(self.m._segments.tasks), 2)
        self.assert_mutual_owners_and_cells()

    def test_symmetric_south_first_cannot_borrow_completed_north_owner(self):
        self.completed_lane(self.north)
        self.m._regional_patrols = (self.south, self.north)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        self.m.status[self.owner].x, self.m.status[self.owner].y = 121., -10.
        self.m.status[self.peer].x, self.m.status[self.peer].y = 3., 0.
        self.m._allocate()
        self.assertEqual(self.south.uid, self.peer)
        self.assertEqual(self.north.uid, self.owner)
        self.assert_mutual_owners_and_cells()

    def test_person_preemption_releases_one_lane_and_preserves_real_target(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        replacement = self.m.uav_ids[2]
        self.idle_only({self.owner, self.peer, replacement})
        self.m._tracking['real_person'] = self.owner
        self.m._allocate()
        self.assertEqual(self.m._tracking['real_person'], self.owner)
        self.assertNotIn(self.owner, self.m._segments.tasks)
        self.assertNotEqual(self.north.uid, self.owner)
        self.assertNotEqual(self.south.uid, self.owner)
        self.assertEqual(len(self.m._segments.tasks), 2)
        self.assert_mutual_owners_and_cells()

    def test_stale_owner_is_cleaned_then_can_return_when_healthy(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        self.m.last_report[self.owner] = sm.rospy.Time.from_sec(self.now-5.)
        self.m._allocate()
        self.assertIsNone(self.south.uid)
        self.assertEqual(self.north.uid, self.peer)
        self.refresh(self.now+2.1)  # original empty-proposal retry is 2 s
        self.m._allocate()
        self.assertEqual(self.south.uid, self.owner)
        self.assert_mutual_owners_and_cells()

    def test_camera_unavailable_owner_is_not_pinned_by_peer_exclusion(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        sample = sm.rospy.Time.now().to_sec()-.01
        self.m._processed_frame_times = {uid: sample for uid in self.m.uav_ids}
        self.m._processed_frame_times[self.owner] = self.now-5.
        self.m._allocate()
        self.assertIsNone(self.south.uid)
        self.assertEqual(self.north.uid, self.peer)
        self.refresh(self.now+2.1)
        sample = sm.rospy.Time.now().to_sec()-.01
        self.m._processed_frame_times = {uid: sample for uid in self.m.uav_ids}
        self.m._allocate()
        self.assertEqual(self.south.uid, self.owner)
        self.assert_mutual_owners_and_cells()

    def test_blocked_lane_releases_owner_after_existing_ten_second_retry_budget(self):
        self.completed_lane(self.south)
        self.idle_only({self.owner, self.peer})
        self.m._blocked_cells = set(self.m.grid.cells)-{(25, 17)}
        self.m._allocate()
        self.assertEqual(self.north.uid, self.peer)
        self.assertEqual(self.south.uid, self.owner)
        self.assertNotIn(self.owner, self.m._segments.tasks)
        self.refresh(self.now+10.1)
        self.m._allocate()
        self.assertIsNone(self.south.uid)
        self.assertGreater(self.south.owner_retry[self.owner], self.now)
        self.m._tracking['real_person'] = self.peer
        self.m._free_search_lease(self.peer)
        self.refresh(self.now+.6)
        self.m._allocate()
        self.assertEqual(self.north.uid, self.owner)
        self.assertNotIn(self.peer, self.m._segments.tasks)
        self.assert_mutual_owners_and_cells()

    def test_real_release_cooldown_keeps_regional_owner_without_peer_capture(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        self.m._release_cooldown[self.owner] = sm.rospy.Time.from_sec(self.now+2.)
        self.m._allocate()
        self.assertEqual(self.north.uid, self.peer)
        self.assertEqual(self.south.uid, self.owner)
        self.assertNotIn(self.owner, self.m._segments.tasks)
        self.refresh(self.now+2.1)
        self.m._allocate()
        self.assertEqual(self.south.uid, self.owner)
        self.assertEqual(self.m._segments.tasks[self.owner].cells[0], (25, 0))
        self.assert_mutual_owners_and_cells()

    def test_concurrent_ticks_cannot_create_duplicate_owners_or_cells(self):
        self.completed_lane(self.south)
        self.keep_only_corners()
        self.idle_only({self.owner, self.peer})
        errors = []
        barrier = threading.Barrier(2)
        def tick():
            try:
                barrier.wait()
                self.m._allocate()
            except Exception as error:
                errors.append(error)
        workers = [threading.Thread(target=tick) for _ in range(2)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(5.)
        self.assertFalse(errors)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(len(self.m._segments.tasks), 2)
        self.assert_mutual_owners_and_cells()


if __name__ == '__main__':
    unittest.main()
