"""West public corners use normal tasks and only actual image dwell as credit."""
import json
import math
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import test_search_segments as fixture
import swarm_manager as sm
import swarm_task as st
from building_viewpoints import valid_view
from robocup_swarm.msg import UavStatus
from search_segments import ATTEND, DONE, TRAVEL


class WestCornerReviewTests(unittest.TestCase):
    uid = fixture.ManagerSegmentTests.uid

    def setUp(self):
        fixture.ManagerSegmentTests.setUp(self)
        m = self.m
        m.grid = st.CoverageGrid(*st.ACTOR_SEARCH_BOUNDS, 7., 6)
        m.allocator, m.lease = st.TaskAllocator(m.grid), st.LeaseManager(m.grid)
        m.uav_ids = ['typhoon_h480_%d' % i for i in range(3)]
        m.status = {uid: UavStatus(uav_id=uid, x=x, y=y, z=4.2, connected=True)
                    for uid, (x, y) in zip(m.uav_ids, ((-40., -50.), (-40., 50.), (-10., 0.)))}
        m._cell_waypoint = {key: (c.cx, c.cy) for key, c in m.grid.cells.items()}
        self.raw = sm.GridMap(210, 130, 1., (-55., -65.), bytes(210*130))
        m._merged_grid = Mock(return_value=self.raw)
        m._g_infl = sm.inflate_grid(self.raw, sm.INFLATE_M)
        m._slam_lock = object()
        m._camera_point_seen, m._west_corner_seen = {}, {}
        self.refresh()

    def refresh(self):
        for uid, status in self.m.status.items():
            status.header.stamp = sm.rospy.Time.from_sec(self.now)
            self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)

    def allocate(self, enabled=True):
        self.refresh()
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', enabled):
            self.m._allocate()

    def corner_tasks(self):
        corners = {look for _, _, _, look in self.m._west_corner_subpoints()}
        return [task for task in self.m._segments.tasks.values()
                if task.purpose == 'building_review' and
                any(look in corners for look in task.look_points)]

    def frame(self, task, sample=None, **changes):
        sample = self.now if sample is None else sample
        point, look = task.points[0], task.look_points[0]
        status = self.m.status[task.uid]
        status.x, status.y, status.z = *point, 4.2
        data = dict(processed=True, source_uav=task.uid, sample_s=sample,
                    x=point[0], y=point[1], z=4.45, down_pitch=0.,
                    yaw=math.atan2(look[1]-point[1], look[0]-point[0]))
        data.update(changes)
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', True):
            self.m._processed_camera_frame_cb(NS(data=json.dumps(data)))

    def ordinary_task(self, uid, key=(10, 8)):
        c = self.m.grid.cell(key)
        return self.m._start_search_segment(uid, [key], [(c.cx, c.cy)], self.now)

    def test_first_exact_actor_corner_has_checked_path_and_no_assignment_credit(self):
        self.allocate()
        tasks = self.corner_tasks()
        self.assertEqual(len(tasks), 1)
        task = tasks[0]
        self.assertIn(task.look_points[0], ((-49.99, -59.99), (-49.99, 59.99)))
        self.assertAlmostEqual(math.dist(task.points[0], task.look_points[0]), 8.95)
        self.assertTrue(valid_view(self.raw, self.m._g_infl, task.look_points[0],
            task.points[0], self.m._segment_flight_waypoint))
        route = sm.plan(self.m._g_infl,
            (self.m.status[task.uid].x, self.m.status[task.uid].y), task.points[0],
            connectivity=8, prevent_corner_cutting=True, max_expansions=2500)
        self.assertTrue(route.success)
        self.assertEqual(task.deadline, self.now+self.m._segment_max_hold)
        self.assertEqual(self.m._west_corner_seen, {})
        self.assertEqual(self.m.grid.cell(task.cells[0]).covered_t, 0.)
        self.assertTrue(any(t.purpose == 'coverage' for t in self.m._segments.tasks.values()))

    def test_first_unseen_corner_precedes_expired_other_corner(self):
        self.m._west_corner_seen['southwest'] = self.now-61.
        self.allocate()
        self.assertEqual(self.corner_tasks()[0].look_points, ((-49.99, 59.99),))
        self.assertEqual(self.m._west_corner_seen, {'southwest': 39.})

    def test_only_actual_corner_image_at_sixty_seconds_triggers_revisit(self):
        self.m._west_corner_seen = {'southwest': 100., 'northwest': 110.}
        self.now = 159.9
        self.refresh()
        idle = list(self.m.uav_ids)
        self.m._allocate_west_corner_review(idle, self.now)
        self.assertEqual(self.corner_tasks(), [])
        self.now = 160.
        self.refresh()
        self.m._allocate_west_corner_review(idle, self.now)
        self.assertEqual(self.corner_tasks()[0].look_points, ((-49.99, -59.99),))
        self.assertEqual(self.m._west_corner_seen, {'southwest': 100., 'northwest': 110.})

    def test_one_idle_with_other_running_normal_search_can_receive_second_corner(self):
        for uid, key in zip(self.m.uav_ids[1:], ((10, 8), (12, 8))):
            self.ordinary_task(uid, key)
        self.m._west_corner_seen['northwest'] = self.now
        self.allocate()
        self.assertEqual(len(self.corner_tasks()), 1)
        self.assertEqual(self.corner_tasks()[0].uid, self.m.uav_ids[0])
        self.assertEqual(self.corner_tasks()[0].look_points, ((-49.99, -59.99),))
        self.assertEqual(len(self.m._segments.tasks), 3)

    def test_no_idle_or_only_remaining_searcher_among_people_cannot_be_taken(self):
        self.m._tracking = {'t0': self.m.uav_ids[1], 't1': self.m.uav_ids[2]}
        self.allocate()
        self.assertEqual(self.corner_tasks(), [])
        self.assertEqual(self.m._segments.tasks[self.m.uav_ids[0]].purpose, 'coverage')
        self.m._tracking['t2'] = self.m.uav_ids[0]
        self.allocate()
        self.assertEqual(self.corner_tasks(), [])

    def test_people_attention_and_both_east_owners_are_protected(self):
        protected = self.m.uav_ids[0]
        self.m._eligible_observations = {('t0', protected): dict(
            source_uav=protected, sample_s=self.now, eligible=True)}
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertNotEqual(self.corner_tasks()[0].uid, protected)
        self.m._drop_segment(self.corner_tasks()[0].uid, 'person_task')
        self.m._regional_patrols = (NS(east=True, uid=self.m.uav_ids[0]),
                                   NS(east=True, uid=self.m.uav_ids[1]))
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertEqual(self.corner_tasks(), [])

    def test_current_pixel_attention_is_not_preempted_by_corner_task(self):
        protected = self.m.uav_ids[0]
        self.m.status[protected].search_state = ATTEND
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertNotEqual(self.corner_tasks()[0].uid, protected)

    def test_bad_or_stale_person_and_corner_times_cannot_hide_first_visit(self):
        uid = self.m.uav_ids[0]
        for bad in ('bad', None, float('nan'), float('inf'), self.now+10., self.now-1.01):
            with self.subTest(sample=bad):
                self.m._eligible_observations = {(uid, 't0'): dict(source_uav=uid, sample_s=bad)}
                self.assertFalse(self.m._west_corner_has_person(uid, self.now))
        for bad in (True, 'bad', float('nan'), float('inf'), self.now+10.):
            with self.subTest(corner=bad):
                self.m._west_corner_seen = {'northwest': bad, 'southwest': bad}
                self.assertIsNone(self.m._west_corner_image_time('northwest', self.now))
                self.assertIsNone(self.m._west_corner_image_time('southwest', self.now))
        self.allocate()
        self.assertEqual(len(self.corner_tasks()), 1)

    def test_whole_loop_concurrent_allocations_reserve_at_most_one_west_task(self):
        barrier = threading.Barrier(6)
        def allocate():
            barrier.wait()
            self.m._allocate()
        with patch.object(sm, 'BUILDING_SUBPOINT_REVIEW', True):
            workers = [threading.Thread(target=allocate) for _ in range(6)]
            for worker in workers: worker.start()
            for worker in workers: worker.join(3.)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertEqual(len(self.corner_tasks()), 1)
        keys = [key for task in self.m._segments.tasks.values() for key in task.cells]
        self.assertEqual(len(keys), len(set(keys)))

    def test_two_real_processed_frames_alone_refresh_one_corner_memory(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.frame(task)
        self.assertEqual(self.m._west_corner_seen, {})
        self.now += .3
        self.frame(task)
        name = 'northwest' if task.look_points[0][1] > 0 else 'southwest'
        self.assertEqual(self.m._west_corner_seen, {name: self.now})
        self.assertIs(self.m._segments.tasks[task.uid], task)
        prior = dict(self.m._west_corner_seen)
        for sample in (self.now, self.now-.1, self.now+1., self.now-1.):
            self.frame(task, sample=sample)
            self.assertEqual(self.m._west_corner_seen, prior)
        self.frame(task, processed=False)
        self.assertEqual(self.m._west_corner_seen, prior)

    def test_los_endpoint_obstacle_cannot_refresh_corner_image_memory(self):
        self.allocate()
        task = self.corner_tasks()[0]
        ix, iy = self.raw.world_to_cell(task.look_points[0])
        cells = bytearray(self.raw.cells)
        cells[iy*self.raw.width+ix] = 1
        self.m._merged_grid.return_value = sm.GridMap(self.raw.width, self.raw.height,
            self.raw.resolution, self.raw.origin, cells)
        self.frame(task)
        self.now += .3
        self.frame(task)
        self.assertEqual(self.m._west_corner_seen, {})

    def test_complete_without_image_keeps_first_visit_due_after_original_failure_cooldown(self):
        self.allocate()
        task = self.corner_tasks()[0]
        status = self.m.status[task.uid]
        status.search_task_id, status.search_revision = task.task_id, task.revision
        status.search_index, status.search_state = 1, DONE
        status.search_path_remaining = 0.
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertEqual(self.m._west_corner_seen, {})
        self.assertEqual(self.m.allocator.failed_visit[task.cells[0]], self.now)
        self.assertNotIn(task.cells[0], self.m._segments.reserved)
        self.assertNotIn(task.uid, self.m._segments.tasks)

    def test_old_version_done_cannot_complete_or_credit_active_corner(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.m._segments.revise(task.uid, task.cells, task.points, self.now)
        status = self.m.status[task.uid]
        status.search_task_id, status.search_revision = task.task_id, task.revision-1
        status.search_index, status.search_state = 1, DONE
        status.search_path_remaining = 0.
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertIs(self.m._segments.tasks[task.uid], task)
        self.assertEqual(task.index, 0)
        self.assertEqual(self.m._west_corner_seen, {})

    def test_person_preemption_unreachable_and_unavailable_maps_release_or_decline(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.m._tracking = {'t0': task.uid}
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertNotIn(task.cells[0], self.m._segments.reserved)
        self.assertEqual(self.m._west_corner_seen, {})
        self.m._tracking = {}
        self.m._drop_segment(self.m.uav_ids[2], 'test_cleanup')
        self.m._g_infl = None
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertEqual(self.corner_tasks(), [])

    def test_checked_path_failure_uses_existing_failure_memory_and_keeps_normal_search(self):
        with patch('search_segment_manager.plan', return_value=NS(success=False)):
            self.allocate()
        self.assertEqual(self.corner_tasks(), [])
        for _, key, _, _ in self.m._west_corner_subpoints():
            self.assertEqual(self.m.allocator.failed_visit[key], self.now)
        self.assertTrue(self.m._segments.tasks)
        self.assertTrue(all(t.purpose == 'coverage' for t in self.m._segments.tasks.values()))

    def test_missing_and_unknown_raw_map_cannot_allocate_or_credit_corner(self):
        self.m._merged_grid.return_value = None
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertEqual(self.corner_tasks(), [])
        unknown = sm.GridMap(self.raw.width, self.raw.height, self.raw.resolution,
                             self.raw.origin, bytes([255])*len(self.raw.cells))
        self.m._merged_grid.return_value = unknown
        self.m._allocate_west_corner_review(list(self.m.uav_ids), self.now)
        self.assertEqual(self.corner_tasks(), [])
        self.assertEqual(self.m._west_corner_seen, {})

    def test_original_deadline_releases_reservation_without_creating_image_credit(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.now = task.deadline
        self.refresh()
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertNotIn(task.cells[0], self.m._segments.reserved)
        self.assertEqual(self.m._west_corner_seen, {})
        self.assertEqual(self.m.allocator.failed_visit[task.cells[0]], self.now)

    def test_disconnected_corner_owner_releases_reservation_with_existing_cooldown(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.m.status[task.uid].connected = False
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertNotIn(task.cells[0], self.m._segments.reserved)
        self.assertEqual(self.m._west_corner_seen, {})
        self.assertEqual(self.m.allocator.failed_visit[task.cells[0]], self.now)

    def test_completion_after_real_corner_image_keeps_image_time_not_completion_time(self):
        self.allocate()
        task = self.corner_tasks()[0]
        self.frame(task)
        self.now += .3
        self.frame(task)
        actual_seen = dict(self.m._west_corner_seen)
        self.now += .2
        self.refresh()
        status = self.m.status[task.uid]
        status.search_task_id, status.search_revision = task.task_id, task.revision
        status.search_index, status.search_state = 1, DONE
        status.search_path_remaining = 0.
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertNotIn(task.cells[0], self.m._segments.reserved)
        self.assertEqual(self.m._west_corner_seen, actual_seen)
        self.assertNotIn(task.cells[0], self.m.allocator.failed_visit)

    def test_flag_off_preserves_normal_auction(self):
        self.allocate(enabled=False)
        self.assertEqual(self.corner_tasks(), [])
        self.assertEqual(self.m._west_corner_seen, {})


if __name__ == '__main__':
    unittest.main()
