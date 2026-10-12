"""Real missed-view failures keep their index and finite blind-review budget."""
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import test_search_segments as fixture
from building_viewpoints import valid_view
from slam_grid import SlamGrid
from swarm_task import ACTOR_SEARCH_BOUNDS, CoverageGrid, LineOfSight, STATE_COVERED, STATE_REVIEW

sa, sm = fixture.sa, fixture.sm
RECORDED = json.loads((Path(__file__).parent/'fixtures'/
    'building_observation_timeouts_053455.json').read_text())


class BuildingObservationTimeoutTests(unittest.TestCase):
    uid = fixture.ManagerSegmentTests.uid

    def setUp(self):
        fixture.ManagerSegmentTests.setUp(self)
        m = self.m
        m.grid = CoverageGrid(*ACTOR_SEARCH_BOUNDS, 7., 6)
        m.allocator, m.lease = sm.TaskAllocator(m.grid), sm.LeaseManager(m.grid)
        m._cell_waypoint = {k: (c.cx, c.cy) for k, c in m.grid.cells.items()}
        self.raw = sm.GridMap(420, 260, .5, (-55., -65.), bytes(420*260))
        m._merged_grid = Mock(return_value=self.raw)
        m._g_infl = sm.inflate_grid(self.raw, sm.INFLATE_M)
        m._camera_point_seen, m._west_corner_seen, m._review_subpoint_views = {}, {}, {}

    def agent(self, record):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        d, focused = record['task'], record['samples'][1]
        cam, pose = focused['frame']['data'], focused['pose_before_receipt']['data']
        packet = record['own_map_snapshots'][str(focused['own_map_sequence'])]['data']
        a.uav_id = d['uav']
        a.offset, a.local_xy, a.local_z = (0., 0.), (pose['x'], pose['y']), pose['z']
        a.yaw, a._pose_sample_s = cam['yaw'], pose['stamp']
        a.state = NS(armed=True, connected=True, mode='OFFBOARD')
        a._plan_lock, a._plan_epoch, a._plan_pending = threading.Lock(), 0, None
        a.path, a.path_target, a._plan_fail_t = [tuple(d['points'][0])], tuple(d['points'][0]), 0.
        a._search_segment = dict(id=d['task'], revision=d['version'], created=d['stamp'],
            until=d['until'], cells=tuple(map(tuple, d['cells'])), points=tuple(map(tuple, d['points'])),
            look_points=tuple(map(tuple, d['look_points'])), purpose='building_review',
            index=0, state=fixture.LOOK, blocked=0, reason='')
        a.assignment = fixture.SearchAssignment(uav_id=a.uav_id, task_type=0,
            cell_ix=d['cells'][0][0], cell_iy=d['cells'][0][1],
            target_x=d['points'][0][0], target_y=d['points'][0][1])
        a._search_scan_key = (*d['cells'][0], *d['points'][0])
        a._search_scan_seen, a._search_scan_first = set(), {}
        a._search_scan_done = a._search_scan_failed = False
        a._search_scan_yaw0 = record['first_LOOK']['camera']['yaw']
        a._search_scan_started_t = record['first_LOOK']['status']['received_ros']
        a._search_scan_directed_first = a._search_scan_directed_last = None
        a._search_scan_directed_done = False
        a._search_last_frame = 0.
        a.cov_grid = CoverageGrid(*ACTOR_SEARCH_BOUNDS, 7., 6)
        a.slam = SlamGrid.from_rle(packet['width'], packet['height'], packet['resolution'],
                                  packet['origin'], packet['rle'])
        raw = sm.GridMap(a.slam.width, a.slam.height, a.slam.resolution,
                        a.slam.origin, a.slam.cells)
        a.grid = sm.inflate_grid(raw, sa.INFLATE_M)
        a.los = LineOfSight(lambda ix, iy: not a.slam.is_free_raw((ix, iy)),
                            a.slam.resolution, a.slam.origin)
        a._world_trusted, a._hold_edge_review = Mock(return_value=True), Mock(return_value=False)
        a._set_search_heading, a._send_vel = Mock(), Mock()
        a._apply_friend_avoidance = lambda vx, vy: (vx, vy)
        self.now = focused['frame']['ros']
        a._processed_camera_frame_cb(NS(data=json.dumps(cam)))
        return a

    def reserve(self, west=False):
        key, point, look = ((0, 0), (-43.6613943083804, -53.6613943083804), (-49.99, -59.99)) if west else (
            (13, 8), (44.5, 4.96), (44.5, -3.99))
        self.m.status[self.uid].x, self.m.status[self.uid].y = point
        task = self.m._start_search_segment(self.uid, [key], [point], self.now,
            purpose='building_review', look_points=[look])
        self.assertTrue(valid_view(self.raw, self.m._g_infl, look, point,
                                  self.m._segment_flight_waypoint))
        return task

    def fail(self, task):
        s = self.m.status[task.uid]
        s.header.stamp = sm.rospy.Time.from_sec(self.now)
        self.m.last_report[task.uid] = s.header.stamp
        s.search_task_id, s.search_revision = task.task_id, task.revision
        s.search_index, s.search_state = task.index, fixture.FAILED
        s.search_blocked_reason = 'observation_timeout'
        s.search_path_remaining = 0.
        self.m._renew_segments(sm.rospy.Time.from_sec(self.now))

    def no_credit(self, key):
        self.assertNotEqual(self.m.grid.cell(key).state, STATE_COVERED)
        self.assertEqual(self.m.grid.cell(key).covered_t, 0.)
        self.assertNotIn(key, self.m.allocator.visit_time)
        self.assertNotIn(key, self.m.allocator.last_seen)

    def test_two_actual_failures_stop_with_failed_index_zero_instead_of_done(self):
        for record in RECORDED['cases']:
            with self.subTest(task=record['task']['task']):
                a = self.agent(record)
                cam = record['samples'][1]['frame']['data']
                look = tuple(record['task']['look_points'][0])
                self.assertTrue(a.slam.is_free_raw(a.slam.world_to_cell(look)))
                self.assertTrue(a.los.visible(cam['x'], cam['y'], *look))
                self.assertFalse(sa.in_view(cam['x'], cam['y'], cam['yaw'], *look,
                                            cam['z'], cam['down_pitch']))
                a._search_scan_step()
                self.assertFalse(a._search_scan_done)
                self.assertEqual(a._search_scan_seen, set())
                self.now = record['scan_timeout_ros']
                a._search_scan_step()
                self.assertTrue(a._search_scan_failed)
                self.now = record['first_DONE']['ros']
                a._segment_control(3., 5., .8, 2., 2.)
                self.assertEqual(a._search_segment['state'], fixture.FAILED)
                self.assertEqual(a._search_segment['index'], 0)
                self.assertEqual(a._search_segment['reason'], 'observation_timeout')
                self.assertEqual(a._search_scan_seen, set())
                self.assertEqual(a.cov_grid.cell(tuple(record['task']['cells'][0])).covered_t, 0.)
                self.assertEqual(a.path, [])
                a._send_vel.assert_called_with(0., 0.)
                status = fixture.UavStatus()
                a._segment_status(status)
                self.assertEqual(status.search_state, fixture.FAILED)
                self.assertEqual(status.search_index, 0)
                self.assertEqual(status.search_blocked_reason, 'observation_timeout')

    def test_actual_scan_success_still_advances_and_other_purposes_keep_old_behavior(self):
        for purpose, done, failed in (('building_review', True, False),
                                     ('coverage', False, True), ('south_patrol', False, True)):
            with self.subTest(purpose=purpose):
                a = self.agent(RECORDED['cases'][0])
                a._search_segment['purpose'] = purpose
                a._search_scan_done, a._search_scan_failed = done, failed
                a._segment_control(3., 5., .8, 2., 2.)
                self.assertEqual(a._search_segment['state'], fixture.DONE)
                self.assertEqual(a._search_segment['index'], 1)

    def test_safe_timeout_releases_without_progress_and_queues_original_eight_ninety(self):
        task = self.reserve()
        self.now += 8.1
        self.fail(task)
        key = task.cells[0]
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertNotIn(key, self.m._segments.reserved)
        self.assertNotIn(key, self.m.allocator.failed_visit)
        self.assertEqual(self.m.grid.cell(key).state, STATE_REVIEW)
        ready, expires = self.m.allocator.revisit_jobs[('blind', key)]
        self.assertAlmostEqual(ready, self.now+8.)
        self.assertAlmostEqual(expires, self.now+90.)
        self.no_credit(key)

    def test_old_blind_deadline_bounds_repeated_timeout_and_next_task(self):
        task = self.reserve()
        key = task.cells[0]
        self.m.allocator.revisit_jobs[('blind', key)] = (101., 145.)
        self.m._review_subpoint_views[key] = {'expires_s': 150., 'options': ()}
        self.now = 108.1
        self.fail(task)
        self.assertEqual(self.m.allocator.revisit_jobs[('blind', key)][1], 145.)
        self.assertEqual(self.m._review_subpoint_views[key]['expires_s'], 145.)
        self.now = 117.
        retry = self.m._start_search_segment(self.uid, task.cells, task.points, self.now,
            purpose='building_review', look_points=task.look_points)
        self.assertEqual(retry.deadline, 145.)
        self.now = 125.
        self.fail(retry)
        self.assertEqual(self.m.allocator.revisit_jobs[('blind', key)], (133., 145.))
        self.assertEqual(self.m._review_subpoint_views[key]['expires_s'], 145.)
        self.no_credit(key)

    def test_pruned_queue_cannot_reopen_active_task_past_its_original_deadline(self):
        task = self.reserve()
        key = task.cells[0]
        self.m.allocator.revisit_jobs[('blind', key)] = (101., 115.)
        self.m._drop_segment(task.uid, 'test_setup')
        self.now = 102.
        task = self.m._start_search_segment(self.uid, task.cells, task.points, self.now,
            purpose='building_review', look_points=task.look_points)
        self.assertEqual(task.deadline, 115.)
        self.m.allocator.revisit_jobs.clear()  # allocator pruned the old episode
        self.now = 115.
        self.fail(task)
        self.assertNotIn(task.uid, self.m._segments.tasks)
        self.assertEqual(self.m.allocator.revisit_jobs, {})
        self.assertEqual(self.m._review_subpoint_views, {})
        self.no_credit(key)

    def test_west_timeout_is_not_hidden_by_global_sixty_second_cooldown(self):
        task = self.reserve(west=True)
        self.now += 8.1
        self.fail(task)
        key = task.cells[0]
        self.assertNotIn(key, self.m.allocator.failed_visit)
        self.assertEqual(self.m._west_corner_seen, {})
        ready, expires = self.m.allocator.revisit_jobs[('blind', key)]
        self.assertAlmostEqual(ready, self.now+8.)
        self.assertAlmostEqual(expires, self.now+90.)
        # West priority must honor the same readiness instead of reissuing
        # this corner in the very tick that the failure released its owner.
        self.m._west_corner_seen['northwest'] = self.now
        self.m.uav_ids.append('typhoon_h480_1')
        self.m.status['typhoon_h480_1'] = fixture.UavStatus(
            uav_id='typhoon_h480_1', x=-35., y=-40., z=4.2, connected=True)
        self.m.last_report['typhoon_h480_1'] = sm.rospy.Time.from_sec(self.now)
        self.now = ready-.01
        for uid in self.m.uav_ids:
            self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        self.m._allocate_west_corner_review(list(self.m.uav_ids), ready-.01)
        self.assertEqual(self.m._segments.tasks, {})
        self.now = ready
        for uid in self.m.uav_ids:
            self.m.last_report[uid] = sm.rospy.Time.from_sec(self.now)
        self.m._allocate_west_corner_review(list(self.m.uav_ids), ready)
        self.assertEqual(len(self.m._segments.tasks), 1)
        replacement = next(iter(self.m._segments.tasks.values()))
        self.assertEqual(replacement.look_points, task.look_points)
        self.assertLessEqual(replacement.deadline, expires)
        self.no_credit(key)

    def test_map_blocked_missing_failed_and_unavailable_keep_old_failure_paths(self):
        for problem in ('raw_missing', 'inflated_missing', 'ground_blocked', 'waypoint_blocked'):
            with self.subTest(problem=problem):
                task = self.reserve()
                self.now += 8.1
                if problem == 'raw_missing':
                    self.m._merged_grid.return_value = None
                elif problem == 'inflated_missing':
                    self.m._g_infl = None
                else:
                    at = task.look_points[0] if problem == 'ground_blocked' else task.points[0]
                    ix, iy = self.raw.world_to_cell(at)
                    cells = bytearray(self.raw.cells)
                    cells[iy*self.raw.width+ix] = 1
                    bad = sm.GridMap(self.raw.width, self.raw.height, self.raw.resolution,
                                     self.raw.origin, cells)
                    self.m._merged_grid.return_value = bad
                    self.m._g_infl = sm.inflate_grid(bad, sm.INFLATE_M)
                self.fail(task)
                self.assertAlmostEqual(self.m.allocator.failed_visit[task.cells[0]], self.now)
                self.assertEqual(self.m.allocator.revisit_jobs, {})
                self.no_credit(task.cells[0])
                self.m._merged_grid.return_value = self.raw
                self.m._g_infl = sm.inflate_grid(self.raw, sm.INFLATE_M)
                self.m.allocator.failed_visit.clear()

    def test_wrong_index_revision_or_future_feedback_cannot_advance_or_queue(self):
        task = self.reserve()
        for bad in ('index', 'revision', 'future'):
            with self.subTest(bad=bad):
                self.now += .1
                s = self.m.status[self.uid]
                s.header.stamp = sm.rospy.Time.from_sec(self.now+1. if bad == 'future' else self.now)
                self.m.last_report[self.uid] = sm.rospy.Time.from_sec(self.now)
                s.search_task_id, s.search_revision = task.task_id, task.revision
                s.search_index, s.search_state = 0, fixture.FAILED
                s.search_blocked_reason = 'observation_timeout'
                s.search_path_remaining = 0.
                if bad == 'index': s.search_index = 1
                if bad == 'revision': s.search_revision = 0
                self.m._renew_segments(sm.rospy.Time.from_sec(self.now))
                self.assertIs(self.m._segments.tasks[self.uid], task)
                self.assertEqual(task.index, 0)
                self.assertEqual(self.m.allocator.revisit_jobs, {})
                self.no_credit(task.cells[0])

    def test_person_task_has_priority_and_historical_image_credit_is_preserved(self):
        task = self.reserve()
        self.m._tracking['t0'] = task.uid
        self.now += .1
        self.fail(task)
        self.assertEqual(self.m._tracking, {'t0': task.uid})
        self.assertEqual(self.m.allocator.revisit_jobs, {})
        self.no_credit(task.cells[0])
        self.m._tracking.clear()
        task = self.reserve()
        key = task.cells[0]
        cell = self.m.grid.cell(key)
        cell.state, cell.covered_t = STATE_COVERED, 80.
        self.m.allocator.last_seen[key] = 80.
        self.now += .1
        self.fail(task)
        self.assertEqual(cell.state, STATE_COVERED)
        self.assertEqual(cell.covered_t, 80.)
        self.assertEqual(self.m.allocator.last_seen[key], 80.)
        self.assertNotIn(key, self.m.allocator.visit_time)
        self.assertEqual(self.m.allocator.revisit_jobs, {})


if __name__ == '__main__':
    unittest.main()
