"""The actual first LOOK must qualify its camera geometry before its timer."""
import copy
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import test_search_segments as fixture
import swarm_agent as sa
import swarm_manager as sm
import swarm_task as st
from search_segments import TRAVEL, LOOK


class CornerArrivalCameraPoseTests(unittest.TestCase):
    message = fixture.AgentSegmentTests.message
    control_fixture = fixture.AgentSegmentTests.control_fixture

    def setUp(self):
        fixture.AgentSegmentTests.setUp(self)
        self.record = json.loads((Path(__file__).parent / 'fixtures' /
            'corner_arrival_camera_pose_1998_031135.json').read_text())
        self.now = self.record['status_received_ros']
        a = self.a = fixture.AgentSegmentTests.directed_fixture(self)
        clock = patch.object(sa.rospy.Time, 'now',
            side_effect=lambda: sa.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)
        a.grid = sm.GridMap(420, 260, .5, (-55., -65.), bytes(420*260))
        a.cov_grid = st.CoverageGrid(-50., 130., -60., 60., 7., 6)
        a.uav_id = 'typhoon_h480_0'
        a.local_xy = tuple(self.record['status_world_xyz'][:2])
        a.local_z = self.record['status_world_xyz'][2]
        a.offset = (0., 0.)
        a.state = NS(connected=True, armed=True, mode='OFFBOARD')
        a._world_trusted = Mock(return_value=True)
        a._pose_sample_s = self.record['status_sample_s']
        point = tuple(self.record['viewpoint'])
        look = tuple(self.record['look_point'])
        a._search_segment.update(id=self.record['task_id'],
            revision=self.record['task_version'], created=self.record['task_created_s'],
            cells=(tuple(self.record['reservation']),), points=(point,),
            look_points=(look,), until=2050., state=TRAVEL)
        a.assignment.uav_id = a.uav_id
        a.assignment.cell_ix, a.assignment.cell_iy = self.record['reservation']
        a.assignment.target_x, a.assignment.target_y = point
        a._search_scan_key = (25, 0, 121.16, -58.5)
        a._search_camera_pose = None
        a._search_scan_started_t = None
        a._search_scan_first = {}
        a._search_scan_seen = set()
        a._search_scan_step = Mock()
        a._search_observable_points = Mock(return_value={})
        a._hold_edge_review = Mock(return_value=False)
        a._request_plan = Mock()
        a._segment_path_key = (self.record['task_id'], 1)
        a.path = [a.world_xy, point]
        a.path_target = point
        a._takeoff_done = True
        a._landing = False

    def image(self, data=None):
        data = copy.deepcopy(data or self.record['latest_camera_before_status']['data'])
        self.a._processed_camera_frame_cb(NS(data=json.dumps(data)))

    def control(self):
        return self.a._segment_control(3., 5., .8, 2., 2.)

    def assert_no_scan_credit(self):
        self.assertEqual(self.a._search_segment['index'], 0)
        self.assertIsNone(self.a._search_scan_started_t)
        self.assertFalse(self.a._search_scan_done)
        self.assertEqual(self.a._search_scan_seen, set())
        self.assertNotEqual(self.a.cov_grid.cell((25, 0)).state, st.STATE_COVERED)

    def test_real_1998_pitch_has_wide_band_but_out_of_range_stays_travel(self):
        self.image()
        a = self.a
        camera = a._search_camera_pose
        self.assertIsNotNone(camera)  # actual assignment already cached images
        distance = math.dist(a.world_xy, self.record['viewpoint'])
        self.assertAlmostEqual(distance, 1.7326990408607486)
        band = sa.ground_distance_band(camera[4], camera[5])
        self.assertGreater(.5*(band[1]-band[0]), distance)
        self.assertGreater(math.dist(camera[1:3], self.record['look_point']), 9.)
        self.control()
        self.assertEqual(a._search_segment['state'], TRAVEL)
        a._search_scan_step.assert_not_called()
        self.assertGreater(math.hypot(*a._send_vel.call_args.args), 0.)
        self.assertEqual(a._last_local_goal, tuple(self.record['viewpoint']))
        self.assert_no_scan_credit()

    def test_real_1998_556_later_frame_also_cannot_start_scan_outside_range(self):
        self.now = self.record['camera_after_status']['received_ros']
        self.a._pose_sample_s = self.now
        self.image(self.record['camera_after_status']['data'])
        self.control()
        self.assertEqual(self.a._search_segment['state'], TRAVEL)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_scan_credit()

    def test_real_later_valid_geometry_enters_look_without_rewriting_task_clock(self):
        self.image()
        self.control()
        task_before = (self.a._search_segment['id'], self.a._search_segment['revision'],
                       self.a._search_segment['created'], self.a._search_segment['until'])
        self.now = self.record['later_observable_camera']['received_ros']
        body = self.record['later_observable_status']
        self.a.local_xy = tuple(body['world_xyz'][:2])
        self.a.local_z = body['world_xyz'][2]
        self.a._pose_sample_s = body['sample_s']
        self.image(self.record['later_observable_camera']['data'])
        self.control()
        self.assertEqual(self.a._search_segment['state'], LOOK)
        self.a._search_scan_step.assert_called_once()
        self.assertEqual(task_before, (self.a._search_segment['id'],
            self.a._search_segment['revision'], self.a._search_segment['created'],
            self.a._search_segment['until']))
        self.assert_no_scan_credit()

    def test_missing_camera_at_exact_waypoint_holds_without_spending_scan_budget(self):
        self.a.local_xy = tuple(self.record['viewpoint'])
        self.control()
        self.assertEqual(self.a._search_segment['state'], TRAVEL)
        self.a._send_vel.assert_called_with(0., 0.)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_scan_credit()

    def test_wrong_source_or_bad_frame_cannot_start_or_credit_scan(self):
        for changes in ({'source_uav': 'typhoon_h480_1'}, {'processed': False},
                        {'sample_s': self.now-1.}, {'sample_s': self.now+.1},
                        {'x': math.nan}, {'z': 0.}, {'down_pitch': math.inf}):
            with self.subTest(changes=changes):
                data = copy.deepcopy(self.record['latest_camera_before_status']['data'])
                data.update(changes)
                self.image(data)
                self.assertIsNone(self.a._search_camera_pose)
                self.control()
                self.assertEqual(self.a._search_segment['state'], TRAVEL)
                self.a._search_scan_step.assert_not_called()
                self.assert_no_scan_credit()

    def test_stale_or_future_cached_camera_does_not_open_scan_timer(self):
        self.image()
        pose = self.a._search_camera_pose
        for sample in (self.now-1., self.now+.03):
            with self.subTest(sample=sample):
                self.a._search_camera_pose = (sample, *pose[1:])
                self.control()
                self.assertEqual(self.a._search_segment['state'], TRAVEL)
                self.a._search_scan_step.assert_not_called()
                self.assert_no_scan_credit()

    def test_camera_world_mismatch_cannot_start_scan(self):
        self.image()
        pose = self.a._search_camera_pose
        self.a._search_camera_pose = (pose[0], pose[1]+5., *pose[2:])
        self.control()
        self.assertEqual(self.a._search_segment['state'], TRAVEL)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_scan_credit()

    def test_pose_freshness_and_trust_remain_required_for_first_look(self):
        self.image()
        for kind in ('stale', 'untrusted'):
            with self.subTest(kind=kind):
                self.a._pose_sample_s = self.now-1. if kind == 'stale' else self.now
                self.a._world_trusted.return_value = kind != 'untrusted'
                self.control()
                self.assertEqual(self.a._search_segment['state'], TRAVEL)
                self.a._search_scan_step.assert_not_called()
                self.assert_no_scan_credit()

    def test_already_look_keeps_original_bounded_range_correction(self):
        self.image()
        self.a._search_segment['state'] = LOOK
        self.a._search_scan_step = sa.SwarmAgent._search_scan_step.__get__(self.a)
        self.control()
        self.assertEqual(self.a._search_segment['state'], LOOK)
        self.assertLessEqual(math.hypot(*self.a._send_vel.call_args.args), .35)
        self.assertFalse(self.a._search_scan_done)
        self.assertEqual(self.a._search_segment['index'], 0)

    def test_ordinary_search_visit_keeps_existing_rules(self):
        self.now = self.record['later_observable_camera']['received_ros']
        self.a.local_xy = tuple(self.record['later_observable_status']['world_xyz'][:2])
        self.a._pose_sample_s = self.record['later_observable_status']['sample_s']
        self.a._search_segment['purpose'] = 'coverage'
        self.a._search_segment['look_points'] = ()
        self.image(self.record['later_observable_camera']['data'])
        self.control()
        self.assertEqual(self.a._search_segment['index'], 1)
        self.a._search_scan_step.assert_not_called()
        self.assertNotEqual(self.a.cov_grid.cell((25, 0)).state, st.STATE_COVERED)

    def test_real_person_preempts_without_starting_corner_look(self):
        a = self.a
        a._camera_cue = dict(candidate_id='blue:8', color='blue', target_id='t1',
            started=self.now, until=self.now+2., bearing=0.,
            xy=(120., -57.), image={'fresh': True})
        a._stable_other_owner = Mock(return_value=False)
        with patch.object(sa, 'candidate_decision', return_value=(0., .3, 'approach')):
            a._control()
        self.assertEqual(a._search_segment['state'], TRAVEL)
        a._search_scan_step.assert_not_called()
        a._send_vel.assert_called_once_with(0., .3)
        self.assert_no_scan_credit()

    def test_blocked_verified_path_holds_before_camera_scan(self):
        self.image()
        self.a._search_segment.update(blocked=1, reason='GOAL_OCCUPIED')
        self.control()
        self.a._send_vel.assert_called_with(0., 0.)
        self.a._search_scan_step.assert_not_called()
        self.assert_no_scan_credit()


if __name__ == '__main__':
    unittest.main()
