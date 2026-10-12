"""Recorded entry velocities and finite braking/yaw preparation regressions."""
import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import Mock
from geometry_msgs.msg import Point
from robocup_swarm.msg import SearchAssignment

from test_directed_hover_view import DirectedHoverViewTests as Setup
from search_segments import TRAVEL, LOOK, FAILED


class DirectedViewSettlingTests(unittest.TestCase):
    message, control_fixture = Setup.message, Setup.control_fixture
    body, image, control, identity = Setup.body, Setup.image, Setup.control, Setup.identity
    assert_no_credit = Setup.assert_no_credit

    def setUp(self):
        Setup.setUp(self)
        self.a._directed_view_settle_enabled = True
        self.a._directed_view_settle_timeout = 8.
        self.a._directed_view_settle_yaw_tol = math.radians(12.)
        self.now = self.record['later_status']['received_ros']
        self.body(self.record['later_status'])
        self.image(self.record['later_camera'])
        self.a._measured_velocity = (self.now, (0., 0.))

    def refresh(self, elapsed=0.):
        self.now += elapsed
        self.a._pose_sample_s = self.now
        camera = list(self.a._search_camera_pose)
        camera[0] = self.now
        self.a._search_camera_pose = tuple(camera)
        self.a._measured_velocity = (self.now, (0., 0.))

    def test_recorded_high_speed_entries_brake_without_starting_scan(self):
        data = json.loads((Path(__file__).parent/'fixtures'/
            'directed_view_entry_motion_060353.json').read_text())
        for record in data['entries']:
            with self.subTest(task=record['task']):
                self.now = record['firstlook']
                status, frame = record['status'], record['camera']['frame']
                a = self.a
                a.local_xy = (status['x'], status['y'])
                a._pose_sample_s = status['stamp']
                a._search_segment.update(index=0, state=TRAVEL, until=self.now+30.,
                    points=(tuple(status['point']),),
                    look_points=(tuple(record['look']),))
                a._search_segment.pop('view_entry', None)
                a._search_camera_pose = (frame['sample_s'], frame['x'], frame['y'],
                    frame['yaw'], frame['z'], frame['down_pitch'])
                velocity = record['measured_velocity']
                a._measured_velocity = (velocity['velocity_sample_s'],
                                         tuple(velocity['velocity_xyz'][:2]))
                a._search_scan_step.reset_mock()
                self.control()
                self.assertEqual(a._search_segment['state'], TRAVEL)
                a._search_scan_step.assert_not_called()
                a._send_vel.assert_called_with(0., 0.)
                self.assertIsNone(a._search_scan_started_t)

    def test_stopped_actual_aligned_camera_can_start_original_scan(self):
        self.control()
        self.assertEqual(self.a._search_segment['state'], LOOK)
        self.a._search_scan_step.assert_called_once()
        self.assert_no_credit()

    def test_turning_177_degrees_does_not_use_image_observation_budget(self):
        camera = list(self.a._search_camera_pose)
        bearing = math.atan2(self.a._search_segment['look_points'][0][1]-camera[2],
                             self.a._search_segment['look_points'][0][0]-camera[1])
        camera[3] = bearing-math.radians(177.)
        self.a._search_camera_pose = tuple(camera)
        before = self.identity()
        self.control()
        self.assertEqual(self.a._search_segment['state'], TRAVEL)
        self.assert_no_credit()
        self.refresh(7.)
        camera = list(self.a._search_camera_pose)
        camera[3] = bearing
        self.a._search_camera_pose = tuple(camera)
        self.control()
        self.assertEqual(self.a._search_segment['state'], LOOK)
        self.assertEqual(before, self.identity())
        self.a._search_scan_step.assert_called_once()

    def test_missing_stale_future_or_nonfinite_actual_velocity_cannot_start(self):
        for measured in (None, (self.now-.31, (0., 0.)),
                         (self.now+.01, (0., 0.)), (self.now, (math.nan, 0.)),
                         (self.now, (0., .36))):
            with self.subTest(measured=measured):
                self.a._measured_velocity = measured
                self.control()
                self.assertEqual(self.a._search_segment['state'], TRAVEL)
                self.assert_no_credit()
        self.a._search_scan_step.assert_not_called()

    def test_wait_is_bounded_without_changing_task_index_or_deadline(self):
        self.a._measured_velocity = (self.now, (1.75, 0.))
        before = self.identity()
        self.control()
        self.refresh(8.01)
        self.control()
        self.assertEqual(self.a._search_segment['state'], FAILED)
        self.assertEqual(self.a._search_segment['reason'], 'observation_timeout')
        self.assertEqual(before, self.identity())
        self.assert_no_credit()

    def test_exact_view_revision_does_not_refresh_preparation_budget(self):
        self.a._measured_velocity = (self.now, (1.75, 0.))
        self.control()
        task = self.a._search_segment
        started = task['view_entry'][1]
        self.refresh(3.)
        msg = SearchAssignment(uav_id=self.a.uav_id, task_type=0,
            search_task_id=task['id'], search_revision=task['revision']+1,
            search_purpose=task['purpose'])
        msg.header.stamp = self.a.assignment.header.stamp.from_sec(self.now)
        msg.search_valid_until = msg.header.stamp.from_sec(task['until'])
        msg.search_cell_ix, msg.search_cell_iy = zip(*task['cells'])
        msg.search_waypoints = [Point(x=x, y=y) for x,y in task['points']]
        msg.search_look_points = [Point(x=x, y=y) for x,y in task['look_points']]
        self.assertTrue(self.a._accept_segment_message(msg))
        self.assertEqual(self.a._search_segment['view_entry'], (0, started))
        self.refresh(5.01)
        self.control()
        self.assertEqual(self.a._search_segment['state'], FAILED)

    def test_blocked_range_correction_remains_zero(self):
        self.a._measured_velocity = (self.now, (1.75, 0.))
        self.control()
        self.refresh(.5)
        camera = list(self.a._search_camera_pose)
        look = self.a._search_segment['look_points'][0]
        camera[1], camera[2], camera[3] = look[0]+8.4, look[1], math.pi
        self.a._search_camera_pose = tuple(camera)
        self.a.local_xy = (camera[1], camera[2])
        self.a._search_segment['points'] = (self.a.world_xy,)
        from unittest.mock import patch
        import search_segment_agent
        with patch.object(search_segment_agent, 'clear_line', return_value=False):
            self.control()
        self.a._send_vel.assert_called_with(0., 0.)
        self.assert_no_credit()

    def test_braking_drift_can_correct_range_but_never_credit_a_frame(self):
        self.a._measured_velocity = (self.now, (1.75, 0.))
        self.control()
        self.refresh(.5)
        look = self.a._search_segment['look_points'][0]
        camera = list(self.a._search_camera_pose)
        camera[1], camera[2], camera[3] = look[0]+8.4, look[1], math.pi
        self.a._search_camera_pose = tuple(camera)
        self.a.local_xy = (camera[1], camera[2])
        self.a._search_segment['points'] = (self.a.world_xy,)
        self.control()
        vx, vy = self.a._send_vel.call_args.args
        self.assertGreater(vx, 0.)
        self.assertLessEqual(math.hypot(vx, vy), .35)
        self.assert_no_credit()

    def test_pitch_only_rejection_does_not_demand_horizontal_motion(self):
        self.a._measured_velocity = (self.now, (1.75, 0.))
        self.control()
        self.refresh(.5)
        camera = list(self.a._search_camera_pose)
        camera[5] = -.1
        self.a._search_camera_pose = tuple(camera)
        self.control()
        self.a._send_vel.assert_called_with(0., 0.)
        self.assert_no_credit()

    def test_legacy_configuration_does_not_require_unsubscribed_velocity(self):
        self.a._directed_view_settle_enabled = False
        self.a._measured_velocity = None
        self.control()
        self.assertEqual(self.a._search_segment['state'], LOOK)


if __name__ == '__main__':
    unittest.main()
