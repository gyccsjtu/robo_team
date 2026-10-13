"""Fast-image yaw cannot acquire a person, refresh evidence, or move the UAV."""
import copy
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                     'coordination/src/robocup_swarm/scripts'))
from fast_tracking_view import fast_yaw_enabled, validate_fast_view
sys.path.insert(0, str(Path(__file__).resolve().parent))
from header_clock_test_support import install_clock_fixture, prepare_flight_fixture
import swarm_agent as sa


class FastYawTests(unittest.TestCase):
    def setUp(self):
        install_clock_fixture(self, sa.rospy)
        self.now = 100.14
        clock = patch.object(sa.rospy.Time, 'now',
                             side_effect=lambda: sa.rospy.Time.from_sec(self.now))
        clock.start()
        self.addCleanup(clock.stop)

    def real(self, **changes):
        v = dict(target_id='t5', local_target_id='t4', sample_s=100.,
                 position_sample_s=100., candidate_id='red:7',
                 world_candidate_id='session:41', width=752., height=480.,
                 fy=376., bearing=.1, x=8., y=0., geometry_ok=True,
                 box=[350., 240., 385., 360.])
        v.update(changes)
        return v

    def data(self, **changes):
        v = dict(schema=1, source_uav='u5', sample_s=100.1, seed_s=100.,
                 seed_id='session:red:7', seed_candidate_id='red:7',
                 candidate_session='session', box=[370., 240., 405., 360.],
                 width=752, height=480, fx=376., bearing=.2, quality=.8,
                 status='tracked', real_detection=False, report_eligible=False,
                 footpoint_valid=False, yaw_only=True)
        v.update(changes)
        return v

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id, a._fast_yaw_enabled = 'u5', True
        a.assignment = NS(task_type=1, target_id='t5', target_x=8., target_y=0.)
        a._tracking_image = self.real()
        a._fast_tracking_image = None
        a._fast_yaw_csv = Mock()
        a.local_xy, a.offset, a.local_z = (0., 0.), (0., 0.), 3.
        a._t_seen = {'t5': 100.}
        a.targets = {'t5': (8., 0., 1., 0.)}
        a._confirm_start, a._last_confirm_t = 98., 100.
        a._visual_observations = {'t5': {'sample_s': 100.}}
        a._tracking_altitude = {'goal': 3.}
        return a

    def send(self, a, **changes):
        a._fast_tracking_view_cb(NS(data=json.dumps(self.data(**changes))))

    def test_disabled_by_default_and_explicit_two_aircraft_selection(self):
        self.assertFalse(fast_yaw_enabled({}, 'typhoon_h480_2'))
        env = dict(FAST_VISUAL_YAW='1', FAST_VISUAL_YAW_UAVS='2,5')
        self.assertTrue(fast_yaw_enabled(env, 'typhoon_h480_2'))
        self.assertTrue(fast_yaw_enabled(env, 'typhoon_h480_5'))
        self.assertFalse(fast_yaw_enabled(env, 'typhoon_h480_1'))
        self.assertFalse(fast_yaw_enabled(dict(env, FAST_VISUAL_YAW='0'), 'typhoon_h480_2'))

    def test_explicit_red_instance_works_despite_local_global_slot_difference(self):
        result = validate_fast_view(self.data(), 'u5', 't5', self.real(), self.now)
        self.assertEqual(result['target_id'], 't5')
        self.assertEqual(result['bound_world_candidate_id'], 'session:41')
        self.assertEqual(result['bearing'], .2)

    def test_other_red_or_camera_or_new_session_is_not_same_person(self):
        for changes in (dict(seed_candidate_id='red:8', seed_id='session:red:8'),
                        dict(candidate_session='restart', seed_id='restart:red:7'),
                        dict(source_uav='u4'), dict(seed_id='session:red:8')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fast_view(self.data(**changes), 'u5', 't5', self.real(), self.now)
        for changes in (dict(candidate_id=None), dict(world_candidate_id=None),
                        dict(target_id='t4')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fast_view(self.data(), 'u5', 't5', self.real(**changes), self.now)

    def test_future_stale_seed_and_same_image_do_not_count_as_updates(self):
        for changes in (dict(sample_s=100.2), dict(sample_s=99.98),
                        dict(seed_s=99.4), dict(sample_s=100.),
                        dict(seed_s=100.1), dict(seed_s=99.9)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fast_view(self.data(**changes), 'u5', 't5', self.real(), self.now)
        for changes in (dict(sample_s=99.), dict(position_sample_s=99.)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fast_view(self.data(), 'u5', 't5', self.real(**changes), self.now)

    def test_quality_contract_box_and_bearing_jump_fail_closed(self):
        for changes in (dict(quality=.34), dict(quality=float('nan')),
                        dict(bearing=float('inf')), dict(bearing=.351),
                        dict(report_eligible=True), dict(real_detection=True),
                        dict(footpoint_valid=True), dict(yaw_only=False),
                        dict(width=640), dict(box=[0., 10., 20., 481.]),
                        dict(box=[0., 10., 20.]), dict(status='lost')):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_fast_view(self.data(**changes), 'u5', 't5', self.real(), self.now)

    def test_angular_wrap_is_not_a_bearing_jump(self):
        v = validate_fast_view(self.data(bearing=-math.pi+.02), 'u5', 't5',
                               self.real(bearing=math.pi-.02), self.now)
        self.assertEqual(v['bearing'], -math.pi+.02)

    def test_callback_does_not_refresh_real_position_altitude_or_evidence(self):
        a = self.agent()
        names = ('_tracking_image', '_t_seen', 'targets', '_confirm_start',
                 '_last_confirm_t', '_visual_observations', '_tracking_altitude')
        before = {n: copy.deepcopy(getattr(a, n)) for n in names}
        self.send(a)
        self.assertEqual(a._fresh_fast_tracking_view(self.now)['bearing'], .2)
        self.assertEqual(before, {n: getattr(a, n) for n in names})
        self.assertEqual(a._fast_yaw_csv.write.call_args.kwargs['event'], 'accept')

    def test_duplicate_and_out_of_order_cannot_replace_current_sample(self):
        a = self.agent()
        self.send(a)
        self.send(a, bearing=.25)
        self.assertEqual(a._fast_tracking_image['bearing'], .2)
        self.send(a, sample_s=100.05, bearing=.25)
        self.assertEqual(a._fast_tracking_image['sample_s'], 100.1)

    def test_new_real_frame_has_priority_and_stale_fast_frame_expires(self):
        a = self.agent()
        self.send(a)
        a._tracking_image['sample_s'] = 100.1
        self.assertIsNone(a._fresh_fast_tracking_view(self.now))
        a._tracking_image['sample_s'] = 100.
        self.assertIsNone(a._fresh_fast_tracking_view(100.251))

    def test_target_change_search_or_camera_rebind_immediately_invalidates_fast_yaw(self):
        for action in ('target', 'search', 'world', 'binding', 'hypothesis'):
            a = self.agent()
            self.send(a)
            if action == 'target':
                a.assignment.target_id = 't4'
            elif action == 'search':
                a.assignment.task_type = 0
            elif action == 'world':
                a._tracking_image['world_candidate_id'] = 'session:42'
            elif action == 'binding':
                a._camera_instance = {'t5': {'sample_s': 100.1, 'candidate_id': 'session:42'}}
            else:
                a._hypothesis_switch_s = {'t5': 100.05}
            with self.subTest(action=action):
                self.assertIsNone(a._fresh_fast_tracking_view(self.now))

    def test_cancellation_clears_cached_fast_image_and_cannot_revive_old_task(self):
        a = self.agent()
        self.send(a)
        a._track_assigned_id = 't5'
        a._discard_old_plan = lambda: None
        a._assign_cb(NS(uav_id='u5', task_type=255, target_id='t5'))
        self.assertIsNone(a._fast_tracking_image)
        a.assignment = NS(task_type=1, target_id='t5')
        self.assertIsNone(a._fresh_fast_tracking_view(self.now))

    def output_agent(self, enable):
        a = self.agent()
        a.yaw, a._look_at, a.altitude_layer = -.5, (8., 0.), 3.
        a._orbit_target, a._confirm_close = 't5', True
        a._takeoff_done, a._landing = True, False
        a._scan_t = self.now
        a._scan = NS(angle_min=-math.pi, angle_increment=math.pi/180.,
                     range_min=.1, range_max=20., ranges=[20.]*361)
        a._est_trustworthy = lambda: True
        a._bounds_recovery_velocity = lambda: None
        a._radar_guard_velocity = lambda vx, vy: (0., 0.)
        a._grid_guard_velocity = a._map_guard_velocity = lambda vx, vy: (vx, vy)
        a._final_scan_cap = lambda vx, vy: (vx, vy)
        a._compute_desired_altitude = lambda: 0.
        a._last_cmd_v, a._last_csv_t = (1., 0.), self.now
        a._climb_latch, a._climb_t0, a._alt_sag = False, None, 0.
        a.pos_pub, a.vel_pub = Mock(), Mock()
        prepare_flight_fixture(a, sa.rospy)
        self.send(a)
        a._fast_yaw_enabled = enable
        return a

    def test_actual_flight_output_only_changes_yaw_and_keeps_safety_stop(self):
        outputs = []
        for enable in (False, True):
            a = self.output_agent(enable)
            a.yaw = 0.
            with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'), patch.object(sa, 'CRASH_DETECT', False):
                a._send_vel(1., 0.)
                a._publish_wall_setpoint_once()
            outputs.append(a.pos_pub.publish.call_args.args[0].pose)
        for output in outputs:
            self.assertEqual((output.position.x, output.position.y, output.position.z), (0., 0., 3.))
        self.assertGreater(outputs[1].orientation.z, outputs[0].orientation.z)
        self.assertEqual(a._fast_yaw_csv.write.call_args.kwargs['yaw_source'], 'fast_image')
        self.assertEqual(a._t_seen, {'t5': 100.})

    def test_velocity_and_position_modes_keep_existing_yaw_rate_limit(self):
        for mode in ('vel', 'pos'):
            a = self.output_agent(True)
            a.yaw = -2.
            with patch.object(sa, 'FLIGHT_OUTPUT', mode), patch.object(sa, 'CRASH_DETECT', False):
                a._send_vel(1., 0.)
                a._publish_wall_setpoint_once()
            if mode == 'vel':
                rate = a.vel_pub.publish.call_args.args[0].twist.angular.z
            else:
                q = a.pos_pub.publish.call_args.args[0].pose.orientation
                output_yaw = 2.*math.atan2(q.z, q.w)
                rate = (output_yaw-a.yaw)/sa.POS_SP_LEAD
            self.assertAlmostEqual(rate, sa.YAW_RATE_MAX)

    def test_full_diagnostic_disk_cannot_interrupt_position_stream(self):
        a = self.output_agent(True)
        a._fast_yaw_csv.write.side_effect = OSError('no space')
        with patch.object(sa, 'FLIGHT_OUTPUT', 'pos'), patch.object(sa, 'CRASH_DETECT', False):
            a._send_vel(1., 0.)
            a._publish_wall_setpoint_once()
        a.pos_pub.publish.assert_called_once()
        self.assertIsNone(a._fast_yaw_csv)


if __name__ == '__main__':
    unittest.main()
