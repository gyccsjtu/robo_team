"""Offline flight output replay: simulation business and monotonic wall transport."""
import math
import os
import sys
import threading
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan


class HeaderClockFlightTests(unittest.TestCase):
    def setUp(self):
        self.clock = NS(sim=100., wall=1000., epoch=0, valid=True, shutdown=False)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        for name, value in (('MOTION_SAFETY', True), ('RADAR_GUARD', 1),
                            ('FLIGHT_OUTPUT', 'pos')):
            self.stack.enter_context(patch.object(sa, name, value))
        self.stack.enter_context(patch.object(sa.rospy.Time, 'now',
            side_effect=lambda: sa.rospy.Time.from_sec(self.clock.sim)))
        for name, callback in (
                ('wall_monotonic', lambda: self.clock.wall),
                ('time_epoch', lambda: self.clock.epoch),
                ('is_time_valid', lambda: self.clock.valid),
                ('is_shutdown', lambda: self.clock.shutdown)):
            self.stack.enter_context(patch.object(sa.rospy, name, side_effect=callback))
        for name in ('loginfo', 'logerr', 'logwarn_throttle', 'loginfo_throttle',
                     'logerr_throttle'):
            self.stack.enter_context(patch.object(sa.rospy, name))
        self.stack.enter_context(patch.dict(os.environ, {'TAKEOFF_REQUIRE_CAMERA': '0'}))
        a = self.a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'typhoon_h480_0'
        a.local_xy = a.offset = (0., 0.)
        a.local_z, a.yaw = 3., .4
        a.altitude_layer = 3.
        a._anchor_done, a._localization_fault = True, False
        a._local_prev_t = self.clock.sim-.05
        a._pose_sample_s = self.clock.sim-.05
        a._scan_t = self.clock.sim-.05
        a._flight_clock_active = True
        a._flight_lock = threading.RLock()
        a._flight_stop = threading.Event()
        a._flight_cache = a._wall_hold_anchor = None
        a._epoch_pose_samples, a._epoch_pose_required = 0, 1
        a._prestream_requested = False
        a._zero_stream_start = a._zero_stream_last = None
        a._takeoff_done, a._landing = True, False
        a._last_cmd_v = a._last_flight_v = (0., 0.)
        a._last_speed, a._alt_sag = .4, 0.
        a._crash_cnt, a._est_hold_n = 3, 4
        a._climb_t0, a._climb_latch = None, False
        a._slam_update = Mock()  # map work is irrelevant to a replayed setpoint
        a.pos_pub, a.vel_pub = Mock(), Mock()
        a.arm_srv, a.mode_srv = Mock(), Mock()
        self.accept_samples()

    def accept_samples(self, xy=(0., 0.), z=None, velocity=(0., 0.), obstacle=None):
        a, c = self.a, self.clock
        pose = PoseStamped()
        pose.header.stamp = sa.rospy.Time.from_sec(c.sim)
        pose.pose.position.x, pose.pose.position.y = xy
        pose.pose.position.z = a.local_z if z is None else z
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(.2), math.cos(.2)
        a._local_cb(pose)
        speed = TwistStamped()
        speed.header.stamp = pose.header.stamp
        speed.twist.linear.x, speed.twist.linear.y = velocity
        a._measured_velocity_cb(speed)
        scan = LaserScan()
        scan.header.stamp = pose.header.stamp
        scan.angle_min, scan.angle_increment = -math.pi, math.pi/180.
        scan.range_min, scan.range_max = .1, 20.
        scan.ranges = [float('inf')]*361
        if obstacle is not None:
            # An obstacle directly ahead in world coordinates for yaw=.4.
            scan.ranges[157] = obstacle
        a._scan_cb(scan)

    def advance(self, sim=.05, wall=.05, **samples):
        self.clock.sim += sim
        self.clock.wall += wall
        self.accept_samples(**samples)

    def tick(self):
        return self.a._publish_wall_setpoint_once()

    def assert_local_hold(self, result, expected=None):
        channel, message = result
        self.assertEqual(channel, 'pos')
        actual = (message.pose.position.x, message.pose.position.y,
                  message.pose.position.z,
                  2.*math.atan2(message.pose.orientation.z, message.pose.orientation.w))
        for value, want in zip(actual, expected or (0., 0., 3., .4)):
            self.assertAlmostEqual(value, want)

    def test_pause_and_source_outage_keep_wall_outputs_without_old_climb(self):
        a = self.a
        a._flight_output(1., 0., .4, .3, target_alt=4., monitor_climb=True)
        a.pos_pub.publish.assert_not_called()
        moving = self.tick()[1]
        self.assertGreater(moving.pose.position.x, 0.)
        self.assertGreater(moving.pose.position.z, 3.)
        self.clock.valid = False
        for _ in range(100):
            self.clock.wall += .05
            self.assert_local_hold(self.tick())
            a._control()  # an invalid-time business call cannot count EKF failure
        self.assertEqual(a.pos_pub.publish.call_count, 101)
        self.assertEqual((a._crash_cnt, a._est_hold_n), (3, 4))
        self.assertIsNone(a._flight_cache)
        a.arm_srv.assert_not_called()
        a.mode_srv.assert_not_called()
        self.assertEqual(self.clock.sim, 100.)

    def test_monotonic_loop_runs_at_twenty_hz_while_simulation_is_frozen(self):
        a, c = self.a, self.clock
        c.valid = False

        def sleep():
            c.wall += .05
            if a.pos_pub.publish.call_count == 12:
                a._flight_stop.set()

        with patch.object(sa.rospy, 'WallRate', return_value=NS(sleep=sleep)) as rate:
            a._wall_flight_loop()
        rate.assert_called_once_with(20.)
        self.assertEqual(a.pos_pub.publish.call_count, 12)
        self.assertEqual(c.sim, 100.)
        self.assertAlmostEqual(c.wall, 1000.6)

    def test_replay_does_not_repeat_safety_acceleration_or_climb_counters(self):
        a = self.a
        with patch.object(a, '_output_motion_safety', wraps=a._output_motion_safety) as safety, \
                patch.object(a, '_guard_executed_climb', wraps=a._guard_executed_climb) as climb:
            a._flight_output(.5, 0., .4, target_alt=4., monitor_climb=True)
            first = self.tick()[1]
            state = (a._climb_t0, a._last_cmd_v, a._last_speed, a._crash_cnt, a._est_hold_n)
            for _ in range(10):
                self.clock.wall += .02
                repeated = self.tick()[1]
                self.assertEqual(repeated.pose, first.pose)
            self.assertEqual(safety.call_count, 1)
            self.assertEqual(climb.call_count, 1)
            self.assertEqual(state, (a._climb_t0, a._last_cmd_v, a._last_speed,
                                     a._crash_cnt, a._est_hold_n))

    def test_expired_command_is_removed_and_cannot_revive_on_source_recovery(self):
        a = self.a
        a._flight_output(1., 0., .5, .4, target_alt=4.)
        self.tick()
        self.clock.wall += .26  # pose/scan still fresh; the command itself expired
        self.assert_local_hold(self.tick())
        self.assertIsNone(a._flight_cache)
        self.advance(xy=(.03, 0.))
        self.assertTrue(a._flight_sources_ready())
        self.assert_local_hold(self.tick())
        a._flight_output(.5, 0., 0.)
        self.assertGreater(self.tick()[1].pose.position.x, .03)

    def test_stale_pose_scan_and_velocity_each_veto_the_entire_old_setpoint(self):
        a = self.a
        for source, expiry in (('pose', .31), ('scan', .51), ('velocity', .31)):
            with self.subTest(source=source):
                for name in ('pose', 'scan', 'velocity'):
                    setattr(a, '_wall_%s_received' % name, self.clock.wall)
                a._flight_output(1., 0., .4, .3, target_alt=4.)
                setattr(a, '_wall_%s_received' % source, self.clock.wall-expiry)
                self.assert_local_hold(self.tick())
                self.assertIsNone(a._flight_cache)

    def test_recent_but_invalid_radar_cannot_keep_old_climb_or_camera_turn(self):
        a = self.a
        a._flight_output(1., 0., .4, .3, target_alt=4.)
        a._scan.ranges = [float('nan')]*361
        self.assert_local_hold(self.tick())
        self.assertIsNone(a._flight_cache)

    def test_new_epoch_retires_tasks_and_needs_new_pose_sequence_and_new_command(self):
        a = self.a
        a.assignment = NS(target_id='old-person')
        a._camera_cue = {'sample_s': 99., 'until': 110.}
        a._tracking_image = {'sample_s': 99.}
        a.targets = {'old-person': (50., 20., 1., 0.)}
        a._t_seen = {'old-person': 99.}
        a.path = [(50., 20.)]
        a._hypothesis_switch_s = {'old-person': 99.}
        a._flight_output(1., 0., .5, .3, target_alt=4.)
        old_epoch_command = a._flight_cache
        self.clock.epoch, self.clock.sim = 1, 5.
        a._reset_flight_time(1)
        self.assertIsNone(a.assignment)
        self.assertIsNone(getattr(a, '_camera_cue', None))
        self.assertEqual((a.targets, a._t_seen, a.path, a._hypothesis_switch_s), ({}, {}, [], {}))
        self.assert_local_hold(self.tick())
        for _ in range(2):
            self.advance()
            self.assertFalse(a._flight_sources_ready())
        self.advance()
        self.assertTrue(a._flight_sources_ready())
        a._flight_cache = old_epoch_command  # even a delayed old writer is fenced
        self.assert_local_hold(self.tick())
        self.assertIsNone(a._flight_cache)
        a._flight_output(.5, 0., 0.)
        self.assertGreater(self.tick()[1].pose.position.x, 0.)

    def test_epoch_reset_during_old_control_calculation_cannot_stamp_it_as_new(self):
        a, c = self.a, self.clock
        a.assignment = NS(target_id='old-person')

        @sa.flight_time_step
        def delayed_control(agent):
            old_goal = agent.assignment
            c.epoch, c.sim = 1, 5.
            agent._reset_flight_time(1)
            for _ in range(3):
                self.advance()
            self.assertTrue(agent._flight_sources_ready())
            self.assertEqual(old_goal.target_id, 'old-person')
            agent._flight_output(1., 0., .5, target_alt=4.)

        delayed_control(a)
        self.assertIsNone(a._flight_cache)
        self.assert_local_hold(self.tick())
        a._flight_output(.5, 0., 0.)
        self.assertGreater(self.tick()[1].pose.position.x, 0.)

    def test_new_epoch_status_broadcast_resumes_without_waiting_for_old_time(self):
        a, c = self.a, self.clock
        a._last_status_t = sa.rospy.Time(100)
        c.epoch, c.sim = 1, 5.
        a._reset_flight_time(1)
        a.state = NS(connected=True, armed=True, mode='OFFBOARD')
        a._calibrate_offset = Mock(return_value=True)
        a._configure_fcu = Mock()
        a._arm_and_offboard = Mock(return_value=True)
        a._control = Mock()
        a._detect_targets = Mock()
        published = []

        def status():
            published.append(c.sim)
            c.shutdown = True

        def sleep():
            c.wall += .05
            c.sim += .05
            self.assertLess(c.sim, 6., 'status still waits for the retired epoch time')

        a._publish_status = Mock(side_effect=status)
        a.ctrl_rate = NS(sleep=sleep)
        a._run_mission()
        self.assertEqual(len(published), 1)
        self.assertLessEqual(published[0], 5.+1./sa.PUB_RATE+.05)

    def test_preempted_sensor_callbacks_cannot_restore_retired_timestamps(self):
        a, c = self.a, self.clock
        for source in ('pose', 'velocity', 'scan'):
            with self.subTest(source=source):
                old_stamp = c.sim+.05
                c.sim = old_stamp
                entered, resume, reset_done = (threading.Event() for _ in range(3))
                errors = []

                def pause():
                    entered.set()
                    if not resume.wait(1.):
                        raise RuntimeError('sensor callback was not resumed')

                class OldStamp(float):
                    def __ge__(self, other):
                        pause()
                        return super().__ge__(other)

                class OldVelocity(tuple):
                    def __getitem__(self, index):
                        if index == 0:
                            pause()
                        return super().__getitem__(index)

                if source == 'pose':
                    a._pose_sample_s = OldStamp(old_stamp-.05)
                    message = PoseStamped()
                    message.pose.position.z = 3.
                    message.pose.orientation.w = 1.
                    callback = a._local_cb
                elif source == 'velocity':
                    a._measured_velocity = OldVelocity((old_stamp-.05, (0., 0.)))
                    message = TwistStamped()
                    callback = a._measured_velocity_cb
                else:
                    a._scan_t = OldStamp(old_stamp-.05)
                    message = LaserScan()
                    message.angle_min, message.angle_increment = -math.pi, math.pi/180.
                    message.range_min, message.range_max = .1, 20.
                    message.ranges = [float('inf')]*361
                    callback = a._scan_cb
                message.header.stamp = sa.rospy.Time.from_sec(old_stamp)

                def old_callback():
                    try:
                        callback(message)
                    except Exception as exc:
                        errors.append(exc)

                def reset():
                    try:
                        a._reset_flight_time(c.epoch)
                    except Exception as exc:
                        errors.append(exc)
                    finally:
                        reset_done.set()

                worker = threading.Thread(target=old_callback, daemon=True)
                reset_worker = threading.Thread(target=reset, daemon=True)
                worker.start()
                try:
                    self.assertTrue(entered.wait(.5), 'callback never reached the sampling check')
                    c.epoch += 1
                    c.sim = 5.
                    reset_worker.start()
                    self.assertFalse(reset_done.wait(.02), 'reset raced through the sensor transaction')
                finally:
                    resume.set()
                    worker.join(.5)
                    if reset_worker.ident is not None:
                        reset_worker.join(.5)
                self.assertFalse(worker.is_alive())
                self.assertFalse(reset_worker.is_alive())
                self.assertEqual(errors, [])
                for _ in range(3):
                    self.advance()
                self.assertTrue(a._flight_sources_ready())
                self.assertLess(a._pose_sample_s, old_stamp)
                self.assertLess(a._measured_velocity[0], old_stamp)
                self.assertLess(a._scan_t, old_stamp)

    def test_epoch_reset_during_wall_message_copy_vetoes_old_climb_before_publish(self):
        a, c = self.a, self.clock
        a._flight_output(1., 0., .5, .3, target_alt=4.)
        deepcopy = sa.copy.deepcopy

        def reset_while_copying(message):
            c.epoch, c.sim, c.valid = 1, 5., False
            return deepcopy(message)

        with patch.object(sa.copy, 'deepcopy', side_effect=reset_while_copying):
            self.assert_local_hold(self.tick())
        self.assertIsNone(a._flight_cache)
        a.pos_pub.publish.assert_called_once()
        self.assertEqual(a.pos_pub.publish.call_args.args[0].header.stamp.to_sec(), 5.)

    def test_source_loss_during_wall_stamping_vetoes_old_goal_before_publish(self):
        a, c = self.a, self.clock
        a._flight_output(1., 0., .5, .3, target_alt=4.)
        calls = []

        def stamp():
            calls.append(c.sim)
            if len(calls) == 2:
                c.valid = False
            return sa.rospy.Time.from_sec(c.sim)

        with patch.object(sa.rospy.Time, 'now', side_effect=stamp):
            self.assert_local_hold(self.tick())
        self.assertIsNone(a._flight_cache)
        a.pos_pub.publish.assert_called_once()

    def test_invalid_source_never_calls_arm_or_offboard_even_after_zero_prestream(self):
        a, c = self.a, self.clock
        a.local_z = 0.
        a.state = NS(armed=False, mode='MANUAL')
        c.valid = False
        steps = []

        def sleep():
            c.wall += .05
            a._publish_wall_setpoint_once()
            steps.append(c.wall)
            if len(steps) == 60:
                c.shutdown = True

        a.ctrl_rate = NS(sleep=sleep)
        self.assertFalse(a._arm_and_offboard())
        self.assertEqual(a.vel_pub.publish.call_count, 60)
        a.pos_pub.publish.assert_not_called()
        for call in a.vel_pub.publish.call_args_list:
            message = call.args[0]
            self.assertEqual((message.twist.linear.x, message.twist.linear.y,
                              message.twist.linear.z, message.twist.angular.z), (0., 0., 0., 0.))
        a.arm_srv.assert_not_called()
        a.mode_srv.assert_not_called()

    def test_cached_wall_prestream_does_not_turn_one_pose_into_stable_samples(self):
        a, c = self.a, self.clock
        a.local_z = 0.
        a.state = NS(armed=False, mode='MANUAL')
        steps = []

        def sleep():
            # All stream/simulation times advance but the one pose never does.
            c.wall += .05
            c.sim += .05
            a._publish_wall_setpoint_once()
            steps.append(c.wall)
            if len(steps) == 100:
                c.shutdown = True

        a.ctrl_rate = NS(sleep=sleep)
        self.assertFalse(a._arm_and_offboard())
        self.assertEqual(a.vel_pub.publish.call_count, 100)
        a.arm_srv.assert_not_called()
        a.mode_srv.assert_not_called()

    def test_real_position_fault_remains_sticky_and_cannot_send_an_old_position(self):
        a = self.a
        a._flight_output(1., 0., .5, target_alt=4.)
        self.advance(xy=(10., 0.))  # a genuine MAVROS discontinuity
        self.assertTrue(a._localization_fault)
        channel, message = self.tick()
        self.assertEqual(channel, 'vel')
        self.assertEqual(message.twist, TwistStamped().twist)
        self.clock.epoch, self.clock.sim = 1, 5.
        a._reset_flight_time(1)
        self.assertTrue(a._localization_fault)
        self.assertFalse(a._flight_sources_ready())

    def test_original_braking_and_position_encoding_still_gate_live_commands(self):
        a = self.a
        self.advance(velocity=(3., 0.), obstacle=4.5)
        for mode in ('vel', 'pos'):
            with self.subTest(mode=mode), patch.object(sa, 'FLIGHT_OUTPUT', mode):
                a._flight_output(.5, 0., 0.)
                channel, message = self.tick()
                self.assertEqual(channel, mode)
                if mode == 'vel':
                    self.assertEqual(message.twist.linear.x, 0.)
                else:
                    self.assertEqual(message.pose.position.x, 0.)

    def test_startup_requires_one_explicit_legal_aircraft(self):
        resolve = sa._clock_sources_from_startup
        self.assertEqual(resolve(['_uav_id:=typhoon_h480_2'], {}), ['typhoon_h480_2'])
        self.assertEqual(resolve([], {'TEAM_TIME_UAVS': 'typhoon_h480_5'}), ['typhoon_h480_5'])
        for value in ('', 'typhoon_h480_6', 'uav_1', 'typhoon_h480_0,typhoon_h480_1', '/clock'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                resolve([], {'TEAM_TIME_UAVS': value})


if __name__ == '__main__':
    unittest.main()
