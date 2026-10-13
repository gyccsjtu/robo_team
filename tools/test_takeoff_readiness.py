"""Offline regressions for overlapping EKF checks and zero-setpoint prestream."""
import sys
import os
import json
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock, PropertyMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent


class TakeoffReadinessTests(unittest.TestCase):
    def test_camera_gate_rejects_lost_calibration_stale_or_unstable_ground(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent._takeoff_camera_frames = (98., 99.9)
        agent._takeoff_ground_samples = [(98.+k*.05, -.1, 0.) for k in range(41)]
        with patch.dict(os.environ, {'TAKEOFF_REQUIRE_CAMERA': '1'}):
            self.assertTrue(agent._camera_takeoff_ready(100.))
            agent._takeoff_camera_frames = (96., 98.)
            self.assertFalse(agent._camera_takeoff_ready(100.))
            agent._takeoff_camera_frames = (98., 99.9)
            agent._takeoff_ground_samples[-1] = (100., -.15, 0.)
            self.assertFalse(agent._camera_takeoff_ready(100.))
            agent._takeoff_ground_samples[-1] = (100., -.1, .1)
            self.assertFalse(agent._camera_takeoff_ready(100.))

    def test_ground_camera_sequence_uses_new_own_frames_without_assignment(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'u0'
        agent.state = SimpleNamespace(armed=False)
        data = dict(source_uav='u0', processed=True, x=0., y=0., z=.17,
                    yaw=0., down_pitch=0., sample_s=100.)
        for now in (100., 100.2, 100.4, 100.6, 100.8, 101., 101.):
            data['sample_s'] = now
            with patch.object(swarm_agent.rospy.Time, 'now',
                              return_value=swarm_agent.rospy.Time.from_sec(now)):
                agent._processed_camera_frame_cb(SimpleNamespace(data=json.dumps(data)))
        self.assertEqual(agent._takeoff_camera_frames, (100., 101.))
        data['sample_s'] = 103.
        with patch.object(swarm_agent.rospy.Time, 'now', return_value=swarm_agent.rospy.Time(103)):
            agent._processed_camera_frame_cb(SimpleNamespace(data=json.dumps(data)))
        self.assertEqual(agent._takeoff_camera_frames, (103., 103.))

    def run_start(self, hook=None, origin=(0.0, 0.0), reject_first=False):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'test_uav'
        agent.local_z, agent.local_xy = 0.0, origin
        agent.state = SimpleNamespace(armed=False, mode='MANUAL')
        agent._anchor_done = False
        clock = SimpleNamespace(now=10.0, steps=0, shutdown=False)
        outputs, modes, arms = [], [], []

        def sleep():
            clock.now += 0.05
            clock.steps += 1
            if hook:
                hook(agent, clock)
            if clock.steps > 500:
                self.fail('takeoff loop did not finish')

        def mode_srv(*args):
            modes.append(clock.now)
            accepted = not reject_first or len(modes) > 1
            if accepted:
                agent.state.mode = 'OFFBOARD'
            return SimpleNamespace(mode_sent=accepted)

        def arm_srv(*args):
            arms.append(clock.now)
            agent.state.armed = True
            return SimpleNamespace(success=True)

        agent.ctrl_rate = SimpleNamespace(sleep=sleep)
        agent._send_vel = lambda vx, vy, vz=None: outputs.append(
            (clock.now, vx, vy, vz))
        agent.mode_srv, agent.arm_srv = mode_srv, arm_srv
        with ExitStack() as stack:
            stack.enter_context(patch.object(swarm_agent.rospy.Time, 'now',
                side_effect=lambda: swarm_agent.rospy.Time.from_sec(clock.now)))
            stack.enter_context(patch.object(swarm_agent.rospy, 'is_shutdown',
                                            side_effect=lambda: clock.shutdown))
            for name in ('loginfo', 'logerr', 'logwarn_throttle'):
                stack.enter_context(patch.object(swarm_agent.rospy, name))
            result = agent._arm_and_offboard()
        self.assertTrue(all(item[1:] == (0.0, 0.0, 0.0) for item in outputs))
        return result, modes, arms, outputs

    def test_ready_aircraft_overlaps_preheat_instead_of_waiting_extra_six_seconds(self):
        result, modes, arms, outputs = self.run_start()
        self.assertTrue(result)
        self.assertGreaterEqual(modes[0] - outputs[0][0], 2.0)
        self.assertLess(modes[0] - outputs[0][0], 2.1)
        self.assertGreaterEqual(len(outputs), swarm_agent.STABLE_NEEDED)
        self.assertGreaterEqual(arms[0], modes[0])

    def test_output_gap_requires_new_continuous_prestream(self):
        def gap(agent, clock):
            if clock.steps == 10:
                clock.now += 0.8
        result, modes, _, outputs = self.run_start(gap)
        self.assertTrue(result)
        after_gap = next(row[0] for row in outputs if row[0] > 11.0)
        self.assertGreaterEqual(modes[0] - after_gap, 2.0 - 1e-6)

    def test_clock_rewind_does_not_reuse_prior_prestream(self):
        def rewind(agent, clock):
            if clock.steps == 10:
                clock.now -= 1.0
        result, modes, _, outputs = self.run_start(rewind)
        self.assertTrue(result)
        reset = next(b[0] for a, b in zip(outputs, outputs[1:]) if b[0] < a[0])
        self.assertGreaterEqual(modes[0] - reset, 2.0 - 1e-6)

    def test_unstable_height_still_delays_offboard(self):
        def unstable(agent, clock):
            agent.local_z = 3.0 if clock.steps < 60 else 0.0
        result, modes, _, _ = self.run_start(unstable)
        self.assertTrue(result)
        self.assertGreaterEqual(modes[0], 14.9)

    def test_invalid_origin_does_not_request_mode_or_arm(self):
        result, modes, arms, _ = self.run_start(origin=(2.0, 0.0))
        self.assertFalse(result)
        self.assertEqual((modes, arms), ([], []))

    def test_fault_and_shutdown_do_not_request_mode_or_arm(self):
        for field in ('fault', 'shutdown'):
            def stop(agent, clock):
                if clock.steps == 10:
                    if field == 'fault':
                        agent._localization_fault = True
                    else:
                        clock.shutdown = True
            result, modes, arms, _ = self.run_start(stop)
            self.assertFalse(result)
            self.assertEqual((modes, arms), ([], []))

    def test_rejected_offboard_keeps_zero_stream_until_accepted(self):
        result, modes, arms, outputs = self.run_start(reject_first=True)
        self.assertTrue(result)
        self.assertEqual(len(modes), 2)
        self.assertGreaterEqual(arms[0], modes[1])
        self.assertLess(max(b[0] - a[0] for a, b in zip(outputs, outputs[1:])), .06)

    def test_only_airborne_offboard_aircraft_is_available_for_dispatch(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'test_uav'
        agent.local_z = 3.0
        agent.assignment = None
        agent.status_pub = Mock()
        agent.cov_grid = SimpleNamespace(world_to_cell=Mock(return_value=(1, 2)))
        cases = [(False, True, 'OFFBOARD', False, False),
                 (True, False, 'OFFBOARD', False, False),
                 (True, True, 'AUTO.LAND', False, False),
                 (True, True, 'OFFBOARD', True, False),
                 (True, True, 'OFFBOARD', False, True)]
        with patch.object(swarm_agent.SwarmAgent, 'world_xy',
                          new_callable=PropertyMock, return_value=(1., 2.)), \
                patch.object(swarm_agent.rospy.Time, 'now',
                             return_value=swarm_agent.rospy.Time(100)):
            for airborne, armed, mode, landing, expected in cases:
                with self.subTest(airborne=airborne, armed=armed, mode=mode, landing=landing):
                    agent._takeoff_done, agent._landing = airborne, landing
                    agent.state = SimpleNamespace(connected=True, armed=armed, mode=mode)
                    agent._publish_status()
                    status = agent.status_pub.publish.call_args.args[0]
                    self.assertEqual(status.connected, expected)
                    if not expected:
                        self.assertEqual((status.cell_ix, status.cell_iy), (-1, -1))
                        self.assertEqual(status.confidence, 0.0)

    def test_climb_watchdog_reports_stall_without_rearming_or_changing_mode(self):
        agent = swarm_agent.SwarmAgent.__new__(swarm_agent.SwarmAgent)
        agent.uav_id = 'test_uav'
        agent.local_z = .1
        agent.state = SimpleNamespace(connected=True, armed=True, mode='OFFBOARD')
        agent.mode_srv, agent.arm_srv = Mock(), Mock()
        with patch.object(swarm_agent.rospy.Time, 'now') as now, \
                patch.object(swarm_agent.rospy, 'logerr_throttle') as log:
            for t in (100, 105, 111):
                now.return_value = swarm_agent.rospy.Time(t)
                agent._check_climb_progress()
            log.assert_called_once()
            agent.mode_srv.assert_not_called()
            agent.arm_srv.assert_not_called()
            # Progress and clock resets restart the timeout.
            agent.local_z = .5
            now.return_value = swarm_agent.rospy.Time(112)
            agent._check_climb_progress()
            self.assertEqual(agent._climb_progress, (112., .5))
            now.return_value = swarm_agent.rospy.Time(90)
            agent._check_climb_progress()
            self.assertEqual(agent._climb_progress, (90., .5))


if __name__ == '__main__':
    unittest.main()
