"""Cropped-view retreat reaches the common output, but never defeats braking.

The aircraft/target positions are constructed sensor inputs, not simulator
truth. The real orbit controller, velocity pipeline and flight encoders run;
only planning bookkeeping and an empty lidar grid are supplied as fixtures.
"""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import test_tracking_view as view_fixture
import test_view_motion_envelope as motion_fixture
import swarm_agent as sa


class ViewSpaceFinalOutputTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.5
        for patcher in (
                patch.object(sa.rospy.Time, 'now', side_effect=lambda:
                             sa.rospy.Time.from_sec(self.now)),
                patch.object(sa, 'VIEW_SPACE_BACKOFF', True),
                patch.object(sa, 'MOTION_SAFETY', True),
                patch.object(sa, 'RADAR_GUARD', True),
                patch.object(sa, 'FLIGHT_INSET', 2.),
                patch.object(sa, 'MAP_GUARD_MARGIN', 2.),
                patch.object(sa, 'flight_bounds', return_value=(-50., 150., -60., 60.))):
            patcher.start()
            self.addCleanup(patcher.stop)

    def agent(self, own_x=50., measured_vx=-.95):
        fixture = motion_fixture.ViewMotionEnvelopeTests()
        fixture.now = self.now
        a = fixture.agent(distance=8.)
        a.local_xy = (own_x, 20.)
        a.local_z = a.altitude_layer = 4.14
        tx, ty = own_x+8., 20.
        a.assignment = NS(task_type=1, target_id='t1', target_x=tx, target_y=ty)
        a.targets = {'t1': (tx, ty, -.9, 0.)}
        a._tracking_image = None
        a._visual_observations = {}
        a._confirm_start = a._last_confirm_t = 0.
        a._confirm_close_tid = 't1'
        a._confirm_close_until = 200.
        a._close_range_ref = None
        a._orbit_entered_t = self.now
        a._giveup_until, a._stale_search_until = {}, {}
        a._flee_run_t, a._escape_spent = {}, set()
        a._last_heading = None
        a._world_trusted = lambda: True
        a._publish_claim = Mock()
        a._pose_sample_s = self.now
        a._measured_velocity = (self.now, (measured_vx, 0.))
        a._last_cmd_v = (-.95, 0.)
        # Empty occupancy is intentional: a new ray need not have reached the
        # fused grid yet, so the final scan and measured-speed gate must work.
        a.grid = NS(world_to_cell=lambda point: (int(point[0]*2), int(point[1]*2)),
                    is_free=lambda cell: True, is_free_raw=lambda cell: True,
                    resolution=.5, origin=(-50., -60.), width=400, height=240)
        a.los = NS(visible=lambda *_args: True)
        # Keep real directional lidar, arena and final-output safety methods.
        for name in ('_radar_guard_velocity', '_map_guard_velocity', '_final_scan_cap'):
            a.__dict__.pop(name, None)
        fixture = view_fixture.TrackingViewTests()
        for sample in (100., 100.2, 100.4):
            raw = fixture.payload(source_uav='u1', target_id='t1', sample_s=sample,
                x=tx, y=ty, box=[325., 300., 375., 480.],
                report_ok=False, person_verdict_ok=True,
                world_candidate_id='u1:session:7')
            a._t_seen['t1'] = sample
            with patch.object(sa.rospy.Time, 'now',
                              return_value=sa.rospy.Time.from_sec(sample+.1)):
                a._tracking_view_cb(NS(data=json.dumps(raw)))
        self.assertIn('minimum_view_range', a._view_motion_context(self.now))
        return a

    def run_orbit(self, agent, mode):
        decisions = []
        original = sa.urgent_close_backoff_velocity

        def record_decision(*args, **kwargs):
            result = original(*args, **kwargs)
            decisions.append(result)
            return result

        with patch.object(sa, 'FLIGHT_OUTPUT', mode), \
                patch.object(sa, 'urgent_close_backoff_velocity',
                             side_effect=record_decision) as urgent, \
                patch.object(agent, '_output_motion_safety',
                             wraps=agent._output_motion_safety) as final_gate:
            agent._fly_orbit()
        urgent.assert_called_once()
        self.assertGreater(urgent.call_args.kwargs['minimum_view_range'], 10.)
        self.assertEqual(decisions[0][2], True)
        self.assertAlmostEqual(decisions[0][0], -1.35)
        self.assertAlmostEqual(decisions[0][1], 0.)
        # The recovery remains a control request, not an official observation.
        self.assertFalse(agent._tracking_image['report_ok'])
        self.assertEqual(agent._visual_observations, {})
        self.assertEqual((agent._confirm_start, agent._last_confirm_t), (0., 0.))
        final_gate.assert_called_once()
        self.assertLess(final_gate.call_args.args[0], 0.)
        return agent.pos_pub if mode == 'pos' else agent.vel_pub

    def assert_stopped_output(self, agent, mode, publisher):
        cmd = publisher.publish.call_args.args[0]
        if mode == 'pos':
            self.assertEqual((cmd.pose.position.x, cmd.pose.position.y), agent.local_xy)
        else:
            self.assertEqual((cmd.twist.linear.x, cmd.twist.linear.y), (0., 0.))
        self.assertEqual(agent._last_flight_v, (0., 0.))

    def test_clear_rear_space_allows_bounded_recovery_in_both_output_modes(self):
        for mode in ('pos', 'vel'):
            with self.subTest(mode=mode):
                a = self.agent()
                publisher = self.run_orbit(a, mode)
                vx, vy = a._last_flight_v
                self.assertLess(vx, -.95)
                self.assertAlmostEqual(vy, 0.)
                self.assertLessEqual(math.hypot(vx, vy), sa.CLOSE_BACKOFF_SPEED)
                cmd = publisher.publish.call_args.args[0]
                if mode == 'pos':
                    self.assertAlmostEqual(cmd.pose.position.x-a.local_xy[0],
                                           vx*sa.POS_SP_LEAD)
                else:
                    self.assertAlmostEqual(cmd.twist.linear.x, vx)

    def test_rear_ray_and_actual_approach_speed_override_urgent_retreat(self):
        for mode in ('pos', 'vel'):
            with self.subTest(mode=mode):
                a = self.agent(measured_vx=-3.)
                # The ray is beyond the old hard stop: intent looks safe,
                # while the measured 3m/s approach requires earlier braking.
                a._scan.ranges[0] = a._scan.ranges[-1] = 4.5
                self.assertGreater(4.5, sa.RADAR_STOP_R)
                publisher = self.run_orbit(a, mode)
                self.assert_stopped_output(a, mode, publisher)

    def test_boundary_measured_brake_overrides_urgent_without_outside_setpoint(self):
        for mode in ('pos', 'vel'):
            with self.subTest(mode=mode):
                # The 10m observation ring still fits in the arena, so this
                # exercises final braking rather than the inward-goal branch.
                a = self.agent(own_x=-43., measured_vx=-3.)
                publisher = self.run_orbit(a, mode)
                self.assert_stopped_output(a, mode, publisher)


if __name__ == '__main__':
    unittest.main()
