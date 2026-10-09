"""Recorded stale grants exercise real callbacks; no physics success is implied."""
import test_tracking_backoff as backoff
from target_motion import handoff_reacquisition_point
import unittest


class HandoffReacquisitionTests(unittest.TestCase):
    def fixture(self, now=2034.264, stamp=2029.892):
        t = backoff.TrackingBackoffTests(); t.setUp(); t.now = now
        a = t.agent
        a._v124_behavior = True; a._handoff_yaw_reacquire = True
        a.targets['t3'] = (57.43, -1.34, 1., 0.)
        a._t_seen['t3'] = stamp
        t.grant()
        return t, a

    def test_recorded_old_point_can_aim_without_changing_retry_or_evidence(self):
        t, a = self.fixture(); a._control()
        self.assertEqual(a._look_at, (57.43, -1.34))
        a._send_vel.assert_called_once_with(0., 0.)
        self.assertEqual(a._giveup_until['t3'], 60.)
        self.assertEqual(a._reset_n[3], 7)
        self.assertEqual(a._t_seen['t3'], 2029.892)
        self.assertIsNone(a._route_pending)
        a._pick_local_goal.assert_not_called()

    def test_renewal_cannot_extend_five_second_window(self):
        t, a = self.fixture(); window = a._handoff_reacquire_window
        t.now += 4.; a._t_seen['t3'] = t.now-.2; a._giveup_until['t3'] = t.now+60
        t.grant(seq=2)
        self.assertEqual(a._handoff_reacquire_window, window)
        t.now = window[3]; a._t_seen['t3'] = t.now-.2
        a._control(); self.assertIsNone(a._look_at)
        a._send_vel.assert_called_once_with(0., 0.)

    def test_stop_clears_window_and_yaw(self):
        t, a = self.fixture(); a._control(); t.grant(seq=2, action='STOP')
        self.assertIsNone(a._handoff_reacquire_window)
        a._control(); self.assertIsNone(a._look_at)

    def test_expired_authority_blocks_yaw_even_with_recent_image(self):
        t, a = self.fixture(); t.now += 11.; a._t_seen['t3'] = t.now-.1
        a._control(); self.assertIsNone(a._look_at)
        a._send_vel.assert_called_once_with(0., 0.)

    def test_successful_fresh_guidance_consumes_window_without_clearing_counters(self):
        t, a = self.fixture(); a._giveup_until.clear(); a._t_seen['t3'] = t.now-.1
        t.grant(seq=2); t.scope['ORBIT_RADIUS'] = 3.
        a.world_xy = (99., 99.); a._fly_orbit = lambda: None
        a._control()
        self.assertIsNone(a._handoff_reacquire_window)
        self.assertEqual(a._reset_n[3], 7)

    def test_new_generation_has_its_own_window(self):
        t, a = self.fixture(); old = a._handoff_reacquire_window
        t.grant(seq=2, action='STOP'); t.now += 2.; t.grant(seq=3, generation=2)
        self.assertEqual(a._handoff_reacquire_window, ('t3', 2, t.now, t.now+5.))
        self.assertNotEqual(a._handoff_reacquire_window, old)
        a._control(); self.assertIsNone(a._look_at)  # Original frame is now older than 5s.

    def test_pure_yaw_hint_rejects_bad_evidence_and_stale_generation(self):
        for stamp, now, window, authorized, cooldown in [
                (16.,20.,('t3',1,20.,25.),False,0.),
                (16.,20.,('t3',2,20.,25.),True,0.),
                (16.,20.,('t1',1,20.,25.),True,0.),
                (14.999,20.,('t3',1,20.,25.),True,0.),
                (20.001,20.,('t3',1,20.,25.),True,0.),
                (16.,20.,('t3',1,20.,25.),True,float('inf')),
                (16.,19.9,('t3',1,20.,25.),True,0.),
                (float('nan'),20.,('t3',1,20.,25.),True,0.),
                (16.,20.,None,True,0.)]:
            with self.subTest(stamp=stamp,window=window):
                self.assertIsNone(handoff_reacquisition_point({'t3':(12.,6.)},
                    {'t3':stamp},'t3',1,window,now,authorized,cooldown))


if __name__ == '__main__':
    unittest.main()
