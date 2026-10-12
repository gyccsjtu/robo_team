"""East-transit regression: an old visual position cannot indefinitely cap speed."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'coordination/src/robocup_swarm/scripts'))
import swarm_agent as sa


class TransitTargetFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.now = 2000.004
        clock = patch.object(sa.rospy.Time, 'now',
            side_effect=lambda: NS(to_sec=lambda: self.now))
        clock.start()
        self.addCleanup(clock.stop)

    def agent(self):
        a = sa.SwarmAgent.__new__(sa.SwarmAgent)
        a.uav_id = 'typhoon_h480_4'
        # 225123 east-patrol status at 2000.004; recorded last green image
        # 1981.928 measured (39.77,4.90). These are team-camera estimates,
        # not actor ground truth or an elimination-status substitute.
        a.local_xy = (39.26267623901367, 20.95037269592285)
        a.offset = (0., 0.)
        a.targets = {}
        a._t_seen = {}
        a._target_state = {}
        a._flee_run_t = {}
        a._walk_since = {}
        a._escape_spent = set()
        a._spend_mode = set()
        return a

    def observe_green(self, a, sample):
        message = NS(eliminated=False, target_id='t0', x=39.77, y=4.90,
                     vx=.90, vy=0., state=0,
                     header=NS(stamp=sa.rospy.Time.from_sec(sample)))
        a._target_cb(message)

    def test_recorded_east_cached_green_does_not_hold_transit_at_point85(self):
        a = self.agent()
        self.now = 1982.004
        self.observe_green(a, 1981.928)
        self.assertAlmostEqual(a._transit_cap(), sa.SPOOK_SPEED)
        original = (dict(a.targets), dict(a._t_seen))
        for now, xy in (
                (1990.004, (31.21626091003418, 20.852737426757812)),
                (2000.004, (39.26267623901367, 20.95037269592285)),
                (2010.056, (47.44158935546875, 21.07386589050293)),
                (2015.004, (51.410247802734375, 20.9747371673584))):
            with self.subTest(now=now):
                self.now, a.local_xy = now, xy
                self.observe_green(a, 1981.928)  # same-image bridge keepalive
                self.assertLess(math.dist(a.world_xy, (39.77, 4.90)), sa.SPOOK_DIST)
                self.assertEqual(a._transit_cap(), sa.MAX_SPEED)
                self.assertEqual((a.targets, a._t_seen), original)
        # At 2020 the recorded aircraft has left the fixed raw point's
        # 22 m circle. Keep the old-image cache unchanged through that case
        # too; the earlier positions specifically reproduce the stale cap.
        self.now = 2020.004
        a.local_xy = (55.29047775268555, 20.865598678588867)
        self.observe_green(a, 1981.928)
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)
        self.assertEqual((a.targets, a._t_seen), original)

    def test_fresh_person_keeps_original_escape_protection_through_ttl(self):
        a = self.agent()
        for age in (0., .1, sa.TARGET_TTL):
            with self.subTest(age=age):
                a.targets = {'t0': (39.77, 4.90, .90, 0.)}
                a._t_seen = {'t0': self.now-age}
                self.assertAlmostEqual(a._transit_cap(), sa.SPOOK_SPEED)
        a._t_seen['t0'] = self.now-sa.TARGET_TTL-.001
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)

    def test_one_stale_target_cannot_mask_fresh_target_in_either_order(self):
        a = self.agent()
        for ids in (('t0', 't1'), ('t1', 't0')):
            a.targets = {tid: (39.77, 4.90, .90, 0.) for tid in ids}
            a._t_seen = {'t0': 1981.928, 't1': self.now-.1}
            self.assertAlmostEqual(a._transit_cap(), sa.SPOOK_SPEED)
        a._t_seen['t1'] = self.now-sa.TARGET_TTL-.1
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)

    def test_unobserved_nonfinite_and_future_clock_samples_do_not_apply_cap(self):
        a = self.agent()
        a.targets = {'t0': (39.77, 4.90, .90, 0.)}
        for sample in (None, 0., -1., math.nan, math.inf, -math.inf, self.now+.01):
            with self.subTest(sample=sample):
                a._t_seen = {} if sample is None else {'t0': sample}
                self.assertEqual(a._transit_cap(), sa.MAX_SPEED)
        a._t_seen = {'t0': 1981.928}
        self.now = 1.0  # new simulation clock; prior sortie's cache is invalid
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)

    def test_fresh_escape_states_and_outside_radius_keep_original_caps(self):
        a = self.agent()
        a.targets, a._t_seen = {'t0': (39.77, 4.90, .90, 0.)}, {'t0': self.now}
        for name in ('_flee_run_t', '_escape_spent', '_spend_mode'):
            with self.subTest(state=name):
                original = getattr(a, name)
                setattr(a, name, {'t0': self.now} if name == '_flee_run_t' else {'t0'})
                self.assertEqual(a._transit_cap(), sa.MAX_SPEED)
                setattr(a, name, original)
        a.targets['t0'] = (a.world_xy[0]+sa.SPOOK_DIST+.01, a.world_xy[1], .9, 0.)
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)

    def test_new_real_image_reinstates_cap_without_judge_feedback(self):
        a = self.agent()
        self.observe_green(a, 1981.928)
        self.assertEqual(a._transit_cap(), sa.MAX_SPEED)
        self.observe_green(a, self.now-.1)
        self.assertAlmostEqual(a._transit_cap(), sa.SPOOK_SPEED)
        self.assertAlmostEqual(a._t_seen['t0'], self.now-.1)


if __name__ == '__main__':
    unittest.main()
