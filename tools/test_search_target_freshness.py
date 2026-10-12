"""A released/stale visual target must not slow search forever."""
import unittest
from test_search_fast_start import SearchFastStartTests, sa


class SearchTargetFreshnessTests(unittest.TestCase):
    setUp = SearchFastStartTests.setUp

    def configure(self, sample):
        a = self.a
        a.targets = {'t1': (10., 0., 0., 0.)}
        a._t_seen = {'t1': sample}
        a._flee_run_t, a._escape_spent, a._spend_mode = {}, set(), set()
        a._transit_cap = sa.SwarmAgent._transit_cap.__get__(a)
        a._search_start_progress = {'metres': 11., 'last': None}

    def test_expired_visual_point_no_longer_caps_actual_search_output(self):
        self.configure(self.now - sa.TARGET_TTL - 1.)
        self.a._control()
        self.assertAlmostEqual(self.commands[-1][0], sa.SEARCH_CRUISE_SPEED)
        self.assertIsNone(self.a._nearest_target_dist())

    def test_live_person_cap_and_safety_stop_still_apply(self):
        self.configure(self.now)
        self.a._control()
        self.assertAlmostEqual(self.commands[-1][0], sa.SPOOK_SPEED)
        self.a._apply_friend_avoidance = lambda vx, vy: (0., 0.)
        self.a._control()
        self.assertEqual(self.commands[-1], (0., 0.))

    def test_invalid_future_or_missing_image_time_cannot_become_a_speed_zone(self):
        for sample in (float('nan'), float('inf'), self.now + 1.):
            self.configure(sample)
            self.assertEqual(self.a._transit_cap(), sa.MAX_SPEED)
            self.assertIsNone(self.a._nearest_target_dist())
        self.a._t_seen.clear()
        self.assertEqual(self.a._transit_cap(), sa.MAX_SPEED)

    def test_old_nearest_point_cannot_hide_another_live_person(self):
        self.configure(self.now - 20.)
        self.a.targets['t2'] = (15., 0., 0., 0.)
        self.a._t_seen['t2'] = self.now
        self.assertEqual(self.a._transit_cap(), sa.SPOOK_SPEED)
        self.assertEqual(self.a._nearest_target_dist(), ('t2', 15.))


if __name__ == '__main__':
    unittest.main()
