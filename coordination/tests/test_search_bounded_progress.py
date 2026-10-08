"""Bounded search movement is distinct from physical stopped evidence."""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from search_observation import SearchSweep


class BoundedProgressTests(unittest.TestCase):
    def sweep(self):
        return SearchSweep(1, (0, 0), (30., 0.), 0.)

    def test_oscillation_latches_despite_half_meter_clock_resets(self):
        s = self.sweep()
        latched = False
        for i in range(231):
            t = i*.1
            p = (1.-math.cos(t*math.pi), 0.)
            s.blocked_reason(t, p, .3, True, 'MOVING')
            latched = s.bounded_stall(t, p)
        self.assertTrue(latched)
        self.assertGreaterEqual(s.progress_s, 20.)
        self.assertLess(s.progress_s, 23.)
        self.assertFalse(s.bounded_rest_ready(24., p, .3, True))
        self.assertFalse(s.bounded_rest_ready(25., p, 0., True))
        for i in range(1, 61):
            ready = s.bounded_rest_ready(25.+i*.1, p, 0., True)
        self.assertTrue(ready)

    def test_slow_genuine_approach_not_stalled(self):
        s = self.sweep()
        for i in range(401):
            self.assertFalse(s.bounded_stall(i*.1, (i*.01, 0.)))

    def test_large_detour_not_stalled(self):
        s = self.sweep()
        for i in range(401):
            self.assertFalse(s.bounded_stall(i*.1, (0., i*.1)))

    def test_missing_freshness_and_long_gap_cannot_complete_window(self):
        for stale in (True, False):
            s = self.sweep()
            for i in range(101):
                s.bounded_stall(i*.1, (0., 0.))
            if stale:
                s.bounded_stall(10.1, (0., 0.), False)
            self.assertFalse(s.bounded_stall(20.1, (0., 0.)))

    def test_stop_rest_requires_fresh_speed_and_bounded_drift(self):
        s = self.sweep()
        for i in range(201):
            s.bounded_stall(i*.1, (0., 0.))
        self.assertFalse(s.bounded_rest_ready(20., (0., 0.), 0., True))
        self.assertFalse(s.bounded_rest_ready(25., (0., 0.), 0., False))
        self.assertFalse(s.bounded_rest_ready(26., (0., 0.), 0., True))
        self.assertFalse(s.bounded_rest_ready(31., (1., 0.), 0., True))
        self.assertFalse(s.bounded_rest_ready(36., (1., 0.), float('nan'), True))
        self.assertFalse(s.bounded_rest_ready(37., (1., 0.), 0., True))
        for i in range(1, 61):
            ready = s.bounded_rest_ready(37.+i*.1, (1., 0.), 0., True)
        self.assertTrue(ready)

    def test_missing_rest_ticks_cannot_prove_six_seconds_stopped(self):
        s = self.sweep()
        for i in range(201):
            s.bounded_stall(i*.1, (0., 0.))
        s.bounded_rest_ready(20., (0., 0.), 0., True)
        self.assertFalse(s.bounded_rest_ready(26., (0., 0.), 0., True))

    def test_next_view_and_new_generation_do_not_borrow_latch(self):
        s = self.sweep()
        for i in range(201):
            s.bounded_stall(i*.1, (0., 0.))
        s.next_view((10., 0.), 21.)
        self.assertFalse(s.bounded_stall(21., (0., 0.)))
        self.assertFalse(SearchSweep(2, (0, 0), (30., 0.), 21.).bounded_stall(21., (0., 0.)))


if __name__ == '__main__':
    unittest.main()
