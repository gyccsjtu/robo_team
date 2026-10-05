from pathlib import Path
import sys
import math
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/robocup_swarm/scripts'))
from radar_observed_map import ObservedMap


class ObservationTests(unittest.TestCase):
    def feed(self, grid, ranges, **overrides):
        args = dict(position=(0., 0.), yaw=0., angle_min=0., angle_increment=math.pi/2,
                    range_min=.5, range_max=8., scan_s=10., pose_s=10., now_s=10.)
        args.update(overrides)
        return grid.feed(ranges, **args)

    def grid(self):
        return ObservedMap(20, 20, 1., (-10., -10.))

    def test_free_ray_hit_and_unknown_off_ray(self):
        grid = self.grid()
        self.assertTrue(self.feed(grid, [5., math.nan, math.nan, math.nan]))
        self.assertEqual(grid.cells[grid.index(3., 0.)], 0)
        self.assertEqual(grid.cells[grid.index(5., 0.)], 100)
        self.assertEqual(grid.cells[grid.index(0., 4.)], -1)

    def test_no_return_does_not_fill_a_free_disk(self):
        grid = self.grid()
        self.feed(grid, [math.inf, math.nan, math.nan, math.nan])
        self.assertEqual(grid.cells[grid.index(7., 0.)], 0)
        self.assertEqual(grid.cells[grid.index(3., 3.)], -1)
        self.assertEqual(grid.cells[grid.index(9., 0.)], -1)

    def test_old_evidence_expires_to_unknown(self):
        grid = self.grid()
        self.feed(grid, [5., math.nan, math.nan, math.nan])
        self.assertTrue(all(v == -1 for v in grid.snapshot(14.)))

    def test_unaligned_stale_future_or_replayed_scan_rejected(self):
        grid = self.grid()
        for override in (dict(pose_s=9.), dict(now_s=11.), dict(now_s=9.)):
            self.assertFalse(self.feed(grid, [5.]*4, **override))
        self.feed(grid, [5.]*4)
        self.assertFalse(self.feed(grid, [5.]*4))

    def test_endpoint_occupancy_wins_over_same_frame_free_ray(self):
        grid = self.grid()
        self.feed(grid, [2., 6., math.nan, math.nan], angle_increment=.001)
        self.assertEqual(grid.cells[grid.index(2., 0.)], 100)

    def test_later_long_ray_preserves_recent_hit_and_original_age(self):
        grid = self.grid()
        self.feed(grid, [2., math.nan, math.nan, math.nan])
        hit = grid.index(2., 0.)
        for t in (10.5, 11., 12., 13.):
            self.feed(grid, [7., math.nan, math.nan, math.nan], scan_s=t, pose_s=t, now_s=t)
            self.assertEqual(grid.cells[hit], 100)
            self.assertEqual(grid.observed_s[hit], 10.)
        self.assertEqual(grid.snapshot(13.01)[hit], -1)

    def test_expired_hit_needs_new_free_ray_and_new_hit_renews_age(self):
        grid = self.grid()
        self.feed(grid, [2., math.nan, math.nan, math.nan])
        hit = grid.index(2., 0.)
        self.feed(grid, [math.nan]*4, scan_s=14., pose_s=14., now_s=14.)
        self.assertEqual(grid.snapshot(14.)[hit], -1)
        self.feed(grid, [7., math.nan, math.nan, math.nan], scan_s=14.5, pose_s=14.5, now_s=14.5)
        self.assertEqual(grid.cells[hit], 0)
        self.assertEqual(grid.observed_s[hit], 14.5)
        self.feed(grid, [2., math.nan, math.nan, math.nan], scan_s=15., pose_s=15., now_s=15.)
        self.assertEqual(grid.cells[hit], 100)
        self.assertEqual(grid.observed_s[hit], 15.)
