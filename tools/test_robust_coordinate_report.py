"""Causal report regressions: depth spikes, fast motion and real log replay."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'coordination/src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class RobustCoordinateReportTests(unittest.TestCase):
    def test_one_depth_spike_is_not_a_turn_at_slower_image_cadences(self):
        for dt in (.2, .4, .8):
            for spike in (.8, 1.1, 1.6):
                with self.subTest(dt=dt, spike=spike):
                    c = bridge.TargetBridgeCore()
                    for i in range(12):
                        c.report(100.+i*.2, 'green', 20., 10., .9, 8., source_uav='u0')
                    last = c.tracks['green'].good_t
                    c.report(last+dt, 'green', 20., 10.+spike, .9, 8., source_uav='u0')
                    point = bridge.official_point_at(c.tracks['green'], last+dt)
                    # At .8 s the previous image is expired: silence is safer
                    # than declaring the lone new depth measurement reliable.
                    if point is not None:
                        self.assertEqual(point, (20., 10.))
                    self.assertEqual(c.tracks['green'].good_t, last)
                    c.report(last+dt+.2, 'green', 20., 10., .9, 8., source_uav='u0')
                    self.assertEqual(c.tracks['green'].good_t, last+dt+.2)

    def test_three_new_turn_images_reanchor_without_waiting_for_old_track_expiry(self):
        c = bridge.TargetBridgeCore()
        for i in range(12):
            t = 100.+i*.2
            c.report(t, 'blue', 2.*(t-100.), 0., .9, 8., source_uav='u0')
        last = c.tracks['blue'].good_t
        corner_x = c.tracks['blue'].good_x
        for dt in (.4, .6):
            c.report(last+dt, 'blue', corner_x, 2.*dt, .9, 8., source_uav='u0')
            self.assertEqual(c.tracks['blue'].good_t, last)
        c.report(last+.8, 'blue', corner_x, 1.6, .9, 8., source_uav='u0')
        tr = c.tracks['blue']
        self.assertEqual(tr.good_t, last+.8)
        point = bridge.official_point_at(tr, last+.9)
        self.assertAlmostEqual(point[0], corner_x)
        # This .4 s segment qualifies fresh coordinates, but is still too
        # short for extrapolation; retain the actual last image temporarily.
        self.assertAlmostEqual(point[1], 1.6)
        c.report(last+1.2, 'blue', corner_x, 2.4, .9, 8., source_uav='u0')
        self.assertAlmostEqual(bridge.official_point_at(tr, last+1.3)[1], 2.6)
        self.assertIsNone(bridge.official_point_at(tr, last+1.81))

    def test_three_inconsistent_depth_errors_cannot_reanchor(self):
        c = bridge.TargetBridgeCore()
        for i in range(12):
            c.report(100.+i*.2, 'green', 20., 10., .9, 8., source_uav='u0')
        last = c.tracks['green'].good_t
        for dt, y in ((.4, 11.1), (.6, 11.8), (.8, 11.1)):
            c.report(last+dt, 'green', 20., y, .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['green'].good_t, last)
        self.assertIsNone(bridge.official_point_at(c.tracks['green'], last+.8))

    def test_single_depth_spike_does_not_replace_stationary_coordinate(self):
        c = bridge.TargetBridgeCore()
        for i in range(12):
            c.report(100.+i*.2, 'green', 20., 10., .9, 8., source_uav='u0')
        last = c.tracks['green'].good_t
        c.report(last+.2, 'green', 20., 11.1, .9, 8., source_uav='u0')
        tr = c.tracks['green']
        self.assertEqual(tr.good_t, last)
        self.assertEqual(bridge.official_point_at(tr, last+.3), (20., 10.))
        self.assertIsNone(bridge.official_point_at(tr, last+.61))
        c.report(last+.4, 'green', 20., 10., .9, 8., source_uav='u0')
        self.assertEqual(tr.good_t, last+.4)

    def test_noisy_fast_walk_preserves_velocity_and_report_time(self):
        for speed in (.5, 1., 2.):
            c = bridge.TargetBridgeCore()
            for i in range(18):
                t = 100.+i*.2
                # An isolated depth error is a measurement, never a new time.
                error = 1.2 if i == 14 else 0.
                c.report(t, 'blue', speed*(t-100.)+error, 10., .9, 8., source_uav='u0')
                if i >= 7:
                    xy = bridge.official_point_at(c.tracks['blue'], t+.1)
                    self.assertIsNotNone(xy)
                    self.assertLess(abs(xy[0]-speed*(t+.1-100.)), .15)
            self.assertAlmostEqual(c.tracks['blue'].good_t, 103.4)

    def test_robust_inlier_time_not_latest_bad_image_limits_prediction(self):
        tr = bridge._Track('white')
        tr.good_history = [(100.+i*.2, float(i)*.2, 0.) for i in range(10)]
        tr.good_history.append((102., 3.2, 0.))
        tr.good_x, tr.good_y, tr.good_t = 3.2, 0., 102.
        self.assertAlmostEqual(bridge.official_point_at(tr, 102.1)[0], 2.1)
        self.assertIsNone(bridge.official_point_at(tr, 102.2))
        self.assertIsNone(bridge.coherent_official_velocity(tr.good_history, 102.))

    def test_consistent_slow_cadence_can_reanchor_but_not_across_missing_images(self):
        c = bridge.TargetBridgeCore()
        for t in (100., 100.8, 101.6):
            c.report(t, 'white', t-100., 0., .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['white'].good_t, 101.6)
        # A remembered trajectory at another place cannot qualify one image.
        c.report(106., 'white', 30., 0., .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['white'].good_t, 101.6)
        self.assertIsNone(bridge.official_point_at(c.tracks['white'], 106.))
        c.report(106.4, 'white', 30., 0., .9, 8., source_uav='u0')
        c.report(106.8, 'white', 30., 0., .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['white'].good_t, 101.6)
        for t in (107.2, 107.6, 108.):
            c.report(t, 'white', 30., 0., .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['white'].good_t, 108.)

    def test_short_visual_gap_does_not_demand_three_more_images_of_known_walk(self):
        c = bridge.TargetBridgeCore()
        for i in range(8):
            t = 100.+i*.2
            c.report(t, 'blue', 2.*(t-100.), 0., .9, 8., source_uav='u0')
        self.assertIsNone(bridge.official_point_at(c.tracks['blue'], 102.4))
        c.report(102.6, 'blue', 5.2, 0., .9, 8., source_uav='u0')
        self.assertEqual(c.tracks['blue'].good_t, 102.6)
        # Reacquisition consumes this actual image; it did not fill the gap.
        point = bridge.official_point_at(c.tracks['blue'], 102.6)
        self.assertAlmostEqual(point[0], 5.2)
        self.assertAlmostEqual(point[1], 0.)

    def test_recorded_green_no_longer_switches_to_over_one_metre_spikes(self):
        samples = json.loads((ROOT/'tools/fixtures/green_coordinate_084026.json').read_text())['samples']
        c = bridge.TargetBridgeCore()
        previous = None
        jumps, valid = [], 0
        for r in samples:
            c.report(r['t'], 'green', r['x'], r['y'], .9, 8., source_uav='typhoon_h480_3')
            tr = c.tracks['green']
            point = bridge.official_point_at(tr, r['t'])
            self.assertIsNone(bridge.official_point_at(tr, tr.good_t+.61))
            if point is None:
                continue
            valid += 1
            if previous is not None and r['t']-previous[0] < .5:
                jumps.append(math.hypot(point[0]-previous[1][0], point[1]-previous[1][1]))
            previous = r['t'], point
        # Baseline: 557/560 usable, largest jump 1.182m, three jumps >1m.
        self.assertGreaterEqual(valid, 550)
        self.assertLess(max(jumps), .85)

    def test_recorded_blue_recovery_does_not_lock_out_usable_images(self):
        reports = json.loads((ROOT/'tools/fixtures/tracking_reports_090835.json').read_text())['reports']
        c = bridge.TargetBridgeCore()
        eligible = accepted = 0
        for r in reports:
            c.report(r['t'], r['tag'], r['x'], r['y'], r['conf'],
                     r['range_m'], r['official_ok'], r['source'])
            if r['tag'] == 'blue' and r['t'] < 2600 and r['official_ok']:
                eligible += 1
                accepted += c.tracks['blue'].accepted_source_samples.get(r['source']) == r['t']
        self.assertEqual(eligible, 233)
        # Baseline accepted 199; this checks actual joined original images.
        self.assertGreaterEqual(accepted, 218)


class GeometryHandoffTests(unittest.TestCase):
    def feed(self, quality=.2, jump=1.2, tag='blue', count=12):
        c = bridge.TargetBridgeCore()
        for i in range(count):
            t = 100.+i*.2
            c.report(t, tag, 0., 0., .9, 9., source_uav='a', position_uncertainty_m=.8)
            c.report(t+.01, tag, jump, 0., .9, 6., source_uav='b', position_uncertainty_m=quality)
        return c

    def test_better_geometry_can_replace_live_smooth_biased_source(self):
        c = self.feed()
        tr = c.tracks['blue']
        self.assertEqual(tr.official_source, 'b')
        self.assertAlmostEqual(bridge.official_point_at(tr, 102.3)[0], 1.2)
        self.assertTrue(all(abs(p[1]-1.2) < 1e-6 for p in tr.good_history))

    def test_no_switch_from_single_frame_equal_quality_or_large_disagreement(self):
        for kwargs in (dict(count=4), dict(quality=.8), dict(jump=2.), dict(tag='red1')):
            c = self.feed(**kwargs)
            self.assertEqual(c.tracks[kwargs.get('tag','blue')].official_source, 'a')

    def test_missing_quality_does_not_guess_which_camera_is_right(self):
        self.assertEqual(self.feed(quality=None).tracks['blue'].official_source, 'a')


if __name__ == '__main__':
    unittest.main()
