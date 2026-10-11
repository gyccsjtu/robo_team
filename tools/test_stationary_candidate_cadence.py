"""Stationary-world evidence must survive the 5 -> 8 Hz inference change."""
import math
import unittest

from perception import perception_real as pr


class StationaryCandidateCadenceTests(unittest.TestCase):
    def track(self, samples):
        last = samples[-1]
        tr = pr.Track('blue', last[1], last[2], .9, (376., 400.),
                      (45., 100.), 8., 1.7, last[0])
        tr.recent_observed_xy = list(samples)
        tr.observed_s = last[0]
        return tr

    def series(self, hz, speed=0.):
        return [(100. + i / hz, 8. + speed * i / hz, .02 * math.sin(i))
                for i in range(int(1.5 * hz) + 1)]

    def test_stationary_world_point_survives_camera_cadence_changes(self):
        for hz in (5, 8, 10):
            with self.subTest(hz=hz):
                tr = self.track(self.series(hz))
                self.assertTrue(tr.stationary_candidate_stable())
                # Rotation can give small camera-relative displacement even
                # for a real stationary object; world evidence rules that out.
                tr.d_view, tr.d_rel = 2.5, .1
                self.assertFalse(tr.attached_to_cam())

    def test_moving_point_and_camera_attached_false_box_keep_protection(self):
        for hz in (5, 8, 10):
            for speed in (1., 2.):
                with self.subTest(hz=hz, speed=speed):
                    tr = self.track(self.series(hz, speed=speed))
                    self.assertFalse(tr.stationary_candidate_stable())
                    # A fixed camera/body artifact moves in world coordinates
                    # with the moving UAV while its relative position stays fixed.
                    tr.d_view, tr.d_rel = 3., 0.
                    self.assertTrue(tr.attached_to_cam())

    def test_initial_motion_does_not_poison_a_new_stationary_suffix(self):
        for hz in (5, 8, 10):
            points = self.series(hz)
            points[:3] = [(p[0], p[1] + 3., p[2]) for p in points[:3]]
            self.assertTrue(self.track(points).stationary_candidate_stable())

    def test_independent_images_and_elapsed_time_are_both_required(self):
        for stamps in ((100., 100.3, 100.6),
                       (100., 100.2, 100.2, 100.6),
                       (100., 100.3, 100.2, 100.6),
                       (100., 100.1, 100.2, 100.3)):
            with self.subTest(stamps=stamps):
                tr = self.track([(t, 8., 0.) for t in stamps])
                self.assertFalse(tr.stationary_candidate_stable())

    def test_image_outage_and_stale_image_do_not_supply_stationary_evidence(self):
        tr = self.track([(t, 8., 0.) for t in (100., 100.2, 101., 101.2)])
        self.assertFalse(tr.stationary_candidate_stable())
        for hz in (5, 8, 10):
            tr = self.track(self.series(hz))
            tr.t += .501
            self.assertFalse(tr.stationary_candidate_stable())

    def test_position_spread_and_nonfinite_observations_remain_rejected(self):
        for points in ([(100., 8., 0.), (100.2, 9., 0.),
                        (100.4, 8., 0.), (100.6, 8., 0.)],
                       [(100., 8., 0.), (100.2, math.nan, 0.),
                        (100.4, 8., 0.), (100.6, 8., 0.)]):
            self.assertFalse(self.track(points).stationary_candidate_stable())


if __name__ == '__main__':
    unittest.main()
