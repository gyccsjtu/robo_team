"""Image-time coordinates, single prediction, and stale-report regression tests."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
from perception import perception_real as vision
import yolo_target_bridge as bridge


class CoordinateTimeTests(unittest.TestCase):
    def test_camera_report_is_measurement_even_if_filter_coasts_and_leads(self):
        tr = vision.Track('brown', 10., 20., .9, (376, 380), (40, 100),
                          8., 1.7, 100.2, observation_s=100.)
        tr.vx = 2.
        with patch.object(vision, 'LAG_COMP', .3), patch.object(vision, 'PUB_EMA', 0.):
            self.assertAlmostEqual(tr.pub_xy()[0], 10.6)
            self.assertEqual(tr.observation_xy(), (10., 20.))
            tr.coast(.4)
            self.assertEqual(tr.observation_xy(), (10., 20.))
        self.assertEqual(tr.observed_s, 100.)

    def test_camera_to_report_uses_only_actual_elapsed_image_time(self):
        core = bridge.TargetBridgeCore()
        tr = vision.Track('brown', 10., 20., .9, (376, 380), (40, 100),
                          8., 1.7, 100.2, observation_s=100.)
        for i in range(12):
            image = 100. + .4*i
            if i:
                tr.update(10.+2.*(image-100.), 20., .4, .9,
                          (376, 380), (40, 100), 8., 1.7, image+.2,
                          observation_s=image)
            x, y = tr.observation_xy()
            core.report(image, 'brown', x, y, .95, 8., source_uav='u0')
        last_image = tr.observed_s
        target = core.tracks['brown']
        for age in (0., .1, .3, .5):
            ev = next(e for e in core.tick(last_image+age) if e['tag'] == 'brown')
            self.assertAlmostEqual(ev['official_x'], 10.+2.*(last_image+age-100.))
            self.assertAlmostEqual(ev['official_y'], 20.)
            self.assertEqual(ev['official_sample_s'], last_image)
            self.assertEqual(target.good_t, last_image)
        self.assertIsNone(bridge.official_point_at(target, last_image+.61))

    def test_half_metre_walk_and_short_one_metre_walk_are_not_stationary(self):
        for speed, times in ((.5, [100.+i*.4 for i in range(6)]),
                             (1., [100.+i*.3 for i in range(4)])):
            with self.subTest(speed=speed):
                history = [(t, speed*(t-100.), 0.) for t in times]
                self.assertIsNone(bridge.stable_official_point(history, times[-1]))
                velocity = bridge.coherent_official_velocity(history, times[-1])
                self.assertIsNotNone(velocity)
                self.assertAlmostEqual(velocity[0], speed)

    def test_regular_and_keepalive_share_prediction_and_expiry(self):
        for speed in (0., .5, 2.):
            b = bridge._mk_bridge()
            for i in range(7):
                t = 100.+i*.3
                b.core.report(t, 'brown', 10.+speed*(t-100.), 20., .95, 8.)
            last = b.core.tracks['brown'].good_t
            b._advance(last+.25)
            b._gate_live['brown'] = True
            point = bridge.official_point_at(b.core.tracks['brown'], b._now())
            ev = next(e for e in b.core.tick(b._now()) if e['tag'] == 'brown')
            self.assertEqual((ev['official_x'], ev['official_y']), point)
            b._keepalive(b._now())
            self.assertEqual(len(b._actor_pubs['brown'].msgs), 1)
            self.assertAlmostEqual(b._actor_pubs['brown'].msgs[0].x, point[0], places=3)
            b._advance(3.)
            b._keepalive(b._now())
            self.assertEqual(len(b._actor_pubs['brown'].msgs), 1)
            self.assertIsNone(bridge.official_point_at(b.core.tracks['brown'], b._now()))

    def test_noise_and_turn_do_not_extend_unmodelled_prediction(self):
        core = bridge.TargetBridgeCore()
        # A single changed point cannot prove whether it is a turn or depth
        # noise. Wait for more images instead of declaring the raw point good.
        t = core.tracks['brown']
        t.good_history = [(100., 0., 0.), (100.4, .8, 0.),
                          (100.8, 1.6, 0.), (101.2, 1.6, .8)]
        t.good_x, t.good_y, t.good_t = 1.6, .8, 101.2
        self.assertIsNone(bridge.coherent_official_velocity(t.good_history, t.good_t))
        self.assertIsNone(bridge.official_point_at(t, 101.3))
        self.assertIsNone(bridge.official_point_at(t, 101.6))
        self.assertIsNone(bridge.official_point_at(t, 101.1))

    def test_camera_handoff_does_not_fit_motion_between_camera_biases(self):
        core = bridge.TargetBridgeCore()
        for i in range(8):
            core.report(100.+i*.2, 'white', 20., 10., .95, 8., source_uav='u0')
        for i in range(4):
            core.report(101.5+i*.2, 'white', 20.3, 10., .95, 8., source_uav='u1')
        tr = core.tracks['white']
        self.assertEqual(tr.official_source, 'u1')
        self.assertEqual(len(tr.good_history), 1)
        self.assertIsNone(bridge.coherent_official_velocity(tr.good_history, tr.good_t))


if __name__ == '__main__':
    unittest.main()
