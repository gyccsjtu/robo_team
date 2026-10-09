import sys
import math
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'perception'))
from recent_motion import RecentMotion


class RecentMotionTests(unittest.TestCase):
    def test_short_false_drift_is_not_enough_blue_motion_evidence(self):
        motion=RecentMotion(10.,3.)
        for i in range(21):motion.observe(i*.1,10.+i*.025,5.)
        self.assertEqual(motion.speed(2.),0.)

    def test_blue_true_motion_becomes_valid_after_measured_span(self):
        motion=RecentMotion(10.,3.)
        for i in range(41):motion.observe(i*.1,float(i)*.1,0.)
        self.assertAlmostEqual(motion.speed(4.),1.)
        self.assertEqual(motion.speed(5.1),0.)

    def test_longer_window_rejects_repeated_stationary_drift(self):
        short=RecentMotion();long=RecentMotion(10.,3.)
        for i in range(101):
            t=i*.1;x=10.+.4*math.sin(t*math.pi/4.)
            short.observe(t,x,5.);long.observe(t,x,5.)
        self.assertGreater(short.speed(10.),.15)
        self.assertLess(long.speed(10.),.15)

    def test_invalid_motion_span_rejected(self):
        for span in [.5,11.,float('nan')]:
            with self.assertRaises(ValueError):RecentMotion(10.,span)

    def test_initial_motion_cannot_keep_stationary_false_target_valid_forever(self):
        motion=RecentMotion()
        for i in range(101):
            t=i*.1
            motion.observe(t,min(t,2.),0.)
        self.assertLess(motion.speed(10.),.01)

    def test_moving_person_and_delayed_original_samples(self):
        motion=RecentMotion()
        for i in range(41):
            t=i*.1;motion.observe(t,t,2*t)
        self.assertAlmostEqual(motion.speed(4.3),5**.5)
        self.assertEqual(motion.speed(5.1),0.)

    def test_static_jitter_does_not_gain_lifetime_motion_credit(self):
        motion=RecentMotion()
        for i in range(81):motion.observe(i*.1,10.+(.1 if i%2 else -.1),5.)
        self.assertLess(motion.speed(8.),.02)

    def test_replays_and_coasting_cannot_refresh_evidence(self):
        motion=RecentMotion();motion.observe(1.,0.,0.);motion.observe(2.,1.,0.)
        self.assertFalse(motion.observe(2.,100.,100.))
        self.assertFalse(motion.observe(1.5,100.,100.))
        self.assertEqual(motion.speed(3.1),0.)
