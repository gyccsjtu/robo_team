import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'perception'))
from recent_motion import RecentMotion


class RecentMotionTests(unittest.TestCase):
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
