"""Image motion estimates must not depend on empty perception-loop cadence."""
import copy
import math
import unittest
from perception import perception_real as pr


def replay_track(loop_hz=0, speed=1., turn=False, dropped=(), latency=.2):
    track = pr.Track('white', 0., 0., .9, (376,400), (40,100),
                     8., 1.7, 100.+latency, observation_s=100.)
    states = []
    for i in range(1, 41):
        image = 100.+i*.25
        receipt = image+latency
        if loop_hz:
            while track.t+1./loop_hz < receipt-1e-6:
                track.coast(1./loop_hz)
        if i in dropped:
            continue
        elapsed = image-100.
        x = speed*(min(elapsed, 5.) if turn else elapsed)
        y = speed*max(0.,elapsed-5.) if turn else 0.
        x += .1 if i == 20 else 0.
        accepted = track.update(x, y, receipt-track.t, .9, (376,400), (40,100),
                               8., 1.7, receipt, observation_s=image)
        if not accepted:
            raise AssertionError('fresh image rejected')
        states.append((track.x,track.y,track.vx,track.vy))
    return track, states


class TrackObservationClockTests(unittest.TestCase):
    def test_straight_motion_and_noise_do_not_depend_on_empty_loop_rate(self):
        _, reference = replay_track()
        for hz in (15, 30, 100):
            _, states = replay_track(hz)
            for expected, actual in zip(reference, states):
                for a,b in zip(expected, actual): self.assertAlmostEqual(a,b,places=8)
            self.assertLess(max(math.hypot(p[2],p[3]) for p in states), 1.07)
            self.assertGreater(min(p[2] for p in states), 0.)

    def test_turns_and_missing_images_use_the_full_observation_interval(self):
        for dropped in ((), (19,), (19,20)):
            _, expected = replay_track(turn=True, dropped=dropped)
            for hz in (15,30,100):
                _, actual = replay_track(hz, turn=True, dropped=dropped)
                for a,b in zip(expected,actual):
                    for x,y in zip(a,b): self.assertAlmostEqual(x,y,places=8)

    def test_clock_spacing_does_not_retimestamp_the_raw_measurement(self):
        tr,_ = replay_track(100, dropped=(38,39))
        self.assertEqual(tr.observed_s,110.)
        self.assertEqual(tr.observation_xy(),(10.,0.))
        history = copy.deepcopy(tr.recent_observed_xy)
        tr.coast(.4)
        self.assertEqual(tr.observed_s,110.)
        self.assertEqual(tr.observation_xy(),(10.,0.))
        self.assertEqual(tr.recent_observed_xy,history)

    def test_duplicate_and_reversed_images_cannot_change_motion(self):
        tr,_ = replay_track(30)
        before = copy.deepcopy(tr.__dict__)
        for sample in (tr.observed_s, tr.observed_s-.25):
            accepted = tr.update(80.,-70.,.2,.9,(376,400),(40,100),
                                 8.,1.7,tr.t+.2,observation_s=sample)
            self.assertFalse(accepted)
            for key in ('t','observed_s','x','y','vx','vy','hits','recent_observed_xy'):
                self.assertEqual(tr.__dict__[key], before[key])

    def test_state_prediction_still_uses_only_unadvanced_time(self):
        tr = pr.Track('blue',0.,0.,.9,(376,400),(40,100),8.,1.7,100.,observation_s=100.)
        tr.vx = 1.
        tr.coast(.20)
        tr.update(.25,0.,.05,.9,(376,400),(40,100),8.,1.7,100.25,observation_s=100.25)
        self.assertAlmostEqual(tr.x,.25)
        self.assertAlmostEqual(tr.vx,1.)


if __name__ == '__main__':
    unittest.main()
