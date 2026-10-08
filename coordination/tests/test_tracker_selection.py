import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from tracker_selection import tracker_rank, takeover_candidate, held_position


class TrackerSelectionTests(unittest.TestCase):
    def test_camera_takeover_intent_requires_lost_owner_and_fresh_in_range_observer(self):
        choices = [('candidate', 10.), ('blind', 4.)]
        observations = [('owner', 8.4), ('candidate', 9.6)]
        self.assertEqual(takeover_candidate('owner', choices, observations, 10., 15.), 'candidate')
        for stamp in (8.5, 9.6):
            self.assertIsNone(takeover_candidate('owner', choices,
                [('owner', stamp), ('candidate', 9.6)], 10., 15.))
        for stamp in (8.9, 10.1, float('nan')):
            self.assertIsNone(takeover_candidate('owner', choices, [('candidate', stamp)], 10., 15.))
        self.assertIsNone(takeover_candidate('owner', [('candidate', 20.)], observations, 10., 15.))

    def choose(self, observations):
        candidates = [('near',5.),('observer',11.)]
        return min(candidates,key=lambda v:tracker_rank(v[0],v[1],observations,10.,15.))[0]

    def test_visible_aircraft_keeps_target_instead_of_blind_nearest(self):
        self.assertEqual(self.choose([('observer',9.6)]),'observer')

    def test_stale_future_or_invalid_evidence_does_not_prefer_observer(self):
        for stamp in (8.9,10.1,float('nan')):
            self.assertEqual(self.choose([('observer',stamp)]),'near')

    def test_observer_outside_dispatch_range_does_not_displace_near_aircraft(self):
        self.assertLess(tracker_rank('near',5.,[('far',9.8)],10.,15.),
                        tracker_rank('far',20.,[('far',9.8)],10.,15.))

    def test_fresh_distant_candidate_keeps_its_camera_for_approach(self):
        seen=[('camera',9.8)]
        self.assertLess(tracker_rank('camera',37.5,seen,10.,20.,seen),
                        tracker_rank('blind',42.5,seen,10.,20.,seen))
        for stamp,distance in [(8.9,37.5),(10.1,37.5),(9.8,45.1)]:
            self.assertEqual(tracker_rank('camera',distance,[('camera',stamp)],10.,20.,
                            [('camera',stamp)])[0],1)
        self.assertEqual(tracker_rank('camera',37.5,[],10.,20.,seen)[0],1)

    def test_hold_is_off_by_default_and_returns_nothing_then(self):
        # hold_s<=0 must reproduce the original behaviour: no held position at all.
        for hold in (0.0, -1.0):
            self.assertIsNone(held_position((3., 4., 100.0), 105.0, hold))

    def test_hold_returns_the_remembered_position_inside_the_window(self):
        self.assertEqual(held_position((3., 4., 100.0), 101.0, 8.0), (3., 4.))
        self.assertEqual(held_position((3., 4., 100.0), 108.0, 8.0), (3., 4.))

    def test_hold_expires_and_refuses_negative_or_undefined_age(self):
        self.assertIsNone(held_position((3., 4., 100.0), 108.1, 8.0))
        self.assertIsNone(held_position((3., 4., 100.0), 99.0, 8.0))
        self.assertIsNone(held_position((3., 4., 100.0), None, 8.0))

    def test_hold_rejects_missing_or_non_finite_state(self):
        self.assertIsNone(held_position(None, 101.0, 8.0))
        self.assertIsNone(held_position((float('nan'), 4., 100.0), 101.0, 8.0))
        self.assertIsNone(held_position((3., 4., float('inf')), 101.0, 8.0))
        self.assertIsNone(held_position((3., 4.), 101.0, 8.0))
