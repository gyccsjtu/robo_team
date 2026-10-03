import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from tracker_selection import tracker_rank


class TrackerSelectionTests(unittest.TestCase):
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
