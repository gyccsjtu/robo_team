from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from cooperative_tracker import CooperativeTracker


class OfficialTrackerTests(unittest.TestCase):
    def test_observation_duration_does_not_invent_elimination_or_teleport(self):
        tracker = CooperativeTracker(official_only=True)
        track = tracker.add_target('t5')
        tracker.assign_observers('t5', ['a'])
        for index in range(401):
            now = index/10.
            tracker.report('a', 't5', now, 2., 3.)
            events = tracker.update(now)
            self.assertFalse(any(event in ('confirmed', 'evade') for _, event in events))
            self.assertFalse(track.eliminated)
        self.assertEqual(tracker.eliminated, [])
        self.assertIsNone(track.last_err)
