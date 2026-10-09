import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from flight_actor_probe import FlightActorProbe


class FlightActorEvidenceTests(unittest.TestCase):
    def observer(self):
        probe = FlightActorProbe.__new__(FlightActorProbe)
        probe.actor_spawned, probe.actor_error = True, None
        probe.visual, probe.confirmed = list(range(5)), list(range(3))
        probe.target_grants = [dict(uav_id='uav_1', generation=2, expires_s=13.)]
        probe.routes = [dict(uav_id='uav_1', generation=2)]
        probe.motion = [('uav_1', 2, 10.), ('uav_1', 2, 12.)]
        return probe

    def truth(self, stamps=(10., 12.)):
        return [dict(sim_s=s, positions={'uav_1': [i, 0., 4.5]}) for i, s in enumerate(stamps)]

    def test_actual_matching_execution_and_displacement_required(self):
        self.assertTrue(self.observer().summary(self.truth())['physical_visual_tracking_verified'])
        probe = self.observer()
        probe.motion = []
        self.assertFalse(probe.summary(self.truth())['physical_visual_tracking_verified'])

    def test_other_generation_motion_and_later_search_motion_are_not_evidence(self):
        probe = self.observer()
        probe.motion = [('uav_1', 1, 10.), ('uav_1', 1, 12.)]
        self.assertFalse(probe.summary(self.truth())['physical_visual_tracking_verified'])
        self.assertFalse(self.observer().summary(self.truth((20., 22.)))['physical_visual_tracking_verified'])

    def test_unmatched_route_and_missing_confirmed_frames_cannot_pass(self):
        probe = self.observer()
        probe.routes = [dict(uav_id='uav_1', generation=1)]
        self.assertFalse(probe.summary(self.truth())['physical_visual_tracking_verified'])
        probe = self.observer()
        probe.confirmed = []
        self.assertFalse(probe.summary(self.truth())['physical_visual_tracking_verified'])
