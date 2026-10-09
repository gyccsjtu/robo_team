import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/robocup_swarm/scripts'))
from fleet_motion_guard import MotionCache, protect


def sample(uid, position, velocity=(0., 0.), stamp=10., seq=1):
    return dict(schema_version=1, run_id='r', uav_id=uid, seq=seq, sample_s=stamp,
                frame='world_enu_xy', position_xy=position, velocity_xy=velocity)


class MotionTests(unittest.TestCase):
    def cache(self, peer, velocity=(0., 0.), own_velocity=(0., 0.)):
        result = MotionCache('r', ['a', 'b'])
        result.receive(sample('a', (0., 0.), own_velocity), 10.)
        result.receive(sample('b', peer, velocity), 10.)
        return result

    def test_crossing_request_stops_before_contact(self):
        result = protect((1., 0.), 'a', self.cache((6., 3.), (-1., 0.)), 10.)
        self.assertEqual(result['velocity_xy'], (0., 0.))

    def test_fast_actual_approach_cannot_be_hidden_by_zero_request(self):
        result = protect((0., 0.), 'a', self.cache((7., 0.), (-1., 0.), (1., 0.)), 10.)
        self.assertEqual(result['reason'], 'RELATIVE_BRAKING_REQUIRED')

    def test_safe_departure_is_allowed(self):
        result = protect((-1., 0.), 'a', self.cache((6., 0.)), 10.)
        self.assertEqual(result['velocity_xy'], (-1., 0.))

    def test_missing_or_stale_member_stops(self):
        cache = self.cache((20., 0.))
        self.assertEqual(protect((1., 0.), 'a', cache, 10.6)['velocity_xy'], (0., 0.))
        del cache.samples['b']
        self.assertEqual(protect((1., 0.), 'a', cache, 10.)['velocity_xy'], (0., 0.))

    def test_old_run_future_and_replayed_samples_rejected(self):
        cache = self.cache((20., 0.))
        self.assertFalse(cache.receive(sample('b', (20., 0.)), 10.))
        self.assertFalse(cache.receive(sample('b', (20., 0.), stamp=11., seq=2), 10.))
        bad = sample('b', (20., 0.), stamp=10.1, seq=2)
        bad['run_id'] = 'other'
        self.assertFalse(cache.receive(bad, 10.1))

    def test_sample_age_is_used_for_approach(self):
        cache = self.cache((5.6, 0.), (-1., 0.))
        self.assertEqual(protect((1., 0.), 'a', cache, 10.4)['velocity_xy'], (0., 0.))
