"""Replay a cropped brown view without changing reporting or flight limits."""
import copy
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'coordination/src/robocup_swarm/scripts'))
from tracking_view import (update_altitude_hold, view_distance_floor,
                           view_height_window)


class CoupledViewGeometryTests(unittest.TestCase):
    # round_02/core_evidence.tar.xz, source uav 3, original image 2107.592.
    # This is own-camera evidence; its x/y are not used in these calculations.
    RAW = dict(target_id='t2', sample_s=2107.592, width=752., height=480.,
               fy=376., box=(358.5, 406.76, 400.1, 479.96),
               camera_tilt=.042605247317489436,
               geometry_ok=True, report_ok=False, person_verdict_ok=True)

    def view(self, elapsed=0., **changes):
        result = dict(self.RAW, sample_s=self.RAW['sample_s']+elapsed)
        result.update(changes)
        return result

    def step(self, previous, view, altitude=3.98, distance=6.48,
             preferred=8.):
        return update_altitude_hold(previous, view, altitude, distance,
                                    2.6, 4.2, preferred_range=preferred)

    def test_actual_crop_reproduces_unchanged_baseline(self):
        state = None
        for i in range(31):
            state = self.step(state, self.view(.2*i), preferred=None)
        self.assertEqual(state['goal'], 3.98)
        self.assertEqual(state['direction'], 0)
        self.assertNotIn('coupled_geometry', state)
        self.assertAlmostEqual(state['range_floor'], 10.104568968987586)
        lo, hi = view_height_window(self.view(), 6.48, 2.6, 4.2)
        self.assertGreater(lo, hi)

    def test_six_seconds_of_new_crop_images_lower_height_and_keep_safe_distance(self):
        state = None
        altitude = 3.98
        untouched = copy.deepcopy(self.RAW)
        for i in range(31):
            before = altitude
            state = self.step(state, self.view(.2*i), altitude=altitude)
            self.assertTrue(state['coupled_geometry'])
            self.assertGreaterEqual(state['range_floor'], 8.)
            self.assertGreaterEqual(state['range_floor']+1e-9,
                                    view_distance_floor(self.view(), altitude))
            self.assertLessEqual(before-state['goal'], .15*.2+1e-9)
            self.assertGreaterEqual(state['goal'], 2.6)
            altitude = state['goal']  # ideal measured response, not fake credit
        self.assertLess(state['goal'], 3.3)
        self.assertLess(state['range_floor'], 8.5)
        self.assertEqual(self.RAW, untouched)
        self.assertFalse(self.RAW['report_ok'])

    def test_target_height_does_not_preempt_measured_altitude(self):
        state = None
        for i in range(31):
            state = self.step(state, self.view(.2*i), altitude=3.98)
            self.assertAlmostEqual(state['range_floor'], 10.104568968987586)
        # Existing measured-height clamp remains intact if the aircraft has
        # not actually descended (e.g. an obstacle safety guard vetoed it).
        self.assertGreaterEqual(state['goal'], 3.98-.25-1e-9)

    def test_complete_healthy_view_is_identical_with_option_enabled(self):
        for distance in (7., 8., 9.5, 12.):
            baseline = candidate = None
            for i in range(31):
                v = self.view(.2*i, box=(340., 240., 385., 380.),
                              report_ok=True)
                baseline = self.step(baseline, v, distance=distance,
                                     preferred=None)
                candidate = self.step(candidate, v, distance=distance)
                self.assertEqual(candidate, baseline)

    def test_tilt_duplicate_and_reverse_time_do_not_integrate(self):
        state = None
        for i in range(31):
            state = self.step(state, self.view(.2*i,
                              camera_tilt=math.radians(9.)))
        self.assertEqual(state['goal'], 3.98)
        state = self.step(None, self.view())
        for i in range(6):
            state = self.step(state, self.view(.2*i))
        before = dict(state)
        for _ in range(30):
            state = self.step(state, self.view(1.))
        self.assertEqual(state, before)
        self.assertEqual(self.step(state, self.view(.8)), before)

    def test_image_gap_requires_new_continuous_pressure(self):
        state = None
        for i in range(4):
            state = self.step(state, self.view(.2*i))
        goal = state['goal']
        state = self.step(state, self.view(1.2))  # .6 s missing images
        self.assertEqual(state['goal'], goal)
        for elapsed in (1.4, 1.6):
            state = self.step(state, self.view(elapsed))
            self.assertEqual(state['goal'], goal)
        state = self.step(state, self.view(1.81))
        self.assertLess(state['goal'], goal)

    def test_after_tilt_only_new_untilted_pressure_can_start_descent(self):
        state = None
        for i in range(20):
            state = self.step(state, self.view(.2*i,
                              camera_tilt=math.radians(9.)))
        for elapsed in (4., 4.2, 4.4):
            state = self.step(state, self.view(elapsed))
            self.assertEqual(state['goal'], 3.98)
        state = self.step(state, self.view(4.61))
        self.assertLess(state['goal'], 3.98)

    def test_restored_full_view_releases_coupled_marker_and_uses_old_controller(self):
        state = None
        for i in range(10):
            state = self.step(state, self.view(.2*i))
        self.assertTrue(state['coupled_geometry'])
        healthy = self.view(2., box=(340., 240., 385., 380.), report_ok=True)
        baseline = self.step(dict(state), healthy, distance=8., preferred=None)
        candidate = self.step(dict(state), healthy, distance=8.)
        self.assertEqual(candidate, baseline)
        self.assertNotIn('coupled_geometry', candidate)

    def test_floor_head_crop_and_unqualified_identity_do_not_gain_descent(self):
        for changes in (dict(box=(358., 0., 400., 480.)),
                        dict(geometry_ok=False),
                        dict(person_verdict_ok=False),
                        dict(report_ok=True),
                        dict(box=(358., 406., 400., 450.))):
            baseline = candidate = None
            for i in range(31):
                v = self.view(.2*i, **changes)
                baseline = self.step(baseline, v, preferred=None)
                candidate = self.step(candidate, v)
                self.assertEqual(candidate, baseline)
        state = None
        for i in range(31):
            state = self.step(state, self.view(.2*i), altitude=2.6,
                              distance=2.)
            self.assertGreaterEqual(state['goal'], 2.6)
            self.assertNotIn('coupled_geometry', state)

    def test_invalid_or_infeasible_ring_is_identical_to_baseline(self):
        for preferred in (float('nan'), float('inf'), -8., 0., 5.,
                          20., 'invalid', object()):
            baseline = candidate = None
            for i in range(10):
                v = self.view(.2*i)
                baseline = self.step(baseline, v, preferred=None)
                candidate = self.step(candidate, v, preferred=preferred)
                self.assertEqual(candidate, baseline)


if __name__ == '__main__':
    unittest.main()
