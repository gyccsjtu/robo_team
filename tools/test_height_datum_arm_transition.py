"""Actual MAVROS startup replay and fail-closed ground-reference transition."""
import ast
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from height_datum import GroundHeightDatum
from test_perception_pose_source import load_pose_functions, Stamp

ROOT = Path(__file__).resolve().parents[1]


class HeightDatumArmTransitionTests(unittest.TestCase):
    def stable(self, datum, start=100.):
        for k in range(21):
            stamp = start+k*.05
            datum.observe(stamp, -.1, False, 0., now=stamp)
        self.assertIsNotNone(datum.offset)

    def test_actual_053455_pose_callback_recovers_recent_qualified_ground_before_arm(self):
        fixture = json.loads((ROOT / 'tools/fixtures/height_datum_arm_transition_053455.json').read_text())
        api = load_pose_functions()
        datum = api['_height_datum'] = GroundHeightDatum(fixture['body_height'])
        now = [0.]
        api['rospy'].Time = NS(now=lambda: Stamp(now[0]))
        grounded_failure, froze, first_armed = False, False, None
        prior_offset = None
        with patch('builtins.print'):
            for row in fixture['samples']:
                now[0] = row['ros']
                api['_height_armed'] = row['armed']
                p = row['pose']
                pose = NS(header=NS(stamp=Stamp(p['sample_s'])),
                    pose=NS(position=NS(**dict(zip(('x','y','z'),p['xyz']))),
                            orientation=NS(**dict(zip(('x','y','z','w'),p['quaternion'])))))
                if row['armed'] and first_armed is None:
                    first_armed = row
                    prior_offset = datum.last_valid_offset
                api['on_pose'](pose, (0., -3., 0.))
                if not row['armed'] and datum.last_valid_sample is not None and datum.offset is None:
                    grounded_failure = True
                    self.assertIsNone(api['pose_at_stamp'](p['sample_s']))
                if row['armed']:
                    froze = True
                    self.assertTrue(datum.frozen)
                    self.assertEqual(datum.offset, prior_offset)
                    self.assertIsNotNone(api['pose_at_stamp'](p['sample_s']))
        self.assertTrue(grounded_failure and froze)
        self.assertEqual(datum.last_valid_sample, 1925.08)
        self.assertAlmostEqual(first_armed['ros']-datum.last_valid_sample, .228)
        self.assertIsNone(api['_pose_fault'])
        self.assertEqual(api['_pose_history'][-1][0], fixture['samples'][-1]['sample'])

    def test_ground_failure_itself_still_cannot_supply_a_new_range(self):
        datum = GroundHeightDatum(.22)
        self.stable(datum)
        offset, sample = datum.offset, datum.last_valid_sample
        self.assertIsNone(datum.observe(101.1, -.2, False, 0., now=101.1))
        self.assertIsNone(datum.offset)
        self.assertEqual((datum.last_valid_offset, datum.last_valid_sample), (offset, sample))
        self.assertEqual(datum.observe(101.2, 3., True, 0., now=101.2), offset)

    def test_original_tilt_limit_keeps_current_ground_range_closed(self):
        datum = GroundHeightDatum(.22)
        self.stable(datum)
        offset = datum.offset
        self.assertIsNone(datum.observe(101.1, -.1, False, math.radians(3.01), now=101.1))
        self.assertIsNone(datum.offset)
        self.assertEqual(datum.observe(101.2, 2., True, 0., now=101.2), offset)

    def test_expired_or_long_unstable_ground_cannot_freeze_previous_reference(self):
        datum = GroundHeightDatum(.22)
        self.stable(datum)
        for k in range(1, 15):
            at = 101.+k*.05
            self.assertIsNone(datum.observe(at, -.3 if k%2 else -.1, False, 0., now=at))
        self.assertIsNone(datum.observe(101.8, 3., True, 0., now=101.8))
        self.assertTrue(datum.frozen)
        self.assertIsNone(datum.observe(102., -.1, False, 0., now=102.))

    def test_no_ground_qualification_airborne_restart_stays_closed(self):
        datum = GroundHeightDatum(.22)
        self.assertIsNone(datum.observe(100., 4., True, 0., now=100.))
        for k in range(30):
            self.assertIsNone(datum.observe(100.1+k*.05, -.1, False, 0., now=100.1+k*.05))
        self.assertIsNone(datum.last_valid_sample)

    def test_unknown_armed_state_never_refreshes_ground_candidate(self):
        datum = GroundHeightDatum(.22)
        for k in range(30):
            at = 100.+k*.05
            self.assertIsNone(datum.observe(at, -.1, None, 0., now=at))
        self.assertIsNone(datum.last_valid_sample)
        self.stable(datum, start=102.)
        sample = datum.last_valid_sample
        self.assertIsNone(datum.observe(103.1, -.1, None, 0., now=103.1))
        self.assertEqual(datum.last_valid_sample, sample)

    def test_duplicate_and_reordered_poses_cannot_refresh_or_freeze(self):
        datum = GroundHeightDatum(.22)
        self.stable(datum)
        value, sample = datum.last_valid_offset, datum.last_valid_sample
        self.assertIsNone(datum.observe(101., -.1, True, 0., now=101.1))
        self.assertFalse(datum.frozen)
        self.assertIsNone(datum.observe(100.9, -.1, False, 0., now=101.2))
        self.assertEqual((datum.last_valid_offset, datum.last_valid_sample), (value, sample))
        self.assertIsNone(datum.observe(101.3, 3., True, 0., now=101.6))

    def test_delayed_armed_pose_cannot_make_old_reference_fresh(self):
        datum = GroundHeightDatum(.22)
        self.stable(datum)
        self.assertIsNone(datum.observe(101.1, 3., True, 0., now=102.))
        self.assertTrue(datum.frozen)

    def test_clock_reset_invalidates_ground_and_frozen_references(self):
        for frozen in (False, True):
            with self.subTest(frozen=frozen):
                datum = GroundHeightDatum(.22)
                self.stable(datum)
                if frozen:
                    datum.observe(101.1, 3., True, 0., now=101.1)
                self.assertIsNone(datum.observe(10., 3., True, 0., now=10.))
                self.assertIsNone(datum.last_valid_sample)
                self.assertIsNone(datum.offset)
                self.assertIsNone(datum.observe(10.1, 3., True, 0., now=10.1))

    def test_nonfinite_sample_does_not_invent_or_refresh_qualification(self):
        for field in ('stamp', 'z', 'tilt', 'now'):
            datum = GroundHeightDatum(.22)
            self.stable(datum)
            old = datum.last_valid_sample
            values = dict(stamp=101.1, z=-.1, armed=False, tilt=0., now=101.1)
            values[field] = float('nan')
            self.assertIsNone(datum.observe(**values))
            self.assertIsNone(datum.offset)
            self.assertEqual(datum.last_valid_sample, old)

    def test_state_callback_does_not_freeze_datum_before_pose_validation(self):
        tree = ast.parse((ROOT/'perception/perception_real.py').read_text())
        callback = next(n for n in ast.walk(tree)
                        if isinstance(n, ast.FunctionDef) and n.name == 'height_state')
        self.assertFalse(any(isinstance(n, ast.Attribute) and n.attr == 'frozen'
                             for n in ast.walk(callback)))


if __name__ == '__main__':
    unittest.main()
