"""Read-only gap images cannot refresh visual evidence or evade capture caps."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from capture_visual_frames import GapCapture


class VisualGapCaptureTests(unittest.TestCase):
    def setUp(self):
        self.capture = GapCapture('white', total_limit=3, per_gap_limit=2)
        self.observation = dict(schema_version=2, run_id='run', uav_id='uav_6',
                                target_id='white', sample_s=10., xyz=[1., 2., 3.])
        self.capture.observe(self.observation, 10.5)

    def test_gap_has_original_time_and_last_observation_but_no_new_evidence(self):
        record = self.capture.select('uav_6', 11., 12.1)
        self.assertEqual(record['kind'], 'gap_image')
        self.assertEqual(record['image_sample_s'], 11.)
        self.assertEqual(record['last_observation'], self.observation)
        self.assertNotIn('observation', record)
        self.assertFalse(record['control_input'])
        self.assertEqual(self.capture.latest['uav_6']['sample_s'], 10.)

    def test_healthy_stream_and_processing_delay_are_not_gap_images(self):
        self.assertIsNone(self.capture.select('uav_6', 10.5, 11.7))
        self.assertIsNone(self.capture.select('uav_6', 11., 11.5))
        self.capture.observe(dict(self.observation, sample_s=11.), 11.6)
        self.assertIsNone(self.capture.select('uav_6', 11., 12.1))

    def test_future_old_unobserved_and_paused_clock_frames_do_not_invent_images(self):
        for uid, stamp, now in [('uav_5', 11., 12.1), ('uav_6', 12., 11.),
                                 ('uav_6', 11., 20.), ('uav_6', 13.1, 14.2)]:
            self.assertIsNone(self.capture.select(uid, stamp, now))
        self.assertIsNotNone(self.capture.select('uav_6', 11., 12.1))
        for _ in range(3):self.assertIsNone(self.capture.select('uav_6', 11., 12.1))

    def test_out_of_order_duplicate_expired_and_future_observations_cannot_reset_cap(self):
        self.capture.select('uav_6', 11., 12.1)
        self.capture.select('uav_6', 11.2, 12.3)
        for sample, now in [(10., 10.7), (9.9, 10.7), (11., 12.1), (13., 12.1)]:
            self.capture.observe(dict(self.observation, sample_s=sample), now)
        self.assertIsNone(self.capture.select('uav_6', 11.4, 12.5))
        self.assertEqual(self.capture.latest['uav_6']['sample_s'], 10.)

    def test_total_limit_survives_new_gaps_and_other_aircraft(self):
        for stamp in [11., 11.2]:
            self.assertIsNotNone(self.capture.select('uav_6', stamp, stamp+1.1))
        self.capture.observe(dict(self.observation, sample_s=12.), 12.5)
        self.assertIsNotNone(self.capture.select('uav_6', 13., 14.1))
        self.capture.observe(dict(self.observation, uav_id='uav_5', sample_s=14.), 14.5)
        self.assertIsNone(self.capture.select('uav_5', 15., 16.1))
