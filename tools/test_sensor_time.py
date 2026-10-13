"""Offline header-clock tests; no ROS module or node is loaded."""

import math
from pathlib import Path
import sys
import threading
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "coordination/src/robocup_swarm/scripts"))
from sensor_time import SensorTimeProvider, TimeUnavailableError


class FakeMonotonic:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class SensorTimeTests(unittest.TestCase):
    def setUp(self):
        self.wall = FakeMonotonic()
        self.clock = SensorTimeProvider(["a"], monotonic=self.wall)

    def test_startup_never_uses_wall_time_or_invalid_headers(self):
        for value in (0, -1, math.nan, math.inf, 1700000000, None):
            self.assertFalse(self.clock.observe("a", value))
        self.assertFalse(self.clock.is_valid())
        with self.assertRaises(TimeUnavailableError):
            self.clock.now()
        self.assertFalse(self.clock.observe("unknown", 1))
        self.assertTrue(self.clock.observe("a", .1))
        self.assertEqual(self.clock.now(), .1)

    def test_duplicate_headers_do_not_refresh_progress(self):
        self.clock.observe("a", 10)
        self.wall.advance(.75)
        self.assertFalse(self.clock.observe("a", 10))
        self.wall.advance(.3)
        self.assertFalse(self.clock.is_valid())
        self.assertEqual(self.clock.now(), 10)
        self.assertAlmostEqual(self.clock.status()["source_progress_age"]["a"], 1.05)

    def test_pause_and_wall_time_do_not_extrapolate(self):
        self.clock.observe("a", 10)
        self.wall.advance(30)
        self.assertEqual(self.clock.now(), 10)
        self.assertFalse(self.clock.is_valid())

    def test_simulation_rate_is_derived_from_headers(self):
        for ratio in (.1, 1., 10.):
            with self.subTest(ratio=ratio):
                wall = FakeMonotonic()
                clock = SensorTimeProvider(["a"], monotonic=wall)
                clock.observe("a", 1)
                wall.advance(.1)
                self.assertTrue(clock.observe("a", 1+ratio*.1))
                self.assertAlmostEqual(clock.now(), 1+ratio*.1)

    def test_default_timeout_accepts_twenty_wall_hz_even_at_low_realtime_factor(self):
        for ratio in (.001, .01, .15, 1.):
            with self.subTest(realtime_factor=ratio):
                wall = FakeMonotonic()
                clock = SensorTimeProvider(["a"], monotonic=wall)
                clock.observe("a", 10)
                for index in range(1, 201):
                    wall.advance(.05)
                    self.assertTrue(clock.observe("a", 10+index*.05*ratio))
                    self.assertTrue(clock.is_valid())
                self.assertAlmostEqual(clock.now(), 10+10*ratio)

    def test_lagging_source_cannot_keep_frozen_global_time_alive(self):
        clock = SensorTimeProvider(["a", "b"], monotonic=self.wall)
        clock.observe("a", 10)
        clock.observe("b", 9.8)
        for index in range(1, 23):
            self.wall.advance(.05)
            clock.observe("b", 9.8+index*.001)
        self.assertEqual(clock.now(), 10)
        self.assertFalse(clock.is_valid())

    def test_small_reordered_header_does_not_change_epoch(self):
        self.clock.observe("a", 10)
        self.wall.advance(.1)
        self.assertFalse(self.clock.observe("a", 9.9))
        self.assertEqual(self.clock.now(), 10)
        self.assertEqual(self.clock.epoch, 0)
        self.assertTrue(self.clock.is_valid())

    def test_obvious_forward_jump_is_rejected_without_poisoning_time(self):
        self.clock.observe("a", 10)
        self.wall.advance(.01)
        self.assertFalse(self.clock.observe("a", 1000))
        self.assertEqual(self.clock.now(), 10)
        self.wall.advance(.01)
        self.assertTrue(self.clock.observe("a", 10.02))

    def test_source_recovery_has_a_new_schedule_generation(self):
        self.clock.observe("a", 10)
        before = self.clock.snapshot().recovery_serial
        self.wall.advance(2)
        self.assertFalse(self.clock.is_valid())
        self.assertTrue(self.clock.observe("a", 12))
        self.assertTrue(self.clock.is_valid())
        self.assertEqual(self.clock.snapshot().recovery_serial, before+1)

    def test_healthier_source_can_take_over_and_foreign_domain_is_rejected(self):
        clock = SensorTimeProvider(["a", "b", "c"], monotonic=self.wall)
        clock.observe("a", 10)
        clock.observe("b", 10)
        self.wall.advance(.1)
        self.assertFalse(clock.observe("c", 10000))
        for sample in (10.2, 10.4, 10.6, 10.8, 11.0, 11.2):
            self.wall.advance(.2)
            self.assertTrue(clock.observe("b", sample))
        self.assertTrue(clock.is_valid())
        self.assertEqual(clock.snapshot().source, "b")

    def test_single_source_reset_requires_progress_and_callbacks_finish_before_valid(self):
        seen = []
        self.clock.add_reset_callback(lambda epoch: seen.append(
            (epoch, self.clock.now(), self.clock.is_valid())))
        self.clock.observe("a", 10)
        self.wall.advance(.1)
        self.assertFalse(self.clock.observe("a", 1))
        self.assertFalse(self.clock.is_valid())
        self.assertFalse(self.clock.observe("a", 1))
        self.wall.advance(.1)
        self.assertTrue(self.clock.observe("a", 1.1))
        self.assertEqual(seen, [(1, 1.1, False)])
        self.assertTrue(self.clock.is_valid())
        self.assertEqual(self.clock.epoch, 1)

    def test_multiple_sources_require_agreement_before_reset(self):
        clock = SensorTimeProvider(["a", "b", "c"], monotonic=self.wall)
        clock.observe("a", 100)
        clock.observe("b", 100)
        clock.observe("c", 100)
        self.wall.advance(.1)
        self.assertFalse(clock.observe("a", 1))
        self.wall.advance(.1)
        self.assertFalse(clock.observe("a", 1.1))
        self.assertEqual(clock.epoch, 0)
        self.assertTrue(clock.observe("b", 1.12))
        self.assertEqual(clock.epoch, 1)
        self.wall.advance(2)
        self.assertFalse(clock.observe("c", 100.1))
        self.assertEqual(clock.now(), 1.12)
        self.assertTrue(clock.observe("c", 1.2))

    def test_reset_callback_failure_keeps_clock_invalid(self):
        def fail(_epoch):
            raise ValueError("cache could not be cleared")
        self.clock.add_reset_callback(fail)
        self.clock.observe("a", 10)
        self.wall.advance(.1)
        self.clock.observe("a", 1)
        self.wall.advance(.1)
        self.clock.observe("a", 1.1)
        self.assertFalse(self.clock.is_valid())
        self.assertIn("cache could not be cleared", self.clock.status()["reset_error"])

    def test_old_epoch_peer_progress_prevents_false_multi_source_reset(self):
        clock = SensorTimeProvider(["a", "b", "c"], monotonic=self.wall)
        for source in clock.sources:
            clock.observe(source, 100)
        self.wall.advance(.1)
        clock.observe("a", 1)
        self.wall.advance(.1)
        clock.observe("c", 100.2)
        self.wall.advance(.1)
        self.assertFalse(clock.observe("b", 1.1))
        self.assertEqual(clock.epoch, 0)
        self.assertEqual(clock.now(), 100.2)
        self.wall.advance(.1)
        self.assertTrue(clock.observe("c", 1.2))
        self.assertEqual(clock.epoch, 1)

    def test_configuration_rejects_invalid_safety_bounds(self):
        for kwargs in (dict(progress_timeout=0), dict(max_rate=math.inf),
                       dict(max_source_skew=-1), dict(reset_confirmations=1)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SensorTimeProvider(["a"], **kwargs)

    def test_reset_callback_does_not_hold_the_provider_state_lock(self):
        observed = []

        def callback(epoch):
            thread = threading.Thread(target=lambda: observed.append(
                (epoch, self.clock.snapshot().valid)), daemon=True)
            thread.start()
            thread.join(.3)
            if thread.is_alive():
                raise RuntimeError("provider state lock held across reset callback")
        self.clock.add_reset_callback(callback)
        self.clock.observe("a", 10)
        self.wall.advance(.1)
        self.clock.observe("a", 1)
        self.wall.advance(.1)
        self.clock.observe("a", 1.1)
        self.assertEqual(observed, [(1, False)])
        self.assertTrue(self.clock.is_valid())

    def test_wait_can_end_on_shutdown_without_initial_time(self):
        self.assertFalse(self.clock.wait_for_valid(30, shutdown=lambda: True))


if __name__ == "__main__":
    unittest.main()
