"""ROS facade tests with a fake transport; never initialize a real ROS node."""

import importlib.util
from pathlib import Path
import sys
import subprocess
import threading
import time
import types
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "coordination/src/robocup_swarm/scripts"
sys.path.insert(0, str(SCRIPTS))
from sensor_time import SensorTimeProvider


class FakeDuration:
    def __init__(self, secs=0, nsecs=0):
        self.value = secs+nsecs/1e9

    @classmethod
    def from_sec(cls, value):
        return cls(value)

    def to_sec(self):
        return self.value


class FakeTime:
    def __init__(self, secs=0, nsecs=0):
        value = secs+nsecs/1e9
        self.secs = int(value)
        self.nsecs = round((value-self.secs)*1e9)

    @classmethod
    def from_sec(cls, value):
        return cls(value)

    @staticmethod
    def now():
        return FakeTime(1700000000)

    def to_sec(self):
        return self.secs+self.nsecs/1e9

    def __add__(self, duration):
        return type(self).from_sec(self.to_sec()+duration.to_sec())

    def __sub__(self, other):
        if isinstance(other, FakeTime):
            return FakeDuration.from_sec(self.to_sec()-other.to_sec())
        return type(self).from_sec(self.to_sec()-other.to_sec())


class FakeROS(types.ModuleType):
    class ROSException(Exception):
        pass

    class ROSInitException(ROSException):
        pass

    class ROSInterruptException(ROSException):
        pass

    Time = FakeTime
    Duration = FakeDuration
    Rate = object
    Publisher = object()

    def __init__(self):
        super().__init__("rospy")
        self.stopped = False
        self.calls = []
        self.subscriptions = []
        self.hooks = []
        self.registered = threading.Event()
        self.logs = []

    def init_node(self, *args, **kwargs):
        self.calls.append((args, kwargs))

    def Subscriber(self, topic, msg_class, callback, callback_args=None, **kwargs):
        subscription = types.SimpleNamespace(topic=topic, callback=callback,
                                             source=callback_args, unregistered=False)
        subscription.unregister = lambda: setattr(subscription, "unregistered", True)
        self.subscriptions.append(subscription)
        self.registered.set()
        return subscription

    def emit(self, source, stamp):
        message = types.SimpleNamespace(header=types.SimpleNamespace(stamp=FakeTime.from_sec(stamp)))
        for subscription in self.subscriptions:
            if subscription.source == source and not subscription.unregistered:
                subscription.callback(message, source)

    def on_shutdown(self, callback):
        self.hooks.append(callback)

    def is_shutdown(self):
        return self.stopped

    def stop(self):
        self.stopped = True
        for callback in self.hooks:
            callback()

    def loginfo(self, *args, **kwargs):
        self.logs.append((args, kwargs))

    logdebug = logwarn = logerr = logfatal = loginfo


class TeamTimeROSTests(unittest.TestCase):
    def setUp(self):
        self.ros = FakeROS()
        geometry = types.ModuleType("geometry_msgs")
        messages = types.ModuleType("geometry_msgs.msg")
        messages.PoseStamped = object
        geometry.msg = messages
        self.module_patch = patch.dict(sys.modules, {"rospy": self.ros,
                                                    "geometry_msgs": geometry,
                                                    "geometry_msgs.msg": messages})
        self.module_patch.start()
        spec = importlib.util.spec_from_file_location("_test_team_time_ros", SCRIPTS/"team_time_ros.py")
        self.facade = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.facade)
        self.threads = []
        self.timers = []

    def tearDown(self):
        self.ros.stop()
        if self.facade._provider is not None:
            self.facade._provider.wake()
        for timer in self.timers:
            timer.shutdown()
            timer.join(.3)
        for thread in self.threads:
            thread.join(.3)
            self.assertFalse(thread.is_alive(), "offline worker did not stop")
        self.module_patch.stop()

    def worker(self, function):
        done, started, errors = threading.Event(), threading.Event(), []

        def run():
            started.set()
            try:
                function()
            except Exception as exc:
                errors.append(exc)
            finally:
                done.set()
        thread = threading.Thread(target=run, daemon=True)
        self.threads.append(thread)
        thread.start()
        self.assertTrue(started.wait(.3))
        return done, errors

    def provider(self, stamp=1., timeout=1., monotonic=None):
        provider = SensorTimeProvider(["a"], progress_timeout=timeout, monotonic=monotonic)
        self.facade._provider = provider
        provider.observe("a", stamp)
        return provider

    def wait_until(self, predicate):
        deadline = time.monotonic()+.3
        while not predicate() and time.monotonic() < deadline:
            threading.Event().wait(.001)
        self.assertTrue(predicate())

    def test_uninitialized_time_is_not_unix_wall_clock(self):
        self.assertFalse(self.facade.is_time_valid())
        self.assertEqual(self.facade.time_epoch(), 0)
        with self.assertRaises(self.ros.ROSException):
            self.facade.Time.now()
        with self.assertRaises(self.facade.TimeUnavailableError):
            self.facade.get_time()

    def test_time_arithmetic_and_transport_are_preserved_without_monkey_patching(self):
        self.provider(.25)
        self.assertAlmostEqual(self.facade.Time.now().to_sec(), .25)
        self.assertEqual(self.facade.get_time(), .25)
        self.assertAlmostEqual(self.facade.get_rostime().to_sec(), .25)
        self.assertEqual((self.facade.Time(3)+self.facade.Duration(2)).to_sec(), 5)
        self.assertEqual((self.facade.Time(3)-self.facade.Time(1)).to_sec(), 2)
        self.assertIs(self.facade.Publisher, self.ros.Publisher)
        self.assertIs(self.ros.Rate, object)
        self.assertEqual(self.ros.Time.now().to_sec(), 1700000000)

    def test_native_header_nanoseconds_survive_provider_and_strict_freshness(self):
        # A fresh interpreter loads real Noetic types rather than this suite's
        # fake transport. Importing/serializing messages does not start a node.
        program = r'''
import io
import sys
sys.path.insert(0, sys.argv[1])
import rospy
import team_time_ros as facade
from geometry_msgs.msg import PoseStamped
from sensor_time import SensorTimeProvider

checks = 0
for seconds in (100, 1000, 50000):
    fractions = sorted({0, 1, 123456789, 987654321, 999999999} |
                       {index*50000000 for index in range(20)})
    for nsecs in fractions:
        header = rospy.Time(seconds, nsecs)
        provider = SensorTimeProvider(['offline_header'], monotonic=lambda: 0.0)
        assert provider.observe('offline_header', header.to_sec())
        facade._provider = provider
        for current in (facade.Time.now(), facade.get_rostime(),
                        facade.Time.from_sec(header.to_sec())):
            assert (current.secs, current.nsecs) == (header.secs, header.nsecs), (
                header, current)
            assert 0.0 <= current.to_sec()-header.to_sec() <= .3
            message = PoseStamped()
            message.header.stamp = current
            payload = io.BytesIO()
            message.serialize(payload)
            decoded = PoseStamped().deserialize(payload.getvalue())
            assert decoded.header.stamp == header
            checks += 1
        assert facade.get_time() == header.to_sec()
    wall = [0.0]
    provider = SensorTimeProvider(['offline_header'], monotonic=lambda: wall[0])
    facade._provider = provider
    for index in range(40):
        wall[0] = index*.05
        header = rospy.Time(seconds+index//20, (index % 20)*50000000)
        assert provider.observe('offline_header', header.to_sec())
        current = facade.Time.now()
        assert current == header
        assert 0.0 <= current.to_sec()-header.to_sec() <= .3
        assert provider.is_valid()
        assert ((current+facade.Duration(1))-current).to_sec() == 1.0
        checks += 1
assert not rospy.core.is_initialized()
print('native precision checks=%d; ROS node remains uninitialized' % checks)
'''
        result = subprocess.run([sys.executable, "-B", "-c", program, str(SCRIPTS)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("ROS node remains uninitialized", result.stdout)

    def test_init_disables_automatic_clock_and_waits_for_allowed_pose_header(self):
        done, errors = self.worker(lambda: self.facade.init_node(
            "team", clock_sources=["typhoon_h480_0"], clock_timeout=.3,
            disable_rostime=False))
        self.assertTrue(self.ros.registered.wait(.3))
        self.assertFalse(done.is_set())
        self.ros.emit("typhoon_h480_0", 0)
        self.assertFalse(done.is_set())
        self.ros.emit("typhoon_h480_0", 1)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])
        self.assertTrue(self.ros.calls[0][1]["disable_rostime"])
        self.assertEqual([sub.topic for sub in self.ros.subscriptions], [
            "/typhoon_h480_0/mavros/local_position/pose"])

    def test_invalid_topic_sources_are_rejected_before_transport_initialization(self):
        for sources in ([], ["/clock"], ["actor_0"], ["typhoon_h480_6"]):
            with self.subTest(sources=sources), self.assertRaises(ValueError):
                self.facade.init_node("team", clock_sources=sources)
        self.assertEqual(self.ros.calls, [])

    def test_fleet_sources_and_positional_disable_rostime_are_supported(self):
        sources = ["typhoon_h480_%d" % index for index in range(6)]
        done, errors = self.worker(lambda: self.facade.init_node(
            "team", None, False, None, False, clock_sources=sources,
            clock_timeout=.3, clock_progress_timeout=.5))
        self.assertTrue(self.ros.registered.wait(.3))
        self.ros.emit("typhoon_h480_0", 1)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])
        self.assertTrue(self.ros.calls[0][0][4])
        self.assertEqual(len(self.ros.subscriptions), 6)
        self.assertEqual(self.facade.time_status()["progress_timeout"], .5)

    def test_startup_timeout_unregisters_sources(self):
        with self.assertRaises(self.ros.ROSInitException):
            self.facade.init_node("team", clock_sources=["typhoon_h480_0"], clock_timeout=.01)
        self.assertTrue(all(sub.unregistered for sub in self.ros.subscriptions))

    def test_shutdown_interrupts_startup_wait(self):
        done, errors = self.worker(lambda: self.facade.init_node(
            "team", clock_sources=["typhoon_h480_0"], clock_timeout=30))
        self.assertTrue(self.ros.registered.wait(.3))
        self.ros.stop()
        self.assertTrue(done.wait(.3))
        self.assertIsInstance(errors[0], self.ros.ROSInterruptException)

    def test_simulation_sleep_ignores_elapsed_wall_time(self):
        provider = self.provider()
        done, errors = self.worker(lambda: self.facade.sleep(.1))
        self.assertFalse(done.wait(.02))
        provider.observe("a", 1.2)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])

    def test_rate_recovery_rebases_deadline_without_catch_up(self):
        wall = [0.0]
        provider = self.provider(monotonic=lambda: wall[0])
        rate = self.facade.Rate(10)
        done, errors = self.worker(rate.sleep)
        wall[0] = 2.0
        provider.wake()
        self.assertFalse(self.facade.is_time_valid())
        provider.observe("a", 5)
        self.wait_until(lambda: rate._recovery == provider.snapshot().recovery_serial)
        self.assertFalse(done.is_set())
        provider.observe("a", 5.11)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])
        self.assertGreater(rate._deadline, provider.now())

    def test_timer_recovery_does_not_burst_old_callbacks(self):
        wall = [0.0]
        provider = self.provider(monotonic=lambda: wall[0])
        fired, events = threading.Event(), []

        def callback(event):
            events.append(event)
            fired.set()
        timer = self.facade.Timer(self.facade.Duration(.01), callback)
        self.timers.append(timer)
        wall[0] = 2.0
        provider.wake()
        self.assertFalse(provider.is_valid())
        provider.observe("a", 5)
        self.wait_until(lambda: timer._rate._recovery == provider.snapshot().recovery_serial)
        self.assertFalse(fired.is_set())
        provider.observe("a", 5.02)
        self.assertTrue(fired.wait(.3))
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].current_real.to_sec(), 5.02)
        self.assertFalse(threading.Event().wait(.01))
        self.assertEqual(len(events), 1)

    def test_epoch_callbacks_and_rate_rebase_use_new_time(self):
        provider = self.provider(10)
        seen = []
        self.facade.add_time_reset_callback(lambda epoch: seen.append(epoch))
        rate = self.facade.Rate(10)
        done, errors = self.worker(rate.sleep)
        provider.observe("a", 1)
        provider.observe("a", 1.1)
        self.assertEqual(self.facade.time_epoch(), 1)
        self.assertEqual(seen, [1])
        self.wait_until(lambda: rate._epoch == provider.epoch)
        self.assertFalse(done.is_set())
        provider.observe("a", 1.3)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])

    def test_wall_rate_keeps_running_when_sensor_time_is_invalid(self):
        rate = self.facade.WallRate(100)
        done, errors = self.worker(rate.sleep)
        self.assertTrue(done.wait(.3))
        self.assertEqual(errors, [])
        self.assertFalse(self.facade.is_time_valid())

    def test_timer_shutdown_wakes_frozen_time(self):
        self.provider()
        timer = self.facade.Timer(self.facade.Duration(.1), lambda _event: self.fail("unexpected timer"))
        self.timers.append(timer)
        timer.shutdown()
        timer.join(.3)
        self.assertFalse(timer.is_alive())

    def test_node_shutdown_interrupts_rate_and_sleep(self):
        provider = self.provider()
        done, errors = self.worker(lambda: self.facade.sleep(10))
        self.ros.stop()
        provider.wake()
        self.assertTrue(done.wait(.3))
        self.assertIsInstance(errors[0], self.ros.ROSInterruptException)

    def test_throttled_diagnostics_do_not_read_native_ros_time(self):
        wall = [0.0]

        def log():
            self.facade.logwarn_throttle(1.0, "source unavailable")
        with patch.object(self.facade, "wall_monotonic", side_effect=lambda: wall[0]), \
                patch.object(FakeTime, "now", side_effect=AssertionError("native time read")):
            log()
            log()
            self.assertEqual(len(self.ros.logs), 1)
            wall[0] = 1.0
            log()
            self.assertEqual(len(self.ros.logs), 2)

    def test_identical_logging_accepts_changed_messages_and_resets_on_epoch(self):
        provider = self.provider(10)

        def log(message):
            self.facade.loginfo_throttle_identical(30, message)
        log("first")
        log("first")
        log("changed")
        self.assertEqual(len(self.ros.logs), 2)
        provider.observe("a", 1)
        provider.observe("a", 1.1)
        log("changed")
        self.assertEqual(len(self.ros.logs), 3)


if __name__ == "__main__":
    unittest.main()
