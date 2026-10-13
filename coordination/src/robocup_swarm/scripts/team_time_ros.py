"""Project ROS facade with header time and independent monotonic waiting.

Import this module as ``rospy`` in team code. ROS transport APIs are delegated;
the real rospy module, Time class and private rostime state are never patched.
Rate/Timer/sleep count simulation seconds, with wall-time bounded waits and no
callback catch-up after an outage or new epoch. WallRate is for safety output.
"""

import math
import inspect
import os
import re
import threading
import time

import rospy as _ros
from sensor_time import SensorTimeProvider, TimeUnavailableError as _ProviderTimeUnavailable


Duration = _ros.Duration
_provider = None
_subscribers = []
_pending_reset_callbacks = []
_init_lock = threading.Lock()
_log_lock = threading.Lock()
_log_history = {}
_log_epoch = 0


class TimeUnavailableError(_ros.ROSException):
    pass


def __getattr__(name):
    return getattr(_ros, name)


def wall_monotonic():
    return time.monotonic()


def _clock():
    if _provider is None:
        raise TimeUnavailableError("team header time has not been initialized")
    return _provider


class Time(_ros.Time):
    @classmethod
    def now(cls):
        return cls.from_sec(get_time())


def get_time():
    try:
        return _clock().now()
    except _ProviderTimeUnavailable as exc:
        raise TimeUnavailableError(str(exc)) from exc


def get_rostime():
    return Time.now()


def is_time_valid():
    return _provider is not None and _provider.is_valid()


def time_epoch():
    return 0 if _provider is None else _provider.epoch


def time_status():
    return dict(valid=False, now_s=None, epoch=0) if _provider is None else _provider.status()


def _throttled_log(level, period, message, args, kwargs, identical):
    """Diagnostics use wall liveness, including when the sensor clock freezes.

    Delegate only unthrottled logging: native rospy's throttle helper reads its
    own global Time class, which is intentionally not patched by this facade.
    """
    global _log_epoch
    if not math.isfinite(period) or period < 0:
        raise ValueError("log throttle period must be finite and nonnegative")
    caller = inspect.currentframe().f_back.f_back
    try:
        key = (caller.f_code.co_filename, caller.f_lineno, level, identical)
    finally:
        del caller
    fingerprint = repr((message, args, kwargs))
    now, epoch = wall_monotonic(), time_epoch()
    with _log_lock:
        if epoch != _log_epoch:
            _log_history.clear()
            _log_epoch = epoch
        previous = _log_history.get(key)
        due = (previous is None or now-previous[0] >= period or
               (identical and fingerprint != previous[1]))
        if due:
            _log_history[key] = (now, fingerprint)
    if due:
        getattr(_ros, "log"+level)(message, *args, **kwargs)


def _log_wrapper(level, identical=False):
    def log(period, message, *args, **kwargs):
        _throttled_log(level, period, message, args, kwargs, identical)
    return log


for _level in ("debug", "info", "warn", "err", "fatal"):
    globals()["log%s_throttle" % _level] = _log_wrapper(_level)
    globals()["log%s_throttle_identical" % _level] = _log_wrapper(_level, identical=True)


def add_time_reset_callback(callback):
    if not callable(callback):
        raise TypeError("reset callback must be callable")
    with _init_lock:
        if _provider is None:
            _pending_reset_callbacks.append(callback)
        else:
            _provider.add_reset_callback(callback)


def init_node(name, *args, clock_sources=None, clock_timeout=30.0,
              clock_progress_timeout=None, clock_max_rate=None, **kwargs):
    global _provider, _subscribers
    sources = clock_sources if clock_sources is not None else tuple(
        "typhoon_h480_%d" % index for index in range(6))
    if isinstance(sources, str):
        sources = sources.split(",")
    sources = tuple(sources)
    if not sources or any(not isinstance(source, str) or
                          not re.fullmatch(r"typhoon_h480_[0-5]", source) for source in sources):
        raise ValueError("clock sources must be official own-UAV namespaces typhoon_h480_0..5")
    if not math.isfinite(clock_timeout) or clock_timeout <= 0:
        raise ValueError("clock_timeout must be finite and positive")
    progress_timeout = (float(os.environ.get("TEAM_TIME_PROGRESS_TIMEOUT", "1.0"))
                        if clock_progress_timeout is None else clock_progress_timeout)
    max_rate = (float(os.environ.get("TEAM_TIME_MAX_RATE", "100.0"))
                if clock_max_rate is None else clock_max_rate)
    provider = SensorTimeProvider(sources, progress_timeout=progress_timeout, max_rate=max_rate)
    if len(args) >= 4:
        positional = list(args)
        positional[3] = True
        args = tuple(positional)
        kwargs.pop("disable_rostime", None)
    else:
        kwargs["disable_rostime"] = True
    with _init_lock:
        if _provider is not None:
            raise _ros.ROSInitException("team header time may only be initialized once per process")
        _ros.init_node(name, *args, **kwargs)
        _provider = provider
        for callback in _pending_reset_callbacks:
            provider.add_reset_callback(callback)
        _pending_reset_callbacks.clear()
        from geometry_msgs.msg import PoseStamped

        def pose_header(message, source):
            try:
                stamp = message.header.stamp.to_sec()
            except (AttributeError, TypeError, ValueError, OverflowError):
                stamp = float("nan")
            provider.observe(source, stamp)

        _ros.on_shutdown(provider.wake)
        _subscribers = [_ros.Subscriber("/%s/mavros/local_position/pose" % source,
                                      PoseStamped, pose_header, callback_args=source,
                                      queue_size=1) for source in sources]
    _ros.loginfo("[SENSOR_TIME] sources=%s progress_timeout=%.3fs max_rate=%.1f; no /clock subscription",
                 ",".join(sources), provider.progress_timeout, provider.max_rate)
    if not provider.wait_for_valid(clock_timeout, _ros.is_shutdown):
        for subscriber in _subscribers:
            subscriber.unregister()
        if _ros.is_shutdown():
            raise _ros.ROSInterruptException("shutdown while waiting for MAVROS header time")
        raise _ros.ROSInitException("no progressing, valid MAVROS pose header within %.1fs" % clock_timeout)


def _interrupt():
    raise _ros.ROSInterruptException("ROS shutdown request")


def sleep(duration):
    seconds = duration.to_sec() if hasattr(duration, "to_sec") else float(duration)
    if not math.isfinite(seconds):
        raise ValueError("sleep duration must be finite")
    if seconds <= 0:
        return
    Rate(1.0 / seconds).sleep()


class Rate:
    def __init__(self, hz, reset=False):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("rate must be finite and positive")
        self.sleep_dur = Duration.from_sec(1.0 / hz)
        self._period = 1.0 / hz
        snapshot = _clock().snapshot()
        if snapshot.now_s is None:
            raise TimeUnavailableError("Rate requires an initial header time")
        self._epoch, self._recovery = snapshot.epoch, snapshot.recovery_serial
        self._deadline = snapshot.now_s + self._period
        self.last_time = Time.from_sec(snapshot.now_s)
        self.last_expected_s = snapshot.now_s
        # All epoch changes rebase safely; no business loop is killed merely
        # because the simulator restarted. Application caches use reset hooks.
        self._reset = reset

    def remaining(self):
        return Duration.from_sec(max(0.0, self._deadline-get_time()))

    def _sleep(self, stopped=None):
        stopped = stopped or (lambda: False)
        provider = _clock()
        while True:
            if _ros.is_shutdown() or stopped():
                _interrupt()
            snapshot = provider.snapshot()
            if not snapshot.valid:
                provider.wait_for_change()
                continue
            if snapshot.epoch != self._epoch or snapshot.recovery_serial != self._recovery:
                self._epoch, self._recovery = snapshot.epoch, snapshot.recovery_serial
                self._deadline = snapshot.now_s+self._period
            if snapshot.now_s >= self._deadline:
                self.last_expected_s = self._deadline
                self.last_time = Time.from_sec(snapshot.now_s)
                following = self._deadline+self._period
                self._deadline = (following if following > snapshot.now_s else
                                  snapshot.now_s+self._period)
                return
            provider.wait_for_change()

    def sleep(self):
        self._sleep()


class WallRate:
    def __init__(self, hz, reset=False):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("wall rate must be finite and positive")
        self._period = 1.0/hz
        self._deadline = wall_monotonic()+self._period
        self.sleep_dur = Duration.from_sec(self._period)
        self._wait = threading.Event()

    def remaining(self):
        return Duration.from_sec(max(0.0, self._deadline-wall_monotonic()))

    def sleep(self):
        while True:
            if _ros.is_shutdown():
                _interrupt()
            remaining = self._deadline-wall_monotonic()
            if remaining <= 0:
                now = wall_monotonic()
                following = self._deadline+self._period
                self._deadline = following if following > now else now+self._period
                return
            self._wait.wait(min(remaining, 0.05))


class TimerEvent:
    def __init__(self, last_expected, last_real, current_expected, current_real, last_duration):
        self.last_expected, self.last_real = last_expected, last_real
        self.current_expected, self.current_real = current_expected, current_real
        self.last_duration = last_duration


class Timer(threading.Thread):
    def __init__(self, period, callback, oneshot=False, reset=False):
        seconds = period.to_sec()
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("timer period must be finite and positive")
        self._period, self._callback = seconds, callback
        self._oneshot = oneshot
        self._rate = Rate(1.0/seconds, reset=reset)
        self._shutdown_event = threading.Event()
        super().__init__(daemon=True)
        self.start()

    def shutdown(self):
        self._shutdown_event.set()
        _clock().wake()

    def run(self):
        last_expected = last_real = last_duration = None
        last_epoch = time_epoch()
        while not _ros.is_shutdown() and not self._shutdown_event.is_set():
            try:
                self._rate._sleep(self._shutdown_event.is_set)
            except _ros.ROSInterruptException:
                return
            if not is_time_valid() or self._shutdown_event.is_set():
                continue
            epoch = time_epoch()
            if epoch != last_epoch:
                last_expected = last_real = last_duration = None
                last_epoch = epoch
            expected = Time.from_sec(self._rate.last_expected_s)
            current = Time.now()
            started = wall_monotonic()
            self._callback(TimerEvent(last_expected, last_real, expected, current, last_duration))
            if self._oneshot:
                return
            last_duration = wall_monotonic()-started
            last_expected, last_real = expected, current
