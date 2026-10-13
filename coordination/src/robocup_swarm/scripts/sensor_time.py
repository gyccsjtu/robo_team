"""A process-local simulation clock derived only from approved pose headers.

No ROS imports, subscriptions, extrapolation, or wall-time fallback live here.
Header progress and wall-time liveness are separate. A reset requires repeated
evidence from one configured source or agreement from two configured sources.
Callbacks run outside the state lock, while observations remain serialized;
the new epoch stays invalid until every reset callback has completed.
"""

from collections import Counter
from dataclasses import dataclass
import math
import threading
import time
from typing import Optional


class TimeUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class ClockSnapshot:
    now_s: Optional[float]
    valid: bool
    epoch: int
    recovery_serial: int
    source: Optional[str]


@dataclass
class _Source:
    stamp: Optional[float] = None
    progress_wall: Optional[float] = None
    reset_stamp: Optional[float] = None
    reset_wall: Optional[float] = None
    reset_count: int = 0
    reset_started_wall: Optional[float] = None


class SensorTimeProvider:
    def __init__(self, sources, progress_timeout=1.0, monotonic=None,
                 max_rate=100.0, jitter_allowance=0.5, max_source_skew=1.0,
                 reset_backjump=1.0, reset_confirmations=2,
                 max_stamp=100000000.0):
        self.sources = tuple(sources)
        if (not self.sources or any(not isinstance(source, str) or not source for source in self.sources)
                or len(set(self.sources)) != len(self.sources)):
            raise ValueError("clock sources must be nonempty and unique")
        for name, value in (("progress_timeout", progress_timeout),
                            ("max_rate", max_rate), ("jitter_allowance", jitter_allowance),
                            ("max_source_skew", max_source_skew),
                            ("reset_backjump", reset_backjump), ("max_stamp", max_stamp)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("%s must be finite and positive" % name)
        if not isinstance(reset_confirmations, int) or reset_confirmations < 2:
            raise ValueError("reset_confirmations must be at least two")
        self.progress_timeout = float(progress_timeout)
        self.max_rate = float(max_rate)
        self.jitter_allowance = float(jitter_allowance)
        self.max_source_skew = float(max_source_skew)
        self.reset_backjump = float(reset_backjump)
        self.reset_confirmations = reset_confirmations
        self.max_stamp = float(max_stamp)
        self.monotonic = monotonic or time.monotonic
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._observations = threading.Lock()
        self._states = {source: _Source() for source in self.sources}
        self._now = None
        self._progress_wall = None
        self._selected = None
        self._epoch = 0
        self._recovery_serial = 0
        self._resetting = False
        self._reset_error = None
        self._retired_sources = set()
        self._callbacks = []
        self._rejected = Counter()

    def _valid_locked(self, wall):
        if (self._now is None or self._resetting or self._reset_error is not None or
                not 0.0 <= wall-self._progress_wall <= self.progress_timeout):
            return False
        return any(state.stamp is not None and state.reset_stamp is None and
                   0.0 <= wall - state.progress_wall <= self.progress_timeout and
                   0.0 <= self._now - state.stamp <= self.max_source_skew
                   for state in self._states.values())

    def snapshot(self):
        with self._lock:
            return ClockSnapshot(self._now, self._valid_locked(self.monotonic()),
                                 self._epoch, self._recovery_serial, self._selected)

    def now(self):
        with self._lock:
            if self._now is None:
                raise TimeUnavailableError("no valid MAVROS pose header has initialized time")
            return self._now

    def is_valid(self):
        return self.snapshot().valid

    @property
    def epoch(self):
        with self._lock:
            return self._epoch

    def add_reset_callback(self, callback):
        if not callable(callback):
            raise TypeError("reset callback must be callable")
        with self._lock:
            self._callbacks.append(callback)

    def wake(self):
        with self._condition:
            self._condition.notify_all()

    def wait_for_change(self, timeout=0.05):
        with self._condition:
            self._condition.wait(timeout)

    def wait_for_valid(self, timeout, shutdown=None):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("startup timeout must be finite and positive")
        shutdown = shutdown or (lambda: False)
        deadline = self.monotonic() + timeout
        with self._condition:
            while not self._valid_locked(self.monotonic()):
                if shutdown():
                    return False
                remaining = deadline - self.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(min(remaining, 0.05))
            return not shutdown()

    def status(self):
        with self._lock:
            wall = self.monotonic()
            return dict(now_s=self._now, valid=self._valid_locked(wall), epoch=self._epoch,
                        recovery_serial=self._recovery_serial, source=self._selected,
                        sources=list(self.sources), progress_timeout=self.progress_timeout,
                        max_rate=self.max_rate, jitter_allowance=self.jitter_allowance,
                        max_source_skew=self.max_source_skew, max_stamp=self.max_stamp,
                        reset_backjump=self.reset_backjump,
                        reset_confirmations=self.reset_confirmations,
                        rejected=dict(self._rejected), reset_error=self._reset_error,
                        source_progress_age={source: (None if state.progress_wall is None else
                                                     wall-state.progress_wall)
                                             for source, state in self._states.items()})

    def _reject(self, reason):
        self._rejected[reason] += 1
        self._condition.notify_all()
        return False

    def _budget(self, elapsed):
        return self.jitter_allowance + self.max_rate * max(0.0, elapsed)

    def _reset_evidence(self, source, stamp, wall):
        state = self._states[source]
        if state.reset_stamp is None or wall-state.reset_wall > self.progress_timeout:
            state.reset_stamp, state.reset_wall, state.reset_count = stamp, wall, 1
            state.reset_started_wall = wall
        elif stamp > state.reset_stamp and stamp-state.reset_stamp <= self._budget(wall-state.reset_wall):
            state.reset_stamp, state.reset_wall = stamp, wall
            state.reset_count += 1
        else:
            return None
        evidence = [(name, value) for name, value in self._states.items()
                    if value.reset_stamp is not None and
                    0.0 <= wall-value.reset_wall <= self.progress_timeout and
                    abs(value.reset_stamp-stamp) <= self.max_source_skew]
        quorum = (len(self.sources) == 1 and state.reset_count >= self.reset_confirmations)
        if len(self.sources) > 1:
            quorum = len(evidence) >= 2
            if quorum:
                first_reset = min(value.reset_started_wall for _, value in evidence)
                if any(value.stamp is not None and value.reset_stamp is None and
                       value.progress_wall > first_reset and
                       value.stamp-stamp >= self.reset_backjump
                       for value in self._states.values()):
                    quorum = False  # a live old-epoch peer contradicts the reset
        return evidence if quorum else None

    def observe(self, source, stamp):
        """Accept one source header; return False for rejected or pending samples.

        A repeated header never refreshes liveness. Large single-source forward
        jumps are bounded by elapsed monotonic time and the configured rate cap.
        Small backwards samples are reordered packets; significant backwards
        samples, or backwards samples after loss of liveness, need reset evidence.
        """
        with self._observations:
            callbacks = None
            with self._condition:
                wall = self.monotonic()
                if source not in self._states:
                    return self._reject("unknown_source")
                try:
                    stamp = float(stamp)
                except (TypeError, ValueError, OverflowError):
                    return self._reject("invalid_stamp")
                if not math.isfinite(stamp) or not 0.0 < stamp <= self.max_stamp:
                    return self._reject("invalid_stamp")
                state = self._states[source]
                was_valid = self._valid_locked(wall)
                if state.stamp is not None and stamp == state.stamp:
                    return self._reject("duplicate")
                if state.stamp is not None and stamp < state.stamp:
                    if state.stamp-stamp < self.reset_backjump and was_valid and state.reset_stamp is None:
                        return self._reject("out_of_order")
                    evidence = self._reset_evidence(source, stamp, wall)
                    if evidence is None:
                        return self._reject("reset_pending")
                    self._states = {name: _Source() for name in self.sources}
                    self._retired_sources = set(self.sources) - {name for name, _ in evidence}
                    for name, previous in evidence:
                        self._states[name] = _Source(previous.reset_stamp, previous.reset_wall)
                    selected, selected_state = max(evidence, key=lambda item: item[1].reset_stamp)
                    self._now = selected_state.reset_stamp
                    self._progress_wall = selected_state.reset_wall
                    self._selected = selected
                    self._epoch += 1
                    self._recovery_serial += 1
                    self._resetting = True
                    callbacks = tuple(self._callbacks)
                    self._condition.notify_all()
                else:
                    if source in self._retired_sources and abs(stamp-self._now) > self.max_source_skew:
                        return self._reject("retired_epoch_source")
                    if state.stamp is not None and stamp-state.stamp > self._budget(wall-state.progress_wall):
                        return self._reject("forward_jump")
                    if self._now is not None:
                        budget = self.max_source_skew + self._budget(wall-self._progress_wall)
                        if abs(stamp-self._now) > budget:
                            return self._reject("different_time_domain")
                        # Other live sources prevent a lone source moving the
                        # team's epoch far ahead of its peers.
                        peers = [value for name, value in self._states.items()
                                 if name != source and value.stamp is not None and
                                 value.reset_stamp is None and
                                 0.0 <= wall-value.progress_wall <= self.progress_timeout]
                        if peers and all(abs(stamp-peer.stamp) >
                                         self.max_source_skew+self._budget(wall-peer.progress_wall)
                                         for peer in peers):
                            return self._reject("peer_time_conflict")
                    state.stamp, state.progress_wall = stamp, wall
                    self._retired_sources.discard(source)
                    state.reset_stamp, state.reset_wall, state.reset_count = None, None, 0
                    state.reset_started_wall = None
                    if self._now is None or stamp > self._now:
                        self._now, self._progress_wall, self._selected = stamp, wall, source
                    if not was_valid and self._valid_locked(wall):
                        self._recovery_serial += 1
                    self._condition.notify_all()
                    return True
            errors = []
            for callback in callbacks:
                try:
                    callback(self._epoch)
                except Exception as exc:
                    errors.append("%s: %s" % (type(exc).__name__, exc))
            with self._condition:
                self._resetting = False
                if errors:
                    self._reset_error = "; ".join(errors)
                self._condition.notify_all()
            return True
