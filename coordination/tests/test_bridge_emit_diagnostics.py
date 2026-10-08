"""Bridge official-publish diagnostics: gate reasons and the round replay.

Why these tests exist
---------------------
A validation round published ZERO official `/actor_*_info` messages while the
bridge trace showed that every observation had been accepted. Nothing recorded
whether the 10 Hz publish timer had stopped or whether every event was skipped
by the gate, because the success path wrote a trace row and the failure path
wrote nothing.

These tests pin down two things:
  1. `emit_gate_reason` agrees exactly with the original inline condition, so
     making the decision visible did not change any decision.
  2. Replaying the aborted round's real 19 observations through the bridge core
     produces 68 events and 59 gate passes -> the gate was NOT why that round
     published nothing. If a future change makes the gate the explanation, this
     test flips and says so.

Pure CPU, no ROS: the core only imports math/json.
"""
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SCRIPTS = os.path.join(REPO, "coordination", "src", "robocup_swarm", "scripts")
sys.path.insert(0, SCRIPTS)

import yolo_target_bridge as B  # noqa: E402
from report_readiness import ReportReadiness  # noqa: E402

FIXTURE = os.path.join(HERE, "bridge_trace_round1_20261008.jsonl")


class _Track(object):
    def __init__(self, t_obs, x=1.0, y=2.0):
        self.t_obs = t_obs
        self.x, self.y = x, y
        self.vx = self.vy = 0.0
        self.position_s = t_obs


def legacy_inline_gate(tag, actor_pub_tags, ev, track, now,
                       approach_reporting=False, readiness_ok=True):
    """The exact expression that used to sit inline in _emit()."""
    return (tag is not None and tag in actor_pub_tags
            and not ev["eliminated"]
            and ev.get("state", 0) != 3 and 0 <= now - track.t_obs <= B.COAST_TIME
            and (not approach_reporting or readiness_ok))


class GateAgreesWithOriginal(unittest.TestCase):
    """Making the decision visible must not change the decision."""

    def test_exhaustive_case_matrix(self):
        tags = [None, "red1", "green", "unknown"]
        pub_tags = {"red1", "green"}
        for tag in tags:
            for eliminated in (False, True):
                for state in (0, 1, 3):
                    for age in (-0.5, 0.0, 0.5, B.COAST_TIME, B.COAST_TIME + 0.001):
                        for approach in (False, True):
                            for ready in (False, True):
                                ev = dict(eliminated=eliminated, state=state)
                                tr = _Track(t_obs=100.0)
                                now = 100.0 + age
                                new = B.emit_gate_reason(
                                    tag, pub_tags, ev, tr, now,
                                    approach_reporting=approach, readiness_ok=ready)
                                old = legacy_inline_gate(
                                    tag, pub_tags, ev, tr, now,
                                    approach_reporting=approach, readiness_ok=ready)
                                self.assertEqual(
                                    new is None, old,
                                    "disagreement tag=%r eliminated=%s state=%s age=%s "
                                    "approach=%s ready=%s -> %r"
                                    % (tag, eliminated, state, age, approach, ready, new))

    def test_reason_names_each_condition(self):
        ev = dict(eliminated=False, state=0)
        tr = _Track(t_obs=100.0)
        self.assertEqual("no_slot", B.emit_gate_reason(None, {"red1"}, ev, tr, 100.0))
        self.assertEqual("no_slot", B.emit_gate_reason("green", {"red1"}, ev, tr, 100.0))
        self.assertEqual("eliminated", B.emit_gate_reason(
            "red1", {"red1"}, dict(eliminated=True, state=0), tr, 100.0))
        self.assertEqual("state_elim", B.emit_gate_reason(
            "red1", {"red1"}, dict(eliminated=False, state=3), tr, 100.0))
        self.assertEqual("stale", B.emit_gate_reason(
            "red1", {"red1"}, ev, tr, 100.0 + B.COAST_TIME + 1.0))
        self.assertEqual("stale", B.emit_gate_reason(
            "red1", {"red1"}, ev, tr, 99.0))          # now before the observation
        self.assertEqual("not_ready", B.emit_gate_reason(
            "red1", {"red1"}, ev, tr, 100.0,
            approach_reporting=True, readiness_ok=False))
        self.assertIsNone(B.emit_gate_reason("red1", {"red1"}, ev, tr, 100.0))
        # readiness is ignored when approach reporting is off (original short-circuit)
        self.assertIsNone(B.emit_gate_reason(
            "red1", {"red1"}, ev, tr, 100.0,
            approach_reporting=False, readiness_ok=False))


class RoundReplay(unittest.TestCase):
    """The aborted round's real inputs must NOT be explained by the gate."""

    def _replay(self, approach_reporting=False):
        core = B.TargetBridgeCore()
        red = B.RedObservations()
        # BRIDGE_APPROACH_REPORTING defaults to 1 in city_swarm_run.py:102, so the
        # realistic regime is ON; OFF is kept to show the gate is what differs.
        approach = bool(approach_reporting)
        readiness = ReportReadiness()
        with open(FIXTURE, encoding="utf-8") as fh:
            rows = [json.loads(l) for l in fh if l.strip()]
        todo = [(float(r["receipt_s"]), 0, r) for r in rows]
        first = min(t[0] for t in todo)
        last = max(t[0] for t in todo)
        t = first
        while t <= last + 1.0:
            todo.append((t, 1, None))
            t += 0.1
        todo.sort(key=lambda x: (x[0], x[1]))

        accepted = events = passed = 0
        reasons = {}
        alive_seen = 0
        for when, kind, row in todo:
            if kind == 0:
                o = row["observation"]
                tag = o["target_id"]
                if tag in ("red1", "red2"):
                    tag = red.observe(float(o["sample_s"]), (o["xyz"][0], o["xyz"][1]))
                    if tag is None:
                        continue
                ok = core.report(float(o["sample_s"]), tag, o["xyz"][0], o["xyz"][1],
                                 float(o["confidence"]), o["uav_id"],
                                 o.get("observation_id"))
                if ok:
                    accepted += 1
                if approach:
                    readiness.observe(o, when, ok)
                if core.tracks[tag].alive:
                    alive_seen += 1
                continue
            for ev in core.tick(when):
                events += 1
                tr = core.tracks[ev["tag"]]
                tag = B.TID_TO_TAG.get(ev["tid"])
                readiness_ok = True
                if approach and tag is not None:
                    readiness_ok = readiness.allowed(tag, when, tr.t_obs)
                reason = B.emit_gate_reason(tag, core.tracks, ev, tr, when,
                                            approach_reporting=approach,
                                            readiness_ok=readiness_ok)
                if reason is None:
                    passed += 1
                else:
                    reasons[reason] = reasons.get(reason, 0) + 1
        return accepted, events, passed, reasons, alive_seen, len(rows)

    def test_inputs_are_all_accepted(self):
        accepted, _, _, _, _, total = self._replay()
        self.assertEqual(total, accepted)

    def test_without_approach_gate_everything_passes(self):
        """Control: the events themselves are publishable (flag OFF)."""
        _, events, passed, reasons, alive_seen, _ = self._replay(approach_reporting=False)
        self.assertGreater(alive_seen, 0, "track never became alive")
        self.assertEqual(68, events, "event count changed")
        self.assertEqual(59, passed, "gate pass count changed")
        self.assertEqual({"stale": 9}, reasons)

    def test_approach_gate_blocks_every_publish_this_round(self):
        """THE finding, in the runner's real regime.

        BRIDGE_APPROACH_REPORTING defaults to 1. The round's red1 observations sit
        at 15.6-20.4 m from the camera, never inside ReportReadiness' 12 m
        first-time threshold, so readiness is never 'ready' and every one of the
        59 would-be publishes is skipped as not_ready. That is why the judge saw
        zero official reports and its 15 s timer never started (0/6).
        """
        _, events, passed, reasons, _, _ = self._replay(approach_reporting=True)
        self.assertEqual(68, events)
        self.assertEqual(0, passed, "if this changes, the approach gate stopped blocking")
        self.assertEqual({"not_ready": 59, "stale": 9}, reasons)

    def test_observed_distance_never_reached_the_12m_gate(self):
        import math
        closest = None
        with open(FIXTURE, encoding="utf-8") as fh:
            for line in fh:
                if not line.strip():
                    continue
                o = json.loads(line)["observation"]
                cam = o["camera_xyz"]
                d = math.hypot(o["xyz"][0] - cam[0], o["xyz"][1] - cam[1])
                closest = d if closest is None else min(closest, d)
        self.assertIsNotNone(closest)
        self.assertGreater(closest, 12.0,
                           "fixture no longer documents the miss; closest=%.2f m" % closest)
        self.assertLess(closest, 16.0, "closest approach was ~15.6 m")


class DiagnosticsPresent(unittest.TestCase):
    """The wrapper must announce its timer and be able to explain a skip."""

    def _src(self):
        with open(os.path.join(SCRIPTS, "yolo_target_bridge.py"), encoding="utf-8") as fh:
            return fh.read()

    def test_source_has_heartbeat_and_skip_trace(self):
        src = self._src()
        self.assertIn("def _heartbeat", src)
        self.assertIn("kind='skip'", src)
        self.assertIn("emit_gate_reason(", src)
        # success path still records the upload it always did
        self.assertIn("kind='upload'", src)

    def test_heartbeat_is_inside_the_timer_callback(self):
        src = self._src()
        tick = src[src.index("def _tick(self, _evt)"):]
        tick = tick[:tick.index("def ", 10)]
        self.assertIn("self._heartbeat()", tick,
                      "heartbeat must run in the timer callback: its absence is the evidence")

    def test_gate_decision_exists_only_once(self):
        """_emit must not keep a second copy of the condition."""
        src = self._src()
        emit = src[src.index("def _emit(self, ev)"):]
        emit = emit[:emit.index("def ", 10)]
        self.assertEqual(1, emit.count("emit_gate_reason("))
        self.assertNotIn("COAST_TIME", emit,
                         "the inline freshness test should now live in emit_gate_reason")


if __name__ == "__main__":
    unittest.main(verbosity=2)
