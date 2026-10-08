"""Boundary tests for the white-evidence diagnostic tools (N12 batch task).

Pure CPU, no ROS, no GPU. Exercises the pure logic of:
  - white_audit3.aircraft-id resolution (binding preferred, directory fallback,
    conflict rejection)
  - truth row filtering (kind + actor id)
  - BoundedCapture pairing/limits/exit conditions of the read-only observer

Run:  python3 test_white_tools.py        (or via unittest discovery)
Note: constructed inputs only; these are logic tests, not physical results.
"""
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "coordination", "docs", "validation",
                                "white_visibility_20261008"))
sys.path.insert(0, os.path.join(REPO, "coordination", "scripts"))

import white_audit3 as A
from white_evidence_observer import BoundedCapture, result_finished


class AircraftId(unittest.TestCase):
    """Two evidence directories must resolve to the correct uav ids."""

    def test_binding_topic_wins(self):
        self.assertEqual("uav_3", A.uav_from_binding(
            {"image_topic": "/uav_3/cgo3_camera/image_raw"}))

    def test_binding_frame_id_fallback(self):
        self.assertEqual("uav_5", A.uav_from_binding(
            {"image_header": {"frame_id": "uav_5/cgo3_camera_optical_frame"}}))

    def test_directory_fallback_maps_h480_N_to_uav_Nplus1(self):
        self.assertEqual("uav_3", A.uav_from_dir("evidence_typhoon_h480_2"))
        self.assertEqual("uav_6", A.uav_from_dir("evidence_typhoon_h480_5_pass"))
        self.assertEqual("uav_1", A.uav_from_dir("evidence_typhoon_h480_0"))

    def test_the_original_defect_is_gone(self):
        # the v2 bug: d.split('_')[2] == 'h480' -> split('h480_')[1] IndexError
        broken = "evidence_typhoon_h480_2".split("_")[2]
        self.assertEqual("h480", broken)              # documents the old bug
        self.assertEqual("uav_3", A.uav_from_dir("evidence_typhoon_h480_2"))

    def test_no_id_returns_none(self):
        self.assertIsNone(A.uav_from_binding({}))
        self.assertIsNone(A.uav_from_dir("evidence_unknown"))


class BindingConflict(unittest.TestCase):
    def test_binding_dir_conflict_is_rejected_not_guessed(self):
        with tempfile.TemporaryDirectory() as tmp:
            alg = os.path.join(tmp, "flight", "algorithm",
                               "evidence_typhoon_h480_2")
            os.makedirs(alg)
            with open(os.path.join(alg, "index.jsonl"), "w") as fh:
                fh.write(json.dumps({
                    "png": "x.png", "image_stamp": 100.0,
                    "camera_binding": {"image_topic": "/uav_5/cgo3_camera/image_raw"}
                }) + "\n")
            entries, rejected = A.build_originals(tmp)
            self.assertEqual([], [e for e in entries if e.get("png") == "x.png"])
            self.assertEqual(1, len(rejected))
            self.assertEqual("BINDING_DIR_CONFLICT", rejected[0]["reason"])


class TruthFiltering(unittest.TestCase):
    def test_only_actor_truth_and_id3_are_white_truth(self):
        rows = [
            {"kind": "actor_truth", "sample": [1.0, 10.0, 20.0, 3]},   # keep
            {"kind": "actor_truth", "sample": [2.0, 11.0, 21.0, 1]},   # other actor
            {"kind": "something_else", "sample": [3.0, 12.0, 22.0, 3]},  # wrong kind
            {"kind": "actor_truth", "sample": [4.0]},                  # malformed
        ]
        with tempfile.TemporaryDirectory() as tmp:
            os.makedirs(os.path.join(tmp, "observers"))
            os.makedirs(os.path.join(tmp, "flight", "algorithm"))
            with open(os.path.join(tmp, "observers",
                                   "independent_visual_accuracy_white.jsonl"), "w") as fh:
                for r in rows:
                    fh.write(json.dumps(r) + "\n")
            kept = []
            for r in A.load_jsonl(os.path.join(
                    tmp, "observers", "independent_visual_accuracy_white.jsonl")):
                if r.get("kind") != "actor_truth":
                    continue
                s = r.get("sample")
                if isinstance(s, (list, tuple)) and len(s) >= 4 and int(s[3]) == 3:
                    kept.append(s)
            self.assertEqual(1, len(kept))
            self.assertEqual(1.0, kept[0][0])


class CaptureLogic(unittest.TestCase):
    """BoundedCapture: pairing, limits, dedupe, exits."""

    POSE = dict(xyz=[0.0, 0.0, 2.0], rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1])
    INTR = [205.47, 205.47, 320.5, 180.5]
    SIZE = [640, 360]

    def make(self, **kw):
        d = tempfile.mkdtemp(prefix="wcap_")
        opts = dict(out_dir=d, enabled=True, min_expected_px=0.0, max_range_m=1e9)
        opts.update(kw)
        return BoundedCapture(**opts), d

    def test_disabled_captures_nothing(self):
        cap, _ = self.make(enabled=False)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.2)
        self.assertFalse(ok)
        self.assertEqual("disabled", reason)

    def test_missing_camera_fields_rejected(self):
        cap, _ = self.make()
        ok, reason, _ = cap.should_capture("uav_1", 10.0, None, 10.0, [30.0, 0.0],
                                           self.INTR, self.SIZE, 10.2)
        self.assertFalse(ok)
        self.assertEqual("missing_camera_fields", reason)

    def test_stale_pose_never_paired_with_image(self):
        cap, _ = self.make()
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 9.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.0)
        self.assertFalse(ok)
        self.assertEqual("pose_image_time_mismatch", reason)

    def test_pose_too_old_rejected(self):
        cap, _ = self.make(pose_max_age_s=0.5)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 99.0)
        self.assertFalse(ok)
        self.assertEqual("pose_too_old", reason)

    def test_no_truth_rejected(self):
        cap, _ = self.make()
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0, None,
                                           self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("no_truth", reason)

    def test_behind_camera_and_outside_image(self):
        cap, _ = self.make()
        # camera looks along +x in the link frame; yaw 180 deg puts the target behind
        behind = dict(xyz=[0.0, 0.0, 2.0],
                      rotation=[-1, 0, 0, 0, -1, 0, 0, 0, 1])
        ok, reason, _ = cap.should_capture("uav_1", 10.0, behind, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("behind_camera", reason)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 500.0], self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("outside_image", reason)

    def test_duplicate_image_time_deduped(self):
        cap, _ = self.make()
        import numpy as np
        img = np.zeros((8, 8, 3), dtype=np.uint8)
        ok, _, payload = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                            [10.0, 0.0], self.INTR, self.SIZE, 10.1)
        self.assertTrue(ok)
        # commit would mark it seen; simulate the seen-key path directly
        cap.seen_keys.add(("uav_1", 10.0))
        ok2, reason2, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                             [10.0, 0.0], self.INTR, self.SIZE, 10.2)
        self.assertFalse(ok2)
        self.assertEqual("duplicate_image_time", reason2)

    def test_same_instant_different_aircraft_both_allowed(self):
        cap, _ = self.make()
        a = cap.should_capture("uav_1", 10.0, self.POSE, 10.0, [10.0, 0.0],
                               self.INTR, self.SIZE, 10.1)
        b = cap.should_capture("uav_2", 10.0, self.POSE, 10.0, [10.0, 0.0],
                               self.INTR, self.SIZE, 10.1)
        self.assertTrue(a[0])
        self.assertTrue(b[0])

    def test_per_uav_rate_limit(self):
        cap, _ = self.make(per_uav_per_sec=1.0)
        cap.last_saved["uav_1"] = 100.0
        ok, reason, _ = cap.should_capture("uav_1", 100.0, self.POSE, 100.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 100.5)
        self.assertFalse(ok)
        self.assertEqual("per_uav_rate_limit", reason)

    def test_total_cap_enforced(self):
        cap, _ = self.make(total_cap=1)
        cap.saved = 1
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("total_cap_reached", reason)

    def test_threshold_heuristic_is_a_sampling_rule(self):
        # far away and tiny -> rejected by the sampling rule, not by a "criterion"
        cap, _ = self.make(min_expected_px=8.0, max_range_m=20.0)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [200.0, 0.0], self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("below_size_and_range_thresholds", reason)

    def test_queue_is_bounded(self):
        cap, _ = self.make(queue_max=3)
        for i in range(10):
            cap.queue.append(i)
        self.assertEqual(3, len(cap.queue))


class ResultFileExit(unittest.TestCase):
    def test_missing_result_file_keeps_waiting(self):
        self.assertFalse(result_finished("/nonexistent/budget.json"))

    def test_unreadable_or_old_result_does_not_stop(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
            fh.write("{not json")
            bad = fh.name
        try:
            self.assertFalse(result_finished(bad))
        finally:
            os.unlink(bad)

    def test_running_status_does_not_stop(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
            json.dump({"attempts": [{"status": "RUNNING"}]}, fh)
            p = fh.name
        try:
            self.assertFalse(result_finished(p))
        finally:
            os.unlink(p)

    def test_terminal_status_stops(self):
        for st in ("ENDED", "FAILED", "STOPPED_AFTER_CONTACT"):
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
                json.dump({"attempts": [{"status": st}]}, fh)
                p = fh.name
            try:
                self.assertTrue(result_finished(p), st)
            finally:
                os.unlink(p)


class Serialization(unittest.TestCase):
    def test_payload_is_json_serializable(self):
        cap, _ = BoundedCapture.__new__(BoundedCapture), None
        import numpy as np
        cap.__init__(out_dir=tempfile.mkdtemp(), enabled=True)
        ok, _, payload = cap.should_capture(
            "uav_1", 10.0, dict(xyz=[0.0, 0.0, 2.0],
                                rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1]),
            10.0, [10.0, 0.0], [205.47, 205.47, 320.5, 180.5], [640, 360], 10.1)
        self.assertTrue(ok)
        json.dumps(payload)          # must not raise on numpy scalars


class SignedExit(unittest.TestCase):
    def test_signal_handlers_are_registered_in_source(self):
        path = os.path.join(REPO, "coordination", "scripts",
                            "white_evidence_observer.py")
        with open(path, encoding="utf-8") as fh:
            src = fh.read()
        self.assertIn("signal.signal(signal.SIGINT", src)
        self.assertIn("signal.signal(signal.SIGTERM", src)
        self.assertIn("rospy.is_shutdown()", src)
        self.assertIn("result_finished(", src)

    def test_observer_publishes_nothing(self):
        path = os.path.join(REPO, "coordination", "scripts",
                            "white_evidence_observer.py")
        with open(path, encoding="utf-8") as fh:
            src = fh.read()
        for forbidden in ("rospy.Publisher", "ServiceProxy", "set_mode",
                          "command_bool", "setpoint_raw"):
            self.assertNotIn(forbidden, src, forbidden)

    def test_perception_real_untouched_by_this_task(self):
        path = os.path.join(REPO, "perception", "perception_real.py")
        with open(path, encoding="utf-8") as fh:
            src = fh.read()
        # the observer must not have leaked truth logic into perception
        self.assertNotIn("white_evidence_observer", src)
        self.assertNotIn("white_dev_observer", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)