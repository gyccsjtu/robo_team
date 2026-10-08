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

    def test_old_original_rejected(self):
        # CONTRACT CHANGE 2026-10-08. The old test asserted pose_too_old for an
        # image 89 s old; that guard measured wall-vs-pose-stamp and so could not
        # tell "no pose for this instant" from "the frame is ancient". The real
        # protection is the ORIGINAL age, checked first.
        cap, _ = self.make(pose_max_age_s=0.5)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 99.0)
        self.assertFalse(ok)
        self.assertEqual("image_too_old", reason)

    def test_pose_too_old_still_available_for_topic_mode(self):
        # Only reachable when the image-age gate is deliberately looser; kept so
        # a dead pose stream cannot be paired silently.
        cap, _ = self.make(pose_max_age_s=1.0, image_max_age_s=5.0)
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, 10.0,
                                           [10.0, 0.0], self.INTR, self.SIZE, 12.0)
        self.assertFalse(ok)
        self.assertEqual("pose_too_old", reason)

    def test_measured_latency_does_not_block_capture(self):
        """REGRESSION for the defect the offline backtest found.

        Measured image latency in a real round is min 0.216 / p50 0.472 /
        max 0.884 s. Service-mode poses carry no independent stamp, so the only
        gate is image age; a 0.5 s old frame MUST still be eligible.
        """
        cap, _ = self.make()
        ok, reason, _ = cap.should_capture("uav_1", 10.0, self.POSE, None,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.472)
        self.assertTrue(ok, reason)

    def test_truth_too_old_rejected(self):
        cap, _ = self.make()
        age = cap.should_capture("uav_1", 10.0, self.POSE, None, [10.0, 0.0],
                                 self.INTR, self.SIZE, 10.1, truth_age_s=9.0)
        self.assertFalse(age[0])
        self.assertEqual("truth_too_old", age[1])

    def test_pose_history_picks_nearest_stamp_not_newest(self):
        cap, _ = self.make(pair_tol_s=0.02)
        cap.note_pose("uav_1", 10.01, self.POSE["xyz"], self.POSE["rotation"])
        cap.note_pose("uav_1", 13.00, self.POSE["xyz"], self.POSE["rotation"],
                      now=13.0)
        # image at 10.0: nearest is 10.01 (0.01 s), NOT the newest 13.0
        ok, reason, _ = cap.should_capture("uav_1", 10.0, None, None,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.05)
        self.assertTrue(ok, reason)

    def test_pose_history_without_match_still_rejects(self):
        # A pose exists, but not for this instant: the honest reason is a
        # pairing mismatch, not "no pose at all".
        cap, _ = self.make()
        cap.note_pose("uav_1", 40.0, self.POSE["xyz"], self.POSE["rotation"])
        ok, reason, _ = cap.should_capture("uav_1", 10.0, None, None,
                                           [10.0, 0.0], self.INTR, self.SIZE, 10.1)
        self.assertFalse(ok)
        self.assertEqual("pose_image_time_mismatch", reason)

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
        for forbidden in ("rospy.Publisher", "set_mode", "command_bool",
                          "setpoint_raw", "mavros/cmd", "mavros/setpoint",
                          "param/set", "arducopter"):
            self.assertNotIn(forbidden, src, forbidden)

    def test_observer_calls_exactly_one_readonly_gazebo_service(self):
        """The pose pull mirrors perception_real.py's own GetLinkState call.

        Control-plane service clients stay forbidden; only the one read-only
        Gazebo link-state query may appear, and it must be the only one.
        """
        path = os.path.join(REPO, "coordination", "scripts",
                            "white_evidence_observer.py")
        with open(path, encoding="utf-8") as fh:
            src = fh.read()
        self.assertEqual(1, src.count("rospy.ServiceProxy("))
        self.assertIn('rospy.ServiceProxy("/gazebo/get_link_state"', src)
        self.assertIn("GetLinkState", src)
        # same link and offset the production node uses
        self.assertIn("::cgo3_camera_link", src)
        self.assertIn("0,0,-0.162", src)

    def test_default_pose_source_mirrors_production(self):
        import white_evidence_observer as O
        args = O.build_arg_parser().parse_args([])
        self.assertEqual("service", args.pose_source)
        self.assertEqual(1.0, args.image_max_age_s)   # same gate as production
        self.assertFalse(args.enable)                 # disabled unless asked

    def test_perception_real_untouched_by_this_task(self):
        path = os.path.join(REPO, "perception", "perception_real.py")
        with open(path, encoding="utf-8") as fh:
            src = fh.read()
        # the observer must not have leaked truth logic into perception
        self.assertNotIn("white_evidence_observer", src)
        self.assertNotIn("white_dev_observer", src)


class LinkResolution(unittest.TestCase):
    """uav_id -> gazebo model -> camera link, wiring first, template fallback."""

    def _args(self, **kw):
        import white_evidence_observer as O
        a = O.build_arg_parser().parse_args([])
        for k, v in kw.items():
            setattr(a, k, v)
        return a

    def test_wiring_is_authoritative(self):
        import white_evidence_observer as O
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
            json.dump({"uavs": [{"uav_id": "uav_3", "model_name": "typhoon_h480_2"}]}, fh)
            p = fh.name
        try:
            links, rows = O.resolve_links(self._args(wiring=p, uavs="uav_3"))
        finally:
            os.unlink(p)
        self.assertEqual("typhoon_h480_2::cgo3_camera_link", links["uav_3"][1])
        self.assertEqual(1, len(rows))

    def test_template_fallback_maps_uav_N_to_h480_Nminus1(self):
        import white_evidence_observer as O
        links, rows = O.resolve_links(self._args(uavs="uav_1,uav_6"))
        self.assertEqual([], rows)
        self.assertEqual("typhoon_h480_0::cgo3_camera_link", links["uav_1"][1])
        self.assertEqual("typhoon_h480_5::cgo3_camera_link", links["uav_6"][1])

    def test_missing_wiring_file_does_not_crash(self):
        import white_evidence_observer as O
        links, rows = O.resolve_links(self._args(wiring="/nope/none.json",
                                                 uavs="uav_2"))
        self.assertEqual("typhoon_h480_1::cgo3_camera_link", links["uav_2"][1])


class Quaternion(unittest.TestCase):
    def test_identity_and_yaw_180(self):
        import white_evidence_observer as O
        import numpy as np
        self.assertTrue(np.allclose(np.eye(3), O.quat_to_R(0, 0, 0, 1)))
        # 180 deg about z
        R = O.quat_to_R(0, 0, 1, 0)
        self.assertTrue(np.allclose(np.diag([-1.0, -1.0, 1.0]), R))


if __name__ == "__main__":
    unittest.main(verbosity=2)