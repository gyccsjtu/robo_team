#!/usr/bin/env python3
"""Reproduce the 222 selected offline candidate checks without editing sources.

Run from any working directory with Python 3.11+ and NumPy installed:
    python -X utf8 -B verify_handoff.py --output-dir /writable/test-results

The checked snapshot is resolved relative to this script, not the shell cwd.
Each test group uses a fresh interpreter. Reports distinguish offline checks
from unperformed ROS/PX4/Gazebo, six-aircraft, and three-dimensional validation.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
CANDIDATE = HERE / "candidate"
GROUPS = {
    "radar": {
        "expected_tests": 71,
        "modules": [
            "test_radar_velocity_guard", "test_radar_observed_map",
            "test_online_radar_planner", "test_position_brake",
            "test_fleet_motion_guard", "test_bounded_escape",
            "test_bounded_escape_agent",
        ],
    },
    "authority": {
        "expected_tests": 88,
        "modules": [
            "test_task_authority_v2", "test_search_lease_v2",
            "test_authority_elimination", "test_search_occupancy",
            "test_route_reservation", "test_search_completion",
            "test_official_tracker", "test_search_observation",
        ],
    },
    "perception": {
        "expected_tests": 63,
        "modules": [
            "test_camera_geometry", "test_camera_track_evidence",
            "test_visual_report_frame_dedup", "test_visual_observation",
            "test_red_observations", "test_official_judge_transport",
            "test_bridge_acceptance", "test_official_report_sources",
            "test_official_report_coast", "test_blue_source_alignment",
        ],
    },
}
SCOPE = (
    "Selected offline pure-module tests and extracted actual adapter methods; "
    "no ROS/PX4/Gazebo execution, six-aircraft flight, or 3D safety validation."
)
PACKAGING_MARKERS = {"CATKIN_IGNORE", "COLCON_IGNORE"}


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def prepare_runtime(output: Path) -> None:
    """All test temporary files stay in the chosen writable output directory."""
    temporary = output / "temporary"
    temporary.mkdir(parents=True, exist_ok=True)
    os.environ.update(TEMP=str(temporary), TMP=str(temporary), TMPDIR=str(temporary),
                      PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8",
                      PYTHONUTF8="1")
    tempfile.tempdir = str(temporary)
    sys.dont_write_bytecode = True


def test_ids(suite: unittest.TestSuite) -> list[str]:
    result = []
    for test in suite:
        if isinstance(test, unittest.TestSuite):
            result.extend(test_ids(test))
        else:
            result.append(test.id())
    return result


def run_group(name: str, output: Path) -> int:
    prepare_runtime(output)
    paths = [CANDIDATE / "coordination/tests", CANDIDATE / "perception",
             CANDIDATE / "coordination/src/robocup_swarm/scripts",
             CANDIDATE / "coordination/src/robocup_navigation/src"]
    sys.path[:0] = [str(path) for path in paths]
    config = GROUPS[name]
    suite = unittest.TestLoader().loadTestsFromNames(config["modules"])
    ids = test_ids(suite)
    with (output / (name + ".txt")).open("w", encoding="utf-8") as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    absent = [module for module in config["modules"]
              if not any(test_id.startswith(module + ".") for test_id in ids)]
    incomplete = bool(result.skipped or result.expectedFailures or absent
                      or result.testsRun != config["expected_tests"])
    status = "FAIL" if not result.wasSuccessful() else "INCOMPLETE" if incomplete else "PASS"
    record = dict(
        group=name, status=status, expected_tests=config["expected_tests"],
        tests_run=result.testsRun, modules=config["modules"], test_ids=ids,
        missing_modules=absent,
        failures=[dict(test_id=test.id(), traceback=trace) for test, trace in result.failures],
        errors=[dict(test_id=test.id(), traceback=trace) for test, trace in result.errors],
        skipped=[dict(test_id=test.id(), reason=reason) for test, reason in result.skipped],
        expected_failures=[dict(test_id=test.id(), traceback=trace)
                           for test, trace in result.expectedFailures],
        unexpected_successes=[test.id() for test in result.unexpectedSuccesses],
        scope=SCOPE,
    )
    write_json(output / (name + ".json"), record)
    return 0 if status == "PASS" else 1


def verify_manifest(path: Path) -> dict:
    """Check SHA-256 against an explicit relative-path source manifest.

    Accepted JSON: {"files": [{"path": "candidate/...", "sha256": "..."}]}.
    Paths may also be candidate-relative. Hash all bytes; no newline or encoding
    normalization is performed. No listed file may escape the candidate tree.
    """
    if not path.is_file():
        return dict(status="MISSING_EVIDENCE", path=str(path),
                    reason="Source manifest is absent; provenance is not verified.")
    problems = []
    checked = []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        entries = data["files"]
        if isinstance(entries, dict):
            entries = [dict(path=name, sha256=digest) for name, digest in entries.items()]
        if not isinstance(entries, list) or not entries:
            raise ValueError("files must contain at least one source file")
        seen = set()
        root = CANDIDATE.resolve()
        for entry in entries:
            relative = Path(entry["path"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("Manifest contains an unsafe path: " + str(relative))
            file = ((HERE if relative.parts[0] == "candidate" else root) / relative).resolve()
            if not file.is_relative_to(root):
                raise ValueError("Manifest path escapes candidate: " + str(relative))
            key = file.relative_to(root).as_posix()
            if key in seen:
                raise ValueError("Duplicate manifest path: " + key)
            seen.add(key)
            expected = entry["sha256"].lower()
            if len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
                raise ValueError("Invalid SHA-256 for " + key)
            if not file.is_file():
                problems.append(dict(path=key, reason="missing_file"))
                continue
            contents = file.read_bytes()
            actual = hashlib.sha256(contents).hexdigest()
            if actual != expected:
                problems.append(dict(path=key, reason="hash_mismatch", actual=actual,
                                     expected=expected))
            if "bytes" in entry and entry["bytes"] != len(contents):
                problems.append(dict(path=key, reason="byte_count_mismatch",
                                     expected=entry["bytes"], actual=len(contents)))
            if "git_blob_oid" in entry:
                blob = b"blob " + str(len(contents)).encode("ascii") + b"\0" + contents
                oid = hashlib.sha1(blob).hexdigest()
                if entry["git_blob_oid"] != oid:
                    problems.append(dict(path=key, reason="git_blob_oid_mismatch",
                                         expected=entry["git_blob_oid"], actual=oid))
            checked.append(key)
        actual_files = {file.relative_to(root).as_posix()
                        for file in root.rglob("*") if file.is_file()
                        and "__pycache__" not in file.parts}
        for key in sorted(actual_files - seen - PACKAGING_MARKERS):
            problems.append(dict(path=key, reason="unlisted_source_file"))
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        return dict(status="FAIL", path=str(path), reason=str(error))
    return dict(status="PASS" if not problems else "FAIL", path=str(path),
                files_checked=len(checked), problems=problems,
                packaging_markers=sorted(actual_files & PACKAGING_MARKERS),
                repository_url=data.get("repository_url"),
                baseline_commit=data.get("baseline_commit"),
                candidate_commit=data.get("candidate_commit"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=HERE / "verification_output")
    parser.add_argument("--manifest", type=Path, default=HERE / "source_manifest.json")
    parser.add_argument("--worker", choices=GROUPS, help=argparse.SUPPRESS)
    args = parser.parse_args()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    prepare_runtime(output)
    if args.worker:
        return run_group(args.worker, output)
    manifest = verify_manifest(args.manifest.expanduser().resolve())
    records = []
    environment = os.environ.copy()
    for name, config in GROUPS.items():
        if manifest["status"] != "PASS":
            records.append(dict(group=name, status="INCOMPLETE", tests_run=0,
                                expected_tests=config["expected_tests"],
                                modules=config["modules"], failures=[], errors=[], skipped=[],
                                reason="Source verification did not pass; tests were not run."))
            continue
        command = [sys.executable, "-X", "utf8", "-B", str(Path(__file__).resolve()),
                   "--output-dir", str(output), "--worker", name]
        result_path = output / (name + ".json")
        if result_path.is_file():
            result_path.unlink()
        completed = subprocess.run(command, cwd=CANDIDATE, env=environment,
                                   text=True, encoding="utf-8", errors="replace",
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (output / (name + "_process.txt")).write_text(
            completed.stdout + completed.stderr, encoding="utf-8")
        try:
            record = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            record = dict(group=name, status="FAIL", tests_run=0,
                          expected_tests=config["expected_tests"],
                          modules=config["modules"], reason="No valid worker result: " + str(error),
                          skipped=[], failures=[], errors=[])
        record["process_exit_code"] = completed.returncode
        if completed.returncode and record["status"] == "PASS":
            record["status"] = "FAIL"
        records.append(record)
    final_manifest = verify_manifest(args.manifest.expanduser().resolve())
    failures = (any(record["status"] == "FAIL" for record in records)
                or manifest["status"] == "FAIL" or final_manifest["status"] == "FAIL")
    complete = (all(record["status"] == "PASS" for record in records)
                and manifest["status"] == "PASS" and final_manifest["status"] == "PASS")
    status = "FAIL" if failures else "PASS" if complete else "INCOMPLETE"
    summary = dict(
        status=status, generated_utc=datetime.now(timezone.utc).isoformat(),
        python=sys.version, executable=sys.executable,
        candidate=str(CANDIDATE), source_manifest=manifest,
        source_manifest_after_tests=final_manifest,
        expected_tests=222, tests_run=sum(record["tests_run"] for record in records),
        failures=sum(len(record["failures"]) for record in records),
        errors=sum(len(record["errors"]) for record in records),
        skipped=sum(len(record["skipped"]) for record in records),
        groups=records, scope=SCOPE,
        unperformed_validation=["ROS/PX4/Gazebo runtime", "six-aircraft physical flight",
                                "3D clearance and collision safety", "Linux publisher file locking"],
    )
    write_json(output / "summary.json", summary)
    lines = ["Overall offline/source verification: " + status, SCOPE,
             "Source manifest before/after tests: %s / %s" % (
                 manifest["status"], final_manifest["status"]),
             "Tests: %s / 222; failures=%s errors=%s skipped=%s" % (
                 summary["tests_run"], summary["failures"], summary["errors"], summary["skipped"])]
    lines.extend("%s: %s (%s / %s tests)" % (
        record["group"], record["status"], record["tests_run"], record["expected_tests"])
        for record in records)
    lines.append("Unperformed checks are not PASS; see summary.json for details.")
    report = "\n".join(lines) + "\n"
    (output / "summary.txt").write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
