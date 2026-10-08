"""Bounded evidence capture for suspicious perception candidates (N12).

Log-only: nothing here feeds detection, tracking or reporting. Each capture is
one PNG plus one JSONL line, so offline review can separate person-model false
positives from wrong box association from colour-region errors.

Bounds (three independent caps, mirroring capture_visual_frames.py):
- token bucket: at most `per_minute` captures per UAV per rolling minute
- total cap: at most `total` captures per UAV; afterwards this instance stops
- every failure is swallowed with a console line; never raises into the loop

Disable entirely with PR_EVIDENCE_CAPTURE=0 (default enabled).
"""
import hashlib
import json
import math
import os
import threading
import time


def file_sha256(path, _cache={}):
    if path not in _cache:
        h = hashlib.sha256()
        try:
            with open(path, 'rb') as fh:
                for chunk in iter(lambda: fh.read(1 << 20), b''):
                    h.update(chunk)
        except OSError:
            _cache[path] = ''
        else:
            _cache[path] = h.hexdigest()
    return _cache[path]


class EvidenceCapture(object):
    def __init__(self, uav, out_dir=None, per_minute=6, total=300, enabled=None):
        self.uav = str(uav)
        root = out_dir or os.environ.get("ROBOCUP_LOG_DIR",
                                         os.path.expanduser("~/robocup_logs"))
        self.dir = os.path.join(str(root), "evidence_%s" % self.uav)
        self.per_minute = int(per_minute)
        self.total_cap = int(total)
        if enabled is None:
            enabled = os.environ.get("PR_EVIDENCE_CAPTURE", "1") == "1"
        self.enabled = bool(enabled)
        self._lock = threading.Lock()
        self._window = []          # wall-clock stamps inside the current minute
        self._count = 0
        self._seq = 0
        self._dropped = 0
        self._written = False

    def _params(self):
        return dict(pr_conf=os.environ.get("PR_CONF", ""),
                    pr_person_verify_conf=os.environ.get("PR_PERSON_VERIFY_CONF", ""),
                    pr_person_verify_iou=os.environ.get("PR_PERSON_VERIFY_IOU", ""),
                    color_model_sha=file_sha256(os.environ.get("PR_WEIGHTS", "")),
                    person_model_sha=file_sha256(
                        os.environ.get("PR_PERSON_VERIFY_WEIGHTS", "")),
                    run_id=os.environ.get("ROBOCUP_RUN_ID", ""))

    def capture(self, image, record):
        """record: dict with the candidate context; PNG saved beside index line."""
        if not self.enabled:
            return False
        try:
            now = time.time()
            with self._lock:
                if self._count >= self.total_cap:
                    self._dropped += 1
                    return False
                self._window = [t for t in self._window if now - t <= 60.0]
                if len(self._window) >= self.per_minute:
                    self._dropped += 1
                    return False
                self._window.append(now)
                self._count += 1
                self._seq += 1
                seq = self._seq
            os.makedirs(self.dir, exist_ok=True)
            import cv2
            name = "frame_%06d_%d" % (seq, int(now * 1000))
            png_path = os.path.join(self.dir, name + ".png")
            ok, buf = cv2.imencode(".png", image)
            if not ok:
                raise ValueError("encode_failed")
            with open(png_path, "wb") as fh:
                fh.write(buf.tobytes())
            row = dict(schema_version=1, uav=self.uav, seq=seq,
                       png=name + ".png", wall_time=now, **record)
            with self._lock:
                with open(os.path.join(self.dir, "index.jsonl"), "a",
                          encoding="utf-8") as fh:
                    fh.write(json.dumps(row, ensure_ascii=False) + "\n")
            self._written = True
            return True
        except Exception as error:
            print("[evidence_%s] capture failed: %s" % (self.uav, error),
                  flush=True)
            return False

    def stats(self):
        with self._lock:
            return dict(uav=self.uav, enabled=self.enabled, written=self._count,
                        dropped=self._dropped, dir=self.dir if self._written else "")
