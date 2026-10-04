"""Client helper for the shared GPU inference service (2026-10-03).

Used by perception_real.py when PR_SHARED_INFER=1. Design rules:

1. Never raise into the caller's detection path. A shared-service problem
   must degrade to in-process inference, because the judge criterion depends on
   a continuous 15 s report and a silently dead perception node is worse than a
   slower one.
2. Report every fallback so the log shows what actually happened rather than
   what was intended.
3. Return the same shape perception_real.py already consumes: a list of
   dicts with cls / conf / xyxy. The in-process path returns ultralytics Boxes,
   so the caller normalizes; see _normalize_boxes in perception_real.py.
"""

import base64
import json
import os
import socket
import time

import numpy as np

TIMEOUT_S = float(os.environ.get("PR_SHARED_TIMEOUT", "2.0"))
RETRY_S = float(os.environ.get("PR_SHARED_RETRY_S", "5.0"))


class Box:
    """Minimal stand-in for an ultralytics Boxes element.

    perception_real.py reads exactly three attributes off each detection:
    cls, conf, xyxy (verified at its single inference call site). Exposing
    those three keeps the shared path interchangeable with the local one.
    """

    __slots__ = ("cls", "conf", "xyxy")

    def __init__(self, cls, conf, xyxy):
        self.cls = cls
        self.conf = conf
        # ultralytics exposes xyxy as [array([x1,y1,x2,y2])]; the consumer
        # does float(b.xyxy[0]) then unpacks 4 values, so mirror that shape.
        self.xyxy = [np.asarray(xyxy, dtype=np.float64)]


class SharedInferenceClient:
    def __init__(self, port=None, host="127.0.0.1", img_fmt=None):
        self.host = host
        self.port = port or int(os.environ.get("PR_SHARED_PORT", "19731"))
        self.img_fmt = img_fmt or os.environ.get("PR_SHARED_IMG_FMT", "png")
        self._sock = None
        self._fh = None
        self._next_retry = 0.0
        self.fallbacks = 0
        self.ok_calls = 0
        self.last_error = ""

    # -- connection ------------------------------------------------------
    def _connect(self):
        if self._fh is not None:
            return True
        if time.time() < self._next_retry:
            return False
        self._next_retry = time.time() + RETRY_S
        try:
            s = socket.create_connection((self.host, self.port), timeout=TIMEOUT_S)
            s.settimeout(TIMEOUT_S)
            self._sock = s
            self._fh = s.makefile("rwb")
            return True
        except Exception as error:
            self.last_error = "connect:%s" % error
            self._close()
            return False

    def _close(self):
        for handle in (self._fh, self._sock):
            try:
                if handle is not None:
                    handle.close()
            except Exception:
                pass
        self._fh = None
        self._sock = None

    def close(self):
        """Release the connection. Callers that open many short-lived clients
        (e.g. tests) must call this, otherwise sockets accumulate."""
        self._close()

    def _rpc(self, payload, timeout=None):
        if not self._connect():
            return None
        try:
            self._fh.write((json.dumps(payload) + "\n").encode("utf-8"))
            self._fh.flush()
            # Makefile readline uses the socket timeout set at connect time.
            line = self._fh.readline()
            if not line:
                self.last_error = "eof"
                self._close()
                return None
            return json.loads(line.decode("utf-8"))
        except socket.timeout:
            self.last_error = "timeout"
            self._close()
            return None
        except Exception as error:
            self.last_error = "rpc:%s" % error
            self._close()
            return None

    # -- public ----------------------------------------------------------
    def available(self):
        reply = self._rpc(dict(op="ping"), timeout=1.0)
        return bool(reply and reply.get("ok") and reply.get("ready"))

    def infer(self, img, uav=""):
        """Return (boxes, meta). boxes is None when the caller must fall back."""
        import cv2
        flag = cv2.IMREAD_COLOR if self.img_fmt == "jpg" else cv2.IMREAD_UNCHANGED
        ok, buf = cv2.imencode(".%s" % self.img_fmt, img)
        if not ok:
            self.last_error = "encode_failed"
            return None, dict(shared=False, error="encode_failed")
        reply = self._rpc(dict(op="infer", uav=uav,
                               img_b64=base64.b64encode(buf.tobytes()).decode("ascii")))
        if not reply or not reply.get("ok"):
            if reply and reply.get("error"):
                self.last_error = str(reply["error"])
            return None, dict(shared=False, error=self.last_error or "no_reply")
        self.ok_calls += 1
        boxes = [Box(d["cls"], d["conf"], d["xyxy"]) for d in reply.get("dets", [])]
        return boxes, dict(shared=True, infer_s=reply.get("infer_s"),
                           queue_s=reply.get("queue_s"),
                           verified_person_boxes=reply.get('verified_person_boxes',[])
                               if reply.get('verification_version') == 1
                               and isinstance(reply.get('verified_person_boxes'),list) else [],
                           verified_person_colors=[c for c in reply.get('verified_person_colors',[])
                               if c in ('green','white')] if reply.get('verification_version') == 1
                               and isinstance(reply.get('verified_person_colors'),list) else [],
                           verified_green_person=(reply.get('verification_version') == 1
                               and reply.get('verified_green_person') is True))

    def note_fallback(self, reason):
        self.fallbacks += 1
        self.last_error = reason

    def stats(self):
        return self._rpc(dict(op="stats"))
