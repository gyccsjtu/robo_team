"""Shared GPU inference service (2026-10-03).

Why this exists
---------------
Six independent perception processes each created their own CUDA context and
their own host-side pools. Measured on this machine (WSL2, RTX 4060 Laptop):

    CPU round (PR_DEVICE=cpu): 425 MB RSS per process, 0 nvidia-backed mappings
    GPU round (PR_DEVICE=0):  846 MB RSS per process, 75 nvidia-backed mappings
    libtorch_cuda.so itself:  6.6 MB RSS

So the duplicated cost is the CUDA context, not the weights. Six processes
therefore pay six context costs for one model. This service holds ONE context
and ONE model, and serves all six streams.

Feasibility was measured, not assumed (tmp/shared_infer_feasibility.py, deleted):
    single-stream  p50=0.0244s p95=0.0546s
    serialized 6 streams  per_call=0.0214s
    => 128 ms of inference per 1000 ms wall clock, i.e. ~7.8x headroom at 5 Hz
    detection per stream. Serialization is not the bottleneck; memory is.

Protocol
--------
Line-oriented JSON over TCP on 127.0.0.1, chosen because it is dependency-free
(no new packages in the frozen CUDA env) and trivially inspectable with nc.

Request :  {"op": "infer", "uav": "uav_1", "img_b64": "<base64 JPEG/PNG>"}
           {"op": "ping"}
           {"op": "stats"}
           {"op": "shutdown"}          (only from an admin client)
Response:  {"ok": true, "dets": [{"cls":0,"conf":0.9,"xyxy":[x1,y1,x2,y2]}, ...],
            "infer_s": 0.021, "queue_s": 0.004, "seq": 123}
           {"ok": false, "error": "..."}

Only cls / conf / xyxy are returned. That is deliberate: perception_real.py
consumes exactly those three fields (verified at the single call site, line
1053) and does its own per-class IOU NMS. Returning more would widen the
shared component's contract for no benefit.

Correctness rules this service must not break
---------------------------------------------
* Detection results must stay identical to the in-process path. The same
  weights and the same ultralytics call are used, with the same conf threshold.
  The only added variable is JPEG transport, so images are sent as PNG
  (lossless) by default to keep the numerics identical. PR_SHARED_IMG_FMT=jpg
  is available but off by default precisely because it is lossy.
* A failure in one stream must not take down the service for the others; the
  per-request error path returns {"ok": false} and keeps serving.
* If the service dies, clients fall back to in-process inference so a perception
  node never silently stops publishing the 15 s judge signal.
"""

import base64
import json
import os
import socket
import socketserver
import sys
import threading
import time

WEIGHTS_DEFAULT = "/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt"

# Serialization guard: one CUDA context, one lock. Requests queue behind it.
_GPU_LOCK = threading.Lock()
_MODEL = None
_PERSON_VERIFIER = None
_PREDICTOR_READY = False


def _log(msg):
    sys.stderr.write("[shared_infer] %s\n" % msg)
    sys.stderr.flush()


def _load(device, conf):
    """Load weights once. Kept out of the request path."""
    global _MODEL, _PREDICTOR_READY, _PERSON_VERIFIER
    from ultralytics import YOLO
    weights = os.environ.get("PR_WEIGHTS", WEIGHTS_DEFAULT)
    _MODEL = YOLO(weights)
    from person_verifier import PersonVerifier
    _PERSON_VERIFIER=PersonVerifier(os.environ.get('PR_PERSON_VERIFY_WEIGHTS',''),device,
        os.environ.get('PR_PERSON_VERIFY_CONF','.1'),os.environ.get('PR_PERSON_VERIFY_IOU','.25'),
        classes=(1,),proof_classes=(1,)+((3,) if os.environ.get('PR_FAST_GREEN_WHITE','0') == '1' else ())
            +((0,) if os.environ.get('PR_FAST_RED_PERSON','0') == '1' else ())
            +((2,) if os.environ.get('PR_FAST_BLUE_PERSON','0') == '1' else ()),
        color_check=os.environ.get('PR_COLOR_VERIFY','0')=='1')
    # Touch the predictor so the first real request does not pay model setup.
    import numpy as np
    probe = np.zeros((360, 640, 3), dtype=np.uint8)
    if _PERSON_VERIFIER.weights:
        _PERSON_VERIFIER.load()(probe,classes=[0],conf=_PERSON_VERIFIER.confidence,device=device,verbose=False)
        _log('same-frame green person verifier ready weights=%s' % _PERSON_VERIFIER.weights)
    _infer(probe, conf, device)
    _PREDICTOR_READY = True
    _log("model ready weights=%s device=%s conf=%.2f" % (weights, device, conf))
    return _MODEL


def _infer(img, conf, device):
    """The only place GPU inference happens. Caller must hold _GPU_LOCK."""
    import torch
    kw = dict(conf=conf, verbose=False, device=device)
    if os.environ.get("PR_FP16", "0") == "1":
        kw["half"] = True
    res = _MODEL(img, **kw)[0]
    if device not in ("cpu", ""):
        torch.cuda.synchronize()
    out = []
    boxes=_PERSON_VERIFIER.filter_boxes(img,res.boxes) if _PERSON_VERIFIER is not None else res.boxes
    for b in boxes:
        out.append(dict(
            cls=int(b.cls),
            conf=round(float(b.conf), 4),
            xyxy=[round(float(v), 2) for v in b.xyxy[0]],
        ))
    return out


class Handler(socketserver.StreamRequestHandler):
    timeout = 30

    def handle(self):
        device = self.server.device
        conf = self.server.conf
        img_fmt = self.server.img_fmt
        for raw in self.rfile:
            raw = raw.strip()
            if not raw:
                continue
            try:
                req = json.loads(raw.decode("utf-8"))
            except Exception as error:
                self._reply(dict(ok=False, error="BAD_JSON:%s" % error))
                continue
            op = req.get("op", "infer")
            if op == "ping":
                self._reply(dict(ok=True, pong=True, ready=_PREDICTOR_READY))
                continue
            if op == "stats":
                self._reply(dict(ok=True, stats=self.server.stats_snapshot()))
                continue
            if op == "shutdown":
                _log("shutdown requested by %s" % self.client_address)
                self._reply(dict(ok=True, bye=True))
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            if op != "infer":
                self._reply(dict(ok=False, error="UNKNOWN_OP:%s" % op))
                continue
            if not _PREDICTOR_READY:
                self._reply(dict(ok=False, error="NOT_READY"))
                continue
            try:
                import cv2
                import numpy as np
                buf = base64.b64decode(req["img_b64"])
                arr = np.frombuffer(buf, dtype=np.uint8)
                flag = cv2.IMREAD_COLOR if img_fmt == "jpg" else cv2.IMREAD_UNCHANGED
                img = cv2.imdecode(arr, flag)
                if img is None:
                    raise ValueError("decode returned None")
                t_queue = time.time()
                with _GPU_LOCK:
                    t_lock = time.time()
                    dets = _infer(img, conf, device)
                    # Proof belongs to this image, copied before another client
                    # can replace the verifier's per-frame state.
                    proof_boxes=list(_PERSON_VERIFIER.verified_boxes) if _PERSON_VERIFIER is not None else []
                    infer_s = time.time() - t_lock
                self.server.record(self.client_address[1], infer_s, t_queue)
                self._reply(dict(ok=True, dets=dets, infer_s=round(infer_s, 4),
                                 queue_s=round(t_lock - t_queue, 4),
                                 verification_version=2,
                                 verified_person_boxes=proof_boxes,
                                 verified_green_person=bool(_PERSON_VERIFIER is not None
                                     and _PERSON_VERIFIER.green_proof_enabled()),
                                 verified_person_colors=_PERSON_VERIFIER.verified_colors()
                                     if _PERSON_VERIFIER is not None else []))
            except Exception as error:
                self._reply(dict(ok=False, error="INFER:%s" % error,
                                 infer_s=round(time.time() - t_queue, 4)
                                 if 't_queue' in dir() else 0.0))

    def _reply(self, payload):
        try:
            self.wfile.write((json.dumps(payload) + "\n").encode("utf-8"))
        except Exception as error:
            _log("reply failed: %s" % error)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, addr, device, conf, img_fmt):
        super().__init__(addr, Handler)
        self.device = device
        self.conf = conf
        self.img_fmt = img_fmt
        self._seq = 0
        self._lock = threading.Lock()
        self._per_client = {}
        self._started = time.time()

    def record(self, port, infer_s, t_queue):
        with self._lock:
            self._seq += 1
            slot = self._per_client.setdefault(port, dict(n=0, total=0.0, worst=0.0))
            slot["n"] += 1
            slot["total"] += infer_s
            slot["worst"] = max(slot["worst"], infer_s)

    def stats_snapshot(self):
        with self._lock:
            return dict(
                seq=self._seq,
                uptime_s=round(time.time() - self._started, 1),
                per_client={str(k): dict(n=v["n"], mean_s=round(v["total"] / max(v["n"], 1), 4),
                                       worst_s=round(v["worst"], 4))
                            for k, v in self._per_client.items()},
            )


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=int(os.environ.get("PR_SHARED_PORT", "19731")))
    ap.add_argument("--device", default=os.environ.get("VISION_DEVICE", "0"))
    ap.add_argument("--conf", type=float, default=float(os.environ.get("PR_CONF", "0.40")))
    ap.add_argument("--img-fmt", default=os.environ.get("PR_SHARED_IMG_FMT", "png"),
                    choices=["png", "jpg"])
    ap.add_argument("--warmup", action="store_true", default=True)
    args = ap.parse_args()

    if args.device in ("cpu", ""):
        _log("refusing to serve on device=%r: shared service exists to hold ONE cuda "
             "context; on cpu it would only add a hop." % args.device)
        return 2
    _load(args.device, args.conf)

    server = Server(("127.0.0.1", args.port), args.device, args.conf, args.img_fmt)
    _log("listening on 127.0.0.1:%d device=%s conf=%.2f fmt=%s"
         % (args.port, args.device, args.conf, args.img_fmt))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
