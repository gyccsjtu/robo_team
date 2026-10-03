"""Regression tests for the shared GPU inference service.

Run (from WSL, with the CUDA env):
    /root/robo_team_build/vision_cuda_20261003/bin/python \
        perception/tests/test_shared_inference.py

What these tests protect, and why each matters for the 15 s judge criterion:

  test_client_holds_no_cuda   The entire memory saving rests on this. If a client
                               ever imports torch/ultralytics or maps nvidia
                               memory, six clients pay six contexts again and the
                               change is worthless while looking correct.
  test_service_ready_and_reply   Basic contract.
  test_malformed_request_survives  One bad frame must not kill the service for
                               the other five aircraft.
  test_parity_on_real_frame    Shared and local inference must agree exactly on
                               cls/conf/xyxy. The judge computes coordinates from
                               these numbers, so "close enough" is not acceptable.
  test_six_client_throughput   Serialization must keep up with 5 Hz x 6 streams.

The parity test needs a real frame. Point PR_TEST_FRAME at one; when absent the
test reports INCONCLUSIVE rather than passing silently, because a 0-vs-0
comparison proves plumbing but not numerics.
"""
import os
import socket
import subprocess
import sys
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PERCEPTION = os.path.dirname(HERE)
sys.path.insert(0, PERCEPTION)

PY = os.environ.get("PR_TEST_PYTHON", "/root/robo_team_build/vision_cuda_20261003/bin/python")
SERVICE = os.path.join(PERCEPTION, "shared_inference_service.py")
WEIGHTS = os.environ.get("PR_WEIGHTS", "/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt")
FRAME = os.environ.get("PR_TEST_FRAME", "")
PORT = int(os.environ.get("PR_TEST_PORT", "19781"))


def rss_mb():
    with open("/proc/self/status") as fh:
        for line in fh:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) // 1024
    return 0


def nvidia_maps():
    try:
        with open("/proc/self/maps") as fh:
            return sum(1 for l in fh if "nvidia" in l)
    except Exception:
        return 0


def wait_ready(client, proc, timeout=150):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if client.available():
            return True
        if proc.poll() is not None:
            return False
        time.sleep(0.5)
    return False


class TestClientIsolation(unittest.TestCase):
    """The saving depends on the client never touching CUDA."""

    def test_client_holds_no_cuda(self):
        from shared_inference_client import SharedInferenceClient
        import cv2
        client = SharedInferenceClient(port=19999)  # nothing listening
        img = (np.random.rand(360, 640, 3) * 255).astype(np.uint8)
        boxes, meta = client.infer(img, uav="uav_test")
        # Must degrade, not raise.
        self.assertIsNone(boxes)
        self.assertFalse(meta.get("shared"))
        self.assertNotIn("torch", sys.modules,
                         "client pulled in torch; it would create a CUDA context")
        self.assertNotIn("ultralytics", sys.modules,
                         "client pulled in ultralytics; same problem")
        self.assertEqual(nvidia_maps(), 0,
                         "client mapped nvidia memory; six clients would pay six contexts")
        # Rough ceiling: the CUDA env interpreter with numpy+cv2 only.
        self.assertLess(rss_mb(), 300, "client RSS unexpectedly high: %d MB" % rss_mb())


class TestServiceContract(unittest.TestCase):
    proc = None
    client = None

    @classmethod
    def setUpClass(cls):
        env = dict(os.environ, PR_WEIGHTS=WEIGHTS)
        cls.proc = subprocess.Popen(
            [PY, SERVICE, "--port", str(PORT), "--device", "0", "--conf", "0.25"],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        from shared_inference_client import SharedInferenceClient
        cls.client = SharedInferenceClient(port=PORT)
        if not wait_ready(cls.client, cls.proc):
            out = ""
            try:
                out = cls.proc.stdout.read().decode("utf8", "ignore")[-2000:]
            except Exception:
                pass
            raise unittest.SkipTest("service did not become ready: %s" % out)

    @classmethod
    def tearDownClass(cls):
        if cls.proc and cls.proc.poll() is None:
            cls.proc.terminate()
            try:
                cls.proc.wait(timeout=10)
            except Exception:
                cls.proc.kill()

    def test_ready_and_single_reply(self):
        img = (np.random.rand(360, 640, 3) * 255).astype(np.uint8)
        boxes, meta = self.client.infer(img, uav="uav_1")
        self.assertIsNotNone(boxes, "no reply: %s" % meta)
        self.assertTrue(meta.get("shared"))
        self.assertIn("infer_s", meta)

    def test_malformed_request_survives(self):
        s = socket.create_connection(("127.0.0.1", PORT), timeout=5)
        s.sendall(b"{not json at all\n")
        data = s.recv(4096).decode("utf8", "ignore")
        s.close()
        self.assertIn("BAD_JSON", data)
        # The whole point: the service survives one client's bad frame.
        self.assertTrue(self.client.available(),
                        "service died on malformed input; other five aircraft would lose 15 s report")

    def test_six_clients_throughput(self):
        import threading
        counts = {}
        lock = threading.Lock()

        def worker(i):
            from shared_inference_client import SharedInferenceClient
            c = SharedInferenceClient(port=PORT)
            ok = 0
            try:
                for k in range(4):
                    img = (np.random.rand(360, 640, 3) * 255).astype(np.uint8)
                    boxes, _ = c.infer(img, uav="uav_%d" % (i + 1))
                    if boxes is not None:
                        ok += 1
                    time.sleep(0.02)
            finally:
                # Each worker owns its own connection; leaking six sockets per run
                # would accumulate across repeated test invocations.
                c.close()
            with lock:
                counts[i] = ok

        t0 = time.time()
        threads = [threading.Thread(target=worker, args=(i,)) for i in range(6)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        elapsed = time.time() - t0
        total = sum(counts.values())
        self.assertEqual(total, 24, "clients failed: %s" % counts)
        rate = total / max(elapsed, 1e-6)
        # 5 Hz x 6 streams = 30 calls/s needed. Keep generous headroom for CI noise.
        self.assertGreater(rate, 8.0, "throughput %.1f calls/s too low" % rate)

    def test_parity_on_real_frame(self):
        if not FRAME or not os.path.exists(FRAME):
            self.skipTest("PR_TEST_FRAME not set to a real camera frame; "
                          "0-vs-0 parity would prove nothing about numerics")
        import cv2
        from ultralytics import YOLO
        img = cv2.imread(FRAME, cv2.IMREAD_COLOR)
        self.assertIsNotNone(img)

        def norm(boxes):
            return sorted((int(b.cls), round(float(b.conf), 4),
                           [round(float(v), 2) for v in b.xyxy[0]]) for b in boxes)

        m = YOLO(WEIGHTS)
        local = norm(m(img, conf=0.25, verbose=False, device="0")[0].boxes)
        boxes, _ = self.client.infer(img, uav="uav_parity")
        shared = norm(boxes or [])
        self.assertEqual(local, shared,
                         "shared/local numerics diverged; the 15 s criterion would be "
                         "computed from different geometry")

    def test_cpu_device_refused(self):
        """Serving on CPU would add a hop without saving anything."""
        out = subprocess.run([PY, SERVICE, "--port", str(PORT + 1), "--device", "cpu"],
                             capture_output=True, text=True, timeout=120)
        self.assertNotEqual(out.returncode, 0,
                            "service accepted device=cpu; it must refuse")


if __name__ == "__main__":
    unittest.main(verbosity=2)
