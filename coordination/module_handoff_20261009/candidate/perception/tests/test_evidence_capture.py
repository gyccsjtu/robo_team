"""N12 tests for the bounded evidence capture. Pure CPU, no ROS/Gazebo.

Run:  python perception/tests/test_evidence_capture.py
"""
import json
import os
import sys
import tempfile
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PERCEPTION = os.path.dirname(HERE)
sys.path.insert(0, PERCEPTION)

from evidence_capture import EvidenceCapture


class FakeBuf:
    def __init__(self, data):
        self._data = data

    def tobytes(self):
        return self._data


def install_fake_cv2():
    """No OpenCV on the test box; capture's bounds are what we test here."""
    fake = type(sys)('fake_cv2')
    fake.imencode = lambda ext, img: (True, FakeBuf(b'PNGDATA' + bytes([id(img) % 251])))
    sys.modules['cv2'] = fake


def remove_fake_cv2():
    sys.modules.pop('cv2', None)


def fake_image(h=48, w=64):
    return np.zeros((h, w, 3), dtype=np.uint8)


class Capture(unittest.TestCase):
    def setUp(self):
        install_fake_cv2()
        self.dir = tempfile.mkdtemp(prefix="ev_test_")

    def tearDown(self):
        remove_fake_cv2()

    def make(self, **kw):
        return EvidenceCapture("uav_test", out_dir=self.dir, **kw)

    def test_disabled_writes_nothing(self):
        ev = self.make(enabled=False)
        self.assertFalse(ev.capture(fake_image(), dict(cls='green')))
        self.assertEqual(ev.stats()['written'], 0)
        self.assertFalse(os.path.isdir(ev.dir))

    def test_capture_writes_png_and_index(self):
        ev = self.make()
        ok = ev.capture(fake_image(), dict(cls='green', color_conf=0.75,
                                           image_stamp=123.5))
        self.assertTrue(ok)
        self.assertTrue(os.path.isfile(os.path.join(ev.dir, "index.jsonl")))
        rows = [json.loads(l) for l in
                open(os.path.join(ev.dir, "index.jsonl"), encoding="utf-8")]
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['cls'], 'green')
        self.assertEqual(rows[0]['schema_version'], 1)
        self.assertTrue(os.path.isfile(os.path.join(ev.dir, rows[0]['png'])))

    def test_token_bucket_limits_per_minute(self):
        ev = self.make(per_minute=2, total=100)
        results = [ev.capture(fake_image(), dict(cls='green')) for _ in range(5)]
        self.assertEqual(sum(1 for r in results if r), 2)
        self.assertEqual(ev.stats()['dropped'], 3)

    def test_total_cap_stops(self):
        ev = self.make(per_minute=100, total=3)
        for i in range(5):
            ev.capture(fake_image(), dict(cls='green', i=i))
        self.assertEqual(ev.stats()['written'], 3)
        rows = [json.loads(l) for l in
                open(os.path.join(ev.dir, "index.jsonl"), encoding="utf-8")]
        self.assertEqual(len(rows), 3)

    def test_bad_image_never_raises(self):
        ev = self.make()
        # Whatever cv2 does with a None/non-array image, capture must not raise
        # and must not poison later captures. (The fake encoder accepts None,
        # real cv2 raises into the except path — both are fine, no raise.)
        try:
            ev.capture(None, dict(cls='green'))
            ev.capture("not-an-image", dict(cls='green'))
        except Exception as error:
            self.fail("capture raised on bad input: %r" % error)
        # A good capture afterwards still works (no poisoned state).
        self.assertTrue(ev.capture(fake_image(), dict(cls='green')))

    def test_index_line_absent_when_encode_fails(self):
        # PNG encode happens before the index append: a failed encode must
        # leave no index line behind (no orphan rows pointing at missing PNGs).
        class ExplodingCv2:
            def imencode(self, ext, img):
                raise ValueError("encode exploded")
        sys.modules['cv2'] = ExplodingCv2
        try:
            ev = self.make()
            self.assertFalse(ev.capture(fake_image(), dict(cls='green')))
        finally:
            install_fake_cv2()   # restore for tearDown
        self.assertFalse(os.path.isfile(os.path.join(ev.dir, "index.jsonl")))


if __name__ == '__main__':
    unittest.main(verbosity=2)
