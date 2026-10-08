"""N12 regression tests: evidence enrichment in PersonVerifier, judgement unchanged.

Pure CPU, no ultralytics/torch. A fake person model stands in for the second
network; a reference copy of the LEGACY filter_boxes algorithm runs the same
inputs so judgement equality is asserted, not assumed.

Note: verified_boxes registration requires color_check=True (legacy behaviour,
protected here). Green/white never touch jersey_color inside the verifier, but
color_check=True imports its filter_boxes, so tests inject a fake module.

Run:  python perception/tests/test_person_verifier_rich.py
"""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PERCEPTION = os.path.dirname(HERE)
sys.path.insert(0, PERCEPTION)

from person_verifier import PersonVerifier, frame_verified, detection_key, overlap


class FakeBox:
    """Mirrors the ultralytics box surface filter_boxes touches: cls/conf/xyxy."""

    def __init__(self, cls, conf, xyxy):
        self.cls = cls
        self.conf = conf
        self.xyxy = [np.asarray(xyxy, dtype=np.float64)]


class FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class FakeModel:
    def __init__(self, people):
        self._people = people   # [(conf, xyxy4)]

    def __call__(self, image, classes=None, conf=None, device=None, verbose=False):
        return FakeResult([FakeBox(0, c, xyxy) for c, xyxy in self._people]),


def install_fake_jersey(shirt_supported=False, fractions=None):
    fake = type(sys)('fake_jersey_color')
    fake.torso_fractions = (lambda image, xyxy: fractions if fractions is not None
                            else {'red': 0.9, 'blue': 0.9})
    fake.supported = lambda color, fr: bool(shirt_supported)
    fake.filter_boxes = lambda image, boxes: boxes
    sys.modules['jersey_color'] = fake
    return fake


def remove_fake_jersey():
    sys.modules.pop('jersey_color', None)


def make_verifier(people, classes=(1,), proof_classes=None, minimum_overlap=.25,
                  color_check=False):
    v = PersonVerifier(weights='fake.pt', device='cpu', confidence=.1,
                       minimum_overlap=minimum_overlap, classes=classes,
                       color_check=color_check, proof_classes=proof_classes)
    v.model = FakeModel(people)   # bypass ultralytics load()
    return v


def box(cls, conf, xyxy):
    return FakeBox(cls, conf, xyxy)


# --- reference copy of the LEGACY algorithm (pre-N12) ------------------------
def legacy_filter(state, image, boxes):
    """Reimplements the pre-N12 behaviour using the same overlap()/detection_key."""
    weights = state['weights']
    classes = state['classes']
    proof_classes = state['proof_classes']
    minimum_overlap = state['minimum_overlap']
    color_check = state['color_check']
    people = state['people_xyxy']
    verified = []
    boxes = list(boxes)
    if not weights or not any(int(b.cls) in classes | proof_classes for b in boxes):
        return boxes, verified
    retained = []
    for box in boxes:
        cid = int(box.cls)
        matched = any(overlap(box.xyxy[0], person) >= minimum_overlap
                      for person in people)
        if cid not in classes or matched:
            retained.append(box)
        shirt_verified = True
        # legacy red/blue torso branches are no-ops without color_check
        if color_check and matched and cid in proof_classes and shirt_verified:
            key = detection_key(cid, box.xyxy[0])
            if key is not None:
                verified.append(dict(cls=key[0], xyxy=list(key[1])))
    return retained, verified
# -----------------------------------------------------------------------------


class JudgementUnchanged(unittest.TestCase):
    G = (10.0, 10.0, 30.0, 90.0)
    B = (50.0, 10.0, 70.0, 90.0)
    R = (90.0, 10.0, 110.0, 90.0)
    FAR = (200.0, 10.0, 220.0, 90.0)

    def run_pair(self, people, boxes, classes=(1,), proof_classes=(1,),
                 color_check=False):
        state = dict(weights='fake.pt', classes=set(classes),
                     proof_classes=set(proof_classes), minimum_overlap=.25,
                     color_check=color_check,
                     people_xyxy=[xyxy for _, xyxy in people])
        v = make_verifier(people, classes=classes, proof_classes=proof_classes,
                          color_check=color_check)
        new_retained = v.filter_boxes(None, boxes)
        legacy_retained, legacy_verified = legacy_filter(state, None, boxes)
        return v, new_retained, legacy_retained, legacy_verified

    def test_retained_bit_exact_green_forced_filter(self):
        people = [(0.9, (0.0, 0.0, 20.0, 80.0))]
        boxes = [box(1, .8, self.G), box(1, .3, self.FAR), box(2, .6, self.B)]
        v, new_r, old_r, _ = self.run_pair(people, boxes)
        self.assertEqual([(int(b.cls), tuple(np.round(b.xyxy[0], 2))) for b in new_r],
                         [(int(b.cls), tuple(np.round(b.xyxy[0], 2))) for b in old_r])

    def test_verified_keys_bit_exact_multi_class(self):
        install_fake_jersey(shirt_supported=True)
        try:
            people = [(0.9, (0.0, 0.0, 20.0, 80.0)),    # covers green box
                      (0.8, (48.0, 5.0, 72.0, 85.0)),   # covers blue box
                      (0.7, (88.0, 5.0, 112.0, 85.0))]  # covers red box
            boxes = [box(1, .8, self.G), box(2, .6, self.B), box(0, .7, self.R)]
            v, _, _, legacy_verified = self.run_pair(
                people, boxes, classes=(1,), proof_classes=(1, 2, 0),
                color_check=True)   # legacy registers only under color_check
        finally:
            remove_fake_jersey()
        new_keys = {(d['cls'], tuple(d['xyxy'])) for d in v.verified_boxes}
        old_keys = {(d['cls'], tuple(d['xyxy'])) for d in legacy_verified}
        self.assertEqual(new_keys, old_keys)
        self.assertEqual(len(new_keys), 3)

    def test_no_person_boxes_matches_legacy_no_match(self):
        people = []
        boxes = [box(1, .8, self.G)]
        v, new_r, old_r, legacy_verified = self.run_pair(people, boxes)
        self.assertEqual([(int(b.cls), tuple(b.xyxy[0])) for b in new_r],
                         [(int(b.cls), tuple(b.xyxy[0])) for b in old_r])
        self.assertEqual(v.verified_boxes, [])
        self.assertEqual(legacy_verified, [])

    def test_frame_verified_works_on_both_structures(self):
        old_style = [dict(cls=1, xyxy=[10.0, 10.0, 30.0, 90.0])]
        new_style = [dict(cls=1, xyxy=[10.0, 10.0, 30.0, 90.0],
                          person_xyxy=[0.0, 0.0, 20.0, 80.0], person_conf=.5,
                          person_iou=.7, n_person_boxes=1, reject_reason=None)]
        key_box = (1, [10.0, 10.0, 30.0, 90.0])
        self.assertEqual(frame_verified(old_style, *key_box),
                         frame_verified(new_style, *key_box))
        self.assertTrue(frame_verified(new_style, *key_box))

    def test_frame_verified_still_rejects_missing_box(self):
        new_style = [dict(cls=1, xyxy=[10.0, 10.0, 30.0, 90.0], person_iou=.7)]
        self.assertFalse(frame_verified(new_style, 1, [40.0, 10.0, 60.0, 90.0]))


class SupportFields(unittest.TestCase):
    def setUp(self):
        install_fake_jersey(shirt_supported=True)

    def tearDown(self):
        remove_fake_jersey()

    def test_support_fields_record_match_partner(self):
        people = [(0.4, (0.0, 0.0, 20.0, 80.0)), (0.1, (100.0, 0.0, 120.0, 80.0))]
        v = make_verifier(people, classes=(1,), proof_classes=(1,),
                          color_check=True)
        v.filter_boxes(None, [box(1, .9, (2.0, 5.0, 18.0, 78.0))])
        self.assertEqual(len(v.verified_boxes), 1)
        entry = v.verified_boxes[0]
        self.assertEqual(entry['cls'], 1)
        self.assertEqual(entry['person_xyxy'], [0.0, 0.0, 20.0, 80.0])
        self.assertEqual(entry['person_conf'], 0.4)
        self.assertEqual(entry['n_person_boxes'], 2)
        self.assertGreater(entry['person_iou'], 0.0)
        self.assertIsNone(entry['reject_reason'])

    def test_best_partner_is_highest_iou(self):
        people = [(0.1, (0.0, 0.0, 20.0, 80.0)), (0.9, (1.0, 1.0, 19.0, 79.0))]
        v = make_verifier(people, classes=(1,), proof_classes=(1,),
                          color_check=True)
        v.filter_boxes(None, [box(1, .9, (2.0, 5.0, 18.0, 78.0))])
        self.assertEqual(v.verified_boxes[0]['person_conf'], 0.9)

    def test_color_check_false_registers_nothing(self):
        # legacy behaviour: without color_check the proof list stays empty even
        # when a person overlaps. Protected so the gate semantics never drift.
        people = [(0.4, (0.0, 0.0, 20.0, 80.0))]
        v = make_verifier(people, classes=(1,), proof_classes=(1,),
                          color_check=False)
        v.filter_boxes(None, [box(1, .9, (2.0, 5.0, 18.0, 78.0))])
        self.assertEqual(v.verified_boxes, [])
        self.assertTrue(v.evidence[0]['matched'])


class RejectReasons(unittest.TestCase):
    def test_no_person_overlap(self):
        install_fake_jersey(shirt_supported=True)
        try:
            v = make_verifier([(0.9, (100.0, 0.0, 120.0, 80.0))],
                              classes=(1,), proof_classes=(1,), color_check=True)
            v.filter_boxes(None, [box(1, .8, (10.0, 10.0, 30.0, 90.0))])
        finally:
            remove_fake_jersey()
        self.assertEqual(len(v.rejected_boxes), 1)
        self.assertEqual(v.rejected_boxes[0]['reject_reason'], 'no_person_overlap')
        self.assertEqual(v.rejected_boxes[0]['best_person_iou'], 0.0)

    def test_proof_disabled_when_channel_off(self):
        # With color_check=False the proof channel never ran: a proof-class box
        # is neither verified nor "rejected" — it is explicitly proof_disabled.
        v = make_verifier([(0.9, (100.0, 0.0, 120.0, 80.0))],
                          classes=(1,), proof_classes=(1,))
        v.filter_boxes(None, [box(1, .8, (10.0, 10.0, 30.0, 90.0))])
        self.assertEqual(v.rejected_boxes, [])
        self.assertEqual(v.evidence[0]['reject_reason'], 'proof_disabled')
        self.assertFalse(v.evidence[0]['verified'])

    def test_shirt_unsupported_via_fake_jersey(self):
        install_fake_jersey(shirt_supported=False)
        try:
            v = make_verifier([(0.9, (0.0, 0.0, 20.0, 80.0))],
                              classes=(0,), proof_classes=(0,), color_check=True)
            v.filter_boxes(None, [box(0, .8, (2.0, 5.0, 18.0, 78.0))])
        finally:
            remove_fake_jersey()
        self.assertEqual(len(v.rejected_boxes), 1)
        self.assertEqual(v.rejected_boxes[0]['reject_reason'], 'shirt_unsupported')

    def test_not_proof_class_recorded_when_frame_enters_path(self):
        # A blue-only frame short-circuits before the verifier path (legacy
        # behaviour), so mix in a green box to make the frame relevant.
        v = make_verifier([(0.9, (0.0, 0.0, 20.0, 80.0))],
                          classes=(1,), proof_classes=(1,))
        v.filter_boxes(None, [box(2, .7, (2.0, 5.0, 18.0, 78.0)),
                              box(1, .8, (10.0, 10.0, 30.0, 90.0))])
        reasons = {e['cls']: e['reject_reason'] for e in v.evidence}
        self.assertEqual(reasons.get(2), 'not_proof_class')
        self.assertEqual(v.rejected_boxes, [])   # blue not in proof set here

    def test_rejected_subset_of_evidence(self):
        v = make_verifier([(0.9, (100.0, 0.0, 120.0, 80.0))],
                          classes=(1,), proof_classes=(1,))
        v.filter_boxes(None, [box(1, .8, (10.0, 10.0, 30.0, 90.0)),
                              box(2, .6, (50.0, 10.0, 70.0, 90.0))])
        self.assertEqual(len(v.evidence), 2)
        failed = [(e['cls'], tuple(e['xyxy'])) for e in v.evidence if not e['verified']]
        for r in v.rejected_boxes:
            self.assertIn((r['cls'], tuple(r['xyxy'])), failed)


class Passthrough(unittest.TestCase):
    def test_no_weights_returns_boxes_and_empty_evidence(self):
        v = PersonVerifier(weights='', device='cpu')
        boxes = [box(1, .8, (10.0, 10.0, 30.0, 90.0))]
        out = v.filter_boxes(None, boxes)
        self.assertEqual(out, boxes)
        self.assertEqual(v.evidence, [])
        self.assertEqual(v.verified_boxes, [])

    def test_irrelevant_classes_only_skip_model(self):
        v = make_verifier([(0.9, (0.0, 0.0, 20.0, 80.0))],
                          classes=(1,), proof_classes=(1,))
        boxes = [box(4, .9, (10.0, 10.0, 30.0, 90.0))]   # brown, unrelated
        out = v.filter_boxes(None, boxes)
        self.assertEqual(out, boxes)
        # N12: the skip is now *recorded*, not silent — brown boxes must be
        # visible to offline attribution with reason verifier_path_not_entered.
        self.assertEqual(len(v.evidence), 1)
        self.assertEqual(v.evidence[0]['cls'], 4)
        self.assertEqual(v.evidence[0]['reject_reason'], 'verifier_path_not_entered')
        self.assertIsNone(v.evidence[0]['n_person_boxes'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
