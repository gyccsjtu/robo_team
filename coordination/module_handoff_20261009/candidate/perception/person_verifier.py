"""Same-frame person verification for custom color detector boxes.

No ROS, torch or CUDA import on the normal shared-client path. Original boxes,
classes, confidences and coordinates are retained; the second model only vetoes
selected classes without an overlapping person detection in this exact image.

2026-10-08 (N12, WorkBuddy): evidence enrichment only, judgement unchanged.
- verified_boxes entries gain support fields (which person box, its conf, the
  actual IoU, per-frame person count). `frame_verified` still keys on
  (cls, xyxy) only, so in-process v2-style dict consumers keep working.
  NOTE: this does NOT make the wire protocol backward compatible - a shared
  client that only accepts verification_version in (1,2) drops v3 replies
  entirely; service and all six clients must upgrade together.
- New `evidence` list: every colour box that entered the verifier path, with a
  reject_reason when it failed proof. New `rejected_boxes` is the failed
  proof-class subset. Neither list feeds any filtering decision.
"""
import math


def detection_key(cls, rectangle):
    try:
        values = [float(v) for v in rectangle]
        if len(values) == 4 and all(math.isfinite(v) for v in values):
            return int(cls), tuple(round(v,2) for v in values)
    except (TypeError, ValueError, OverflowError):
        pass
    return None


def frame_verified(records, cls, rectangle):
    key = detection_key(cls,rectangle)
    return key is not None and any(isinstance(r,dict)
        and detection_key(r.get('cls'),r.get('xyxy')) == key for r in records)


def overlap(first, second):
    a,b=list(first),list(second)
    if len(a)!=4 or len(b)!=4 or not all(math.isfinite(float(v)) for v in a+b):return 0.
    area_a=max(0.,a[2]-a[0])*max(0.,a[3]-a[1])
    area_b=max(0.,b[2]-b[0])*max(0.,b[3]-b[1])
    intersection=max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    union=area_a+area_b-intersection
    # float(): torch/numpy element arithmetic above yields 0-d tensors; a bare
    # return leaked a Tensor into verified_boxes and killed json.dumps in the
    # shared service reply (codex review #1, live-reproduced 2026-10-08).
    return float(intersection/union) if union>0 else 0.


# Classes whose shirt colour is actually checked inside this module (red=0,
# blue=2). For green(1)/white(3) the torso check happens in perception_real,
# so `shirt_verified=True` here only means "not vetoed here", never "proved".
SHIRT_CHECKED_HERE = (0, 2)


def _to_float_list(xyxy):
    """N12 fix: ultralytics boxes may hold torch Tensors or numpy values;
    json.dumps on a list(Tensor) raises TypeError inside the shared service
    reply. Convert element-wise to plain Python floats."""
    try:
        return [float(v) for v in xyxy]
    except (TypeError, ValueError):
        return None


def _reject_reason(color_check, is_proof_class, matched, shirt_verified):
    if not color_check:
        # The proof channel is off entirely; a proof-class box that is not
        # "verified" was not vetoed, it simply was never checked.
        return 'proof_disabled' if is_proof_class else 'not_proof_class'
    if not is_proof_class:
        return 'not_proof_class'
    if not matched:
        return 'no_person_overlap'
    if not shirt_verified:
        return 'shirt_unsupported'
    return None


class PersonVerifier:
    def __init__(self, weights='', device='cpu', confidence=.1, minimum_overlap=.25, classes=(1,), color_check=False, proof_classes=None):
        self.weights=weights
        self.device=device
        self.confidence=float(confidence)
        self.minimum_overlap=float(minimum_overlap)
        self.classes=set(classes)
        self.proof_classes=set(classes if proof_classes is None else proof_classes)
        self.verified_boxes=[]
        self.evidence=[]          # N12: full per-box record for the current frame
        self.rejected_boxes=[]    # N12: failed proof-class entries of `evidence`
        self.color_check=bool(color_check)
        self.model=None

    def green_proof_enabled(self):
        return bool(self.weights) and 1 in self.classes and self.color_check

    def verified_colors(self):
        if not self.weights or not self.color_check:
            return []
        return [color for cid,color in ((1,'green'),(3,'white'),(0,'red'),(2,'blue')) if cid in self.proof_classes]

    def load(self):
        if self.model is None:
            from pathlib import Path
            if not Path(self.weights).is_file():raise FileNotFoundError(self.weights)
            from ultralytics import YOLO
            self.model=YOLO(self.weights)
            if self.model.names.get(0)!='person':raise ValueError('Verifier class 0 must be person')
        return self.model

    def filter_boxes(self, image, boxes):
        self.verified_boxes=[]
        self.evidence=[]
        self.rejected_boxes=[]
        boxes=list(boxes)
        if self.color_check:
            from jersey_color import filter_boxes
            boxes=filter_boxes(image,boxes)
        if not self.weights or not any(int(b.cls) in self.classes | self.proof_classes for b in boxes):
            # N12: the verifier path is not entered for this frame (no box of a
            # proof/forced class). Record that fact so brown/white boxes are not
            # invisible to offline attribution. Judgement unchanged.
            for box in boxes:
                cid=int(box.cls)
                if cid not in self.classes and cid not in self.proof_classes:
                    self.evidence.append(dict(
                        cls=cid, xyxy=_to_float_list(box.xyxy[0]),
                        color_conf=float(box.conf), n_person_boxes=None,
                        best_person_iou=None, best_person_conf=None,
                        best_person_xyxy=None, in_classes_filter=False,
                        is_proof_class=False, matched=None, shirt_verified=None,
                        shirt_checked_here=False, shirt_fractions=None,
                        verified=False, reject_reason='verifier_path_not_entered'))
            return boxes
        self.load()
        result=self.model(image,classes=[0],conf=self.confidence,device=self.device,verbose=False)[0]
        # N12: keep (xyxy, conf) pairs; the matched semantics below use the
        # same overlap values as the original generator expression.
        people=[(b.xyxy[0], float(b.conf)) for b in result.boxes if int(b.cls)==0]
        retained=[]
        for box in boxes:
            cid=int(box.cls)
            ious=[overlap(box.xyxy[0],person_xyxy) for person_xyxy,_ in people]
            matched=any(iou>=self.minimum_overlap for iou in ious)
            if cid not in self.classes or matched:
                retained.append(box)
            shirt_verified=True
            shirt_fractions=None
            shirt_checked_here=cid in SHIRT_CHECKED_HERE
            if self.color_check and cid == 0 and cid in self.proof_classes:
                from jersey_color import torso_fractions,supported
                fractions=torso_fractions(image,box.xyxy[0])
                shirt_fractions=fractions
                shirt_verified=fractions is not None and supported('red',fractions)
            if self.color_check and cid == 2 and cid in self.proof_classes:
                from jersey_color import torso_fractions,supported
                fractions=torso_fractions(image,box.xyxy[0])
                shirt_fractions=fractions
                shirt_verified=fractions is not None and supported('blue',fractions)
            is_proof_class=cid in self.proof_classes
            passed=self.color_check and matched and is_proof_class and shirt_verified
            best_idx=ious.index(max(ious)) if ious and max(ious)>0 else -1
            best_iou=ious[best_idx] if best_idx>=0 else 0.0
            if passed:
                key=detection_key(cid,box.xyxy[0])
                if key is not None:
                    self.verified_boxes.append(dict(
                        cls=key[0], xyxy=list(key[1]),
                        person_xyxy=(_to_float_list(people[best_idx][0]) if best_idx>=0 else None),
                        person_conf=(float(people[best_idx][1]) if best_idx>=0 else None),
                        person_iou=float(best_iou),
                        n_person_boxes=len(people),
                        shirt_fractions=shirt_fractions,
                        shirt_checked_here=shirt_checked_here,
                        reject_reason=None))
            evidence=dict(
                cls=cid,
                xyxy=_to_float_list(box.xyxy[0]),
                color_conf=float(box.conf),
                n_person_boxes=len(people),
                best_person_iou=float(best_iou),
                best_person_conf=(float(people[best_idx][1]) if best_idx>=0 else None),
                best_person_xyxy=(_to_float_list(people[best_idx][0]) if best_idx>=0 else None),
                in_classes_filter=(cid in self.classes),
                is_proof_class=is_proof_class,
                matched=matched,
                shirt_verified=shirt_verified,
                shirt_checked_here=shirt_checked_here,
                shirt_fractions=shirt_fractions,
                verified=passed,
                reject_reason=_reject_reason(self.color_check, is_proof_class,
                                             matched, shirt_verified))
            self.evidence.append(evidence)
            if self.color_check and is_proof_class and not passed:
                self.rejected_boxes.append(dict(
                    cls=evidence['cls'], xyxy=evidence['xyxy'],
                    color_conf=evidence['color_conf'],
                    n_person_boxes=evidence['n_person_boxes'],
                    best_person_iou=evidence['best_person_iou'],
                    best_person_conf=evidence['best_person_conf'],
                    reject_reason=evidence['reject_reason']))
        return retained
