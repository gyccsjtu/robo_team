"""Same-frame person verification for custom color detector boxes.

No ROS, torch or CUDA import on the normal shared-client path. Original boxes,
classes, confidences and coordinates are retained; the second model only vetoes
selected classes without an overlapping person detection in this exact image.
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
    return intersection/union if union>0 else 0.


class PersonVerifier:
    def __init__(self, weights='', device='cpu', confidence=.1, minimum_overlap=.25, classes=(1,), color_check=False, proof_classes=None):
        self.weights=weights
        self.device=device
        self.confidence=float(confidence)
        self.minimum_overlap=float(minimum_overlap)
        self.classes=set(classes)
        self.proof_classes=set(classes if proof_classes is None else proof_classes)
        self.verified_boxes=[]
        self.color_check=bool(color_check)
        self.model=None

    def green_proof_enabled(self):
        return bool(self.weights) and 1 in self.classes and self.color_check

    def verified_colors(self):
        if not self.weights or not self.color_check:
            return []
        return [color for cid,color in ((1,'green'),(3,'white')) if cid in self.proof_classes]

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
        boxes=list(boxes)
        if self.color_check:
            from jersey_color import filter_boxes
            boxes=filter_boxes(image,boxes)
        if not self.weights or not any(int(b.cls) in self.classes | self.proof_classes for b in boxes):return boxes
        self.load()
        result=self.model(image,classes=[0],conf=self.confidence,device=self.device,verbose=False)[0]
        people=[b.xyxy[0] for b in result.boxes if int(b.cls)==0]
        retained=[]
        for box in boxes:
            cid=int(box.cls)
            matched=any(overlap(box.xyxy[0],person)>=self.minimum_overlap for person in people)
            if cid not in self.classes or matched:
                retained.append(box)
            if self.color_check and matched and cid in self.proof_classes:
                key=detection_key(cid,box.xyxy[0])
                if key is not None:self.verified_boxes.append(dict(cls=key[0],xyxy=list(key[1])))
        return retained
