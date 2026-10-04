"""Same-frame person verification for custom color detector boxes.

No ROS, torch or CUDA import on the normal shared-client path. Original boxes,
classes, confidences and coordinates are retained; the second model only vetoes
selected classes without an overlapping person detection in this exact image.
"""
import math


def overlap(first, second):
    a,b=list(first),list(second)
    if len(a)!=4 or len(b)!=4 or not all(math.isfinite(float(v)) for v in a+b):return 0.
    area_a=max(0.,a[2]-a[0])*max(0.,a[3]-a[1])
    area_b=max(0.,b[2]-b[0])*max(0.,b[3]-b[1])
    intersection=max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    union=area_a+area_b-intersection
    return intersection/union if union>0 else 0.


class PersonVerifier:
    def __init__(self, weights='', device='cpu', confidence=.1, minimum_overlap=.25, classes=(1,), color_check=False):
        self.weights=weights
        self.device=device
        self.confidence=float(confidence)
        self.minimum_overlap=float(minimum_overlap)
        self.classes=set(classes)
        self.color_check=bool(color_check)
        self.model=None

    def load(self):
        if self.model is None:
            from pathlib import Path
            if not Path(self.weights).is_file():raise FileNotFoundError(self.weights)
            from ultralytics import YOLO
            self.model=YOLO(self.weights)
            if self.model.names.get(0)!='person':raise ValueError('Verifier class 0 must be person')
        return self.model

    def filter_boxes(self, image, boxes):
        boxes=list(boxes)
        if self.color_check:
            from jersey_color import filter_boxes
            boxes=filter_boxes(image,boxes)
        if not self.weights or not any(int(b.cls) in self.classes for b in boxes):return boxes
        self.load()
        result=self.model(image,classes=[0],conf=self.confidence,device=self.device,verbose=False)[0]
        people=[b.xyxy[0] for b in result.boxes if int(b.cls)==0]
        return [box for box in boxes if int(box.cls) not in self.classes
                or any(overlap(box.xyxy[0],person)>=self.minimum_overlap for person in people)]
