"""Optional appearance veto; the primary detector keeps its boxes and colours.

Inputs are detections on the SAME image. No motion, world position, official
feedback, or absence of a secondary actor detection constitutes decoy evidence.
"""
import math
import os


def iou(a, b):
    inter = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    return inter/max((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter, 1e-9)


def same_object(a, b):
    if len(a) != 4 or len(b) != 4 or not all(math.isfinite(v) for v in (*a, *b)):
        return False
    aw, ah, bw, bh = a[2]-a[0], a[3]-a[1], b[2]-b[0], b[3]-b[1]
    if min(aw, ah, bw, bh) <= 0.:
        return False
    return (iou(a, b) >= .60 and .60 <= aw/bw <= 1.50 and .60 <= ah/bh <= 1.50
            and abs(a[0]+a[2]-b[0]-b[2]) <= .50*min(aw, bw)
            and abs(a[1]+a[3]-b[1]-b[3]) <= .30*min(ah, bh))


def strong_decoys(primary, secondary):
    """Return primary indices with unique, uncontradicted sculpture evidence.

    Different colours on near-identical primary boxes can describe one object.
    Two spatially distinct people matched to one secondary box are ambiguous.
    Secondary scores are NOT compared to primary scores from a different model.
    """
    decoys = [b for b in secondary if b['label'] == 'distractor_person'
              and math.isfinite(b['confidence']) and b['confidence'] >= .85]
    actors = [b for b in secondary if b['label'] in ('red','green','blue','white','brown')
              and b['confidence'] >= .30]
    matched = [[j for j, b in enumerate(decoys) if same_object(a['box'], b['box'])]
               for a in primary]
    result = {}
    for i, choices in enumerate(matched):
        if len(choices) != 1:
            continue
        j = choices[0]
        peers = [k for k, ms in enumerate(matched) if j in ms]
        if any(iou(primary[i]['box'], primary[k]['box']) < .85 for k in peers):
            continue
        if any(iou(primary[i]['box'], b['box']) >= .30 for b in actors):
            continue  # conflicting secondary classification: retain primary
        result[i] = decoys[j]['confidence']
    return result


class DistractorVerifier:
    """Three fresh, associated images spanning .30 s; no sticky blacklists."""
    def __init__(self):
        self.history = []
        self.last_sample = -math.inf

    def update(self, primary, secondary, sample_s):
        if not math.isfinite(sample_s) or sample_s <= self.last_sample:
            return set()  # repeated/out-of-order images cannot manufacture votes
        self.last_sample = sample_s
        evidence = strong_decoys(primary, secondary)
        previous = [h for h in self.history if 0. < sample_s-h['sample_s'] <= .40]
        next_history, rejected = [], set()
        for i, confidence in evidence.items():
            box = primary[i]['box']
            matches = []
            for h in previous:
                if not same_object(box, h['box']):
                    continue
                peers = [primary[k]['box'] for k in evidence
                         if same_object(primary[k]['box'], h['box'])]
                if all(iou(a,b) >= .85 for a in peers for b in peers):
                    matches.append(h)  # never lend one history to two people
            # Histories of coincident blue/white boxes describe the same image,
            # so merge unique times, never inflate the number of observations.
            unique = all(iou(a['box'], b['box']) >= .85 for a in matches for b in matches)
            samples = sorted({s for h in matches for s in h['samples'] if sample_s-s <= 1.}) if unique else []
            samples.append(sample_s)
            next_history.append(dict(box=list(box), sample_s=sample_s, samples=samples))
            if len(samples) >= 3 and samples[-1]-samples[0] >= .30-1e-6:
                rejected.add(i)
        self.history = next_history
        return rejected


class SecondaryAppearance:
    """Shared detector, separate temporal evidence for each physical camera."""
    def __init__(self, mode=None, weights=None, configure_model=None):
        self.mode = mode or os.environ.get('PR_DISTRACTOR_MODE','off')
        self.weights = weights or os.environ.get('PR_DISTRACTOR_WEIGHTS','')
        if self.mode not in ('off','shadow','enforce'):
            raise ValueError('invalid PR_DISTRACTOR_MODE')
        self.model = None
        self.histories = {}
        if self.mode != 'off':
            from ultralytics import YOLO
            import numpy as np
            from pathlib import Path
            if not Path(self.weights).is_file():
                raise ValueError('missing PR_DISTRACTOR_WEIGHTS')
            self.model = YOLO(self.weights)
            if configure_model is not None:
                configure_model(self.model)
            required={'red','green','blue','white','brown','distractor_person'}
            if set(self.model.names.values()) != required:
                raise ValueError('secondary weights must contain five actors and distractor_person')
            self.model.predict(np.zeros((480,752,3),np.uint8),imgsz=640,
                               conf=.30,device='cpu',verbose=False)
        print('[DISTRACTOR_READY] mode=%s weights=%s; primary geometry preserved' %
              (self.mode,self.weights),flush=True)

    def filter(self, image, boxes, names, sample_s, camera='stereo'):
        boxes=list(boxes)
        if self.model is None:return boxes,[]
        history=self.histories.setdefault(camera,DistractorVerifier())
        if not boxes:
            history.update([],[],sample_s)
            return boxes,[]
        primary=[dict(label=names[int(b.cls)],confidence=float(b.conf),
                      box=[float(v) for v in b.xyxy[0]]) for b in boxes]
        result=self.model.predict(image,imgsz=640,conf=.30,device='cpu',verbose=False)[0]
        secondary=[dict(label=result.names[int(b.cls)],confidence=float(b.conf),
                        box=[float(v) for v in b.xyxy[0]]) for b in result.boxes]
        strong=strong_decoys(primary,secondary)
        rejected=history.update(primary,secondary,sample_s)
        evidence=[dict(box=primary[i]['box'],confidence=c,
                       status='secondary_distractor' if i in rejected else 'secondary_pending')
                  for i,c in strong.items()]
        # A lone negative is diagnostic only: the 121245 trial contains a real
        # white actor which an earlier verifier called a sculpture at .855.
        # Do not mark pending boxes ambiguous (which silently stops reports).
        # Confirmation needs three distinct images; a new positive/missing
        # secondary observation breaks the negative streak immediately.
        if self.mode=='enforce':boxes=[b for i,b in enumerate(boxes) if i not in rejected]
        return boxes,evidence
