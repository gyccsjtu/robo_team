"""Optional appearance veto; the primary detector keeps its boxes and colours.

Inputs are detections on the SAME image. No motion, world position, official
feedback, or absence of a secondary actor detection constitutes decoy evidence.
"""
import math
import os
import time

ACTOR_LABELS = ('red', 'green', 'blue', 'white', 'brown')


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


def unique_actor_evidence(primary, secondary):
    """Same-image, same-colour actor matches with no competing object owner."""
    usable = [b for b in secondary if math.isfinite(b['confidence'])
              and b['confidence'] >= .30]
    matches = [[j for j, b in enumerate(usable)
                if same_object(a['box'], b['box'])] for a in primary]
    result = {}
    for i, choices in enumerate(matches):
        if len(choices) != 1 or primary[i]['label'] not in ACTOR_LABELS:
            continue
        j = choices[0]
        candidate = usable[j]
        if candidate['label'] != primary[i]['label']:
            continue
        if sum(j in peers for peers in matches) != 1:
            continue  # even coincident cross-colour primary boxes are ambiguous
        if any(b['label'] != candidate['label']
               and iou(primary[i]['box'], b['box']) >= .30 for b in usable):
            continue  # a nearby, overlapping decoy/other colour cannot certify
        result[i] = candidate
    return result


class DistractorVerifier:
    """Three fresh images confirm a box; a fixed 2 s hold cannot renew itself."""
    CONFIRMED_HOLD_S = 2.

    def __init__(self):
        self.history = []
        self.last_sample = -math.inf
        self.confirmed = []
        self.held_rejections = {}

    def update(self, primary, secondary, sample_s):
        self.held_rejections = {}
        if not math.isfinite(sample_s) or sample_s <= self.last_sample:
            if math.isfinite(sample_s) and sample_s < self.last_sample:
                self.confirmed = []  # do not carry an old clock epoch's hold
            return set()  # repeated/out-of-order images cannot manufacture votes
        self.last_sample = sample_s
        actor_proof = unique_actor_evidence(primary, secondary)
        self.confirmed = [h for h in self.confirmed
                          if 0. <= sample_s-h['confirmed_s'] <= self.CONFIRMED_HOLD_S
                          and not any(same_object(primary[i]['box'], h['box'])
                                      for i in actor_proof)]
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
                # Only a new three-image negative can renew the fixed deadline.
                # Empty/pending images and held half-body boxes never move it.
                self.confirmed = [h for h in self.confirmed
                                  if not same_object(box, h['box'])]
                self.confirmed.append(dict(box=list(box), confirmed_s=sample_s,
                                           confidence=confidence))
        self.history = next_history
        # A camera's old box must have exactly one current owner, and that owner
        # exactly one old box. Coincident/nearby people cannot share a hold.
        matches = [[j for j, h in enumerate(self.confirmed)
                    if same_object(a['box'], h['box'])] for a in primary]
        for i, choices in enumerate(matches):
            if i in rejected or len(choices) != 1:
                continue
            j = choices[0]
            if sum(j in peers for peers in matches) != 1:
                continue
            if any(b['label'] in ACTOR_LABELS and math.isfinite(b['confidence'])
                   and b['confidence'] >= .30
                   and iou(primary[i]['box'], b['box']) >= .30 for b in secondary):
                continue  # an ambiguous fresh actor is not sticky decoy evidence
            rejected.add(i)
            self.held_rejections[i] = self.confirmed[j]['confidence']
        return rejected


class SecondaryAppearance:
    """Shared detector, separate temporal evidence for each physical camera."""
    def __init__(self, mode=None, weights=None):
        self.mode = mode or os.environ.get('PR_DISTRACTOR_MODE','off')
        self.weights = weights or os.environ.get('PR_DISTRACTOR_WEIGHTS','')
        if self.mode not in ('off','shadow','enforce'):
            raise ValueError('invalid PR_DISTRACTOR_MODE')
        self.model = None
        self.histories = {}
        self.last_inference_s = 0.0
        if self.mode != 'off':
            from ultralytics import YOLO
            import numpy as np
            import torch
            from pathlib import Path
            if not Path(self.weights).is_file():
                raise ValueError('missing PR_DISTRACTOR_WEIGHTS')
            self.model = YOLO(self.weights)
            required={'red','green','blue','white','brown','distractor_person'}
            labels = (self.model.names.values() if isinstance(self.model.names, dict)
                      else self.model.names)
            if {str(label).strip().lower() for label in labels} != required:
                raise ValueError('secondary weights must contain five actors and distractor_person')
            threads=int(os.environ.get('PR_TORCH_THREADS','2'))
            if threads < 1:
                raise ValueError('PR_TORCH_THREADS must be positive')
            self.model.add_callback('on_predict_start',
                                    lambda predictor: torch.set_num_threads(threads)
                                    if predictor.device.type == 'cpu' else None)
            self.model.predict(np.zeros((480,752,3),np.uint8),imgsz=640,
                               conf=.30,device='cpu',verbose=False)
        print('[DISTRACTOR_READY] mode=%s weights=%s; primary geometry preserved' %
              (self.mode,self.weights),flush=True)

    def filter(self, image, boxes, names, sample_s, camera='stereo'):
        boxes=list(boxes)
        self.last_inference_s=0.0
        for b in boxes:
            # Box objects can be reused by callers; proof belongs to this image.
            b.secondary_actor_confirmed = False
            b.secondary_actor_label = ''
            b.secondary_actor_sample_s = None
            b.secondary_actor_camera = ''
            b.secondary_actor_confidence = 0.
            b.secondary_actor_box = None
        if self.model is None:return boxes,[]
        history=self.histories.setdefault(camera,DistractorVerifier())
        fresh = math.isfinite(sample_s) and sample_s > history.last_sample
        if not boxes:
            history.update([],[],sample_s)
            return boxes,[]
        primary=[dict(label=str(names[int(b.cls)]).strip().lower(),confidence=float(b.conf),
                      box=[float(v) for v in b.xyxy[0]]) for b in boxes]
        started=time.perf_counter()
        try:
            result=self.model.predict(image,imgsz=640,conf=.30,device='cpu',verbose=False)[0]
        except Exception as exc:
            # A failed verifier cannot turn a primary actor into a decoy. Drop
            # partial temporal votes so the next image starts from evidence.
            history.history=[]
            now=time.monotonic()
            if now-getattr(self,'last_error_wall',-math.inf) >= 5.0:
                print('[DISTRACTOR_ERROR] camera=%s sample=%.3f error=%s' %
                      (camera,sample_s,exc),flush=True)
                self.last_error_wall=now
            return boxes,[]
        finally:
            self.last_inference_s=time.perf_counter()-started
        secondary=[dict(label=str(result.names[int(b.cls)]).strip().lower(),confidence=float(b.conf),
                        box=[float(v) for v in b.xyxy[0]]) for b in result.boxes]
        if fresh:
            for i, proof in unique_actor_evidence(primary, secondary).items():
                if getattr(boxes[i], 'appearance_status', 'actor') != 'actor':
                    continue
                boxes[i].secondary_actor_confirmed = True
                boxes[i].secondary_actor_label = proof['label']
                boxes[i].secondary_actor_sample_s = sample_s
                boxes[i].secondary_actor_camera = camera
                boxes[i].secondary_actor_confidence = proof['confidence']
                boxes[i].secondary_actor_box = list(proof['box'])
        strong=strong_decoys(primary,secondary)
        rejected=history.update(primary,secondary,sample_s)
        evidence=[]
        for i,c in strong.items():
            status=('secondary_distractor_hold' if i in history.held_rejections else
                    'secondary_distractor' if i in rejected else 'secondary_pending')
            evidence.append(dict(box=primary[i]['box'],confidence=c,
                                 primary_label=primary[i]['label'],status=status))
            if self.mode=='enforce':
                # Keep the physical box in the track chain so image steering
                # and the scoped rejection event refer to the same instance.
                # The existing report and dispatch gates reject non-actor
                # statuses; confirmed decoys never become official reports.
                boxes[i].appearance_status=('distractor' if i in rejected else 'ambiguous')
                boxes[i].distractor_confidence=max(
                    float(getattr(boxes[i],'distractor_confidence',0.)),c)
                boxes[i].secondary_decoy_confirmed=i in rejected
        for i, confidence in history.held_rejections.items():
            if i in strong:
                continue
            evidence.append(dict(box=primary[i]['box'], confidence=confidence,
                                 primary_label=primary[i]['label'],
                                 status='secondary_distractor_hold'))
            if self.mode == 'enforce':
                boxes[i].appearance_status = 'distractor'
                boxes[i].distractor_confidence = max(
                    float(getattr(boxes[i], 'distractor_confidence', 0.)), confidence)
                boxes[i].secondary_decoy_confirmed = True
        return boxes,evidence
