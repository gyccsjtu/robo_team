#!/usr/bin/env python3
"""Frozen secondary veto evaluated against primary recall on reviewed images.

Single-frame *potential* veto is stricter than temporal runtime veto: even one
actor wrongly vetoed here fails the deployment gate. Does not claim live recall.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'perception'))
from distractor_verifier import strong_decoys, iou


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--primary', required=True)
    p.add_argument('--secondary', required=True)
    p.add_argument('--manifest', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    import cv2
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(1); cv2.setNumThreads(1)
    models = [YOLO(args.primary), YOLO(args.secondary)]
    counts, baseline, after, details = Counter(), Counter(), Counter(), []
    for row in map(json.loads, Path(args.manifest).read_text().splitlines()):
        if row['split'] not in ('val', 'test') or not row.get('reviewed'):
            continue
        image = cv2.imread(row['image']); h, w = image.shape[:2]
        labels = []
        for line in Path(row['label']).read_text().splitlines():
            c,x,y,bw,bh = map(float, line.split())
            labels.append((int(c), [(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]))
        predictions = []
        for model in models:
            result = model.predict(image,imgsz=640,conf=.30,device='cpu',verbose=False)[0]
            predictions.append([dict(label=model.names[int(b.cls[0])],
                box=b.xyxy[0].tolist(),confidence=float(b.conf[0])) for b in result.boxes])
        primary, secondary = predictions
        primary = [b for b in primary if b['confidence'] >= (.30 if b['label']=='white' else .40)]
        veto = strong_decoys(primary, secondary)
        # Colour-independent geometry protects any real actor, including a
        # primary colour mistake. Primary localization boxes stay unchanged.
        harm=[]; decoys=[]
        for i in veto:
            b = primary[i]['box']
            if any(c<5 and iou(b,gb)>=.30 for c,gb in labels): harm.append(i)
            if any(c==5 and iou(b,gb)>=.30 for c,gb in labels): decoys.append(i)
        names=['red','green','blue','white','brown','distractor_person']
        for c, gb in labels:
            if c>=5: continue
            matches=[i for i,b in enumerate(primary) if b['label']==names[c] and iou(b['box'],gb)>=.5]
            baseline[names[c]]+=bool(matches)
            after[names[c]]+=any(i not in veto for i in matches)
        counts['images']+=1;counts['actor_vetoes']+=len(harm)
        counts['decoy_vetoes']+=len(decoys);counts['decoy_veto_frames']+=bool(decoys)
        counts['all_vetoes']+=len(veto)
        if veto: details.append(dict(image=row['image'],group=row['group'],
            primary=primary,secondary=secondary,veto=sorted(veto),harm=harm,decoys=decoys))
        if counts['images']%50==0: print(dict(counts),flush=True)
    out=dict(counts=counts,primary_recall=baseline,after_recall=after,
        gate_pass=not counts['actor_vetoes'] and baseline==after and counts['decoy_vetoes']>0,
        note='Previously inspected validation/test layouts; not new blind or live test.',details=details,
        sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in
            (args.primary,args.secondary,str(ROOT/'perception/distractor_verifier.py'))})
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    Path(args.output).write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k not in ('details','sha256')},indent=2))


if __name__ == '__main__': main()
