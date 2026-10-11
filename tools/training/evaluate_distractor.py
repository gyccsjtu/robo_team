#!/usr/bin/env python3
"""Compare old/new detector on reviewed independent test images at runtime thresholds.

This measures image recall and false actor boxes on sculptures. A passing report
does not claim a passing temporal tracker or a passing official simulation.
"""
import argparse
import ast
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location('train_distractor', Path(__file__).parents[1]/'train_distractor.py')
training = importlib.util.module_from_spec(spec); spec.loader.exec_module(training)


def runtime_instance_predicate():
    """Load the exact pure runtime geometry without starting ROS or its node."""
    source = Path(__file__).resolve().parents[2]/'perception/perception_real.py'
    tree = ast.parse(source.read_text())
    required = {'_box_iou', 'same_appearance_instance', 'blue_white_torso_vote',
                'filter_blue_white_duplicate_boxes'}
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in required]
    if {n.name for n in nodes} != required:
        raise ValueError('Runtime instance predicate moved; evaluation must be updated')
    import numpy as np
    scope = {'np': np}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
    return scope['same_appearance_instance'], scope['filter_blue_white_duplicate_boxes'], training.sha256(source)


same_instance, blue_white_filter, RUNTIME_SOURCE_SHA256 = runtime_instance_predicate()


def runtime_color_survivors(image, predictions):
    """Apply the production image-color filter before per-class thresholds.

    A blue box with confidence .30-.399 can still remove an overlapping white
    box in production. Retain this exact input ordering and input threshold.
    """
    boxes = [SimpleNamespace(cls=cls, xyxy=[box], conf=confidence, index=i)
             for i, (cls, box, confidence) in enumerate(predictions) if cls < 5]
    return {box.index for box in blue_white_filter(image, boxes)}


def iou(a, b):
    inter = max(0., min(a[2],b[2])-max(a[0],b[0])) * max(0.,min(a[3],b[3])-max(a[1],b[1]))
    area_a, area_b = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
    return inter/max(area_a+area_b-inter, 1e-12)


def score_image(labels, predictions, usable_indices=None):
    total, matched, usable = Counter(), Counter(), Counter()
    used, real_matched = set(), set()
    conflicts = []
    color_suppressed = []
    for cls, box in labels:
        total[cls] += 1
        matches = [(iou(box,pbox), i) for i,(pcls,pbox,_) in enumerate(predictions)
                   if pcls == cls and i not in used]
        if matches:
            overlap, idx = max(matches)
            if overlap >= .5:
                used.add(idx); matched[cls] += 1
                # Only a prediction actually assigned to this actor may be
                # exempted below. A neighbouring actor GT cannot excuse an
                # additional torso/head false box on a sculpture. When the
                # assigned box fits a sculpture at least as well, abstain.
                nearer_actor = overlap > max((iou(predictions[idx][1], gb)
                                              for gc, gb in labels if gc == 5), default=0.)
                if cls < 5 and nearer_actor:
                    real_matched.add(idx)
                # Single frames cannot demonstrate the resolver's three-image
                # agreement, even when an actor has a strong class margin.
                conflict = cls < 5 and (not nearer_actor or
                           any(pc == 5 and cf >= .50 and same_instance(predictions[idx][1], pb)
                               for pc, pb, cf in predictions))
                if conflict:
                    conflicts.append(dict(actor_class=cls, actor_box=predictions[idx][1]))
                elif cls < 5 and usable_indices is not None and idx not in usable_indices:
                    color_suppressed.append(dict(actor_class=cls, actor_box=predictions[idx][1]))
                else:
                    usable[cls] += 1
    false_decoy, partial_decoy = [], []
    for idx, (cls, box, confidence) in enumerate(predictions):
        if cls >= 5 or idx in real_matched:
            continue
        for gc, gb in labels:
            if gc != 5:
                continue
            overlap = iou(box, gb)
            ix = max(0., min(box[2],gb[2])-max(box[0],gb[0]))
            iy = max(0., min(box[3],gb[3])-max(box[1],gb[1]))
            contained = ix*iy/max((box[2]-box[0])*(box[3]-box[1]), 1e-12)
            cx, cy = (box[0]+box[2])/2, (box[1]+box[3])/2
            partial = gb[0] <= cx <= gb[2] and gb[1] <= cy <= gb[3] and contained >= .80
            if overlap >= .30 or partial:
                false_decoy.append((cls,box,confidence))
                if partial and overlap < .30:
                    partial_decoy.append((cls,box,confidence))
                break
    return dict(total=total, matched=matched, usable=usable,
                false_decoy=false_decoy, partial_decoy=partial_decoy, actor_conflicts=conflicts,
                color_suppressed=color_suppressed)


def evaluate(weights, rows, device='cpu', candidate=False):
    from ultralytics import YOLO
    from PIL import Image
    import numpy as np
    model = YOLO(str(weights))
    allowed = (training.NAMES,) if candidate else (training.NAMES, training.NAMES[:5])
    if list(model.names.values()) not in allowed:
        raise ValueError('Unexpected checkpoint class order')
    totals, matches, usable, false_frames, partial_frames, negative_false_frames, details = Counter(), Counter(), Counter(), 0, 0, 0, []
    for row in rows:
        with Image.open(row['image']) as im:
            w,h=im.size
            image_bgr=np.asarray(im.convert('RGB'))[:,:,::-1].copy()
        gt = [(c, [(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h])
              for c,x,y,bw,bh in training.read_labels(row['label'])]
        result = model.predict(row['image'], imgsz=640, conf=.30, device=device, verbose=False)[0]
        raw_pred=[]
        for b in result.boxes:
            cls,conf=int(b.cls[0]),float(b.conf[0])
            raw_pred.append((cls,b.xyxy[0].tolist(),conf))
        survivors=runtime_color_survivors(image_bgr,raw_pred)
        pred=[]; usable_indices=set()
        for raw_index,(cls,box,conf) in enumerate(raw_pred):
            if conf >= (.30 if cls == 3 else .40):
                if raw_index in survivors:
                    usable_indices.add(len(pred))
                pred.append((cls,box,conf))
        scored = score_image(gt,pred,usable_indices)
        negative_false = [p for p in pred if p[0] < 5] if not gt else []
        negative_false_frames += bool(negative_false)
        totals.update(scored['total']); matches.update(scored['matched']); usable.update(scored['usable'])
        false_frames += bool(scored['false_decoy']); partial_frames += bool(scored['partial_decoy'])
        details.append(dict(image=row['image'], group=row['group'],
                            false_actor_boxes_on_decoy=scored['false_decoy'],
                            partial_actor_boxes_on_decoy=scored['partial_decoy'],
                            actor_distractor_conflicts=scored['actor_conflicts'],
                            actor_boxes_removed_by_runtime_color_filter=scored['color_suppressed'],
                            actor_boxes_in_empty_negative=negative_false,
                            matched=dict(scored['matched']), usable=dict(scored['usable']),
                            labels=dict(scored['total'])))
    return dict(weights=str(weights), sha256=hashlib.sha256(Path(weights).read_bytes()).hexdigest(),
                images=len(rows), false_decoy_frames=false_frames, partial_decoy_frames=partial_frames,
                empty_negative_false_frames=negative_false_frames,
                recall={training.NAMES[i]: matches[i]/totals[i] if totals[i] else None for i in range(6)},
                usable_actor_recall={training.NAMES[i]: usable[i]/totals[i] if totals[i] else None for i in range(5)},
                totals={training.NAMES[i]:totals[i] for i in range(6)}, frames=details)


def compare(baseline, candidate):
    raw_ok = all(candidate['recall'][n] is not None and baseline['recall'][n] is not None and
                 candidate['recall'][n]+1e-12 >= baseline['recall'][n]-.01 for n in training.NAMES[:5])
    usable_ok = all(candidate['usable_actor_recall'][n] is not None and baseline['usable_actor_recall'][n] is not None and
                    candidate['usable_actor_recall'][n]+1e-12 >= baseline['usable_actor_recall'][n]-.01 for n in training.NAMES[:5])
    decoy_ok = candidate['false_decoy_frames'] == 0
    return dict(actor_recall_within_one_percentage_point=raw_ok,
                usable_actor_recall_within_one_percentage_point=usable_ok,
                image_decoy_gate=decoy_ok, image_acceptance=raw_ok and usable_ok and decoy_ok)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', required=True);p.add_argument('--baseline', required=True)
    p.add_argument('--candidate', required=True);p.add_argument('--output', required=True)
    p.add_argument('--device', default='cpu')
    p.add_argument('--original-val', action='store_true', help='Also compare the untouched legacy 340-image validation subset')
    p.add_argument('--validation-only', action='store_true',
                   help='Select candidate using original validation and new-layout validation; do not inspect test images')
    args=p.parse_args()
    all_rows,_=training.audit_manifest(args.manifest)
    if args.validation_only and not args.original_val:
        p.error('--validation-only requires --original-val to protect all actor colors')
    rows=[r for r in all_rows if (r['split']=='val' and r['group']!='legacy_original_val')
          ] if args.validation_only else [r for r in all_rows if r['split']=='test']
    if not rows:
        raise ValueError('No reviewed images in requested evaluation split')
    provenance=training.verify_candidate_provenance(args.candidate,rows,independent=not args.validation_only)
    baseline=evaluate(args.baseline,rows,args.device)
    candidate=evaluate(args.candidate,rows,args.device,candidate=True)
    result=dict(baseline=baseline,candidate=candidate,
                **compare(baseline,candidate), candidate_provenance=provenance,
                evaluation_scope='checkpoint_selection_validation' if args.validation_only else 'held_out_test',
                runtime_instance_predicate_source_sha256=RUNTIME_SOURCE_SHA256,
                evaluator_source_sha256=training.sha256(__file__),
                temporal_tracking_acceptance='not_tested', official_match_acceptance='not_tested')
    if args.original_val:
        # Original archive images and the separate negatives archive share a
        # split group. Their source directories distinguish the untouched 340.
        legacy=[r for r in all_rows if r.get('original_split')=='val' and
                r['group']=='legacy_original_val' and Path(r['image']).parents[2].name=='yolo_ds']
        if len(legacy) != 340:
            raise ValueError('Expected exactly the original 340 validation images')
        legacy_provenance=training.verify_candidate_provenance(args.candidate,legacy,independent=False)
        old=evaluate(args.baseline,legacy,args.device)
        new=evaluate(args.candidate,legacy,args.device,candidate=True)
        legacy_comparison=compare(old,new)
        result['original_validation']=dict(baseline=old,candidate=new,provenance=legacy_provenance,
                                           **legacy_comparison, limitation='Checkpoint-selection validation, not an independent test')
        result['image_acceptance'] = result['image_acceptance'] and legacy_comparison['image_acceptance']
        negatives=[r for r in all_rows if r.get('original_split')=='val' and
                   r['group']=='legacy_original_val' and r not in legacy]
        if len(negatives) != 34 or any(training.read_labels(r['label']) for r in negatives):
            raise ValueError('Expected the 34 separately reviewed original empty negatives')
        training.verify_candidate_provenance(args.candidate,negatives,independent=False)
        old_negative=evaluate(args.baseline,negatives,args.device)
        new_negative=evaluate(args.candidate,negatives,args.device,candidate=True)
        negative_ok = new_negative['empty_negative_false_frames'] <= old_negative['empty_negative_false_frames']
        result['original_negative_validation']=dict(baseline=old_negative,candidate=new_negative,
                                                    false_frames_not_increased=negative_ok)
        result['image_acceptance'] = result['image_acceptance'] and negative_ok
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('baseline','candidate','original_validation','original_negative_validation')},indent=2))
    if not result['image_acceptance']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
