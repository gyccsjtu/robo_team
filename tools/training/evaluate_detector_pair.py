#!/usr/bin/env python3
"""Offline actor detector + sculpture verifier experiment, never a ROS node.

Policies are fixed before inference. A per-image rejection is only a proposed
veto: this tool does not claim temporal tracking or flight acceptance.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time


def load_evaluator():
    spec = importlib.util.spec_from_file_location(
        'pair_image_evaluation', Path(__file__).with_name('evaluate_distractor.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def iou(a, b):
    intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(
        0., min(a[3], b[3])-max(a[1], b[1]))
    return intersection/max((a[2]-a[0])*(a[3]-a[1]) +
                            (b[2]-b[0])*(b[3]-b[1])-intersection, 1e-12)


def valid_prediction(prediction):
    try:
        cls, box, confidence = prediction
        return (isinstance(cls, int) and 0 <= cls <= 5 and len(box) == 4 and
                all(math.isfinite(v) for v in box) and math.isfinite(confidence) and
                0 <= confidence <= 1 and box[2] > box[0] and box[3] > box[1])
    except (TypeError, ValueError):
        return False


def match_instance(a, b, policy, strict_match):
    if policy.get('shape') == 'runtime_same_appearance_instance':
        return strict_match(a, b)
    aw, ah, bw, bh = a[2]-a[0], a[3]-a[1], b[2]-b[0], b[3]-b[1]
    lo, hi = policy['width_height_ratio']
    return (iou(a, b) >= policy['iou'] and lo <= aw/bw <= hi and
            lo <= ah/bh <= hi and abs(a[0]+a[2]-b[0]-b[2])/2 <=
            policy['center_dx_fraction']*min(aw, bw) and
            abs(a[1]+a[3]-b[1]-b[3])/2 <= policy['center_dy_fraction']*min(ah, bh))


def proposed_vetoes(actors, verifier, policy, strict_match):
    """Mutually unique geometry; confidence margin uses the verifier only.

    Conflicting nearby/overlapping primary boxes cause abstention, not NMS.
    No primary detection is added/relabelled. Invalid verifier output abstains.
    """
    if not all(valid_prediction(v) for v in actors+verifier):
        return [], [{'reason': 'invalid_prediction'}]
    decoys = [p for p in verifier if p[0] == 5 and p[2] >= .50]
    candidates = [[j for j, (_, db, _) in enumerate(decoys)
                   if match_instance(ab, db, policy, strict_match)]
                  for _, ab, _ in actors]
    reverse = Counter(j for choices in candidates for j in choices)
    rejected, evidence = [], []
    for index, choices in enumerate(candidates):
        if not choices:
            continue
        if len(choices) != 1 or reverse[choices[0]] != 1:
            evidence.append(dict(actor_index=index, reason='ambiguous_instance'))
            continue
        _, db, dc = decoys[choices[0]]
        # Partial duplicate boxes also make assignment ambiguous. Do not let
        # relaxed cross-model matching silently merge neighboring actors.
        if any(i != index and iou(ab, db) >= .30
               for i, (_, ab, _) in enumerate(actors)):
            evidence.append(dict(actor_index=index, reason='overlapping_primary'))
            continue
        competing = max((conf for cls, box, conf in verifier
                         if cls < 5 and iou(box, db) >= .50), default=0.)
        qualifies = (dc >= policy['minimum_decoy_confidence'] and
                     dc-competing >= policy['same_model_class_margin']-1e-9)
        evidence.append(dict(actor_index=index, decoy_confidence=dc,
                             verifier_actor_confidence=competing,
                             reason='proposed_veto' if qualifies else 'weak_verifier'))
        if qualifies:
            rejected.append(index)
    return rejected, evidence


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def aggregate(scores, names):
    total, matched = Counter(), Counter()
    for score in scores:
        total.update(score['total']); matched.update(score['matched'])
    return dict(images=len(scores),
                recall={names[c]: matched[c]/total[c] if total[c] else None for c in range(5)},
                matched={names[c]: matched[c] for c in range(5)},
                labels={names[c]: total[c] for c in range(5)},
                false_decoy_frames=sum(bool(s['false_decoy']) for s in scores),
                empty_false_frames=sum(bool(s.get('empty_false')) for s in scores))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    if (output/'result.json').exists():
        parser.error('Use a new output directory; prior results cannot be overwritten')
    policy = json.loads(Path(args.policy).read_text())
    ev = load_evaluator()
    rows, audit = ev.training.audit_manifest(args.manifest)
    rows = [r for r in rows if r['split'] in ('val', 'test')]
    for row in rows:
        row['evaluation_group'] = ('legacy340' if Path(row['image']).parents[2].name == 'yolo_ds'
                                   else 'negative34') if row['group'] == 'legacy_original_val' else row['group']
    weights = policy['primary_weights']+[policy['verifier_weights']]
    expected = policy['primary_weights_sha256']+policy['verifier_weights_sha256']
    if [digest(p) for p in weights] != expected:
        raise ValueError('Weights differ from the predeclared policy')
    import numpy as np
    from PIL import Image
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(1)
    predictions, timing = {}, {}
    for weight in weights:
        model = YOLO(weight)
        if list(model.names.values()) != ev.training.NAMES[:5 if weight != weights[-1] else 6]:
            raise ValueError('Unexpected model labels: '+weight)
        cache, elapsed = {}, []
        for n, row in enumerate(rows):
            start = time.perf_counter()
            result = model.predict(row['image'], imgsz=policy['imgsz'],
                                   conf=policy['model_conf'], device=args.device, verbose=False)[0]
            cache[row['image']] = [(int(b.cls[0]), b.xyxy[0].tolist(), float(b.conf[0])) for b in result.boxes]
            elapsed.append(time.perf_counter()-start)
        predictions[weight] = cache
        timing[weight] = dict(images=len(rows), median_s=float(np.median(elapsed[1:])),
                              p95_s=float(np.quantile(elapsed[1:], .95)), first_s=elapsed[0])
        print('inferred', weight, timing[weight], flush=True)
    report = dict(policy=policy, policy_sha256=digest(args.policy),
                  manifest_sha256=digest(args.manifest), evaluator_sha256=digest(__file__),
                  runtime_source_sha256=ev.RUNTIME_SOURCE_SHA256,
                  inference_device=args.device, timing=timing, groups={}, details=[],
                  temporal_acceptance='not_tested', flight_acceptance='not_tested', deployed=False)
    grouped = {}
    for row in rows:
        with Image.open(row['image']) as im:
            width, height = im.size
            image = np.asarray(im.convert('RGB'))[:, :, ::-1].copy()
        labels = [(c, [(x-w/2)*width, (y-h/2)*height, (x+w/2)*width, (y+h/2)*height])
                  for c, x, y, w, h in ev.training.read_labels(row['label'])]
        for primary in weights[:-1]:
            raw = predictions[primary][row['image']]
            survivors = ev.runtime_color_survivors(image, raw)
            actors = [p for i, p in enumerate(raw) if i in survivors and p[2] >=
                      policy['actor_thresholds']['white' if p[0] == 3 else 'other']]
            for config in [None]+policy['policies']:
                name = config['name'] if config else 'primary_alone'
                rejected, evidence = proposed_vetoes(actors, predictions[weights[-1]][row['image']],
                                                     config, ev.same_instance) if config else ([], [])
                filtered = [p for i, p in enumerate(actors) if i not in rejected]
                score = ev.score_image(labels, filtered)
                score['empty_false'] = bool(filtered) and not labels
                grouped.setdefault((row['evaluation_group'], primary, name), []).append(score)
                if rejected or evidence or score['false_decoy']:
                    baseline = ev.score_image(labels, actors)
                    report['details'].append(dict(image=row['image'], group=row['evaluation_group'],
                        primary=primary, policy=name, rejected=[actors[i] for i in rejected],
                        evidence=evidence, missed_additional={ev.training.NAMES[c]: max(0, baseline['matched'][c]-score['matched'][c]) for c in range(5)},
                        false_decoy=score['false_decoy']))
    for (group, primary, name), scores in grouped.items():
        report['groups'].setdefault(group, {}).setdefault(primary, {})[name] = aggregate(scores, ev.training.NAMES)
    (output/'predictions.json').write_text(json.dumps(predictions)+'\n')
    (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report['groups'], indent=2), flush=True)


if __name__ == '__main__':
    main()
