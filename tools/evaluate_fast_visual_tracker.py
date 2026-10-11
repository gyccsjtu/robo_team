#!/usr/bin/env python3
"""Offline, image-only comparison of fast tracking with per-frame YOLO boxes.

YOLO is a noisy reference, NOT ground truth. This tool cannot validate world
coordinates, feet, actor identity or competition elimination. Intermediate YOLO
results never enter the tracker. There are no ROS imports or publishers.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def distribution(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return dict(n=0, min=None, p50=None, p95=None, max=None)
    percentiles = np.percentile(values, [0, 50, 95, 100])
    return dict(n=len(values), **dict(zip(('min', 'p50', 'p95', 'max'),
                                        map(float, percentiles))))


def load_manifest(path, max_frames=200, source=None):
    """Sort each camera by capture time, rejecting duplicate/invalid samples."""
    path = Path(path).resolve()
    rows, seen = [], set()
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        raw = json.loads(line)
        camera = str(raw['source_uav'])
        if source is not None and camera != source:
            continue
        stamp = float(raw['image_stamp'])
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('Invalid image_stamp at manifest line %d' % line_number)
        key = (camera, stamp)
        if key in seen:
            raise ValueError('Duplicate camera timestamp at line %d' % line_number)
        seen.add(key)
        image = Path(raw['image'])
        if not image.is_absolute():
            image = path.parent / image
        if not image.is_file():
            raise ValueError('Missing image at line %d: %s' % (line_number, image))
        rows.append(dict(image=str(image.resolve()), source_uav=camera,
                         image_stamp=stamp))
    # The cap selects a chronological prefix, not whichever camera sorted first.
    return sorted(rows, key=lambda row: (row['image_stamp'], row['source_uav']))[:max_frames]


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0., x2 - x1) * max(0., y2 - y1)
    area_a = max(0., a[2] - a[0]) * max(0., a[3] - a[1])
    area_b = max(0., b[2] - b[0]) * max(0., b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-9)


def unique_matches(predictions, references, min_iou=.15, margin=.10):
    """Mutual best same-label matching; ambiguous alternatives are excluded.

    This only establishes unambiguous geometric box correspondence, not actual
    identity. In particular a confident match does not prove no red-person swap.
    """
    matches, reasons = [], Counter()
    scores = np.zeros((len(predictions), len(references)), dtype=float)
    for i, prediction in enumerate(predictions):
        for j, reference in enumerate(references):
            if prediction['label'] == reference['label']:
                scores[i, j] = box_iou(prediction['box'], reference['box'])
    for i in range(len(predictions)):
        if not len(references) or float(scores[i].max()) < min_iou:
            reasons['no_reference_overlap'] += 1
            continue
        j = int(scores[i].argmax())
        alternatives = sorted(scores[i], reverse=True)
        reverse_alternatives = sorted(scores[:, j], reverse=True)
        if ((len(alternatives) > 1 and alternatives[0] - alternatives[1] < margin)
                or (len(reverse_alternatives) > 1 and
                    reverse_alternatives[0] - reverse_alternatives[1] < margin)
                or int(scores[:, j].argmax()) != i):
            reasons['ambiguous_reference'] += 1
            continue
        matches.append((i, j))
    return matches, reasons


def box_difference(predicted, reference):
    pc = ((predicted[0] + predicted[2]) / 2., (predicted[1] + predicted[3]) / 2.)
    rc = ((reference[0] + reference[2]) / 2., (reference[1] + reference[3]) / 2.)
    return dict(iou=box_iou(predicted, reference),
                center_px=math.hypot(pc[0] - rc[0], pc[1] - rc[1]),
                bottom_px=abs(predicted[3] - reference[3]),
                bottom_center_px=math.hypot(pc[0] - rc[0], predicted[3] - reference[3]))


def load_reference_cache(directory, config):
    if directory is None:
        return {}
    directory = Path(directory)
    saved_config = json.loads((directory / 'reference_config.json').read_text())
    for key in ('weights_sha256', 'imgsz', 'confidence', 'labels'):
        if saved_config.get(key) != config[key]:
            raise ValueError('Reference cache configuration differs: %s' % key)
    cache = {}
    for line in (directory / 'reference.jsonl').read_text().splitlines():
        row = json.loads(line)
        cache[(row['source_uav'], row['image_stamp'], row['image_sha256'])] = row
    return cache


class ReferenceDetector:
    """Load model lazily; cache reuse has no misleading new timing samples."""
    def __init__(self, args):
        self.args, self.model, self.load_wall_s = args, None, None
        self.warmup_wall_s = None

    def detect(self, image):
        if self.model is None:
            started = time.perf_counter()
            import torch
            from ultralytics import YOLO
            torch.set_num_threads(self.args.threads)
            try:
                torch.set_num_interop_threads(self.args.threads)
            except RuntimeError:
                pass
            self.model = YOLO(str(self.args.weights))
            self.load_wall_s = time.perf_counter() - started
            started = time.perf_counter()
            self._predict(image)
            self.warmup_wall_s = time.perf_counter() - started
        started = time.perf_counter()
        result = self._predict(image)
        detections = []
        for box in result.boxes:
            label = str(result.names[int(box.cls[0])]).lower()
            if label not in self.args.labels:
                continue
            detections.append(dict(label=label, box=[float(v) for v in box.xyxy[0].tolist()],
                                   confidence=float(box.conf[0])))
        return detections, time.perf_counter() - started

    def _predict(self, image):
        return self.model.predict(image, imgsz=self.args.imgsz, conf=self.args.confidence,
                                  device=self.args.device, verbose=False)[0]


def evaluate(args):
    from perception.fast_visual_tracker import FastVisualTracker
    rows = load_manifest(args.manifest, args.max_frames, args.source)
    if not rows:
        raise ValueError('No frames selected')
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(args.threads)
    config = dict(weights=str(args.weights.resolve()), weights_sha256=sha256(args.weights),
                  tracker_file=str(Path(inspect.getfile(FastVisualTracker)).resolve()),
                  tracker_sha256=sha256(inspect.getfile(FastVisualTracker)),
                  evaluator_sha256=sha256(Path(__file__)),
                  confidence=args.confidence, imgsz=args.imgsz, device=args.device,
                  labels=args.labels, opencv_threads=args.threads,
                  opencv_version=cv2.__version__)
    cache = load_reference_cache(args.reference_cache, config)
    (args.output / 'reference_config.json').write_text(json.dumps(config, indent=2) + '\n')
    detector = ReferenceDetector(args)
    counts, reasons, matching_reasons = Counter(), Counter(), Counter()
    gaps, tracker_times, seed_times, yolo_times, differences, hold_differences = [], [], [], [], [], []
    states, per_source = {}, defaultdict(Counter)
    comparisons, image_count = [], 0
    with (args.output / 'reference.jsonl').open('x') as reference_file, \
            (args.output / 'tracking.jsonl').open('x') as tracking_file:
        for number, row in enumerate(rows):
            image = cv2.imread(row['image'], cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Cannot decode image: %s' % row['image'])
            image_hash = sha256(row['image'])
            key = (row['source_uav'], row['image_stamp'], image_hash)
            reference = cache.get(key)
            if reference is None:
                detections, elapsed = detector.detect(image)
                reference = dict(row, image_sha256=image_hash, detections=detections,
                                 inference_wall_s=elapsed, reused=False)
                yolo_times.append(elapsed)
            else:
                reference = dict(reference, **row, reused=True)
                detections = reference['detections']
                counts['cached_reference_frames'] += 1
            reference_file.write(json.dumps(reference) + '\n')
            camera, stamp = row['source_uav'], row['image_stamp']
            state = states.setdefault(camera, dict(index=0, previous=None, tracker=None, seeds={}))
            interval = None if state['previous'] is None else stamp - state['previous']
            if interval is not None:
                gaps.append(interval)
                if interval > args.max_gap:
                    counts['camera_gaps_over_limit'] += 1
                    per_source[camera]['gaps_over_limit'] += 1
            anchor = state['index'] % args.seed_every == 0
            event = dict(row, anchor=anchor, image_interval_s=interval)
            counts['frames'] += 1
            per_source[camera]['frames'] += 1
            if anchor:
                # YOLO corrections replace all hypotheses on every scheduled
                # anchor. No middle-frame reference leaks into this state.
                state['tracker'] = FastVisualTracker(max_age_s=args.max_age,
                    max_gap_s=args.max_gap, max_tracks=args.max_tracks)
                selected = sorted(detections, key=lambda d: d['confidence'], reverse=True)[:args.max_tracks]
                seeds = [dict(item, id='%s:%d:%d' % (camera, state['index'], j))
                         for j, item in enumerate(selected)]
                state['seeds'] = {item['id']: item for item in seeds}
                started = time.perf_counter()
                results = state['tracker'].seed(image, stamp, seeds)
                seed_times.append(time.perf_counter() - started)
                event['results'] = results
                counts['anchor_frames'] += 1
                counts['anchor_detections'] += len(seeds)
                for result in results:
                    if result['status'] != 'seeded':
                        reasons['seed:' + str(result.get('reason', result['status']))] += 1
            else:
                started = time.perf_counter()
                results = state['tracker'].step(image, stamp)
                elapsed = time.perf_counter() - started
                tracker_times.append(elapsed)
                counts['intermediate_frames'] += 1
                opportunities = len(state['seeds'])
                counts['intermediate_object_opportunities'] += opportunities
                per_source[camera]['object_opportunities'] += opportunities
                valid = [result for result in results if result['status'] == 'tracked']
                counts['tracked_objects'] += len(valid)
                per_source[camera]['tracked_objects'] += len(valid)
                if opportunities:
                    counts['intermediate_frames_with_seed'] += 1
                if valid:
                    counts['intermediate_frames_with_track'] += 1
                returned_ids = set(result['id'] for result in results)
                for identifier in state['seeds']:
                    if identifier not in returned_ids:
                        reasons['no_live_track'] += 1
                for result in results:
                    if result['status'] != 'tracked':
                        reasons[str(result.get('reason', result['status']))] += 1
                    if (result.get('control_eligible') or result.get('report_eligible') or
                            result.get('footpoint_valid')):
                        raise AssertionError('Shadow tracker granted production qualification')
                matches, unmatched = unique_matches(valid, detections)
                matching_reasons.update(unmatched)
                counts['unambiguous_reference_matches'] += len(matches)
                measurements = []
                for prediction_index, reference_index in matches:
                    prediction, actual = valid[prediction_index], detections[reference_index]
                    difference = box_difference(prediction['box'], actual['box'])
                    held = box_difference(state['seeds'][prediction['id']]['box'], actual['box'])
                    differences.append(difference)
                    hold_differences.append(held)
                    measurements.append(dict(id=prediction['id'], label=prediction['label'],
                        reference_index=reference_index, tracked=difference, held_anchor=held))
                event.update(results=results, tracker_wall_s=elapsed, matches=measurements,
                             association_exclusions=dict(unmatched), object_opportunities=opportunities)
                if valid and image_count < args.max_comparisons:
                    drawn = image.copy()
                    for detection in detections:
                        a, b, c, d = map(lambda x: int(round(x)), detection['box'])
                        cv2.rectangle(drawn, (a, b), (c, d), (0, 220, 0), 1)
                    for prediction in valid:
                        a, b, c, d = map(lambda x: int(round(x)), prediction['box'])
                        cv2.rectangle(drawn, (a, b), (c, d), (0, 150, 255), 1)
                    cv2.putText(drawn, 'GREEN=YOLO reference ORANGE=tracker (NOT ground truth)',
                                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
                    name = 'comparison_%03d.jpg' % image_count
                    if not cv2.imwrite(str(args.output / name), drawn):
                        raise OSError('Failed comparison image write')
                    comparisons.append(dict(image=name, source_uav=camera, image_stamp=stamp,
                                            reviewed=False))
                    image_count += 1
            tracking_file.write(json.dumps(event) + '\n')
            state['previous'], state['index'] = stamp, state['index'] + 1
            if (number + 1) % 25 == 0:
                print(json.dumps(dict(processed=number + 1, tracked=counts['tracked_objects'])), flush=True)
    fields = ('iou', 'center_px', 'bottom_px', 'bottom_center_px')
    summary = dict(
        scope='Image-only shadow replay; YOLO boxes are NOT ground truth. No world-coordinate, footpoint, identity or flight success claim.',
        reference_warning='Ambiguous matches are excluded from box differences but counted. Matched-only statistics do not include lost tracks. Box bottom is not a verified footpoint.',
        paired_selection_warning='Tracker and held-box differences use only successful, uniquely matched intermediate outputs. Improvement on this selected subset does not describe all seeded-object opportunities or prove a reduction in identity switches.',
        manifest=str(args.manifest.resolve()), manifest_sha256=sha256(args.manifest),
        config=config, parameters=dict(seed_every=args.seed_every, max_age_s=args.max_age,
            max_gap_s=args.max_gap, max_tracks=args.max_tracks, max_frames=args.max_frames,
            source=args.source), counts=dict(counts), source_counts=dict(per_source),
        image_interval_s=distribution(gaps),
        consecutive_pairs_within_gap=sum(value <= args.max_gap for value in gaps),
        valid_intermediate_object_fraction=(counts['tracked_objects'] / counts['intermediate_object_opportunities']
            if counts['intermediate_object_opportunities'] else None),
        matched_subset_fraction_of_opportunities=(len(differences) / counts['intermediate_object_opportunities']
            if counts['intermediate_object_opportunities'] else None),
        failure_reasons=dict(reasons), association_exclusions=dict(matching_reasons),
        tracker_wall_ms=distribution(1000 * value for value in tracker_times),
        seed_wall_ms=distribution(1000 * value for value in seed_times),
        yolo_wall_ms_fresh_only=distribution(1000 * value for value in yolo_times),
        model_load_wall_s=detector.load_wall_s, model_warmup_wall_s=detector.warmup_wall_s,
        tracker_vs_yolo_reference={field: distribution(d[field] for d in differences) for field in fields},
        held_anchor_vs_same_matched_reference={field: distribution(d[field] for d in hold_differences) for field in fields},
        comparison_images=comparisons,
        high_frequency_validation_supported=any(value <= args.max_gap for value in gaps))
    if not summary['high_frequency_validation_supported']:
        summary['limitation'] = 'All per-camera intervals exceed max_gap_s; this is a sparse-input safety smoke test, not a high-frequency tracking evaluation.'
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing results are never overwritten')
    parser.add_argument('--max-frames', type=int, default=200)
    parser.add_argument('--seed-every', type=int, default=3)
    parser.add_argument('--source', help='Optional source_uav selection')
    parser.add_argument('--reference-cache', type=Path, help='An earlier evaluator output directory')
    parser.add_argument('--max-age', type=float, default=.6)
    parser.add_argument('--max-gap', type=float, default=.3)
    parser.add_argument('--max-tracks', type=int, default=6)
    parser.add_argument('--max-comparisons', type=int, default=8)
    parser.add_argument('--confidence', type=float, default=.30)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--labels', nargs='+', default=['red', 'green', 'blue', 'white', 'brown'])
    args = parser.parse_args()
    if (args.max_frames <= 0 or args.seed_every < 2 or args.max_gap <= 0 or args.max_age <= 0
            or args.max_tracks <= 0 or args.threads <= 0 or args.max_comparisons < 0
            or not 0 < args.confidence < 1):
        parser.error('Invalid sampling/threshold settings; seed-every must be at least 2')
    if not args.weights.is_file():
        parser.error('Weights must be an existing local file')
    try:
        evaluate(args)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
