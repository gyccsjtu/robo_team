#!/usr/bin/env python3
"""Offline footpoint evidence audit; never publishes or grants report eligibility.

Consumes evaluate_fast_visual_tracker outputs and verifies their native images.
YOLO box bottoms are only a comparison, never anatomical foot ground truth.
Optional human annotations provide pixel-error measurements, not world truth.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


FX = FY = 376.0
CX, CY = 376.0, 240.0
WIDTH, HEIGHT = 752, 480
OPT_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def distribution(values):
    values = [float(x) for x in values if math.isfinite(float(x))]
    if not values:
        return dict(n=0, p50=None, p95=None, max=None)
    return dict(n=len(values), **dict(zip(('p50', 'p95', 'max'),
                                        map(float, np.percentile(values, [50, 95, 100])))))


def rotation_pitch(degrees):
    angle = math.radians(degrees)
    return np.array([[math.cos(angle), 0., math.sin(angle)], [0., 1., 0.],
                     [-math.sin(angle), 0., math.cos(angle)]])


def ground_xy(u, v, height, rotation):
    """Same calibrated optical/body convention as perception, with camera at XY=0."""
    if not all(math.isfinite(float(x)) for x in (u, v, height)) or height <= 0.:
        return None
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        return None
    ray = rotation @ OPT_TO_BODY @ np.array([(u-CX)/FX, (v-CY)/FY, 1.])
    if ray[2] >= -1e-6:
        return None
    return (-height / ray[2] * ray)[:2]


def sensitivity_case(height, horizontal_range, pitch, pixel_delta):
    """Exact perturbation, not an uncertainty guarantee or actual-frame error."""
    rotation = rotation_pitch(pitch)
    foot_world = np.array([horizontal_range, 0., -height])
    optical = (rotation @ OPT_TO_BODY).T @ foot_world
    if optical[2] <= 0.:
        return dict(height_m=height, horizontal_range_m=horizontal_range,
                    pitch_deg=pitch, eligible_geometry=False, reason='behind_camera')
    u, v = CX + FX*optical[0]/optical[2], CY + FY*optical[1]/optical[2]
    sin_down = height / math.hypot(height, horizontal_range)
    base = ground_xy(u, v, height, rotation)
    errors = {}
    for name, du, dv in (('bottom_down', 0., pixel_delta),
                         ('bottom_up', 0., -pixel_delta),
                         ('horizontal_right', pixel_delta, 0.)):
        point = ground_xy(u+du, v+dv, height, rotation)
        errors[name + '_delta_m'] = (None if point is None else
                                    float(np.linalg.norm(point-base)))
        errors[name + '_pixel_in_frame'] = bool(0 <= u+du < WIDTH and 0 <= v+dv < HEIGHT)
    return dict(height_m=height, horizontal_range_m=horizontal_range,
                pitch_deg=pitch, pixel_delta=pixel_delta, projected_foot_px=[u, v],
                sin_down=sin_down, foot_inside_image=bool(0 <= u < WIDTH and 0 <= v < HEIGHT),
                eligible_geometry=bool(0 < height < 6. and sin_down >= .30 and
                                       0 <= u < WIDTH and 0 <= v < HEIGHT), **errors)


def stereo_sensitivity(depths, baseline=.12):
    """Ideal rectified Z=f*b/disparity; actual disparity must be validated first."""
    result = []
    for depth in depths:
        disparity = FX * baseline / depth
        row = dict(optical_depth_m=depth, baseline_m=baseline, disparity_px=disparity)
        for delta in (.25, .5, 1.):
            values = [abs(FX*baseline/(disparity+sign*delta)-depth)
                      for sign in (-1., 1.) if disparity+sign*delta > 0.]
            row['worst_depth_change_%spx_m' % delta] = max(values)
        result.append(row)
    return result


def load_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def key(row):
    return str(row['source_uav']), float(row['image_stamp'])


def validate_image(reference):
    path = Path(reference['image'])
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != reference['image_sha256']:
        raise ValueError('Native image hash mismatch: %s' % path)
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (HEIGHT, WIDTH):
        raise ValueError('Native camera dimensions must be 752x480: %s' % path)


def audit(evaluation, output, annotations=None, max_samples=24):
    reference_rows = load_jsonl(evaluation / 'reference.jsonl')
    references = {key(row): row for row in reference_rows}
    if len(reference_rows) != len(references):
        raise ValueError('Duplicate reference image stamps')
    for reference in reference_rows:
        validate_image(reference)
    annotation_index = {}
    if annotations is not None:
        for row in load_jsonl(annotations):
            index = key(row) + (str(row['track_id']),)
            if index in annotation_index:
                raise ValueError('Duplicate annotation')
            annotation_index[index] = row
    samples, differences, marked_errors = [], [], []
    counts = Counter()
    seen = set()
    for event in load_jsonl(evaluation / 'tracking.jsonl'):
        frame_key = key(event)
        if frame_key in seen:
            raise ValueError('Duplicate tracking image stamp')
        seen.add(frame_key)
        reference = references.get(frame_key)
        if reference is None:
            raise ValueError('Tracking image has no reference entry')
        if str(Path(event['image']).resolve()) != str(Path(reference['image']).resolve()):
            raise ValueError('Tracking/reference image paths differ')
        counts['frames'] += 1
        if event.get('anchor'):
            continue
        matches = {str(item['id']): item for item in event.get('matches', [])}
        for result in event.get('results', []):
            counts['result_' + str(result['status'])] += 1
            box = result.get('box')
            if box is None or len(box) != 4 or not all(math.isfinite(float(x)) for x in box):
                continue
            match = matches.get(str(result['id']))
            sample = dict(source_uav=event['source_uav'], image_stamp=event['image_stamp'],
                image=reference['image'], image_sha256=reference['image_sha256'],
                track_id=result['id'], label=result['label'], tracker_box=box,
                tracker_status=result['status'], reason=result.get('reason'),
                yolo_reference_box=None, bottom_difference_px=None,
                review_reasons=[])
            if result['status'] != 'tracked':
                sample['review_reasons'].append('tracking_failed')
            if match is not None:
                actual = reference['detections'][int(match['reference_index'])]
                if actual['label'] != result['label']:
                    raise ValueError('Matched reference has a different label')
                bottom_difference = abs(float(box[3])-float(actual['box'][3]))
                sample.update(yolo_reference_box=actual['box'],
                              bottom_difference_px=bottom_difference)
                differences.append(bottom_difference)
            else:
                sample['review_reasons'].append('no_unique_yolo_reference')
            if box[3] >= HEIGHT - 6 or box[0] <= 6 or box[2] >= WIDTH - 6:
                sample['review_reasons'].append('near_image_boundary')
            if box[3]-box[1] < 65:
                sample['review_reasons'].append('small_person_box')
            annotation = annotation_index.get(frame_key + (str(result['id']),))
            if annotation is not None:
                if annotation.get('image_sha256') != reference['image_sha256']:
                    raise ValueError('Annotation image hash mismatch')
                if (annotation.get('reviewed') is True and annotation.get('same_person') is True
                        and annotation.get('foot_visible') is True):
                    foot = annotation.get('foot_px')
                    if (not isinstance(foot, list) or len(foot) != 2 or
                            not all(math.isfinite(float(x)) for x in foot) or
                            not 0 <= foot[0] < WIDTH or not 0 <= foot[1] < HEIGHT):
                        raise ValueError('A visible reviewed foot requires valid foot_px')
                    if result['status'] == 'tracked':
                        marked_errors.append(math.hypot((box[0]+box[2])/2.-foot[0],
                                                       box[3]-foot[1]))
                        counts['human_annotated_tracked_feet'] += 1
                else:
                    counts['annotations_not_eligible_for_pixel_error'] += 1
            samples.append(sample)
    # Select difficult cases and spread them across labels/times; raw image paths
    # remain unchanged. No automatic anatomical labels are invented.
    by_error = sorted(samples, key=lambda row: row['bottom_difference_px'] or 0., reverse=True)
    prioritized = []
    for reason in ('tracking_failed', 'no_unique_yolo_reference', 'near_image_boundary'):
        prioritized.extend([row for row in samples if reason in row['review_reasons']][:3])
    for label in sorted({row['label'] for row in samples}):
        prioritized.extend([row for row in by_error if row['label'] == label][:3])
    prioritized.extend(by_error)
    chosen, selected = [], set()
    for row in prioritized:
        # At most one example per person/0.25s prevents 24 adjacent near-duplicates.
        bucket = (row['source_uav'], row['label'], int(row['image_stamp'] * 4))
        if bucket in selected:
            continue
        selected.add(bucket)
        chosen.append(dict(row, reviewed=False, same_person=None,
                           foot_visible=None, foot_px=None, note=''))
        if len(chosen) >= max_samples:
            break
    sensitivities = [sensitivity_case(h, r, p, delta)
                    for h in (3., 4., 5.) for r in (5., 8., 10., 12.)
                    for p in (-8., 0., 8.) for delta in (1., 2., 5., 8.)]
    summary = dict(scope='Offline footpoint evidence and theoretical sensitivity only.',
        source_evaluation=str(evaluation.resolve()), counts=dict(counts),
        native_images_verified=len(references),
        tracker_vs_yolo_bottom_difference_px=distribution(differences),
        tracker_bottom_vs_human_foot_px=distribution(marked_errors),
        world_error_evaluable=False,
        world_error_reason='No independent world ground truth, calibrated image-time pose validation, '
                           'or anatomical foot labels inferred from YOLO. Pixel agreement is not <1m proof.',
        footpoint_evaluable=bool(marked_errors),
        report_eligible=False, manual_review_samples=len(chosen),
        sensitivity_warning='Hypothetical known camera poses/flat ground. Not measured world errors; '
                            'height and pose uncertainty, occlusion and identity errors are excluded.',
        sensitivity_cases=sensitivities,
        stereo_sensitivity=stereo_sensitivity((5., 8., 10., 12.)),
        stereo_warning='Official 0.12m baseline is small at tracking range. Existing sample contains '
                       'left images only; synchronized right images/calibration are required. '
                       'Depth of a torso surface is not the actor ground-center coordinate.',
        annotation_instructions='Review raw image, confirm same person, label visible ground contact '
            'foot_px=[u,v] in the native 752x480 image. Mark occluded/ambiguous feet false; '
            'do not copy a detector bottom. Paired feet/raised feet need an explicit annotation convention.')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (output / 'annotation_template.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in chosen))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory, never overwrite')
    parser.add_argument('--annotations', type=Path)
    parser.add_argument('--max-samples', type=int, default=24)
    args = parser.parse_args()
    if args.max_samples <= 0:
        parser.error('max-samples must be positive')
    try:
        summary = audit(args.evaluation, args.output, args.annotations, args.max_samples)
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    print(json.dumps({k: v for k, v in summary.items() if k != 'sensitivity_cases'}, indent=2))


if __name__ == '__main__':
    main()
