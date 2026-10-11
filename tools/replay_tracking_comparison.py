#!/usr/bin/env python3
"""Offline same-image comparison; never starts ROS, a simulator, or control.

Infer each verified burst JPEG once, then compare the current ImageMOT with
the actual PixelCueTracker definitions from pinned commit 997cfa0. This tests
image association/cue wiring, not the old world/bridge/controller pipeline.
Identity births and bbox continuity are proxies, not ground-truth ID switches.
"""
import argparse
import ast
from bisect import bisect_left
from collections import Counter, defaultdict
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
LEGACY_COMMIT = '997cfa006a41d60e58eff724c5762ca13a3d9cc3'
LEGACY_PATH = 'perception/perception_real.py'
DISCLAIMER = ('Offline same JPEG/detection comparison of image association only. '
              'Not an old/new whole-match causal comparison; no actor identity '
              'ground truth, world accuracy, official report or flight acceptance.')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def header_stamp(header):
    stamp = header.get('stamp', {}) if isinstance(header, dict) else {}
    if not isinstance(stamp, dict):
        return None
    seconds, nanos = finite(stamp.get('secs')), finite(stamp.get('nsecs'))
    if seconds is None or nanos is None or not 0 <= nanos < 1e9:
        return None
    return seconds+nanos*1e-9


def load_legacy(repo):
    """Compile the pinned, pure definitions, without importing its ROS node.

    Read git objects rather than a potentially edited checkout. Association is
    not reimplemented or patched. Record every executed definition's hash.
    """
    command = ['git', '-C', str(repo)]
    pointer = repo/'.git'
    if os.name != 'nt' and pointer.is_file():
        match = re.fullmatch(r'gitdir:\s*([A-Za-z]):[/\\](.+)', pointer.read_text().strip())
        if match:
            git_dir = Path('/mnt')/match[1].lower()/match[2].replace('\\', '/')
            if not git_dir.is_dir():
                raise RuntimeError('Windows worktree Git directory unavailable: '+str(git_dir))
            command.extend(['--git-dir='+str(git_dir), '--work-tree='+str(repo)])
    source = subprocess.check_output(command+['show', LEGACY_COMMIT+':'+LEGACY_PATH])
    tree = ast.parse(source.decode('utf8'), filename=LEGACY_PATH)
    constants = {'CLASSES', 'ROI_TOP', 'ROI_BOT', 'FX', 'FY', 'CX', 'CY',
                 'IMG_W', 'IMG_H', 'M_OPT2LINK', 'CAM_OFF_BL', 'PZ_SCALE'}
    definitions = {'target_class_mapping', 'target_boxes_only', '_box_iou',
                   'same_appearance_instance', 'ActorDistractorResolver',
                   'pixel_cue_box_ok', 'PixelCueTracker', 'blue_white_torso_vote',
                   'filter_blue_white_duplicate_boxes', 'quat_to_R', 'R_to_quat',
                   'interpolate_rotation', 'camera_pose_from_local',
                   'image_world_bearing'}
    selected, found, hashes = [], set(), {}
    for node in tree.body:
        names = set()
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            names = {node.name} & definitions
        elif isinstance(node, ast.Assign):
            names = {part.id for target in node.targets for part in ast.walk(target)
                     if isinstance(part, ast.Name)} & constants
        if names:
            selected.append(node)
            found.update(names)
            for name in names:
                hashes[name] = sha256(ast.get_source_segment(source.decode('utf8'), node).encode())
    if found != constants | definitions:
        raise RuntimeError('Pinned source definitions missing: '+str((constants | definitions)-found))
    import numpy as np
    namespace = dict(np=np, math=math, os=os, __name__='pinned_legacy_image_association')
    exec(compile(ast.Module(body=selected, type_ignores=[]), LEGACY_PATH, 'exec'), namespace)
    return NS(**namespace), dict(commit=LEGACY_COMMIT, source_path=LEGACY_PATH,
        source_sha256=sha256(source), executed_definition_sha256=hashes,
        loading='actual pinned git definitions, no ROS node import')


def load_index(recording, sources=(), include_sparse=False, index_bytes=None):
    images, poses, counts = [], defaultdict(dict), Counter()
    previous = {}
    input_stream = (recording/'events.jsonl').open(encoding='utf8') if index_bytes is None else io.StringIO(index_bytes.decode('utf8'))
    with input_stream as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except (TypeError, ValueError):
                counts['malformed_json_lines'] += 1
                continue
            if not isinstance(row, dict):
                counts['malformed_json_lines'] += 1
                continue
            source, topic = row.get('source_uav'), row.get('topic', '')
            if not isinstance(topic, str):
                counts['invalid_topic_lines'] += 1
                continue
            if not source and topic.startswith('/typhoon_h480_'):
                source = topic.split('/')[1]
            if not source or (sources and source not in sources):
                continue
            if topic == '/%s/mavros/local_position/pose' % source:
                payload = row.get('payload', {})
                if not isinstance(payload, dict):
                    counts['invalid_pose_lines'] += 1
                    continue
                stamp = header_stamp(payload.get('header', {}))
                if stamp is not None and stamp > 0 and isinstance(payload.get('pose'), dict):
                    poses[source][stamp] = payload['pose']
                continue
            if not row.get('image_file'):
                continue
            if row.get('image_kind') != 'burst' and not include_sparse:
                counts['sparse_images_excluded'] += 1
                continue
            if row.get('image_saved') is not True:
                counts['unsaved_images_excluded'] += 1
                continue
            stamp = finite(row.get('sample_s'))
            original = header_stamp(row.get('image_header', {}))
            if stamp is None or stamp <= 0 or original is None or abs(stamp-original) > 1e-6:
                counts['invalid_original_image_stamp'] += 1
                continue
            if source in previous and stamp <= previous[source]:
                counts['nonincreasing_stamp_in_recording_order'] += 1
            previous[source] = stamp
            row.update(source_uav=source, sample_s=stamp, index_line=line_number)
            images.append(row)
    images.sort(key=lambda row: (row['sample_s'], row['source_uav'], row['index_line']))
    return images, {source: sorted(series.items()) for source, series in poses.items()}, counts


def camera_at(series, stamp, legacy, slack, max_gap):
    """Original pose stamps only; match the node's interpolation and 50 ms slack.

    A constant same-UAV ENU translation cancels in legacy bearing/depth matching,
    so no invented world spawn offset or ground-height datum is required here.
    """
    if not series:
        return None
    i = bisect_left(series, (stamp,))
    if i < len(series) and abs(series[i][0]-stamp) < 1e-8:
        bracket = [series[i]]
    elif len(series) == 1:
        bracket = series if abs(stamp-series[0][0]) <= slack else []
    elif i == 0:
        bracket = series[:2] if series[0][0]-stamp <= slack else []
    elif i == len(series):
        bracket = series[-2:] if stamp-series[-1][0] <= slack else []
    else:
        bracket = series[i-1:i+1]
    if not bracket or (len(bracket) == 2 and not 0 < bracket[1][0]-bracket[0][0] <= max_gap):
        return None
    values = []
    try:
        for ts, pose in bracket:
            msg = NS(pose=NS(position=NS(**pose['position']), orientation=NS(**pose['orientation'])))
            values.append((ts, legacy.camera_pose_from_local(msg, (0., 0., 0.))))
        if len(values) == 1:
            return values[0][1]
        (a, first), (b, second) = values
        alpha = (stamp-a)/(b-a)
        xyz = tuple(first[j]+alpha*(second[j]-first[j]) for j in range(3))
        return xyz+(legacy.interpolate_rotation(first[3], second[3], alpha),)
    except (KeyError, TypeError, ValueError, AttributeError):
        return None


def common_preprocess(image, detections, resolver, mapping, names, stamp, legacy):
    """Shared appearance resolver and the production .85 same-color NMS."""
    actors, _ = resolver.resolve(detections, mapping, names, stamp)
    by_class, filtered = defaultdict(list), []
    for box in actors:
        by_class[int(box.cls)].append(box)
    for boxes in by_class.values():
        kept = []
        for box in sorted(boxes, key=lambda b: float(b.conf), reverse=True):
            xyxy = [float(v) for v in box.xyxy[0]]
            if not any(legacy._box_iou(xyxy, [float(v) for v in other.xyxy[0]]) > .85 for other in kept):
                kept.append(box)
        filtered.extend(kept)
    return legacy.filter_blue_white_duplicate_boxes(image, filtered)


def cue_box(box, image, pose, legacy):
    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
    u, v = (x1+x2)/2., (y1+y2)/2.
    result = dict(cls=legacy.CLASSES[int(box.cls)], conf=float(box.conf), u=u, v=v,
                  h=y2-y1, box=[x1, y1, x2, y2], width=image.shape[1], height=image.shape[0],
                  image_track_id=getattr(box, 'image_track_id', None),
                  appearance_status=box.appearance_status,
                  appearance_confidence=box.appearance_confidence,
                  distractor_confidence=box.distractor_confidence,
                  bearing=None, cam_x=None, cam_y=None)
    if pose is not None:
        x, y, z, rotation = pose
        result.update(bearing=legacy.image_world_bearing(u, v, rotation), cam_x=x, cam_y=y,
                      camera_tilt=math.acos(max(-1., min(1., float(rotation[2, 2])))))
    return result


def legacy_ids(tracker, stamp, boxes):
    """Read original internal assignments, including not-yet-publishable IDs.

    The old update API emits only ready cues. Identical ambiguous boxes are not
    guessed: report None if reading its state does not identify a unique owner.
    """
    result = {}
    for index, box in boxes:
        owners = [tr for tr in tracker.tracks if tr['sample_s'] == stamp and tr['cls'] == box['cls']
                  and all(abs(tr[field]-box[field]) < 1e-8 for field in ('u', 'v', 'h'))]
        duplicates = sum(other['cls'] == box['cls'] and all(
            abs(other[field]-box[field]) < 1e-8 for field in ('u', 'v', 'h')) for _, other in boxes)
        result[index] = '%s:%d' % (box['cls'], owners[0]['id']) if len(owners) == duplicates == 1 else None
    return result


def numeric_stats(values):
    ordered = sorted(values)
    if not ordered:
        return dict(count=0, status='ABSTAIN')
    return dict(count=len(ordered), mean=sum(ordered)/len(ordered), median=ordered[(len(ordered)-1)//2],
                p95=ordered[int((len(ordered)-1)*.95)], maximum=ordered[-1])


class Continuity:
    def __init__(self, gap):
        self.gap, self.times, self.previous, self.counts = gap, defaultdict(list), None, Counter()

    def observe(self, stamp, rows):
        current = {row['id'] for row in rows if row.get('id')}
        self.counts['frames'] += 1
        self.counts['detections'] += len(rows)
        self.counts['assigned_detections'] += sum(bool(row.get('id')) for row in rows)
        for key in current:
            self.times[key].append(stamp)
        if self.previous and 0 < stamp-self.previous[0] <= self.gap:
            previous_ids = {row['id'] for row in self.previous[1] if row.get('id')}
            self.counts['adjacent_previous_ids'] += len(previous_ids)
            self.counts['adjacent_retained_ids'] += len(current & previous_ids)
            # Strictly unique overlapping boxes are only a continuity proxy.
            for row in rows:
                options = [old for old in self.previous[1] if old['cls'] == row['cls']
                           and bbox_iou(old['box'], row['box']) >= .5]
                if len(options) != 1 or sum(other['cls'] == row['cls'] and
                        bbox_iou(options[0]['box'], other['box']) >= .5 for other in rows) != 1:
                    continue
                old = options[0]
                if row.get('id') and old.get('id'):
                    self.counts['unique_bbox_continuity_pairs'] += 1
                    self.counts['changed_id_on_unique_bbox_proxy'] += row['id'] != old['id']
        self.previous = stamp, rows

    def summary(self):
        spans, observations = [], []
        for times in self.times.values():
            start, before = times[0], times[0]
            for stamp in times[1:]:
                if stamp-before > self.gap:
                    spans.append(before-start)
                    start = stamp
                before = stamp
            spans.append(before-start)
            observations.append(len(times))
        denominator = self.counts['adjacent_previous_ids']
        return dict(self.counts, unique_observed_ids=len(self.times),
            adjacent_id_retention_ratio=self.counts['adjacent_retained_ids']/denominator if denominator else None,
            continuity_gap_s=self.gap, observed_contiguous_id_span_s=numeric_stats(spans),
            observations_per_id=numeric_stats(observations),
            identity_switch_ground_truth='ABSTAIN; births and bbox pairing are proxies only')


def bbox_iou(a, b):
    overlap = max(0., min(a[2], b[2])-max(a[0], b[0]))*max(0., min(a[3], b[3])-max(a[1], b[1]))
    return overlap/max((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-overlap, 1e-9)


def verified_image(recording, row):
    path = (recording/row['image_file']).resolve()
    if recording.resolve() not in path.parents or path.suffix.lower() not in ('.jpg', '.jpeg'):
        raise ValueError('JPEG path escapes recording directory or has wrong extension')
    raw = path.read_bytes()
    if not row.get('jpeg_sha256') or sha256(raw) != row['jpeg_sha256']:
        raise ValueError('Missing or incorrect saved JPEG sha256')
    import cv2
    import numpy as np
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Saved JPEG cannot be decoded')
    if image.shape[:2] != (row.get('height'), row.get('width')) or image.shape[:2] != (480, 752):
        raise ValueError('Image dimensions do not match original index or pinned 752x480 intrinsics')
    return image


def run(args):
    # Freeze "latest" before reading: a new trial may change that symlink while
    # inference runs, but its metadata/JPEGs must not replace this trial's input.
    args.recording = args.recording.resolve()
    legacy, provenance = load_legacy(args.legacy_repo)
    # A live recorder can append while inference runs. Freeze the actual bytes
    # parsed, rather than hashing a later, different version of its index.
    index_bytes = (args.recording/'events.jsonl').read_bytes()
    images, poses, index_counts = load_index(args.recording, args.uav, args.include_sparse, index_bytes)
    metadata_path = args.recording/'metadata.json'
    metadata_bytes = metadata_path.read_bytes() if metadata_path.is_file() else b''
    try:
        captured_metadata = json.loads(metadata_bytes.decode('utf8')) if metadata_bytes else {}
    except (ValueError, TypeError):
        captured_metadata = {}
    if not isinstance(captured_metadata, dict):
        captured_metadata = {}
    finalized = captured_metadata.get('writer_drained') is True
    total_selected = len(images)
    if args.limit:
        images = images[:args.limit]
    args.out.mkdir(parents=True, exist_ok=False)
    summary = dict(schema_version=1, mode='offline_same_image_association_comparison',
        formal_competition=False, control_or_truth_inputs=False, scope=DISCLAIMER,
        legacy=provenance, original_index_sha256=sha256(index_bytes), original_index_bytes_read=len(index_bytes),
        recording_finalized=finalized, recording_metadata_sha256=sha256(metadata_bytes) if metadata_bytes else None,
        recorded_commit=captured_metadata.get('commit'), recorded_source_sha256=captured_metadata.get('source_sha256'),
        arguments={key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        selected_images=total_selected, replay_image_limit=len(images), index_counts=dict(index_counts),
        ordered_by='original sample_s, source_uav, index line; separate trackers per UAV',
        pose_policy='original MAVROS pose stamps, rotation slerp; both associations skip missing pose; no yaw/receive-time fallback',
        comparison_policy='shared detection/appearance/NMS; old cue eligibility .45, new MOT low-score association',
        effective_legacy_geometry={name: getattr(legacy, name).tolist() if hasattr(getattr(legacy, name), 'tolist')
            else getattr(legacy, name) for name in ('ROI_TOP', 'ROI_BOT', 'FX', 'FY', 'CX', 'CY', 'CAM_OFF_BL')},
        status='ABSTAIN_NO_SAVED_BURST_IMAGES' if not images else 'INPUT_INSPECTED_ONLY')
    write_summary = lambda: (args.out/'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')
    if args.inspect_only or not images:
        write_summary()
        print(json.dumps(dict(status=summary['status'], selected_images=total_selected, out=str(args.out))))
        return 0 if args.inspect_only else 2
    summary['status'] = 'INCOMPLETE_RUNTIME_INITIALIZATION'
    write_summary()
    weights_raw = args.weights.read_bytes()
    if weights_raw.startswith(b'version https://git-lfs.github.com/spec/v1'):
        raise RuntimeError('Weights are a Git LFS pointer, not real detector weights')
    sys.path.insert(0, str(ROOT))
    import cv2
    import numpy as np
    import torch
    import ultralytics
    from ultralytics import YOLO
    from perception.image_mot import ImageMOT
    torch.set_num_threads(args.threads)
    started = time.perf_counter()
    model = YOLO(str(args.weights))
    mapping = legacy.target_class_mapping(model.names)
    summary.update(weights_sha256=sha256(weights_raw), detector_names=model.names,
        environment=dict(python=sys.version, executable=sys.executable, prefix=sys.prefix,
            cv2=cv2.__version__, numpy=np.__version__, torch=torch.__version__, ultralytics=ultralytics.__version__),
        model_load_ms=(time.perf_counter()-started)*1000,
        current_source_sha256={str(path.relative_to(ROOT)): sha256(path.read_bytes()) for path in (
            ROOT/'perception/image_mot.py', ROOT/'perception/vendor/ultralytics_trackers/SOURCE.md')},
        steady_inference_policy='first inference measured separately; excluded from steady timings')
    states, failures, inference_ms = {}, Counter(), []
    try:
        with (args.out/'frames.jsonl').open('x', encoding='utf8') as output:
            for row in images:
                frame = {key: row[key] for key in ('source_uav', 'sample_s', 'image_file', 'jpeg_sha256', 'index_line')}
                source, stamp = row['source_uav'], row['sample_s']
                try:
                    image = verified_image(args.recording, row)
                except (ValueError, OSError) as exc:
                    failures[str(exc)] += 1
                    frame.update(status='SKIPPED_INVALID_JPEG', reason=str(exc))
                    output.write(json.dumps(frame, allow_nan=False)+'\n')
                    continue
                if source not in states:
                    threshold = {color: args.white_conf if color == 'white' else args.conf for color in legacy.CLASSES}
                    states[source] = dict(mot=ImageMOT(legacy.CLASSES, source,
                        session='replay_'+sha256(source.encode())[:8],
                        low_conf=args.low_conf, class_thresholds=threshold, max_absence_s=args.max_absence,
                        max_frame_gap_s=args.max_frame_gap), old=legacy.PixelCueTracker(),
                        resolver=legacy.ActorDistractorResolver(), known=set(), counts=Counter(),
                        timing=defaultdict(list), metrics={name: Continuity(args.continuity_gap) for name in (
                            'botsort_all', 'botsort_common_eligible', 'legacy_common_eligible')})
                state = states[source]
                start = time.perf_counter()
                detected = model(image, conf=args.low_conf, device=args.device,
                                 agnostic_nms=False, verbose=False)[0]
                # Explicit GPU completion makes elapsed wall time comparable.
                if str(args.device).lower() != 'cpu' and torch.cuda.is_available():
                    torch.cuda.synchronize()
                latency = (time.perf_counter()-start)*1000
                inference_ms.append(latency)
                raw = [dict(detector_cls=int(box.cls), conf=float(box.conf),
                            box=[float(v) for v in box.xyxy[0]]) for box in detected.boxes]
                start = time.perf_counter()
                boxes = common_preprocess(image, detected.boxes, state['resolver'], mapping, model.names, stamp, legacy)
                frame['shared_preprocess_ms'] = (time.perf_counter()-start)*1000
                pose = camera_at(poses.get(source, []), stamp, legacy, args.pose_slack, args.pose_max_gap)
                frame['image_time_camera_pose_local_enu'] = (
                    dict(xyz=list(pose[:3]), rotation=pose[3].tolist()) if pose is not None else None)
                if pose is None:
                    # Feeding only the new tracker extra images would confound
                    # the same-input comparison on the next valid pose frame.
                    state['counts']['frames'] += 1
                    state['counts']['legacy_no_pose_frames'] += 1
                    state['counts']['both_associations_skipped_no_pose'] += 1
                    frame.update(status='ABSTAIN_NO_COMPARABLE_POSE', detector_ms=latency,
                        detections_inferred_once=raw,
                        botsort_status='ABSTAIN_NO_IMAGE_TIME_POSE',
                        legacy_status='ABSTAIN_NO_IMAGE_TIME_POSE')
                    output.write(json.dumps(frame, ensure_ascii=False, allow_nan=False)+'\n')
                    continue
                start = time.perf_counter()
                accepted = state['mot'].annotate(image, boxes, stamp)
                frame['botsort_association_ms'] = (time.perf_counter()-start)*1000
                pixels = [cue_box(box, image, pose, legacy) for box in boxes]
                eligible = [(i, box) for i, box in enumerate(pixels) if legacy.pixel_cue_box_ok(
                    box['cls'], box['conf'], *box['box'])]
                start = time.perf_counter()
                new_cues = state['mot'].cues(stamp, [box for _, box in eligible]) if pose is not None else []
                frame['botsort_cue_ms'] = (time.perf_counter()-start)*1000
                old_cues, old_ids = [], {}
                old_fresh = pose is not None and stamp > state['old'].last_frame
                start = time.perf_counter()
                if pose is not None:
                    old_cues = state['old'].update(stamp, [box for _, box in eligible])
                frame['legacy_update_and_cue_ms'] = (time.perf_counter()-start)*1000
                if old_fresh:
                    old_ids = legacy_ids(state['old'], stamp, eligible)
                output_rows = []
                for i, box in enumerate(pixels):
                    threshold = args.white_conf if box['cls'] == 'white' else args.conf
                    key = box['image_track_id']
                    continued_low = bool(accepted and key in state['known'] and box['conf'] < threshold)
                    state['counts']['low_score_continued_old_id'] += continued_low
                    output_rows.append(dict(index=i, cls=box['cls'], conf=box['conf'], box=box['box'],
                        appearance_status=box['appearance_status'], botsort_id=key, legacy_id=old_ids.get(i),
                        common_cue_eligible=any(j == i for j, _ in eligible),
                        below_class_creation_threshold=box['conf'] < threshold,
                        botsort_low_score_continuation=continued_low))
                    if key:
                        state['known'].add(key)
                def metric_rows(indices, field):
                    return [dict(id=output_rows[i][field], cls=output_rows[i]['cls'], box=output_rows[i]['box']) for i in indices]
                if accepted:
                    state['metrics']['botsort_all'].observe(stamp, metric_rows(range(len(pixels)), 'botsort_id'))
                if accepted and old_fresh:
                    indices = [i for i, _ in eligible]
                    state['metrics']['botsort_common_eligible'].observe(stamp, metric_rows(indices, 'botsort_id'))
                    state['metrics']['legacy_common_eligible'].observe(stamp, metric_rows(indices, 'legacy_id'))
                state['counts']['frames'] += 1
                state['counts']['botsort_rejected_duplicate_or_out_of_order'] += not accepted
                state['counts']['legacy_no_pose_frames'] += pose is None
                state['counts']['comparable_frames'] += bool(accepted and old_fresh)
                state['counts']['botsort_ready_cues'] += len(new_cues)
                state['counts']['legacy_ready_cues'] += len(old_cues)
                for key in ('botsort_association_ms', 'botsort_cue_ms', 'legacy_update_and_cue_ms'):
                    if accepted and old_fresh:
                        state['timing'][key].append(frame[key])
                frame.update(status='REPLAYED', detector_ms=latency, detections_inferred_once=raw,
                    detections_after_shared_preprocessing=output_rows, botsort_cues=new_cues,
                    legacy_cues=old_cues, botsort_frame_accepted=accepted,
                    legacy_status='PROCESSED' if old_fresh else 'REJECTED_TIMESTAMP' if pose is not None else 'ABSTAIN_NO_IMAGE_TIME_POSE')
                output.write(json.dumps(frame, ensure_ascii=False, allow_nan=False)+'\n')
        comparable = sum(state['counts']['comparable_frames'] for state in states.values())
        summary['status'] = ('ABSTAIN_NO_VALID_IMAGES' if not inference_ms else
            'ABSTAIN_NO_COMPARABLE_FRAMES' if not comparable else
            'PARTIAL_INVALID_IMAGES' if failures else 'REPLAY_PARTIAL_RECORDING' if not finalized else 'REPLAY_COMPLETE')
    except Exception as exc:
        summary.update(status='INCOMPLETE_ERROR', error=type(exc).__name__+': '+str(exc))
        raise
    finally:
        summary.update(jpeg_failures=dict(failures), actual_inference_calls=len(inference_ms),
            first_inference_ms=inference_ms[0] if inference_ms else None,
            steady_detector_ms=numeric_stats(inference_ms[1:]),
            sources={source: dict(counts=dict(state['counts']), image_mot_stats=state['mot'].stats,
                timing_ms={name: numeric_stats(values) for name, values in state['timing'].items()},
                continuity={name: metric.summary() for name, metric in state['metrics'].items()})
                for source, state in states.items()})
        write_summary()
    print(json.dumps(dict(status=summary['status'], actual_inference_calls=len(inference_ms), out=str(args.out))))
    return 0 if summary['status'] == 'REPLAY_COMPLETE' else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True, help='Observer directory containing events.jsonl/images')
    parser.add_argument('--out', type=Path, required=True, help='New output directory; never overwrite prior results')
    parser.add_argument('--legacy-repo', type=Path, default=ROOT.parent/'robo_team_tracking_review_20261011')
    parser.add_argument('--weights', type=Path, default=Path(os.environ.get('PR_WEIGHTS', ROOT/'weights/best_yolo11n_bino_v1.pt')))
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--uav', action='append', default=[], help='Repeat to select sources; default all saved bursts')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--include-sparse', action='store_true', help='Diagnostic only; sparse JPEGs are not a continuous MOT sequence')
    parser.add_argument('--inspect-only', action='store_true', help='Validate index/source pin only; never load detector')
    parser.add_argument('--conf', type=float, default=.40)
    parser.add_argument('--white-conf', type=float, default=.30)
    parser.add_argument('--low-conf', type=float, default=.10)
    parser.add_argument('--max-absence', type=float, default=3.)
    parser.add_argument('--max-frame-gap', type=float, default=3.)
    parser.add_argument('--continuity-gap', type=float, default=.5)
    parser.add_argument('--pose-slack', type=float, default=.05)
    parser.add_argument('--pose-max-gap', type=float, default=.3)
    args = parser.parse_args()
    if not (0 < args.low_conf < min(args.conf, args.white_conf) <= max(args.conf, args.white_conf) <= 1
            and min(args.max_absence, args.max_frame_gap, args.continuity_gap, args.pose_max_gap) > 0
            and 0 <= args.pose_slack <= .05 and args.limit >= 0 and args.threads > 0):
        parser.error('Invalid thresholds, timing bounds, frame limit or CPU thread count')
    try:
        return run(args)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        parser.exit(1, 'Replay failed: %s\n' % exc)


if __name__ == '__main__':
    sys.exit(main())
