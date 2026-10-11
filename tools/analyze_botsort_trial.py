#!/usr/bin/env python3
"""Offline analysis of read-only trial evidence, with explicit missing-data abstention."""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import re

ACTORS = {'green': (0,), 'blue': (1,), 'brown': (2,), 'white': (3,),
          'red': (4, 5), 'red1': (4, 5), 'red2': (4, 5)}


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def stats(values):
    ordered = sorted(value for item in values if (value := finite(item)) is not None)
    if not ordered:
        return {'count': 0, 'status': 'ABSTAIN'}
    def percentile(fraction):
        position = (len(ordered)-1)*fraction
        lower = math.floor(position)
        upper = min(lower+1, len(ordered)-1)
        return ordered[lower]+(ordered[upper]-ordered[lower])*(position-lower)
    return dict(count=len(ordered), min=ordered[0], mean=sum(ordered)/len(ordered),
                median=percentile(.5), p95=percentile(.95), max=ordered[-1])


def events(path):
    with path.open(encoding='utf8') as source:
        for line_number, line in enumerate(source, 1):
            try:
                yield json.loads(line)
            except (ValueError, TypeError):
                yield {'topic': 'observer/malformed_line', 'line_number': line_number}


def xy(pose):
    point = pose.get('position', {})
    x, y = finite(point.get('x')), finite(point.get('y'))
    return None if x is None or y is None else (x, y)


def interpolate(samples, stamp, max_gap):
    """Bracket one unverified source only; never extrapolate past disappearance."""
    if stamp is None or not samples:
        return None
    index = bisect_left(samples, (stamp, -float('inf'), -float('inf')))
    if index < len(samples) and abs(samples[index][0]-stamp) <= 1e-8:
        return samples[index][1:]
    if index == 0 or index == len(samples):
        return None
    a, b = samples[index-1], samples[index]
    if not 0. < b[0]-a[0] <= max_gap:
        return None
    fraction = (stamp-a[0])/(b[0]-a[0])
    return (a[1]+fraction*(b[1]-a[1]), a[2]+fraction*(b[2]-a[2]))


def frame_series(frames, gap=1.):
    ordered = sorted(frames.items())
    if not ordered:
        return {'status': 'ABSTAIN', 'original_frames': 0}
    gaps, longest, start = [], 0., ordered[0][0]
    births, replacements, previous_ids, previous_stamp = 0, 0, set(), None
    for stamp, ids in ordered:
        new_ids = ids-previous_ids
        births += len(new_ids)
        if previous_stamp is not None:
            elapsed = stamp-previous_stamp
            if elapsed > gap:
                gaps.append(elapsed)
                longest = max(longest, previous_stamp-start)
                start = stamp
            elif previous_ids:
                replacements += len(new_ids)
        previous_ids, previous_stamp = ids, stamp
    longest = max(longest, ordered[-1][0]-start)
    return dict(original_frames=len(ordered), first_sample_s=ordered[0][0],
        last_sample_s=ordered[-1][0], max_gap_s=max((b[0]-a[0] for a,b in zip(ordered,ordered[1:])),default=0.),
        gap_threshold_s=gap, reacquisitions_after_gap=len(gaps), missing_spans_s=gaps,
        longest_observation_span_s=longest, observed_id_births=births,
        replacement_candidate_events=replacements,
        unique_ids=len(set().union(*(ids for _,ids in ordered))),
        observation_span_limitations='Successive observations may have different IDs or depict different people',
        identity_ground_truth='ABSTAIN: ID births/replacements are proxies, not verified identity switches')


def rows(payload):
    if isinstance(payload, list):
        return [row for row in payload if isinstance(row, dict)]
    if not isinstance(payload, dict):
        return []
    if isinstance(payload.get('candidates'), list):
        inherited = {key: payload.get(key) for key in ('source_uav', 'sample_s', 'real_detection')}
        return [{**inherited, **row}
                for row in payload['candidates'] if isinstance(row, dict)]
    return [payload]


def summarize(args):
    event_path = args.recording / 'events.jsonl'
    metadata_path = args.recording / 'metadata.json'
    metadata = json.loads(metadata_path.read_text(encoding='utf8')) if metadata_path.exists() else {}
    # Neither API has been shown to follow the animated visible actor mesh.
    # Keep each API independent; conflicting samples must never form a bracket.
    gazebo_samples = defaultdict(lambda: defaultdict(list))
    observed_start, observed_end = None, None
    observed_wall_start, observed_wall_end = None, None
    for event in events(event_path):
        payload = event.get('payload', {})
        stamp = finite(event.get('receive_ros_s'))
        wall = finite(event.get('receive_wall_s'))
        if wall is not None:
            observed_wall_start = wall if observed_wall_start is None else min(observed_wall_start, wall)
            observed_wall_end = wall if observed_wall_end is None else max(observed_wall_end, wall)
        if stamp is not None:
            observed_start = stamp if observed_start is None else min(observed_start, stamp)
            observed_end = stamp if observed_end is None else max(observed_end, stamp)
        if stamp is None or not isinstance(payload, dict):
            continue
        if event['topic'] == '/gazebo/model_states':
            for name, model in payload.get('models', {}).items():
                point = xy(model.get('pose', {}))
                if name.startswith('actor_') and point is not None:
                    gazebo_samples[event['topic']][name].append((stamp, *point))
        elif event['topic'] == 'observer/get_model_state' and payload.get('success'):
            point = xy(payload.get('pose', {}))
            if point is not None:
                gazebo_samples[event['topic']][payload['name']].append((stamp, *point))
    gazebo_samples = {source:{name:sorted(set(points)) for name,points in actors.items()}
                      for source,actors in gazebo_samples.items()}
    interval_start = finite(metadata.get('ros_start_s'))
    interval_end = finite(metadata.get('ros_end_s'))
    if interval_start is None:
        interval_start = observed_start
    if interval_end is None:
        interval_end = observed_end
    wall_start = finite(metadata.get('wall_start_s'))
    wall_end = finite(metadata.get('wall_end_s'))
    if wall_start is None:
        wall_start = observed_wall_start
    if wall_end is None:
        wall_end = observed_wall_end
    def in_interval(stamp):
        return (stamp is not None and interval_start is not None and interval_end is not None
                and interval_start <= stamp <= interval_end)
    counters, stages, images, final_topics = Counter(), defaultdict(lambda: defaultdict(set)), Counter(), {}
    reports, report_gaps, latencies = defaultdict(Counter), defaultdict(list), defaultdict(list)
    diagnostic_counts = defaultdict(lambda: defaultdict(Counter))
    diagnostic_differences = defaultdict(lambda: defaultdict(list))
    bridge_state_counts = defaultdict(Counter)
    observed_pose_uavs = set()
    for event in events(event_path):
        topic, payload = event['topic'], event.get('payload', {})
        counters[topic] += 1
        received = finite(event.get('receive_ros_s'))
        sample = finite(event.get('sample_s'))
        if sample is not None and received is not None:
            latencies[topic].append(received-sample)
        if '/mavros/local_position/pose' in topic:
            observed_pose_uavs.add(topic.split('/')[1])
        if event.get('image_file') and event.get('image_saved', True):
            if (args.recording / event['image_file']).is_file():
                images[event.get('image_kind', 'sparse')] += 1
            else:
                images['missing_saved_files'] += 1
        if topic in ('/score', '/time_usage', '/left_actors'):
            final_topics[topic] = dict(receive_ros_s=received, payload=payload)
        if topic == '/swarm/target_states' and isinstance(payload, dict):
            target = payload.get('target_id')
            if target:
                bridge_state_counts[target]['emitted'] += 1
                fresh = (sample is not None and received is not None and
                         0. <= received-sample <= 2.5 and
                         finite(payload.get('x')) is not None and finite(payload.get('y')) is not None)
                if fresh and not payload.get('eliminated'):
                    bridge_state_counts[target]['nominal_2_5s_sample_window'] += 1
                    stages['published_bridge_state_original_samples:'+target][sample].add('bridge_state')
                else:
                    bridge_state_counts[target]['outside_sample_window_or_eliminated'] += 1
        if topic == '/swarm/tracking_view' and isinstance(payload, dict):
            source, target = payload.get('source_uav'), payload.get('target_id')
            original = finite(payload.get('sample_s'))
            if source and target and original is not None and payload.get('real_detection') is True:
                key = '%s/%s' % (source, target)
                identity = str(payload.get('track_id', payload.get('world_candidate_id', 'unknown')))
                stages['published_real_tracking_views:'+key][original].add(identity)
                if payload.get('geometry_ok') is True:
                    stages['published_geometry_valid_views:'+key][original].add(identity)
        if topic == '/coordination/target_report':
            for row in rows(payload):
                source, target = row.get('source_uav'), row.get('target_id')
                original = finite(row.get('sample_s'))
                if source and target and original is not None and row.get('real_detection') is True:
                    key = 'published_original_target_reports:%s/%s' % (source, target)
                    stages[key][original].add(str(row.get('candidate_id', row.get('track_id', 'unknown'))))
        if topic.startswith('/actor_') and topic.endswith('_info') and isinstance(payload, dict):
            tag = topic[len('/actor_'):-len('_info')]
            reports[tag]['published'] += 1
            report_gaps[tag].append(received)
            cls = payload.get('cls', tag)
            x, y = finite(payload.get('x')), finite(payload.get('y'))
            for source, actors in gazebo_samples.items():
                count = diagnostic_counts[source][tag]
                count['published_reports'] += 1
                candidates = [interpolate(actors.get('actor_%d' % aid, ()), received, args.truth_max_gap)
                              for aid in ACTORS.get(cls, ACTORS.get(tag, ()))]
                candidates = [point for point in candidates if point is not None]
                if x is None or y is None or not candidates:
                    count['no_source_bracket_or_report_coordinate'] += 1
                    continue
                difference = min(math.hypot(x-point[0],y-point[1]) for point in candidates)
                diagnostic_differences[source][tag].append(difference)
                count['source_bracketed_comparisons'] += 1
    report_summary = {}
    for tag, count in reports.items():
        timing = frame_series({stamp: {'actor_report'} for stamp in report_gaps[tag] if stamp is not None})
        report_summary[tag] = dict(counts=dict(count), timing=timing,
            coordinate_accuracy='ABSTAIN: no independently verified visible actor mesh pose',
            full_official_validity='ABSTAIN: coordinate accuracy alone does not evaluate every judge condition',
            status='ABSTAIN')
    gazebo_diagnostic = {
        source:dict(status='UNVERIFIED_GAZEBO_DIAGNOSTIC', source_topic=source,
            source_sample_counts={name:len(points) for name,points in actors.items()},
            reports={tag:dict(counts=dict(count),
                horizontal_coordinate_difference_m=stats(diagnostic_differences[source][tag]))
                for tag,count in sorted(diagnostic_counts[source].items())},
            time_basis='ActorInfo receive time vs this API receive time; headerless transport',
            limitations='Visible animated actor mesh pose correspondence is unverified; this is not location accuracy. '
                        'No cross-source interpolation or extrapolation; red compares either available red model.')
        for source,actors in sorted(gazebo_samples.items())}
    inference, raw_mot, csv_counters, seen_inference = [], defaultdict(lambda: defaultdict(set)), Counter(), set()
    for root in args.csv_root:
        for path in root.rglob('*.csv'):
            try:
                with path.open(encoding='utf8', errors='replace', newline='') as source:
                    for row in csv.DictReader(source):
                        csv_counters['rows'] += 1
                        image_s = finite(row.get('image_stamp'))
                        received_s = finite(row.get('ros_time'))
                        row_wall = finite(row.get('wall_time'))
                        if wall_start is not None and wall_end is not None:
                            if row_wall is None:
                                csv_counters['missing_wall_time_attribution_ABSTAIN'] += 1
                                continue
                            if not wall_start <= row_wall <= wall_end:
                                csv_counters['outside_recording_wall_interval'] += 1
                                continue
                        if not in_interval(received_s if received_s is not None else image_s):
                            csv_counters['outside_recording_interval'] += 1
                            continue
                        csv_counters['rows_in_recording_interval'] += 1
                        duration = finite(row.get('inference_s'))
                        token = (str(path), image_s)
                        if image_s is not None and duration is not None and token not in seen_inference:
                            seen_inference.add(token)
                            inference.append(duration)
                        label = row.get('source', '')
                        if label.startswith('yolo_raw_mot:') and image_s is not None:
                            key = '%s/%s' % (path.stem, row.get('class_name', 'unknown'))
                            raw_mot[key][image_s].add(label[len('yolo_raw_mot:'):])
                        view_s = finite(row.get('view_sample_s'))
                        if (view_s is not None and received_s is not None and
                                0. <= received_s-view_s <= .5):
                            key = 'logged_accepted_control_view:%s/%s' % (path.stem,row.get('target_id', ''))
                            stages[key][view_s].add('accepted_image')
                csv_counters['files_read'] += 1
            except (OSError, csv.Error) as exc:
                csv_counters['unreadable_files'] += 1
    judge = {'status': 'ABSTAIN', 'reason': 'no official judge log supplied'}
    if args.judge_log and args.judge_log.exists():
        text = args.judge_log.read_text(encoding='utf8', errors='replace')
        ids = sorted(set(int(value) for value in re.findall(r'^actor_([0-5]) is OK\s*$',text,re.M)))
        times = [float(value) for value in re.findall(r'^Time usage: ([0-9.]+)',text,re.M)]
        judge = dict(development_judge_eliminated_ids=ids, count=len(ids),
                     mission_finished='Mission finished' in text, reported_time_usage_s=times,
                     failure_lines=[line for line in text.splitlines() if 'Warning:' in line or 'lost; penalty' in line],
                     status='OBSERVED_DEV_LOG', formal_competition=False)
    return dict(schema_version=1, mode='development_simulation', formal_competition=False,
        overall_verdict='ABSTAIN: no formal competition, baseline comparison or continuous collision evidence',
        evidence=dict(recording=str(args.recording), metadata=metadata, observed_topics=dict(counters),
                      six_mavros_pose_streams=sorted(observed_pose_uavs),
                      unverified_gazebo_sample_counts={source:{name:len(points) for name,points in actors.items()}
                                                      for source,actors in gazebo_samples.items()},
                      images=dict(images), csv=dict(csv_counters),
                      csv_roots=[str(path) for path in args.csv_root],
                      csv_ros_interval_s=[interval_start, interval_end],
                      csv_wall_interval_s=[wall_start, wall_end],
                      csv_attribution=('ROS and wall time filtered' if wall_start is not None and wall_end is not None
                                       else 'ABSTAIN: wall interval unavailable; ROS-only filtering cannot separate clock resets')),
        published_view_and_report_continuity={key:frame_series(value) for key,value in sorted(stages.items())},
        raw_image_mot_id_continuity={key:frame_series(value) for key,value in sorted(raw_mot.items())},
        actor_info_reports=report_summary, official_topics_last_observed=final_topics, development_judge=judge,
        report_coordinate_accuracy='ABSTAIN: Gazebo APIs have not been verified against the visible actor mesh',
        unverified_gazebo_diagnostic=gazebo_diagnostic,
        bridge_state_emissions={key:dict(value) for key,value in sorted(bridge_state_counts.items())},
        bridge_state_limitations='2.5s is the default sample freshness window; emitted states do not prove '
                                'an agent had authority, accepted geometry or moved. Repeated stamps count once.',
        original_sample_receive_age_s={key:stats(value) for key,value in sorted(latencies.items())},
        cpu_inference_s=stats(inference), isolated_tracker_cpu_cost='ABSTAIN: existing inference CSV is pipeline timing',
        baseline_effect='ABSTAIN: replay the SAME recorded burst with both trackers before comparing',
        truth_limitations='Gazebo API values are observer-only unverified diagnostics, not independent actor pose truth. '
                          'Animated actor mesh correspondence and headerless timing are unverified; APIs remain separate. '
                          'Overall report coordinate accuracy is ABSTAIN even when source brackets exist.',
        sparse_image_limitations='Sparse frames are diagnostics. Only image_kind=burst is a replay candidate; '
                                 'JPEG compression and capture gaps must be reported. No synthetic frames.',
        collision_verdict='ABSTAIN: no continuous contact evidence',
        flight_clearance_verdict='ABSTAIN: sampled poses do not prove continuous clearance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--out', type=Path, help='JSON destination; default recording/analysis.json')
    parser.add_argument('--csv-root', type=Path, action='append', default=[])
    parser.add_argument('--judge-log', type=Path)
    parser.add_argument('--truth-max-gap', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.truth_max_gap) or args.truth_max_gap <= 0.:
        parser.error('--truth-max-gap must be finite and positive')
    result = summarize(args)
    destination = args.out or args.recording / 'analysis.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps(dict(analysis=str(destination), verdict=result['overall_verdict'],
                         images=result['evidence']['images'], cpu_inference=result['cpu_inference_s'],
                         judge=result['development_judge'],reports=result['actor_info_reports']),
                     ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
