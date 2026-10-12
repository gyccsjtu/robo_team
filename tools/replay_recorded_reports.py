#!/usr/bin/env python3
"""Offline replay of recorded camera messages through the real bridge.

No ROS connection, actor truth, or flight commands. Recorder receipt times
approximate bridge receipt times; this is not a deterministic judge replay.
"""
import argparse
import gzip
import json
import os
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record')
    parser.add_argument('output')
    parser.add_argument('--tags', nargs='+', default=['white', 'brown'])
    args = parser.parse_args()
    # Match the recorded run_sculpture.sh / run_match.sh configuration.
    for key, value in {'BRIDGE_REPORT_POLICY': 'fresh', 'BRIDGE_DROP_TIME': '4.0',
                       'BRIDGE_NEW_TRACK_CONF': '.55', 'BRIDGE_NEW_TRACK_FRAMES': '2',
                       'BRIDGE_EXTRAP_MAX_D': '.5'}.items():
        os.environ.setdefault(key, value)
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root/'coordination/src/robocup_swarm/scripts'))
    import yolo_target_bridge as bridge
    bridge._core_log = lambda *a: None
    node = bridge._mk_bridge()
    frames = []
    with gzip.open(args.record, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['event'] != 'image_report':
                continue
            frame = row['data'].copy()
            frame['candidates'] = [c for c in frame['candidates']
                                   if c['target_id'] in args.tags]
            if frame['candidates']:
                frames.append((row['ros'], frame))
    if not frames:
        raise SystemExit('no matching camera frames')
    events = [(now, 0, frame) for now, frame in frames]
    start, end = min(t for t, _ in frames), max(t for t, _ in frames)
    events += [(start+i/15., 1, None) for i in range(int((end-start+1.)*15.))]
    observations, published = [], []
    for tag in args.tags:
        def record(message, tag=tag):
            tr = node.core.tracks[tag]
            published.append(dict(tag=tag, time=node._now(), x=message.x,
                                  y=message.y, sample=tr.good_t,
                                  source=tr.official_source,
                                  candidate=tr.official_candidate_id))
        node._actor_pubs[tag].publish = record
    for now, kind, frame in sorted(events, key=lambda e: (e[0], e[1])):
        node._advance(now-node._now())
        if kind:
            node._tick(None)
            continue
        node._report_cb(bridge._FakeString(json.dumps(frame)))
        for candidate in frame['candidates']:
            tr = node.core.tracks[candidate['target_id']]
            key = node.core._source_key(frame['source_uav'], candidate['candidate_id'])
            observations.append(dict(time=now, sample=frame['sample_s'],
                source=frame['source_uav'], candidate=candidate['candidate_id'],
                tag=candidate['target_id'], input_ok=candidate['official_view_ok'],
                xy=candidate['xyz'][:2], range=candidate['range_m'],
                accepted=tr.accepted_source_samples.get(key) == frame['sample_s'],
                selected=tr.official_source, selected_candidate=tr.official_candidate_id,
                selected_sample=tr.good_t, selected_range=tr.range_m,
                gate=node._gate_live.get(tr.tag, False),
                point=bridge.official_point_at(tr, now),
                history=len(tr.source_history.get(key, []))))
    summary = {}
    for tag in args.tags:
        points = [p for p in published if p['tag'] == tag]
        gaps = sorted([(b['time']-a['time'], a['time'], b['time'])
                       for a, b in zip(points, points[1:])], reverse=True)
        summary[tag] = dict(count=len(points), longest_gaps=gaps[:10])
    Path(args.output).write_text(json.dumps(dict(summary=summary,
        published=published, observations=observations), indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
