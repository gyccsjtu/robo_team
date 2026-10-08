"""Offline replay of recorded bridge inputs/uploads. Truth labels only, no ROS."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/robocup_swarm/scripts'))
from yolo_target_bridge import TargetBridgeCore


def error_at(stamp, xy, truth):
    times = [r[0] for r in truth]
    i = bisect.bisect_left(times, stamp)
    if not 0 < i < len(truth):
        return None
    a, b = truth[i-1], truth[i]
    if not 0 < b[0]-a[0] <= .2:
        return None
    f = (stamp-a[0])/(b[0]-a[0])
    return math.dist(xy, [a[j]+f*(b[j]-a[j]) for j in (1, 2)])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True)
    p.add_argument('--tag', default='blue')
    p.add_argument('--output', required=True)
    args = p.parse_args()
    root = Path(args.run)
    truth = [json.loads(l)['sample'] for l in
             (root/'observers'/('independent_visual_accuracy_'+args.tag+'.jsonl')).open()
             if json.loads(l).get('kind') == 'actor_truth']
    raw = {}
    for file in (root/'flight/algorithm').glob('perception_*.csv'):
        uid = 'uav_'+str(int(file.stem.rsplit('_', 1)[-1])+1)
        for row in csv.DictReader(file.open()):
            if row.get('event') == 'track_publish' and row.get('class_name') == args.tag:
                raw[(uid, float(row['original_s']))] = [float(row['raw_target_x']), float(row['raw_target_y'])]
    cores = dict(baseline=TargetBridgeCore(), raw=TargetBridgeCore(),
                 raw_aligned=TargetBridgeCore(blue_alignment=True))
    rows, missing_raw = [], 0
    for line in (root/'flight/algorithm/bridge_trace.jsonl').open():
        d = json.loads(line)
        if d['kind'] == 'input':
            obs = d['observation']
            if obs['target_id'] != args.tag:
                continue
            coords = raw.get((obs['uav_id'], obs['sample_s']))
            missing_raw += coords is None
            for name, core in cores.items():
                xy = coords if name != 'baseline' and coords is not None else obs['xyz'][:2]
                tag = args.tag
                core.report(obs['sample_s'], tag, *xy, obs['confidence'],
                            obs['uav_id'], obs['observation_id'])
        elif d['kind'] == 'upload' and d['tag'] == args.tag:
            values = {}
            for name, core in cores.items():
                tag = args.tag
                event = next((x for x in core.tick(d['prediction_s']) if x['tag'] == tag), None)
                if event:
                    xy = [round(event['x'], 3), round(event['y'], 3)]
                    values[name] = dict(xy=xy, error=error_at(d['receipt_s'], xy, truth))
            rows.append(dict(receipt_s=d['receipt_s'], actual=d['requested_xy'],
                actual_error=error_at(d['receipt_s'], d['requested_xy'], truth), variants=values))
    summary = dict(rows=len(rows), missing_raw=missing_raw,
        baseline_max_delta=max((math.dist(x['actual'], x['variants']['baseline']['xy'])
                               for x in rows if 'baseline' in x['variants']), default=None))
    for name in ('actual', 'baseline', 'raw', 'raw_aligned'):
        es = [x['actual_error'] if name == 'actual' else x['variants'].get(name, {}).get('error') for x in rows]
        es = [v for v in es if v is not None]
        summary[name] = dict(aligned=len(es), under_1m=sum(v<1 for v in es), maximum=max(es, default=None))
    Path(args.output).write_text(json.dumps(dict(control_input=False,
        meaning='recorded input/upload-time replay, not physical or complete ROS callback replay',
        summary=summary, rows=rows), indent=2, allow_nan=False))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
