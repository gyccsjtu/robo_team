#!/usr/bin/env python3
"""Offline counterfactual at saved actual report times, without oracle gating.

Association comes from the saved bridge's selected visual_source events.
Truth is used only AFTER selecting a sensor estimate, to score both outputs.
This cannot replay scheduling or candidates missing from the old event log.
"""
import argparse
import bisect
import collections
import csv
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       'coordination/src/robocup_swarm/scripts'))
from visual_triangulation import camera_bearing, MultiViewTrack
from yolo_target_bridge import _robust_official_fit, TID_TO_TAG


def replay(audit, views):
    images = {}
    for path in sorted(views.glob('tracking_view_*.csv')):
        source = path.stem[len('tracking_view_'):]
        for r in csv.DictReader(path.open()):
            if r['report_ok'] != 'True' or r['range_method'] != 'foot_ground':
                continue
            camera = [float(r[k]) for k in ('camera_x','camera_y','camera_z','qx','qy','qz','qw')]
            box = [float(r[k]) for k in ('x1','y1','x2','y2')]
            bearing = camera_bearing(camera, box)
            if bearing:
                images[(source, float(r['sample_s']), r['target_id'], r['track_id'])] = bearing
    events = [json.loads(line) for line in audit.open()]
    truth = collections.defaultdict(list)
    for d in events:
        if d['event'] == 'truth' and d['success']:
            truth[d['actor']].append((.5*(d['ros']+d['request_ros']), d['x'], d['y']))
    times = {k:[p[0] for p in v] for k,v in truth.items()}

    def position(actor, t):
        i = bisect.bisect_left(times.get(actor, []), t)
        if i == 0 or i == len(times[actor]):
            return None
        a,b = truth[actor][i-1:i+1]
        if b[0]-a[0] > .6:
            return None
        f = (t-a[0])/(b[0]-a[0])
        return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]))

    tracks = collections.defaultdict(MultiViewTrack)
    errors = collections.defaultdict(list)
    counters = collections.Counter()
    selected = []
    for d in sorted(events, key=lambda d:d['wall']):
        if d['event'] == 'visual_source':
            r = d['data']; tag = TID_TO_TAG[r['target_id']]
            if tag.startswith('red'):
                continue
            if r.get('hypothesis_switch'):
                switch = r['hypothesis_switch']['switch_s']
                key = (tag, switch)
                if key not in counters:
                    counters[key] = 1; tracks[tag] = MultiViewTrack()
            key = (r['source_uav'],r['sample_s'],r.get('local_target_id',r['target_id']),
                   r['candidate_id'].rsplit(':',1)[-1])
            ray = images.get(key)
            if ray:
                counters['matched_views'] += 1
                # CSV report_ok is image qualification. The old bridge's
                # official_view_ok is a separate source-bias acceptance gate.
                tracks[tag].add(dict(r,tag=tag,bearing=ray,official_ok=True))
        elif d['event'] == 'report' and not d['tag'].startswith('red'):
            tag = d['tag']; old = (d['x'],d['y'])
            result = tracks[tag].point_at(d['ros'], _robust_official_fit, old)
            if result is not None:
                counters['corrected_reports'] += 1
            actor = {'green':0,'blue':1,'brown':2,'white':3}[tag]
            actual = position(actor,d['ros'])
            if actual is None:
                counters['no_truth_bracket'] += 1
                continue
            new = result or old
            old_err, new_err = (math.hypot(p[0]-actual[0],p[1]-actual[1]) for p in (old,new))
            errors[tag].append((old_err,new_err,result is not None))
            if result is not None:
                selected.append(dict(tag=tag,ros=d['ros'],old_error=old_err,new_error=new_err))
    def metrics(rows):
        return dict(reports=len(rows),changed=sum(r[2] for r in rows),
                    old_bad=sum(r[0]>=1. for r in rows),new_bad=sum(r[1]>=1. for r in rows),
                    newly_bad=sum(r[0]<1.<=r[1] for r in rows),
                    fixed=sum(r[1]<1.<=r[0] for r in rows),
                    old_median=statistics.median(r[0] for r in rows),
                    new_median=statistics.median(r[1] for r in rows))
    return dict(counts={k:v for k,v in counters.items() if isinstance(k,str)},
                by_tag={tag:metrics(rows) for tag,rows in errors.items()},
                changed_reports=selected,
                limitation='Conditional on saved source association and original publication times; '
                           'does not simulate new flight paths or missing old candidate events.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-jsonl',type=Path,required=True)
    p.add_argument('--views',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    result = replay(args.audit_jsonl,args.views)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='changed_reports'},indent=2))
