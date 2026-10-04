#!/usr/bin/env python3
"""Read-only per-track motion replay; annotations never enter flight control."""
import argparse
import collections
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'perception'))
from recent_motion import RecentMotion


def replay(algorithm, annotation=None, manifest=None):
    summary=collections.defaultdict(lambda:dict(samples=0,old_motion_pass=0,new_motion_pass=0))
    files={}
    for file in sorted(Path(algorithm).glob('perception*.csv')):
        data=file.read_bytes()
        if manifest is not None:
            expected=manifest['input_files'][file.name]
            data=data[:expected['bytes']]
            if hashlib.sha256(data).hexdigest()!=expected['sha256']:
                raise ValueError('REPLAY_INPUT_CHANGED: '+file.name)
        files[file.name]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))
        rows=list(csv.DictReader(io.StringIO(data.decode('utf-8'))))
        raw=collections.defaultdict(list)
        for row in rows:
            if row.get('event')=='yolo_detection' and row.get('class_name')=='blue':
                raw[float(row['image_stamp'])].append(row)
        seen=set();motions={}
        for row in rows:
            if (row.get('event') not in ('track_publish','track_reject')
                    or row.get('class_name')!='blue' or not row.get('track_id')):
                continue
            key=(row['track_id'],float(row['image_stamp']))
            if key in seen:continue
            seen.add(key)
            candidates=raw.get(key[1],[])
            if not candidates:continue
            position=[float(row['target_x']),float(row['target_y'])]
            measured=min(candidates,key=lambda q:math.dist(position,
                [float(q['target_x']),float(q['target_y'])]))
            xy=[float(measured['target_x']),float(measured['target_y'])]
            if math.dist(position,xy)>3.:continue
            group=('annotation_neighborhood' if annotation is not None
                   and math.dist(xy,annotation)<=5. else 'other_unlabelled')
            counter=summary[group];counter['samples']+=1
            for label,window,span in [('old',4.,1.),('new',10.,3.)]:
                motion=motions.setdefault((key[0],label),RecentMotion(window,span))
                motion.observe(key[1],*xy)
                counter[label+'_motion_pass']+=int(motion.speed(key[1])>=.15)
    return dict(control_input=False,formal_competition_pass=False,
        diagnostic_annotation_xy=annotation,summary=dict(summary),input_files=files,
        limitations='Raw-to-track nearest association within 3m; not a full detector or judge replay. '
                    'Other samples have no truth label. Coasting and duplicate image stamps excluded.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm',required=True)
    parser.add_argument('--annotation-xy',nargs=2,type=float)
    parser.add_argument('--output',required=True)
    parser.add_argument('--input-manifest',help='Reproduce a prior checkpoint using its byte prefix and SHA')
    args=parser.parse_args()
    manifest=json.loads(Path(args.input_manifest).read_text()) if args.input_manifest else None
    value=replay(args.algorithm,args.annotation_xy,manifest)
    with Path(args.output).open('x') as stream:
        json.dump(value,stream,indent=2)
    print(json.dumps(value['summary']))
