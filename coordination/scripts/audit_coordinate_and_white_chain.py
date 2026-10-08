"""Read-only archival chain audit; truth is a label and never control input."""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys


def load(path):
    return [json.loads(line) for line in path.open() if line.strip()]


def truth_at(t, rows, times):
    i = bisect.bisect_left(times, t)
    if i < len(rows) and times[i] == t:
        return rows[i][1:3]
    if not 0 < i < len(rows): return None
    a,b = rows[i-1],rows[i]
    if not 0 < b[0]-a[0] <= .2: return None
    alpha = (t-a[0])/(b[0]-a[0])
    return tuple(a[j]+alpha*(b[j]-a[j]) for j in (1,2))


def coordinate_stages(root, trace, csv_paths):
    """Compare positions at their own times; do not sum scalar errors."""
    track_rows = {}
    for path in csv_paths:
        uid = 'uav_'+str(int(path.stem.rsplit('_',1)[1])+1)
        for row in csv.DictReader(path.open()):
            if row.get('event') == 'track_publish' and row.get('original_s'):
                track_rows[(uid,row['class_name'],float(row['original_s']))] = row
    output = {}
    for color in ('green','blue','brown','white'):
        path = root/'observers'/('independent_visual_accuracy_'+color+'.jsonl')
        if color == 'green' and not path.exists():
            path = root/'observers/independent_visual_accuracy.jsonl'
        if not path.exists(): continue
        truth = sorted((r['sample'] for r in load(path) if r.get('kind')=='actor_truth'),key=lambda r:r[0])
        times = [r[0] for r in truth]
        rows=[]; uploads=[]
        for item in trace:
            if item['kind']=='input' and item['observation']['target_id']==color:
                obs=item['observation']; point=truth_at(obs['sample_s'],truth,times)
                tr=track_rows.get((obs['uav_id'],color,obs['sample_s']))
                if point is None: continue
                raw=(float(tr['raw_target_x']),float(tr['raw_target_y'])) if tr else None
                rows.append(dict(uid=obs['uav_id'],image_s=obs['sample_s'],
                    raw_error_at_image_m=math.dist(raw,point) if raw else None,
                    input_error_at_image_m=math.dist(obs['xyz'][:2],point),
                    input_minus_raw_m=math.dist(obs['xyz'][:2],raw) if raw else None,
                    age_at_receipt_s=item['receipt_s']-obs['sample_s'],accepted=item['accepted']))
            elif item['kind']=='upload' and item.get('tag')==color:
                point=truth_at(item['receipt_s'],truth,times)
                if point is not None:
                    original=truth_at(item.get('original_s',0),truth,times)
                    uploads.append(dict(receipt_s=item['receipt_s'],
                        error_m=math.dist(item['requested_xy'],point),xy=item['requested_xy'],
                        truth_at_receipt_xy=point,original_s=item.get('original_s'),
                        position_s=item.get('position_s'), fused_xy=item.get('fused_xy'),
                        velocity=item.get('velocity'),window=item.get('window'),
                        target_movement_since_original_xy=[point[i]-original[i] for i in (0,1)]
                            if original is not None else None))
        output[color]=dict(inputs=rows,uploads=uploads,
            summary=dict(input_rows=len(rows),raw_matched=sum(r['raw_error_at_image_m'] is not None for r in rows),
                input_over_1m=sum(r['input_error_at_image_m']>=1 for r in rows),
                uploads=len(uploads),uploads_over_1m=sum(r['error_m']>=1 for r in uploads)))
    return output


def audit(root):
    truth_path = root/'observers/independent_visual_accuracy_white.jsonl'
    truth = sorted((r['sample'] for r in load(truth_path) if r.get('kind')=='actor_truth'), key=lambda x:x[0])
    times = [r[0] for r in truth]
    trajectory_path = root/'flight/city_trajectory.jsonl'
    distances = []
    for row in load(trajectory_path):
        point = truth_at(row['sample_s'],truth,times)
        if point is None: continue
        nearest = min((math.dist(p[:2],point),uid) for uid,p in row['positions'].items())
        distances.append(dict(sample_s=row['sample_s'],distance_m=nearest[0],uid=nearest[1]))
    trace_path = root/'flight/algorithm/bridge_trace.jsonl'
    trace = load(trace_path)
    counts = Counter((r['kind'],r.get('observation',{}).get('target_id',r.get('tag')))
                     for r in trace)
    white = dict(nearest=min(distances,key=lambda r:r['distance_m']) if distances else None,
        aligned_trajectory_samples=len(distances),
        samples_with_any_uav_within_22m=sum(r['distance_m']<=22 for r in distances),
        samples_with_any_uav_within_12m=sum(r['distance_m']<=12 for r in distances),
        bridge_kinds={str(k):v for k,v in counts.items() if k[1]=='white'})
    rejects = Counter(); originals = {}; csv_paths=[]; raw_rows=[]; gates=[]
    for path in (root/'flight/algorithm').glob('perception_*.csv'):
        csv_paths.append(path)
        uid = 'uav_'+str(int(path.stem.rsplit('_',1)[1])+1)
        for row in csv.DictReader(path.open()):
            if row.get('class_name') != 'white': continue
            rejects[row.get('event','')+':'+row.get('source','')] += 1
            if row.get('event') == 'yolo_detection' and row.get('image_stamp'):
                point = truth_at(float(row['image_stamp']),truth,times)
                raw_rows.append(dict(uid=uid,image_s=float(row['image_stamp']),
                    raw_xy=[float(row['target_x']),float(row['target_y'])],
                    raw_error_m=math.dist((float(row['target_x']),float(row['target_y'])),point) if point else None,
                    distance_m=float(row['range_m']), confidence=float(row['confidence']),
                    person_frame_verified=row.get('person_frame_verified'),
                    person_iou=row.get('person_iou'),person_conf=row.get('person_conf'),
                    reject_reason=row.get('reject_reason')))
            if row.get('original_s'):
                key = (uid,row['original_s'])
                originals.setdefault(key,dict(uid=uid,original_s=row['original_s'],events=set()))['events'].add(row.get('event'))
                if row.get('event') == 'track_reject' and row.get('track_miss') == '0':
                    gates.append({k: row.get(k) for k in ('ros_time','original_s','track_id',
                        'hits','person_hits','fresh_person_allowed','person_current_verified',
                        'source','range_m','raw_target_x','raw_target_y','target_x','target_y')})
                    gates[-1]['uid'] = uid
    inputs = [r for r in trace if r.get('kind')=='input' and r['observation']['target_id']=='white']
    input_summary = []
    for row in inputs:
        obs = row['observation']; cam = obs['camera_xyz']
        input_summary.append(dict(receipt_s=row['receipt_s'],image_s=obs['sample_s'],
            uid=obs['uav_id'],age_s=row['receipt_s']-obs['sample_s'],
            horizontal_distance_m=math.dist(obs['xyz'][:2],cam[:2]),
            accepted=row['accepted'],alive=row['alive'], readiness=row.get('report_readiness')))
    white.update(raw_detections=raw_rows,
                 bridge_white_inputs=inputs, bridge_input_summary=input_summary,
                 fresh_rejection_rows=gates,
                 csv_event_rows=dict(rejects), distinct_track_original_keys=len(originals),
                 note='CSV rows are not distinct images; nearest distance does not prove in-FOV or visibility')
    # Exercise current pure policy classes with archival inputs. This is not a
    # replay of ROS scheduling, frame association, route control or physics.
    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(repo/'perception'))
    sys.path.insert(0,str(repo/'coordination/src/robocup_swarm/scripts'))
    from fresh_person import FreshPerson
    from report_readiness import ReportReadiness
    proof=FreshPerson(); proof_counts=[]
    for stamp in (1969.844,1970.496,1971.836):
        proof.observe(stamp,True); proof_counts.append(proof.hits)
    readiness=ReportReadiness()
    decisions=[readiness.observe(r['observation'],r['receipt_s'],r['accepted']) for r in inputs]
    assert proof_counts == [1,2,1]
    white['production_policy_replay']=dict(person_hits=proof_counts,
        readiness_decisions=decisions,scope='pure policy only; not complete callback or physics replay')
    files=[trajectory_path,trace_path]+sorted((root/'observers').glob('independent_visual_accuracy*.jsonl'))+csv_paths
    return dict(control_input=False,scope='closed round read-only labels, no entity simulation',white=white,
        coordinate_stages=coordinate_stages(root,trace,csv_paths),
        inputs=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=audit(args.run)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result['white'][k] for k in ('nearest',
        'aligned_trajectory_samples','bridge_kinds','production_policy_replay')},indent=2))
