"""Closed short-round detection audit. Truth labels never feed control."""
import argparse,csv,json,math,hashlib
import sys
from pathlib import Path
from collections import Counter
from types import SimpleNamespace
from audit_coordinate_and_white_chain import truth_at,load
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'perception'))
from motion_identity import MotionIdentity
from white_discovery import blue_discovery


def audit(root):
    truth={};files=[]
    for tag in ('green','blue','brown','white','red'):
        name='independent_visual_accuracy'+('' if tag=='green' else '_'+tag)+'.jsonl'
        path=root/'observers'/name
        if not path.exists():continue
        files.append(path)
        for row in load(path):
            if row.get('kind')=='actor_truth':
                sample=row['sample'];truth.setdefault(int(sample[3]),[]).append(sample)
    for rows in truth.values():rows.sort(key=lambda r:r[0])
    times={key:[r[0] for r in rows] for key,rows in truth.items()}
    details=[];approach=[]
    for path in (root/'flight/algorithm').glob('perception_*.csv'):
        files.append(path);uid='uav_'+str(int(path.stem.rsplit('_',1)[1])+1)
        rows=list(csv.DictReader(path.open()))
        identities={};seen=set()
        for row in rows:
            if (row.get('event')!='track_reject' or row.get('class_name')!='blue'
                    or not row.get('original_s')):continue
            key=(row['track_id'],row['original_s'])
            if key in seen:continue
            seen.add(key)
            identity=identities.setdefault(row['track_id'],MotionIdentity())
            stamp=float(row['original_s']);xy=(float(row['raw_target_x']),float(row['raw_target_y']))
            identity.observe(stamp,*xy)
            track=SimpleNamespace(cls='blue',observed_s=stamp,raw_xy=xy,
                hits=int(row['hits']),miss=int(row['track_miss']),h=float(row['height_m']),
                rng=float(row['range_m']),score_ema=float(row['score_ema']),blue_identity=identity,
                person_support=SimpleNamespace(hits=int(row['person_hits']),
                    current_verified=row['person_current_verified']=='True'))
            if blue_discovery(track,float(row['ros_time']),1.3,2.4,float(row['verdict_score'])):
                point=truth_at(stamp,truth.get(1,[]),times.get(1,[]))
                approach.append(dict(uid=uid,image_s=stamp,track_id=row['track_id'],reason=row['source'],
                    raw_xy=xy,range_m=track.rng,blue_truth_error_m=math.dist(xy,point) if point else None))
        for row in rows:
            if row.get('event')!='yolo_detection' or row.get('class_name')!='blue':continue
            stamp=float(row['image_stamp']);xy=(float(row['target_x']),float(row['target_y']))
            labels={}
            for actor, samples in truth.items():
                point=truth_at(stamp,samples,times[actor])
                if point is not None:labels[actor]=math.dist(xy,point)
            gates=Counter()
            for reject in rows:
                if (reject.get('event')!='track_reject' or reject.get('class_name')!='blue'
                        or not reject.get('original_s')):continue
                if (abs(float(reject['original_s'])-stamp)<=.01
                        and reject.get('raw_target_x') and math.dist(xy,
                        (float(reject['raw_target_x']),float(reject['raw_target_y'])))<=.05):
                    gates[reject['source']]+=1
            details.append(dict(uid=uid,image_s=stamp,raw_xy=xy,range_m=float(row['range_m']),
                person_verified=row.get('person_frame_verified')=='True',
                person_iou=row.get('person_iou'),blue_truth_error_m=labels.get(1),
                nearest_actor=min(labels,key=labels.get) if labels else None,
                nearest_actor_error_m=min(labels.values()) if labels else None,
                matched_rejection_rows=dict(gates)))
    proved=[r for r in details if r['person_verified']]
    aligned=[r for r in proved if r['blue_truth_error_m'] is not None]
    return dict(control_input=False,scope='closed raw detections and rejection records, not an image identity certificate',
        summary=dict(blue_boxes=len(details),person_proved_boxes=len(proved),truth_aligned_proved_boxes=len(aligned),
            blue_originals=len({(r['uid'],r['image_s']) for r in details}),
            person_proved_originals=len({(r['uid'],r['image_s']) for r in proved}),
            proved_under_1m_originals=len({(r['uid'],r['image_s']) for r in aligned if r['blue_truth_error_m']<1}),
            proved_boxes_under_1m=sum(r['blue_truth_error_m']<1 for r in aligned),
            proved_boxes_under_3m=sum(r['blue_truth_error_m']<3 for r in aligned),
            proved_minimum_blue_error_m=min((r['blue_truth_error_m'] for r in aligned),default=None),
            proved_maximum_blue_error_m=max((r['blue_truth_error_m'] for r in aligned),default=None)),
        details=details,approach_replay=dict(candidates=approach,
            caveat='reconstructs tracked rejection originals only; missing pre-rejection originals, not complete association replay'),
        inputs=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=audit(a.run);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result['summary']))
