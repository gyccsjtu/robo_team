import sys, json, time, collections, statistics
from pathlib import Path
import numpy as np
import cv2
import torch
sys.path.insert(0, '/home/gycc/桌面/RoboCup_Team/tools/training')
import evaluate_distractor as ev
from ultralytics import YOLO

sys.path.insert(0, '/tmp/robocup_appearance_verifier')
root=Path('/home/gycc/桌面/RoboCup_Team')
data=Path('/home/gycc/robocup_distractor_data')
rows,_=ev.training.audit_manifest(data/'dataset_v3_preserved/reviewed_manifest.jsonl')
split='test' if '--reference-test' in sys.argv else 'val'
rows=[dict(image=json.loads(l)['image'],label=None,group='flight_57f3690_unreviewed') for l in open('/home/gycc/robocup_distractor_data/flight_57f3690/frames.jsonl')]
split='flight'
base=YOLO(str(root/'weights/best_yolo11n_bino_v1.pt'))
verify=YOLO(str(data/'runs/yolo11n_bino_distractor_v3/weights/last.pt'))
summary=collections.defaultdict(lambda: {'base_matched':collections.Counter(), 'new_matched':collections.Counter(), 'base_usable':collections.Counter(), 'new_usable':collections.Counter(), 'total':collections.Counter(), 'base_false':0,'new_false':0,'removed':[]})
timings=[]
for index,row in enumerate(rows):
    im=cv2.imread(row['image']);h,w=im.shape[:2]
    labels=[(c,[(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]) for c,x,y,bw,bh in (ev.training.read_labels(row['label']) if row['label'] else [])]
    pred=[(int(b.cls),b.xyxy[0].tolist(),float(b.conf)) for b in base.predict(im,imgsz=640,conf=.3,device=0,verbose=False)[0].boxes]
    retained=[]; removed=[]
    verified=[(int(b.cls),b.xyxy[0].tolist(),float(b.conf)) for b in verify.predict(im,imgsz=640,conf=.3,device=0,verbose=False)[0].boxes]
    from types import SimpleNamespace as NS
    from perception.appearance_verifier import AppearanceVerifier
    primary=[NS(cls=c,xyxy=[b],conf=f,appearance_status='actor') for c,b,f in pred]
    secondary=[NS(cls=c,xyxy=[b],conf=f) for c,b,f in verified]
    kept,evidence=AppearanceVerifier().apply(primary,secondary,verify.names,1000.)
    all_kept=[(b.cls,b.xyxy[0],b.conf) for b in kept]
    survivors=ev.runtime_color_survivors(im,all_kept)
    retained=[p for i,(b,p) in enumerate(zip(kept,all_kept)) if i in survivors and b.appearance_status=='actor']
    removed=evidence
    def score(raw):
        survivors=ev.runtime_color_survivors(im,raw); filtered=[]; usable=set()
        for i,(c,b,f) in enumerate(raw):
            if f>=(.3 if c==3 else .4):
                if i in survivors:usable.add(len(filtered))
                filtered.append((c,b,f))
        return ev.score_image(labels,filtered,usable)
    a,b=score(pred),score(retained)
    key=row['group']; s=summary[key]
    for prefix,result in [('base',a),('new',b)]:
        s[prefix+'_matched'].update(result['matched']); s[prefix+'_usable'].update(result['usable'])
        s[prefix+'_false']+=bool(result['false_decoy'])
    s['total'].update(a['total'])
    if removed:s['removed'].append(dict(image=row['image'],boxes=removed))
    if index%40==0:print('image',index,'/',len(rows),flush=True)
result=dict(policy=dict(checkpoint='v3_last',imgsz=640,ambiguous_confidence=.50,strong_confidence=.75,iou=.75,model_class_margin=.2,unique_instance=True),groups=summary,crop_latency_median_s=None,deployed=False)
Path('/tmp/robocup_actual_verifier_probe_'+split+'.json').write_text(json.dumps(result,indent=2)+'\n')
for group,s in summary.items():print(group,{k:v for k,v in s.items() if k!='removed'},'removed',len(s['removed']),flush=True)
