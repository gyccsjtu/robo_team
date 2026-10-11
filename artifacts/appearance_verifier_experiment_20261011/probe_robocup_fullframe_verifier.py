import sys, json, time, collections, statistics
from pathlib import Path
import numpy as np
import cv2
import torch
sys.path.insert(0, '/home/gycc/桌面/RoboCup_Team/tools/training')
import evaluate_distractor as ev
from ultralytics import YOLO

root=Path('/home/gycc/桌面/RoboCup_Team')
data=Path('/home/gycc/robocup_distractor_data')
rows,_=ev.training.audit_manifest(data/'dataset_v3_preserved/reviewed_manifest.jsonl')
split='test' if '--reference-test' in sys.argv else 'val'
rows=[r for r in rows if r['split']==split]
base=YOLO(str(root/'weights/best_yolo11n_bino_v1.pt'))
verify=YOLO(str(data/'runs/yolo11n_bino_distractor_v3/weights/last.pt'))
summary=collections.defaultdict(lambda: {'base_matched':collections.Counter(), 'new_matched':collections.Counter(), 'base_usable':collections.Counter(), 'new_usable':collections.Counter(), 'total':collections.Counter(), 'base_false':0,'new_false':0,'removed':[]})
timings=[]
for index,row in enumerate(rows):
    im=cv2.imread(row['image']);h,w=im.shape[:2]
    labels=[(c,[(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]) for c,x,y,bw,bh in ev.training.read_labels(row['label'])]
    pred=[(int(b.cls),b.xyxy[0].tolist(),float(b.conf)) for b in base.predict(im,imgsz=640,conf=.3,device=0,verbose=False)[0].boxes]
    retained=[]; removed=[]
    verified=[(int(b.cls),b.xyxy[0].tolist(),float(b.conf)) for b in verify.predict(im,imgsz=640,conf=.3,device=0,verbose=False)[0].boxes]
    def same(a,b):
        aw,ah=a[2]-a[0],a[3]-a[1];bw,bh=b[2]-b[0],b[3]-b[1]
        return (ev.iou(a,b)>=.75 and .65<=aw/bw<=1.5 and .65<=ah/bh<=1.5 and
                abs(a[0]+a[2]-b[0]-b[2])<=.3*min(aw,bw) and
                abs(a[1]+a[3]-b[1]-b[3])<=.3*min(ah,bh))
    for cls,box,conf in pred:
        decoys=[x for x in verified if x[0]==5 and x[2]>=.75 and same(box,x[1])]
        actors=[x for x in verified if x[0]<5 and same(box,x[1])]
        reject=(len(decoys)==1 and sum(same(b,decoys[0][1]) for c,b,f in pred)==1 and
                decoys[0][2]>=max((x[2] for x in actors),default=0)+.2)
        if reject:removed.append(dict(cls=cls,box=box,conf=conf,verifier=decoys))
        else:retained.append((cls,box,conf))
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
result=dict(policy=dict(checkpoint='v3_last',imgsz=640,confidence=.75,iou=.75,model_class_margin=.2,unique_instance=True),groups=summary,crop_latency_median_s=None,deployed=False)
Path('/tmp/robocup_fullframe_verifier_probe_'+split+'.json').write_text(json.dumps(result,indent=2)+'\n')
for group,s in summary.items():print(group,{k:v for k,v in s.items() if k!='removed'},'removed',len(s['removed']),flush=True)
