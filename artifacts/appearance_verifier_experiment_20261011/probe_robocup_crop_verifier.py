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
    for cls,box,conf in pred:
        x1,y1,x2,y2=box;bw,bh=x2-x1,y2-y1
        left,top=max(0,int(x1-.15*bw)),max(0,int(y1-.15*bh))
        right,bottom=min(w,int(x2+.15*bw+1)),min(h,int(y2+.15*bh+1))
        crop=im[top:bottom,left:right]
        t=time.perf_counter()
        results=verify.predict(crop,imgsz=256,conf=.3,device=0,verbose=False)[0]
        timings.append(time.perf_counter()-t)
        hits=[]
        for b in results.boxes:
            bb=b.xyxy[0].tolist(); mapped=[bb[0]+left,bb[1]+top,bb[2]+left,bb[3]+top]
            hits.append((int(b.cls),mapped,float(b.conf),ev.iou(box,mapped)))
        decoys=[b for b in hits if b[0]==5 and b[2]>=.9 and b[3]>=.5]
        actors=[b for b in hits if b[0]<5]
        reject=(len(decoys)==1 and decoys[0][3]>=max((b[3] for b in actors),default=0)+.15)
        if reject:removed.append(dict(cls=cls,box=box,conf=conf,verifier=hits))
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
result=dict(policy=dict(checkpoint='v3_last',crop_padding=.15,imgsz=256,confidence=.9,iou=.5,overlap_margin=.15),groups=summary,crop_latency_median_s=statistics.median(timings),deployed=False)
Path('/tmp/robocup_crop_verifier_probe_'+split+'.json').write_text(json.dumps(result,indent=2)+'\n')
for group,s in summary.items():print(group,{k:v for k,v in s.items() if k!='removed'},'removed',len(s['removed']),flush=True)
