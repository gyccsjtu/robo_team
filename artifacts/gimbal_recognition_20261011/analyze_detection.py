"""Offline only. Controlled-row diagnostic, not a general dataset recall score.

Actor-only horizontal regions are manually checked on recorded native frames.
Remove decoy regions; require six separated boxes in the known left-to-right
color order. This measures complete-row detections within this short benchmark.
It does not measure world-position accuracy or competition elimination.
"""
import collections
import json
import statistics
import time
from pathlib import Path
import cv2
import torch
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parent
rows=[json.loads(line) for line in (ROOT/'detections.jsonl').read_text().splitlines()]
regions={16:(240,400),12:(218,432),8:(165,478),5:(75,567)}
order=['red','red','white','brown','blue','green']

def match_row(boxes, distance, threshold):
    left,right=regions[distance]
    items=sorted((b for b in boxes if b['conf']>=threshold and
                  left<(b['xyxy'][0]+b['xyxy'][2])/2<right),key=lambda b:b['xyxy'][0]+b['xyxy'][2])
    selected=[]
    # Ordered matching prevents a decoy or duplicate half-body from becoming
    # the second red instance. Center spacing is scaled to the known range.
    last=-1e9
    for item in items:
        center=(item['xyxy'][0]+item['xyxy'][2])/2
        if item['color']==order[len(selected)] and center-last>100./distance:
            selected.append(item)
            last=center
            if len(selected)==6:return selected
    return None

summary={}
for distance in (16,12,8,5):
    phase='actors_%dm_aim'%distance
    rr=[r for r in rows if r['camera']=='gimbal' and r['phase']==phase]
    summary[phase]=dict(frames=len(rr),actor_region_x=list(regions[distance]),
        complete_row_at_025=sum(match_row(r['boxes'],distance,.25) is not None for r in rr),
        complete_row_at_040=sum(match_row(r['boxes'],distance,.4) is not None for r in rr))
    chosen=rr[len(rr)//2]
    image=cv2.imread(str(ROOT/chosen['frame']))
    for box in chosen['boxes']:
        x1,y1,x2,y2=map(round,box['xyxy'])
        cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,255),1)
        cv2.putText(image,'%s %.2f'%(box['color'],box['conf']),(x1,max(12,y1-4)),
                    cv2.FONT_HERSHEY_SIMPLEX,.32,(0,255,255),1,cv2.LINE_AA)
    cv2.imwrite(str(ROOT/(phase+'_example.png')),image)

# Same saved 16 m photographs: test preprocessing, without changing optics,
# flight, sensor parameters or the trained weights. Central crop loses side FOV.
torch.set_num_threads(2)
model=YOLO(str(ROOT.parent/'yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt'))
replays=[]
for row in rows:
    if row['camera']!='gimbal' or row['phase']!='actors_16m_aim':continue
    image=cv2.imread(str(ROOT/row['frame']))
    for method,im,imgsz,xoffset in [('full_640',image,640,0),('full_960',image,960,0),('center_crop_640',image[:,140:500],640,140)]:
        start=time.monotonic()
        result=model.predict(im,imgsz=imgsz,conf=.25,device='cpu',verbose=False)[0]
        elapsed=time.monotonic()-start
        boxes=[]
        for b in result.boxes:
            xy=b.xyxy.cpu().numpy()[0].tolist()
            xy[0]+=xoffset;xy[2]+=xoffset
            boxes.append(dict(color=result.names[int(b.cls.item())],conf=float(b.conf.item()),xyxy=xy))
        replays.append(dict(method=method,frame=row['frame'],boxes=boxes,inference_wall_s=elapsed,
             complete_at_025=match_row(boxes,16,.25) is not None,
             complete_at_040=match_row(boxes,16,.4) is not None))
variants={}
for method in ('full_640','full_960','center_crop_640'):
    rr=[r for r in replays if r['method']==method]
    variants[method]=dict(frames=len(rr),complete_row_at_025=sum(r['complete_at_025'] for r in rr),
        complete_row_at_040=sum(r['complete_at_040'] for r in rr),
        median_warm_inference_ms=1000*statistics.median(r['inference_wall_s'] for r in rr[1:]))
result=dict(gimbal_centered_view=summary,offline_16m_same_frame_variants=variants,
    total_images=len(rows),gimbal_images=sum(r['camera']=='gimbal' for r in rows),
    camera_inference_ms={c:1000*statistics.median(r['inference_wall_s'] for r in rows[2:] if r['camera']==c)
                         for c in ('gimbal','stereo')},
    limitations=['Single controlled layout; short correlated samples; no statistical generalization',
                'Manual actor regions and ordered detection diagnostic are not full human bounding-box annotations',
                'No coordinate accuracy, moving pursuit, six-UAV throughput or elimination validation',
                'Cloud view contains own landing gear; cropping loses peripheral view'])
(ROOT/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
(ROOT/'offline_variants.json').write_text(json.dumps(replays,indent=2)+'\n')
print(json.dumps(result,indent=2))
