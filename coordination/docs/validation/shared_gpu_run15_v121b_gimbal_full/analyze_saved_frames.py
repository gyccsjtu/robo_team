import os,json,pathlib,sys,cv2,numpy as np,time
os.environ['OMP_NUM_THREADS']='1';os.environ['OPENBLAS_NUM_THREADS']='1'
from ultralytics import YOLO
repo=pathlib.Path('/mnt/d/a/.robocup/robo_team');sys.path.insert(0,str(repo/'perception'))
from person_verifier import overlap
root=pathlib.Path('/root/robocup_runs/codex_city_teammate_v121b_20261004/flight/visual_frames')
color=YOLO(repo/'weights/best_yolo11n_bino_v1.pt');person=YOLO(repo/'weights/yolo11n_person.pt')
out=root.parent/'saved_frame_color_audit.jsonl'
with out.open('w') as f:
 for line in (root/'frames.jsonl').read_text().splitlines():
  row=json.loads(line);im=cv2.imread(str(root/row['image']));boxes=color(im,conf=.4,device=0,verbose=False)[0].boxes
  people=person(im,classes=[0],conf=.1,device=0,verbose=False)[0].boxes
  results=[]
  for b in boxes:
   xy=[float(x) for x in b.xyxy[0]];x1,y1,x2,y2=xy;w=x2-x1;h=y2-y1
   xa,xb=max(0,int(x1+.25*w)),min(im.shape[1],int(x2-.25*w));ya,yb=max(0,int(y1+.20*h)),min(im.shape[0],int(y1+.60*h))
   crop=im[ya:yb,xa:xb];stats={}
   if crop.size:
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV);H,S,V=cv2.split(hsv)
    masks={'red':((H<12)|(H>168))&(S>80)&(V>45),'green':(H>=35)&(H<=90)&(S>65)&(V>40),'blue':(H>90)&(H<140)&(S>65)&(V>40),'white':(S<65)&(V>160),'brown':(H>=8)&(H<35)&(S>65)&(V>35)&(V<190)}
    stats={k:round(float(m.mean()),4) for k,m in masks.items()}
   results.append({'class':color.names[int(b.cls)],'confidence':float(b.conf),'xyxy':xy,'person_iou':float(max((overlap(xy,p.xyxy[0]) for p in people),default=0.)),'torso_color_fraction':stats})
  f.write(json.dumps({'image':row['image'],'observation':row['observation'],'boxes':results})+'\n')
print('saved',out)
