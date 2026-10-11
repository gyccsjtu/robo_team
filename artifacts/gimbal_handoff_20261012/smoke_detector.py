"""Run the auxiliary image path on saved native CGO3 images, without ROS."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np,torch
from ultralytics import YOLO
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'coordination/src/robocup_swarm/scripts'))
from gimbal_attention import image_bearing
torch.set_num_threads(1);cv2.setNumThreads(1)
model=YOLO(str(root/'artifacts/yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt'))
folder=root/'artifacts/gimbal_recognition_20261011/frames'
files=sorted(folder.glob('*actors_12m_aim_gimbal.png'))[:12]
k=(640,360,[205.469637,0,320.5,0,205.469637,180.5,0,0,1],[0.]*5)
rows=[]
for path in files:
    im=cv2.imread(str(path));assert im.shape[:2]==(360,640)
    started=time.monotonic()
    out=model.predict(im,imgsz=960,conf=.45,device='cpu',verbose=False)[0]
    elapsed=time.monotonic()-started;usable=[]
    for box in out.boxes:
        color=out.names[int(box.cls.item())]
        if color not in {'red','green','white','brown','blue'}:continue
        try:bearing,elevation=image_bearing(box.xyxy.cpu().numpy()[0].tolist(),k,np.eye(3))
        except ValueError:continue
        usable.append(dict(color=color,confidence=float(box.conf.item()),bearing=bearing))
    rows.append(dict(file=path.name,inference_wall_s=elapsed,usable_boxes=usable))
assert len(rows)==12 and all(r['usable_boxes'] for r in rows)
result=dict(scope='Saved images only; bearings here use identity rotation for input validation, not world coordinates; no recall or match claim',
            names=model.names,median_hot_inference_s=float(np.median([r['inference_wall_s'] for r in rows[1:]])),frames=rows)
(Path(__file__).parent/'detector_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='frames'}))
