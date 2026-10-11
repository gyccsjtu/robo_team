"""Read recorded team frames only. No runtime publishers or threshold tuning."""
from pathlib import Path
import json,torch,cv2
from ultralytics import YOLO
root=Path(__file__).resolve().parents[2]
torch.set_num_threads(1)
weights=[root/'artifacts/yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt',Path('/home/gycc/robocup_distractor_data/runs/yolo11n_bino_distractor_v3/weights/best.pt')]
models=[YOLO(str(w)) for w in weights]
frames=[root/'artifacts/tracking_phase2_20261012/camera_views/u2_2009.984.jpg']
rows=[]
for path in frames:
 row={'image':str(path),'annotation':'Visually reviewed sculpture; one frame cannot establish temporal rejection.','predictions':[]}
 for model,weight in zip(models,weights):
  result=model.predict(cv2.imread(str(path)),imgsz=640,conf=.30,device='cpu',verbose=False)[0]
  row['predictions'].append({'weight':str(weight),'boxes':[{'label':model.names[int(b.cls[0])],'confidence':float(b.conf[0]),'box':b.xyxy[0].tolist()} for b in result.boxes]})
 rows.append(row)
Path(__file__).with_suffix('.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows,indent=2))
