"""Offline comparison only; no ROS publication or production weight changes."""
import importlib.util,json,time
from pathlib import Path
import torch
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parents[2]; OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('ev',ROOT/'tools/training/evaluate_distractor.py')
ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
torch.set_num_threads(1)
rows,_=ev.training.audit_manifest('/home/gycc/robocup_distractor_data/dataset_v3_preserved/reviewed_manifest.jsonl')
groups={
 'legacy_validation340':[r for r in rows if r['split']=='val' and r['group']=='legacy_original_val' and Path(r['image']).parents[2].name=='yolo_ds'],
 'empty_negative34':[r for r in rows if r['split']=='val' and r['group']=='legacy_original_val' and Path(r['image']).parents[2].name!='yolo_ds'],
 'sculpture_layout_b8':[r for r in rows if r['split']=='val' and r['group']!='legacy_original_val'],
 'previously_evaluated_layout_d10':[r for r in rows if r['split']=='test']}
weights={'production_v1':ROOT/'weights/best_yolo11n_bino_v1.pt','remote_v3_clean':OUT/'weights/best_yolo11n_bino_v3_clean.pt'}
output={'scope':'Fixed existing datasets, not a fresh independent held-out test. Remote training membership not verified. No threshold selection. No deployment.','thresholds':{'white':.30,'other':.40,'imgsz':640},'groups':{},'native_frames':{}}
for name,rs in groups.items():
 print(name,len(rs),flush=True);output['groups'][name]={}
 for model,path in weights.items():
  report=ev.evaluate(path,rs,device=0,candidate=False)
  output['groups'][name][model]=report
  print(model,json.dumps({k:v for k,v in report.items() if k not in ('frames','weights','sha256')}),flush=True)
 (OUT/'comparison.json').write_text(json.dumps(output,indent=2)+'\n')
frames=[ROOT/'artifacts/run_audit_20261011_175916/frames/0011_typhoon_h480_2.png']
frames+=sorted((ROOT/'artifacts/run_audit_20261011_180725/frames').glob('*typhoon_h480_2.png'))[:12]
for name,path in weights.items():
 model=YOLO(str(path))
 for frame in frames:
  result=model.predict(str(frame),imgsz=640,conf=.30,device=0,verbose=False)[0]
  output['native_frames'].setdefault(str(frame),{})[name]=[dict(cls=model.names[int(b.cls[0])],confidence=float(b.conf[0]),xyxy=b.xyxy[0].tolist()) for b in result.boxes]
 print(name,'native frames complete',flush=True)
(OUT/'comparison.json').write_text(json.dumps(output,indent=2)+'\n')
