"""Offline native-frame pair probe; sparse frames cannot establish temporal acceptance."""
import importlib.util,json,time
from pathlib import Path
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('pair',ROOT/'tools/training/evaluate_detector_pair.py');p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
ev=p.load_evaluator();policy=json.loads((OUT/'policy.json').read_text());torch.set_num_threads(1)
frames=[ROOT/'artifacts/run_audit_20261011_175916/frames/0011_typhoon_h480_2.png']
frames+=sorted((ROOT/'artifacts/run_audit_20261011_180725/frames').glob('*typhoon_h480_2.png'))[:12]
weights=policy['primary_weights']+[policy['verifier_weights']]
models={w:YOLO(str(ROOT/w)) for w in weights}
report={'scope':'Recorded sparse native frames; not dense temporal validation. Only the first frame has an individually verified sculpture annotation here.','frames':[],'cpu_benchmark':{}}
for path in frames:
 im=np.asarray(Image.open(path).convert('RGB'))[:,:,::-1].copy()
 preds={}
 for w,model in models.items():
  result=model.predict(im,imgsz=640,conf=.30,device='cpu',verbose=False)[0]
  preds[w]=[(int(b.cls[0]),b.xyxy[0].tolist(),float(b.conf[0])) for b in result.boxes]
 row={'image':str(path),'predictions':preds,'combinations':{}}
 for primary in weights[:-1]:
  raw=preds[primary];survive=ev.runtime_color_survivors(im,raw)
  actors=[v for i,v in enumerate(raw) if i in survive and v[2]>=(.30 if v[0]==3 else .40)]
  row['combinations'][primary]={'primary_alone':actors}
  for config in policy['policies']:
   reject,evidence=p.proposed_vetoes(actors,preds[weights[-1]],config,ev.same_instance)
   row['combinations'][primary][config['name']]={'kept':[v for i,v in enumerate(actors) if i not in reject],'proposed_vetoes':[actors[i] for i in reject],'evidence':evidence}
 report['frames'].append(row)
print('native frames complete',flush=True)
# Same input and warmed models; no disk decode in this benchmark.
im=np.asarray(Image.open(frames[0]).convert('RGB'))[:,:,::-1].copy();primary=models[weights[1]];secondary=models[weights[2]]
for mode in ('primary_only','primary_plus_verifier'):
 duration=[]
 for k in range(24):
  start=time.perf_counter();primary.predict(im,imgsz=640,conf=.30,device='cpu',verbose=False)
  if mode=='primary_plus_verifier':secondary.predict(im,imgsz=640,conf=.30,device='cpu',verbose=False)
  duration.append(time.perf_counter()-start)
 report['cpu_benchmark'][mode]={'median_s':float(np.median(duration)),'p95_s':float(np.quantile(duration,.95)),'samples':len(duration),'torch_threads':1,'six_uav_load_tested':False}
(OUT/'native_and_latency.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['cpu_benchmark'],indent=2),flush=True)
for row in report['frames']:
 primary=weights[1];print(Path(row['image']).name,{k:len(v if isinstance(v,list) else v['kept']) for k,v in row['combinations'][primary].items()},flush=True)
