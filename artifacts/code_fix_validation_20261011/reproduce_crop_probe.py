import sys, json, time
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, '/home/gycc/桌面/RoboCup_Team/tools')
import train_distractor as data
cv2.setNumThreads(2)
root=Path('/home/gycc/robocup_distractor_data')
rows,audit=data.audit_manifest(root/'dataset_v3_preserved/reviewed_manifest.jsonl')

def feature(im,b):
 h,w=im.shape[:2];c,x,y,bw,bh=b
 x0=max(0,int((x-bw/2)*w));x1=min(w,int((x+bw/2)*w+1))
 y0=max(0,int((y-bh/2)*h));y1=min(h,int((y+bh/2)*h+1))
 if x1-x0<8 or y1-y0<16:return None
 a=cv2.cvtColor(im[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY)
 a=cv2.resize(a,(64,128)).astype(np.float32)
 gx=cv2.Sobel(a,cv2.CV_32F,1,0,ksize=1);gy=cv2.Sobel(a,cv2.CV_32F,0,1,ksize=1)
 mag=np.hypot(gx,gy);angle=np.mod(np.arctan2(gy,gx),np.pi)
 bins=np.minimum(8,(angle*9/np.pi).astype(int))
 cell=np.arange(128)[:,None]//8*8+np.arange(64)[None,:]//8
 hist=np.bincount((cell*9+bins).ravel(),weights=mag.ravel(),minlength=128*9).reshape(128,9)
 hist/=np.sqrt(np.sum(hist**2,axis=1,keepdims=True)+1e-6)
 return np.r_[hist.ravel(),np.log((x1-x0)/(y1-y0)),1.]
sets={s:[] for s in ('train','val','test')}
for r in rows:
 if r['split']=='test':continue
 im=cv2.imread(r['image'])
 for b in data.read_labels(r['label']):
  f=feature(im,b)
  if f is not None:sets[r['split']].append((f,b[0],r['group']))
print({s:len(v) for s,v in sets.items()},flush=True)
x=np.array([p[0] for p in sets['train']],np.float32); y=np.array([int(p[1]==5) for p in sets['train']],np.int32)
weight=np.where(y==1, max(1.,float(sum(y==0))/max(1,sum(y==1))),1.)
regularizer=np.eye(x.shape[1]);regularizer[-1,-1]=.01
coef=np.linalg.solve((x.T*weight)@x+regularizer, x.T@(weight*(y*2.-1.)))
v=np.array([p[0] for p in sets['val']],np.float32); labels=np.array([p[1] for p in sets['val']]); pred=v@coef
threshold=max(.1,float(max(pred[labels!=5]))+.05);sign=1.
summary=dict(audit=audit,threshold=threshold,sign=sign,by_class={data.NAMES[c]:dict(n=int(sum(labels==c)),rejected=int(sum(pred[labels==c]>=threshold)),max_margin=float(max(pred[labels==c])) if any(labels==c) else None) for c in range(6)},scope='GT crops of existing validation only; not detector crops or flight acceptance',model='linear grayscale gradient cells, ridge alpha=1 fixed')
out=root/'crop_probe_20261011';out.mkdir(exist_ok=True);np.save(out/'candidate.npy',coef);(out/'probe.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
