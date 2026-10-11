"""Offline validation only: compare new decoder with previously saved ground probe.
Recorded external camera poses are assessment labels, never an input to flight.
"""
import collections,gzip,json,math,sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'coordination/src/robocup_swarm/scripts'))
from gimbal_attention import AttitudeHistory,measured_packet,camera_rotation,quat_matrix
records=[json.loads(l) for l in gzip.open(root/'artifacts/gimbal_probe_20261011/fixed_ports_samples.jsonl.gz','rt')]
end={}
for r in records:end[r['phase']]=max(end.get(r['phase'],0),r['ros'])
errors=collections.defaultdict(list);rejected=0
def label_at(history,stamp):
    # External assessment was recorded at 5 Hz, slower than flight telemetry.
    # This interpolation is evaluation-only; production remains <=0.15 s.
    for a,b in zip(history.rows,history.rows[1:]):
        if a[0]<=stamp<=b[0] and b[0]-a[0]<=.35:
            qa,qb=np.array(a[1]),np.array(b[1])
            if np.dot(qa,qb)<0:qb=-qb
            q=qa+(qb-qa)*(stamp-a[0])/(b[0]-a[0])
            return (tuple(q/np.linalg.norm(q)),0)
for uid in (0,1):
    for r in records:
        if r['event']!='mavlink' or r['msgid']!=285 or r['topic'].split('/')[1]!='typhoon_h480_%d'%uid:continue
        data=measured_packet(NS(msgid=r['msgid'],framing_status=r['framing_status'],
            len=r['length'],payload64=r['payload64']),r['ros'])
        if data is None:rejected+=1;continue
        stamp,q,flags=data
        if r['ros']<end[r['phase']]-2.:continue
        body=AttitudeHistory();label=AttitudeHistory()
        for row in records:
            if abs(row['ros']-stamp)>.4:continue
            if row['event']=='body_pose' and row['topic'].split('/')[1]=='typhoon_h480_%d'%uid:
                body.add(row['stamp'],row['q'])
            elif row['event']=='external_camera_pose' and row['uav']==uid and row['success']:
                label.add(row['ros'],row['q'])
        b,g=body.at(stamp),label_at(label,stamp)
        if b is None or g is None:continue
        relative=camera_rotation((q,flags),b).T@quat_matrix(g[0])
        angle=math.degrees(math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1))))
        errors[str(uid)].append(angle)
out=dict(scope='Saved ground probe only; no live service access, no coordinate or flight use',
    rejected_packets=rejected,errors_deg={u:dict(n=len(e),median=float(np.median(e)),maximum=max(e)) for u,e in errors.items()})
assert len(errors)==2 and all(len(e)>30 and max(e)<.3 for e in errors.values()),out
(Path(__file__).parent/'attitude_replay.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
