"""Offline observer-scheduled comparison, not full ROS callback/physical replay."""
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[4]
SCRIPTS=ROOT/'coordination/src/robocup_swarm/scripts'
sys.path.insert(0,str(SCRIPTS))
from yolo_target_bridge import TargetBridgeCore
Q=ROOT/'coordination/docs/validation/fast_competition_v137_20261006/round_1_seed_8159'
rows=[json.loads(l) for l in (Q/'observers/independent_visual_accuracy_brown.jsonl').open()]
truth=sorted(r['sample'] for r in rows if r['kind']=='actor_truth')
times=[r[0] for r in truth]
official=[r['sample'] for r in rows if r['kind']=='official']
visual=[]
for line in (Q/'flight/city_events.jsonl').open():
    r=json.loads(line)
    if r.get('kind')=='visual' and r['value']['target_id']=='brown':
        visual.append((r['sample_s'],r['value']))
visual.sort(key=lambda r:r[0])
def point(t):
    i=bisect.bisect_left(times,t)
    if not 0<i<len(times):return None
    a,b=truth[i-1],truth[i]
    if b[0]-a[0]>.2 or math.dist(a[1:3],b[1:3])>max(.5,3*(b[0]-a[0])):return None
    w=(t-a[0])/(b[0]-a[0])
    return tuple(a[k]+w*(b[k]-a[k]) for k in (1,2))
cores=[TargetBridgeCore(),TargetBridgeCore(brown_alignment=True)]
results=[[],[]];idx=0;baseline_errors=[]
for t,x,y in official:
    while idx<len(visual) and visual[idx][0]<=t:
        _,v=visual[idx];idx+=1
        for c in cores:
            c.report(v['sample_s'],'brown',*v['xyz'][:2],v['confidence'],v['uav_id'],v['observation_id'])
    expected=point(t)
    for k,c in enumerate(cores):
        events=c.tick(t)
        ev=next((e for e in events if e['tag']=='brown' and not e['eliminated']),None)
        xy=(ev['x'],ev['y']) if ev and 0<=t-c.tracks['brown'].t_obs<=1.5 else None
        error=math.dist(xy,expected) if xy and expected else None
        results[k].append(dict(t=t,error=error,xy=xy))
        if k==0 and xy:baseline_errors.append(math.dist(xy,(x,y)))
def metrics(rows):
    start=last=None;longest=0.
    for r in rows:
        t=r['t']
        if r['error'] is not None and r['error']<1.:
            if start is None or t-last>1.:start=t
            longest=max(longest,t-start);last=t
        else:start=last=None
    return dict(outputs=sum(r['xy'] is not None for r in rows),
        accurate=sum(r['error'] is not None and r['error']<1. for r in rows),longest_s=longest)
a,b=map(metrics,results)
breaks=[]
for t in (2068.636,2084.836,2169.436,2176.936,2177.436,2186.836):
    i=min(range(len(official)),key=lambda i:abs(official[i][0]-t))
    breaks.append(dict(t=t,old_error=results[0][i]['error'],new_error=results[1][i]['error']))
fixed=sum(r['old_error'] is not None and r['old_error']>=1. and r['new_error'] is not None and r['new_error']<1. for r in breaks)
gate=(len(baseline_errors)==len(official) and max(baseline_errors)<.01 and fixed>=1
    and b['accurate']>=a['accurate'] and b['longest_s']>=12.-1e-6 and b['outputs']>=a['outputs'])
summary=dict(kind='observer_scheduled_not_complete_callbacks',visual_count=len(visual),official_count=len(official),
    baseline_reproduction_max_m=max(baseline_errors),baseline=a,candidate=b,breaks=breaks,fixed_breaks=fixed,enable_candidate=gate,
    source_sha256={n:hashlib.sha256((SCRIPTS/n).read_bytes()).hexdigest() for n in ('source_aligned_fusion.py','yolo_target_bridge.py')})
Path(__file__).with_name('brown_replay_result.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
