import bisect,json,math,pathlib,sys
root=pathlib.Path('/root/robocup_runs/competition_recovery_r2_20261007/round_1_seed_8159')
sys.path.insert(0,str(root/'flight/swarm_sources/coordination/src/robocup_swarm/scripts'))
truth={4:[],5:[]}
for l in (root/'observers/independent_visual_accuracy_red.jsonl').open():
 r=json.loads(l)
 if r['kind']=='actor_truth':truth[r['sample'][3]].append(r['sample'])
times={a:[r[0] for r in rows] for a,rows in truth.items()}
def error(t,xy):
 ds=[]
 for a,rows in truth.items():
  i=bisect.bisect_left(times[a],t)
  if i==0 or i>=len(rows) or rows[i][0]-rows[i-1][0]>.5:continue
  p,q=rows[i-1],rows[i];f=(t-p[0])/(q[0]-p[0])
  ds.append(math.hypot(xy[0]-(p[1]+f*(q[1]-p[1])),xy[1]-(p[2]+f*(q[2]-p[2]))))
 return min(ds) if ds else None
source=(root/'flight/swarm_sources/coordination/src/robocup_swarm/scripts/yolo_target_bridge.py').read_text()
trace=[json.loads(l) for l in (root/'flight/algorithm/bridge_trace.jsonl').open()]
out={}
for name,aligned,distance,duration in [('original',False,1.5,1.2),('distance_only',False,3.,1.2),('distance_and_time',False,3.,1.5),('source_aligned',True,1.5,1.2),('aligned_with_limits',True,3.,1.5)]:
 scope={'__name__':'offline_probe','__file__':'offline_probe'}
 text=source.replace("tag == 'brown'", "tag in ('brown','red1','red2')") if aligned else source
 exec(compile(text,'offline_bridge_probe','exec'),scope)
 scope['EXTRAP_MAX_D']=distance;scope['EXTRAP_MAX_T']=duration
 core=scope['TargetBridgeCore'](brown_alignment=aligned)
 samples=[]
 for r in trace:
  if r['kind']=='input' and r['observation']['target_id'] in ('red1','red2'):
   o=r['observation'];core.report(o['sample_s'],o['target_id'],*o['xyz'][:2],o['confidence'],o['uav_id'],o['observation_id'])
  if r['kind']=='upload' and r['tag'] in ('red1','red2'):
   events={e['tag']:e for e in core.tick(r['prediction_s'])}
   if r['tag'] not in events:continue
   e=events[r['tag']];xy=[round(e['x'],3),round(e['y'],3)]
   samples.append(dict(stamp=r['prediction_s'],xy=xy,error=error(r['prediction_s'],xy),
    old_error=error(r['prediction_s'],r['requested_xy']),old_xy=r['requested_xy']))
 valid=[r['error'] for r in samples if r['error'] is not None]
 out[name]=dict(count=len(samples),accurate=sum(e<1. for e in valid),maximum=max(valid),mean=sum(valid)/len(valid),
    exact_old_max_difference=max(math.dist(r['xy'],r['old_xy']) for r in samples),
    new_bad=sum(r['old_error']<1.<=r['error'] for r in samples),new_good=sum(r['error']<1.<=r['old_error'] for r in samples),samples=samples)
path=pathlib.Path(__file__).resolve().parent/'red_extrapolation_probe.json'
path.write_text(json.dumps({'variants':out,'limitations':['Uses actual input/upload order and exact prediction time; no ROS callback or new motion replay.','Truth only classifies offline error; no production code or parameter changed.','No 15 second success proven.']},indent=2)+'\n')
for name,r in out.items():print(name,{k:v for k,v in r.items() if k!='samples'})
