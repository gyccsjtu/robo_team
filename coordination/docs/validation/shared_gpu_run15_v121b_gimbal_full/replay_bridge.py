import pathlib,json,sys,bisect,math,statistics
root=pathlib.Path('/root/robocup_runs/codex_city_teammate_v121b_20261004/flight')
sys.path.insert(0,'/mnt/d/a/.robocup/robo_team/coordination/src/robocup_swarm/scripts')
import yolo_target_bridge as m
incoming=[json.loads(l) for l in (root/'city_events.jsonl').read_text().splitlines() if json.loads(l)['kind']=='visual'];incoming.sort(key=lambda r:r['sample_s'])
truth={};times={}
for tag in ['green','brown','white']:
 f=root/('independent_visual_accuracy'+('' if tag=='green' else '_'+tag)+'.jsonl')
 truth[tag]=[json.loads(l)['sample'] for l in f.read_text().splitlines() if json.loads(l)['kind']=='actor_truth'];times[tag]=[r[0] for r in truth[tag]]
def error(tag,t,xy):
 i=bisect.bisect_left(times[tag],t)
 if i==0 or i==len(times[tag]):return None
 a,b=truth[tag][i-1],truth[tag][i]
 if b[0]-a[0]>.2:return None
 f=(t-a[0])/(b[0]-a[0]);return math.dist(xy,[a[j]+f*(b[j]-a[j]) for j in [1,2]])
all_results=[]
for window in [.15,.3,.6]:
 for alpha in [.1,.35,.7]:
  m.OBS_WINDOW=window;m.VEL_ALPHA=alpha;c=m.TargetBridgeCore();idx=0;errors={tag:[] for tag in truth}
  start=1930.964
  for tick in range(6001):
   now=start+tick*.1
   while idx<len(incoming) and incoming[idx]['sample_s']<=now:
    r=incoming[idx]['value'];tag=r['target_id']
    if tag in truth:c.report(r['sample_s'],tag,*r['xyz'][:2],r['confidence'])
    idx+=1
   for out in c.tick(now):
    tag=out['tag'];tr=c.tracks[tag]
    if tag not in truth or not 0<=now-tr.t_obs<=m.COAST_TIME:continue
    err=error(tag,now,[out['x'],out['y']])
    if err is not None:errors[tag].append((now,err))
  metrics={}
  for tag,rows in errors.items():
   longest=0.;since=previous=None
   for now,err in rows:
    if err>=1 or (previous is not None and now-previous>1):since=None
    if err<1:
     if since is None:since=now
     longest=max(longest,now-since)
    previous=now
   metrics[tag]={'samples':len(rows),'mean_error':statistics.mean(e for _,e in rows) if rows else None,'fraction_below1':sum(e<1 for _,e in rows)/len(rows) if rows else None,'longest_good_s':round(longest,2)}
  value={'window':window,'alpha':alpha,'metrics':metrics,'offline_only':True,'official_judge_executed':False};all_results.append(value);print(json.dumps(value),flush=True)
(root/'bridge_replay_candidates.json').write_text(json.dumps(all_results,indent=2))
