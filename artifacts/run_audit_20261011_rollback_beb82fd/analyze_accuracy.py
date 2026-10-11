import bisect, collections, json, math, sys
path=sys.argv[1]
truth=collections.defaultdict(list); reports=[]; sources=[]
for line in open(path):
    try:d=json.loads(line)
    except ValueError:continue
    if d['event']=='truth' and d['success']:
        truth[d['actor']].append((.5*(d['request_ros']+d['ros']),d['x'],d['y']))
    elif d['event']=='report':reports.append(d)
    elif d['event']=='visual_source':sources.append(d)
times={i:[p[0] for p in pts] for i,pts in truth.items()}
def pos(aid,t):
    ts=times.get(aid,[]); idx=bisect.bisect_left(ts,t)
    if not ts or idx==0 or idx==len(ts):return None
    a,b=truth[aid][idx-1:idx+1]
    if b[0]-a[0]>.6:return None
    f=(t-a[0])/(b[0]-a[0])
    return (a[1]+f*(b[1]-a[1]),a[2]+f*(b[2]-a[2]))
def nearest(tag,x,y,t):
    ids={'green':[0],'blue':[1],'brown':[2],'white':[3],'red1':[4,5],'red2':[4,5],'red':[4,5]}[tag]
    # A referee deletion can occur between the final truth sample and our
    # receipt of its last valid report. Do not assign that packet to the other
    # red actor merely because it is the only surviving interpolation series.
    for aid in ids:
        pts=truth.get(aid,[])
        if pts and 0. <= t-pts[-1][0] <= .6 and math.hypot(x-pts[-1][1],y-pts[-1][2]) < 2.:
            return None
    ds=[]
    for aid in ids:
        p=pos(aid,t)
        if p:ds.append((math.hypot(x-p[0],y-p[1]),aid))
    return min(ds) if ds else None
errors=collections.defaultdict(list); bad=[]
for r in reports:
    hit=nearest(r['tag'],r['x'],r['y'],r['ros'])
    if hit:
        e,aid=hit; errors[r['tag']].append((r['ros'],e,aid))
        if e>=1:bad.append(dict(r,error=e,actor=aid))
print('truth_samples', {i:len(v) for i,v in truth.items()}, 'reports',len(reports))
for tag,rows in sorted(errors.items()):
    es=sorted(x[1] for x in rows);good=sum(e<1 for e in es)
    print(tag, 'n',len(es),'under_1m',good,'p50',round(es[len(es)//2],3),'p90',round(es[int(.9*(len(es)-1))],3),'max',round(max(es),3), 'actors',dict(collections.Counter(r[2] for r in rows)), 'last',rows[-1])
print('last_bad',bad[-6:])
raw=collections.defaultdict(list)
for r in sources:
    d=r['data'];tag={'t0':'green','t1':'blue','t2':'brown','t3':'white','t4':'red2','t5':'red1'}.get(d.get('target_id'))
    if tag is None:continue
    hit=nearest(tag,d['x'],d['y'],d['sample_s'])
    if hit:raw[(tag,d['source_uav'],d['official_view_ok'])].append(hit[0])
for key,es in sorted(raw.items()):
    es.sort(); print('raw',key,'n',len(es),'p50',round(es[len(es)//2],2),'p90',round(es[int(.9*(len(es)-1))],2))
