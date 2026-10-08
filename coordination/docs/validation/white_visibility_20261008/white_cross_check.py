import json, csv, glob, os, math

R = '/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159'
A = R + '/flight/algorithm'
V = '/mnt/d/a/.robocup/robo_team/coordination/docs/validation/white_visibility_20261008/white_visibility_v140.json'

d = json.load(open(V))
frames = d['frames']
in_fov = [f for f in frames if f.get('in_fov')]

print('=== in-FOV expected pixel height distribution ===')
hs = sorted(f['expected_h_px'] for f in in_fov)
ds = sorted(f['horiz_m'] for f in in_fov)
def pct(a, p):
    return a[min(len(a)-1, int(len(a)*p))]
for p, label in ((0.0,'min'), (0.25,'p25'), (0.5,'p50'), (0.75,'p75'), (0.95,'p95'), (1.0,'max')):
    print('  %-4s dist=%6.1f m   expected_h=%5.1f px' % (label, pct(ds,p), pct(hs,p)))
print('  frames with expected_h >= 8 px : %d / %d' % (sum(1 for h in hs if h >= 8), len(hs)))
print('  frames with expected_h >= 12 px: %d / %d' % (sum(1 for h in hs if h >= 12), len(hs)))
print('  frames within 22 m : %d' % sum(1 for x in ds if x <= 22))
print('  frames within 12 m : %d' % sum(1 for x in ds if x <= 12))

print()
print('=== the three recorded white detections: visibility at those instants ===')
# 机号映射：perception_typhoon_h480_N.csv 的模型名对应 uav_(N+1)
want = [(1970.076, 'uav_3'), (1970.848, 'uav_3'), (1999.208, 'uav_3')]
for t, uav in want:
    cand = [f for f in frames if f['uav'] == uav and abs(f['t_image'] - t) <= 0.5]
    for f in cand[:2]:
        if f.get('mid') is None:
            print('  t=%.3f %s: %s' % (t, uav, f.get('reason')))
            continue
        print('  t=%.3f %s: %s  dist=%.1f m  depth=%.1f  exp_h=%s px  uv=(%.0f,%.0f)' % (
            t, uav, f.get('reason'), f.get('horiz_m', -1), f['mid']['depth'],
            ('%.1f' % f['expected_h_px']) if f.get('expected_h_px') else 'n/a',
            f['mid']['u'] or -1, f['mid']['v'] or -1))

print()
print('=== the 45 white CSV rows: which aircraft, exact file -> logical id ===')
print('  mapping: perception_typhoon_h480_N.csv  ==  uav_(N+1)')
for fn in sorted(glob.glob(A + '/perception_typhoon_h480_*.csv')):
    rows = list(csv.DictReader(open(fn)))
    w = [r for r in rows if (r.get('class_name') or '').lower() == 'white']
    if w:
        n = os.path.basename(fn).replace('perception_typhoon_h480_', '').replace('.csv', '')
        print('  h480_%s -> uav_%d : %d white rows' % (n, int(n)+1, len(w)))
        for r in w:
            if r.get('event') == 'yolo_detection':
                print('      t=%s conf=%s w=%s h=%s range=%s height=%s n_person=%s reason=%s' % (
                    r.get('ros_time'), r.get('confidence'), r.get('bbox_w'),
                    r.get('bbox_h'), r.get('range_m'), r.get('height_m'),
                    r.get('n_person_boxes'), r.get('reject_reason')))

print()
print('=== in-FOV frames vs saved original images ===')
# 本轮保存的原图（frames.jsonl + evidence 通道）
saved = []
try:
    for f in (json.loads(l) for l in open(R + '/observers/frames/frames.jsonl')):
        saved.append((float(f['observation']['sample_s']), f['observation']['uav_id']))
except Exception as e:
    print('  frames.jsonl read err', e)
for d2 in glob.glob(A + '/evidence_*'):
    idx = os.path.join(d2, 'index.jsonl')
    if os.path.isfile(idx):
        for line in open(idx):
            try:
                r = json.loads(line)
                saved.append((float(r.get('image_stamp', -1)), 'h480_' + os.path.basename(d2).split('_')[-1]))
            except Exception:
                pass
print('  saved original images indexed: %d' % len(saved))
print('  in-FOV frames whose moment has a saved image (+/-0.3 s): %d / %d' % (
    sum(1 for f in in_fov if any(abs(f['t_image']-s[0]) <= 0.3 for s in saved)), len(in_fov)))