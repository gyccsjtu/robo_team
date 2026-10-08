import json, math
import numpy as np

M = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
R = '/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159'
cams = [json.loads(l) for l in open(R + '/flight/algorithm/search_camera_frames.jsonl')]
FX, CX, CY = 205.46963709898583, 320.5, 180.5


def proj(c, world):
    Rm = np.array(c['camera_rotation'], dtype=float).reshape(3, 3)
    cam = np.array(c['camera_xyz'], dtype=float)
    d = M.T @ Rm.T @ (np.array(world, dtype=float) - cam)
    if abs(d[2]) < 1e-9:
        return None
    return (FX * d[0] / d[2] + CX, FX * d[1] / d[2] + CY, float(d[2]))


def fmt(out):
    if out is None:
        return 'behind/degenerate'
    u, v, dep = out
    inside = 0 <= u < 640 and 0 <= v < 360 and dep > 0
    return 'uv=(%6.1f,%5.1f) depth=%6.1f %s' % (u, v, dep, 'IN' if inside else 'OUT')


truth = [json.loads(l)['sample'] for l in
         open(R + '/observers/independent_visual_accuracy_white.jsonl')]

print('=== SANITY 1: same actor, same uav, different times (u/v must move) ===')
for t in (1942.028, 1950.0, 1960.0, 1970.0, 1990.0):
    cs = [c for c in cams if c['uav_id'] == 'uav_1' and abs(float(c['image_s']) - t) < 0.3]
    if not cs:
        print('  t=%.1f no camera frame' % t)
        continue
    c = cs[0]
    near = min(truth, key=lambda s: abs(float(s[0]) - float(c['sample_s'])))
    out = proj(c, [near[1], near[2], 1.0])
    print('  uav_1 t=%.1f cam=(%7.1f,%6.1f) truth=(%7.1f,%7.1f) -> %s' % (
        float(c['image_s']), c['camera_xyz'][0], c['camera_xyz'][1],
        near[1], near[2], fmt(out)))

print()
print('=== SANITY 2: blue observations with person_frame_verified=true (should be IN) ===')
frames = [json.loads(l) for l in open(R + '/observers/frames/frames.jsonl')]
for f in frames[:6]:
    o = f['observation']
    cs = [c for c in cams if c['uav_id'] == o['uav_id']
          and abs(float(c['image_s']) - float(o['sample_s'])) < 0.35]
    if not cs:
        continue
    c = cs[0]
    xyz = o['xyz']
    out = proj(c, [xyz[0], xyz[1], 1.0])
    print('  %s t=%.3f blue xyz=(%6.1f,%6.1f) -> %s' % (
        o['uav_id'], o['sample_s'], xyz[0], xyz[1], fmt(out)))

print()
print('=== SANITY 3: six aircraft at the same time (u/v should differ by viewpoint) ===')
for t in (1950.0, 1970.0):
    print('  t~%.0f' % t)
    for uav in ('uav_1', 'uav_2', 'uav_3', 'uav_4', 'uav_5', 'uav_6'):
        cs = [c for c in cams if c['uav_id'] == uav and abs(float(c['image_s']) - t) < 0.3]
        if not cs:
            continue
        c = cs[0]
        near = min(truth, key=lambda s: abs(float(s[0]) - float(c['sample_s'])))
        out = proj(c, [near[1], near[2], 1.0])
        print('     %s cam=(%7.1f,%6.1f) -> %s' % (
            uav, c['camera_xyz'][0], c['camera_xyz'][1], fmt(out)))