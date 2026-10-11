#!/bin/bash
# get_sensor_cost.sh —— 一键读出 score_cal 节点的全部 7 个 sensor 参数
# 用法：bash tools/get_sensor_cost.sh
set -e
NS="/score_cal"
for p in mono_cam stereo_cam laser1d laser2d laser3d gimbal bino_cam; do
    v=$(rosparam get ${NS}/${p} 2>/dev/null || echo "NOT_SET")
    echo "${NS}/${p} = ${v}"
done
echo "---- 计算 sensor_cost ----"
python3 - <<'PY'
ns = "/score_cal"
weights = {
    "mono_cam": 500,
    "stereo_cam": 1000,
    "laser1d": 200,
    "laser2d": 1000,
    "laser3d": 2000,
    "gimbal": 200,
    "bino_cam": 1000,
}
import subprocess, json
defaults = dict(mono_cam=1, stereo_cam=0, laser1d=0, laser2d=1, laser3d=0, gimbal=1, bino_cam=0)
total = 0
out = {}
for k, w in weights.items():
    v = subprocess.run(
        ["rosparam", "get", f"{ns}/{k}"],
        capture_output=True, text=True
    ).stdout.strip()
    try:
        vi = int(v)
    except Exception:
        vi = defaults[k]
    out[k] = dict(count=vi, cost=w, total=vi*w, source='official_default' if not v else 'parameter')
    total += vi * w
print(json.dumps(out, indent=2, ensure_ascii=False))
print(f"sensor_cost = {total}")
print(f"传感器扣分项 = {total * 0.003:.3f}；不读取裁判分数，数量口径由正式裁判确认")
PY
