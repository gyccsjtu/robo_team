#!/usr/bin/env bash
# verify_lamp_post_fix_20261006.sh
# 6 项指标核验脚本（针对 docs/lamp_post_stuck_fix_20261005.md 第六节验证表）
# 用法：在 ROS 主机跑完 `bash run_match.sh` 后，跑 `bash verify_lamp_post_fix_20261006.sh`
#      输出会同时落到终端与 logs/verify_report_20261006.txt

set -u

cd "$(dirname "$0")"

OUT=logs/verify_report_20261006.txt
mkdir -p logs
exec > >(tee -a "$OUT") 2>&1

echo "============================================================"
echo " RoboCup 三问题修复验证报告  $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

if ! ls logs/uav_*.log 2>/dev/null | head -1 | grep -q .; then
  echo "ERROR: logs/uav_*.log 不存在，先跑 bash run_match.sh"
  exit 1
fi

echo
echo "----- 指标 1: 三面堵死告警总数 -----"
N1=$(grep -h "三面堵死" logs/uav_*.log 2>/dev/null | wc -l)
echo "  count = $N1   阈值 < 30   $([ "$N1" -lt 30 ] && echo PASS || echo FAIL)"

echo
echo "----- 指标 2: vout=(0,0) 死锁 -----"
N2=$(grep -h "vout=(0.00,0.00)" logs/uav_*.log 2>/dev/null | wc -l)
echo "  count = $N2   阈值 = 0    $([ "$N2" -eq 0 ] && echo PASS || echo FAIL)"

echo
echo "----- 指标 3: 首次目标捕获时间 -----"
T3=$(grep -h "开始捕获" logs/uav_*.log 2>/dev/null | head -1)
echo "  $T3"
if echo "$T3" | grep -qE 't=[0-9.]+'; then
  T3NUM=$(echo "$T3" | grep -oE 't=[0-9.]+' | head -1 | sed 's/t=//')
  awk -v t="$T3NUM" 'BEGIN{printf "  t=%.1fs  阈值 < 90s  %s\n", t, (t<90?"PASS":"FAIL")}'
fi

echo
echo "----- 指标 4: 位置散布（起飞后 5s 起，每 2s 取样一次）-----"
python3 - <<'PYEOF'
import csv, os, glob
from collections import defaultdict
samples = defaultdict(list)
for fn in sorted(glob.glob('logs/uav_*.csv')):
    uid = os.path.basename(fn).split('.')[0]
    with open(fn) as f:
        r = csv.reader(f)
        next(r, [])
        rows = [row for row in r if row]
    # 取 z>4m 后的前 20 个采样
    air = [r for r in rows if len(r) >= 4 and float(r[3]) > 4.0][:20]
    for r in air[::2]:
        samples[uid].append((float(r[1]), float(r[2])))
if not samples:
    print("  无可用采样 (csv 为空或飞机未起飞)")
else:
    for uid, pts in sorted(samples.items()):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if len(pts) >= 2:
            from math import sqrt
            spread = max(sqrt((x-xs[0])**2+(y-ys[0])**2) for x,y in zip(xs,ys))
            tag = "PASS (散布开)" if spread > 3.0 else "WARN (可能仍集中在起飞点附近)"
            print(f"  {uid}: 起{(xs[0]:+.1f},{ys[0]:+.1f)} 散布 {spread:.1f}m  {tag}")
        else:
            print(f"  {uid}: 仅 {len(pts)} 个采样")
PYEOF

echo
echo "----- 指标 5: EMA 双轨制 reset/streak 分布 -----"
SCORE_LINE=$(grep "SCORE_STATS" logs/*.log 2>/dev/null | tail -1)
if [ -z "$SCORE_LINE" ]; then
  echo "  WARN: 未找到 SCORE_STATS 输出（score_cal.py F 修复可能未生效或进程未正常退出）"
else
  echo "  $SCORE_LINE"
  # 解析 5 桶 streak 分布
  python3 - <<PYEOF
import re
line = """$SCORE_LINE"""
keys = ['streak_lt1','streak_1to5','streak_5to10','streak_10to15','streak_ge15']
vals = {}
for k in keys:
    m = re.search(rf'{k}=(\d+)', line)
    vals[k] = int(m.group(1)) if m else 0
total = sum(vals.values())
long_streak = vals['streak_5to10'] + vals['streak_10to15'] + vals['streak_ge15']
ratio = (long_streak / total * 100) if total else 0
print(f"  总 streak 样本 = {total}")
print(f"  长 streak(≥5s) = {long_streak} = {ratio:.1f}%   阈值 ≥ 60% (EMA 抗误检)")
print(f"  桶分布: <1s={vals['streak_lt1']}  1-5s={vals['streak_1to5']}  5-10s={vals['streak_5to10']}  10-15s={vals['streak_10to15']}  ≥15s={vals['streak_ge15']}")
print(f"  判定: {'PASS' if ratio >= 60 else 'FAIL'}")
PYEOF
fi

echo
echo "----- 指标 6: 撞 lamp_post 检查（按需人工）------"
echo "  csv 中 lamp_post_191 @ (-3,-3.7) 附近世界坐标采样:"
grep -h "world_xy" logs/uav_*.csv 2>/dev/null | python3 -c "
import sys, math
hit = 0
near = 0
for line in sys.stdin:
    parts = line.replace(',', ' ').split()
    try:
        # 找两个连续的浮点数作为 x,y
        nums = [float(x) for x in parts if x.replace('.','').replace('-','').isdigit()]
        if len(nums) < 2: continue
        x,y = nums[0], nums[1]
        d = math.hypot(x-(-3), y-(-3.7))
        if d < 0.5: hit += 1
        elif d < 1.0: near += 1
    except: pass
print(f'  距 lamp_post_191 中心 <0.5m 采样 = {hit}  (撞杆阈值)')
print(f'  距 <1.0m 采样 = {near}            (贴杆预警)')
print(f'  判定: {\"PASS\" if hit == 0 else \"FAIL\"}')
"

echo
echo "============================================================"
echo " 报告保存于: $OUT"
echo "============================================================"