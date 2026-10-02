#!/bin/bash
# 官方世界「穿城区」长航线实飞 —— 验证 A* 全局绕行 + 雷达局部避障两级配合
#
# 航线 (-60,60) -> (10,-25)：由 pick_route.py 从 8 个候选中选出
#   直线 110.1m，A* 规划 136.3m（多绕 26m），沿线 7 栋建筑
#   ⇒ 必穿城区，A* 必然给绕行航点，雷达必然看到建筑
#
# 判据：
#   · 全部航点到达
#   · 最低高度 > 4.0m（不坠地）
#   · 轨迹不穿任何建筑 AABB（用 buildings_from_world 校验）
#   · obs 应动态非零（确实看见建筑）
#   · blocked 不应长期 True（不卡死）
#
# 🔴 09-29 修正：`--alt` 6.0 -> 5.5。
#   官方 score_cal.py 明文 `z > 6.0 ⇒ score=0 且任务终止`；实测高度波动
#   5.93~6.13 m，用 6.0 必然踩线判 0 分。6 机起飞点/航线全按 5.5 m 走。
HERE="$HOME/robocup_real"
WORLD="$HOME/XTDrone/robocup/base.world"
X=-60.0; Y=60.0; GX=10.0; GY=-25.0

echo "=== world = $WORLD ==="
ls -la "$WORLD" || exit 1

pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[g]zclient' 2>/dev/null
pkill -9 -f '[b]in/px4' 2>/dev/null; pkill -9 -f '[m]avros_node' 2>/dev/null
pkill -9 -f '[r]osmaster' 2>/dev/null; pkill -9 -f '[r]oslaunch' 2>/dev/null
pkill -9 -f '[m]ultirotor_communication' 2>/dev/null
sleep 5
source "$HERE/env_robocup.sh" >/dev/null 2>&1

echo "--- 起官方世界 ---"
setsid nohup roslaunch "$HERE/world_only.launch" world:="$WORLD" gui:=false \
    > /tmp/tc_world.log 2>&1 </dev/null &
disown
for i in $(seq 1 80); do
  if pgrep -f '[g]zserver' >/dev/null 2>&1 && \
     timeout 10 rosservice list 2>/dev/null | grep -q '/gazebo/get_model_state'; then
    echo "gzserver 就绪（第 $((i*3)) 秒）"; break
  fi
  sleep 3
done
pgrep -f '[g]zserver' >/dev/null || { echo "WORLD_FAIL"; tail -10 /tmp/tc_world.log; exit 1; }
GZPID=$(pgrep -f '[g]zserver' | head -1)
[ -n "$GZPID" ] && taskset -pc 0-5 "$GZPID" >/dev/null 2>&1

echo "--- 起 PX4+MAVROS 出生点=($X,$Y) ---"
setsid nohup roslaunch "$HERE/uav_lidar_clean.launch" x:="$X" y:="$Y" \
    > /tmp/tc_uav.log 2>&1 </dev/null &
disown
for i in $(seq 1 60); do
  OUT=$(timeout 12 rosservice call /gazebo/get_model_state \
        "{model_name: 'typhoon_h480_0', relative_entity_name: 'world'}" 2>/dev/null)
  if echo "$OUT" | grep -q 'success: True'; then echo "模型已生成"; break; fi
  sleep 5
done
for i in $(seq 1 60); do
  S=$(timeout 8 rostopic echo -n1 /typhoon_h480_0/mavros/state 2>/dev/null)
  if echo "$S" | grep -q 'connected: True'; then echo "MAVROS connected"; break; fi
  sleep 5
done

cd "$HERE/_radar_test"
# 全程抓帧，供事后重放校验
python3 -u "$HERE/_radar_test/dump_scan.py" --uav typhoon_h480_0 \
  --out /tmp/tc_dump.pkl --every 10 --dur 420 > /tmp/tc_dump.log 2>&1 &
DUMP_PID=$!
sleep 2

echo "--- 实飞（穿城区 110m）---"
timeout 420 python3 -u radar_avoid.py --uav typhoon_h480_0 --pose-source gazebo \
  --start "$X" "$Y" --goal "$GX" "$GY" --world "$WORLD" \
  --alt 5.5 --speed 1.2 --verbose --hold 10 > /tmp/rf_city.log 2>&1
echo "CITY_FLIGHT_DONE rc=$?"
kill $DUMP_PID 2>/dev/null; sleep 1

echo "--- 结果 ---"
python3 - <<'PY'
import math, re, os, sys
HERE = os.path.expanduser('~/robocup_real/_radar_test')
sys.path.insert(0, HERE)
for c in (os.path.expanduser('~/robocup_real/_demo2026'),):
    if os.path.isfile(os.path.join(c, 'astar_plan.py')):
        sys.path.insert(0, c); break
import astar_plan as A
bs = A.buildings_from_world(os.path.expanduser('~/XTDrone/robocup/base.world'))

pts = []
for ln in open('/tmp/rf_city.log', errors='ignore'):
    m = re.search(r'pos=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)', ln)
    if m:
        pts.append(tuple(float(m.group(i)) for i in (1, 2, 3)))
if len(pts) < 2:
    print('  样本不足 (%d)' % len(pts)); raise SystemExit
sx, sy, _ = pts[0]; ex, ey, ez = pts[-1]
print('  起点=(%.2f,%.2f) 终点=(%.2f,%.2f)' % (sx, sy, ex, ey))
print('  终点距目标=%.2f m' % math.hypot(ex - 10.0, ey - (-25.0)))
print('  最低高度=%.2f m  最高高度=%.2f m' % (min(p[2] for p in pts), max(p[2] for p in pts)))
print('  航程(累计)=%.2f m'
      % sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1)))
# 穿墙检查（按采样点位置是否落在建筑 AABB 内）
hits = {}
for (px, py, _) in pts:
    for b in bs:
        nm, cx, cy, hx, hy = b[0], float(b[1]), float(b[2]), float(b[3]), float(b[4])
        if abs(px - cx) < hx and abs(py - cy) < hy:
            hits[nm] = hits.get(nm, 0) + 1
print('  落入建筑 AABB 的采样点: %s' % (hits if hits else '无 ✅'))
# 最小净空（到最近建筑表面）
dmin, near_nm = 1e9, None
for (px, py, _) in pts:
    for b in bs:
        nm, cx, cy, hx, hy = b[0], float(b[1]), float(b[2]), float(b[3]), float(b[4])
        dx = max(abs(px - cx) - hx, 0.0); dy = max(abs(py - cy) - hy, 0.0)
        d = math.hypot(dx, dy)
        if d < dmin:
            dmin, near_nm = d, nm
print('  到建筑表面最小距离=%.2f m (%s)  [CLEARANCE=0.80]' % (dmin, near_nm))
print('  ==> 避障 %s' % ('PASS' if dmin > 0.80 else 'FAIL'))
PY
echo "--- 到达? ---"
grep -o '航点 [0-9] 完成\|全部到达\|航点 [0-9] 超时' /tmp/rf_city.log | sort | uniq -c
echo "--- gnd 分布 ---"
grep -o 'gnd=[0-9]*' /tmp/rf_city.log | sort | uniq -c | sort -rn | head -3
echo "--- obs 分布 ---"
grep -o 'obs=[0-9]*' /tmp/rf_city.log | sort -t= -k2 -n | uniq -c | tail -6
echo "--- blocked 统计 ---"
grep -o 'blocked=[A-Za-z]*' /tmp/rf_city.log | sort | uniq -c
echo "--- 偏角去重（非0=绕行）---"
grep -o '子目标偏目标=[-+0-9.]*°' /tmp/rf_city.log | sort -u | head -12
echo "--- shift 分布 ---"
grep -o 'shift=[-+0-9.]*' /tmp/rf_city.log | sort | uniq -c | sort -rn | head -6
echo "--- 抓帧 ---"
tail -2 /tmp/tc_dump.log; ls -la /tmp/tc_dump.pkl 2>&1 | tail -1
