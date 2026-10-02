#!/bin/bash
# 随机地图真飞 —— 验证「官方 map_generator 产出的随机图 + 两级避障」
#
# 与 fly_city.sh 的区别：
#   1. world 不是固定 base.world，而是官方 map_generator.py 当次随机生成的
#      robocup.world（每次位置都不同）；
#   2. A* 障碍源用**官方 black_box.txt 真值矩形**（--black-box），
#      不解析 world（避开 mesh/key/yaw/双份 pose 四个坑）；
#   3. 巡航高度 5.5 m（官方 score_cal.py 明文 z>6.0 ⇒ 0 分）。
#
# 用法：GEN=~/robocup_real/_genvm/try1 bash fly_rand.sh
HERE="$HOME/robocup_real"
GEN="${GEN:-$HOME/robocup_real/_genvm/try1}"
WORLD="$GEN/robocup.world"
BB="$GEN/black_box.txt"
X="${X:-0.0}"; Y="${Y:--3.0}"; GX="${GX:-110.0}"; GY="${GY:--35.0}"
ALT="${ALT:-5.5}"; SPEED="${SPEED:-1.2}"

echo "=== 随机图目录 $GEN ==="
ls -la "$WORLD" "$BB" || exit 1
echo "=== world 非空检查（官方 fallback bug 会产出 0 字节）==="
[ -s "$WORLD" ] || { echo "WORLD_EMPTY（官方生成器 fallback）"; exit 1; }

pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[g]zclient' 2>/dev/null
pkill -9 -f '[b]in/px4' 2>/dev/null; pkill -9 -f '[m]avros_node' 2>/dev/null
pkill -9 -f '[r]osmaster' 2>/dev/null; pkill -9 -f '[r]oslaunch' 2>/dev/null
pkill -9 -f '[m]ultirotor_communication' 2>/dev/null
sleep 5
source "$HERE/env_robocup.sh" >/dev/null 2>&1

echo "--- 起随机世界 ---"
setsid nohup roslaunch "$HERE/world_only.launch" world:="$WORLD" gui:=false \
    > /tmp/tr_world.log 2>&1 </dev/null &
disown
for i in $(seq 1 80); do
  if pgrep -f '[g]zserver' >/dev/null 2>&1 && \
     timeout 10 rosservice list 2>/dev/null | grep -q '/gazebo/get_model_state'; then
    echo "gzserver 就绪（第 $((i*3)) 秒）"; break
  fi
  sleep 3
done
pgrep -f '[g]zserver' >/dev/null || { echo "WORLD_FAIL"; tail -10 /tmp/tr_world.log; exit 1; }
GZPID=$(pgrep -f '[g]zserver' | head -1)
[ -n "$GZPID" ] && taskset -pc 0-5 "$GZPID" >/dev/null 2>&1

echo "--- 起 PX4+MAVROS 出生点=($X,$Y) ---"
setsid nohup roslaunch "$HERE/uav_lidar_clean.launch" x:="$X" y:="$Y" \
    > /tmp/tr_uav.log 2>&1 </dev/null &
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
setsid nohup python3 -u "$HERE/_radar_test/dump_scan.py" --uav typhoon_h480_0 \
  --out /tmp/tr_dump.pkl --every 10 --dur 500 > /tmp/tr_dump.log 2>&1 </dev/null &
DUMP_PID=$!
sleep 2

echo "--- 实飞（随机图 $(basename $GEN)，alt=$ALT）---"
timeout 600 python3 -u radar_avoid.py --uav typhoon_h480_0 --pose-source gazebo \
  --start "$X" "$Y" --goal "$GX" "$GY" \
  --black-box "$BB" \
  --alt "$ALT" --speed "$SPEED" --verbose --hold 10 > /tmp/rf_rand.log 2>&1
echo "RAND_FLIGHT_DONE rc=$?"
kill $DUMP_PID 2>/dev/null; sleep 1

echo "--- 结果（用官方 black_box.txt 真值校验）---"
BB="$BB" GX="$GX" GY="$GY" python3 - <<'PY'
import math, re, os, sys
for c in (os.path.expanduser('~/robocup_real/_radar_test'),
          os.path.expanduser('~/robocup_real/_demo2026')):
    if os.path.isdir(c):
        sys.path.insert(0, c)
import astar_plan as A
bld = A.boxes_from_black_box(os.environ['BB'])
GX = float(os.environ['GX']); GY = float(os.environ['GY'])

pts = []
for ln in open('/tmp/rf_rand.log', errors='ignore'):
    m = re.search(r'pos=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)', ln)
    if m:
        pts.append(tuple(float(m.group(i)) for i in (1, 2, 3)))
if len(pts) < 2:
    print('  样本不足 (%d)' % len(pts)); raise SystemExit
sx, sy, _ = pts[0]; ex, ey, ez = pts[-1]
print('  起点=(%.2f,%.2f) 终点=(%.2f,%.2f)' % (sx, sy, ex, ey))
print('  终点距目标=%.2f m' % math.hypot(ex - GX, ey - GY))
print('  最低高度=%.2f m  最高高度=%.2f m' % (min(p[2] for p in pts), max(p[2] for p in pts)))
print('  航程(累计)=%.2f m' % sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
                              for i in range(len(pts)-1)))
hits = {}
for (px, py, _) in pts:
    for (nm, cx, cy, hx, hy, _y) in bld:
        if abs(px - cx) < hx and abs(py - cy) < hy:
            hits[nm] = hits.get(nm, 0) + 1
print('  落入障碍真值的采样点: %s' % (hits if hits else '无 ✅'))
dmin, near_nm = 1e9, None
for (px, py, _) in pts:
    for (nm, cx, cy, hx, hy, _y) in bld:
        dx = max(abs(px - cx) - hx, 0.0); dy = max(abs(py - cy) - hy, 0.0)
        d = math.hypot(dx, dy)
        if d < dmin:
            dmin, near_nm = d, nm
print('  到障碍表面最小距离=%.2f m (%s)  [CLEARANCE=0.80]' % (dmin, near_nm))
print('  ==> 避障 %s' % ('PASS' if dmin > 0.80 and not hits else 'FAIL'))
PY
echo "--- 到达? ---"
grep -o '航点 [0-9]* 完成\|全部到达\|航点 [0-9]* 超时' /tmp/rf_rand.log | sort | uniq -c
echo "--- gnd / blocked / obs 统计 ---"
grep -o 'gnd=[0-9]*' /tmp/rf_rand.log | sort | uniq -c | sort -rn | head -3
grep -o 'blocked=[A-Za-z]*' /tmp/rf_rand.log | sort | uniq -c
grep -o 'obs=[0-9]*' /tmp/rf_rand.log | sort -t= -k2 -n | uniq -c | tail -5
echo "--- 偏角去重（非0=绕行）---"
grep -o '子目标偏目标=[-+0-9.]*°' /tmp/rf_rand.log | sort -u | head -12
echo "--- 障碍源 ---"
grep -o '障碍源 = .*' /tmp/rf_rand.log | head -2
