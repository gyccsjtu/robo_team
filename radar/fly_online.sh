#!/bin/bash
# 在线建图模式真飞 —— 机上**零文件**，靠机载雷达自己建图、自己绕
#
# 与 fly_autonomous.sh 的唯一区别：
#   算法端多开 --online-map：
#     ① 雷达每帧累积在线占据栅格（不读任何地图文件、不订阅任何真值话题）
#     ② 每 --replan-period 秒用 A* 在**已探明**障碍上重算剩余航点
#     ③ A* 失败 / 路径病态 ⇒ 保持原航点不动 ⇒ 退化为「直飞 + 雷达反应式」
#   即：起飞时对地图一无所知（比赛实况），飞着飞着自己把路找出来。
#
# ⚠ world 文件只传给 gzserver：那是**仿真器加载场地**（等价于比赛现场
#   组委会摆好的场景），不传给算法进程，算法对它一无所知。
#
# 前置（缺一不可）：
#   ~/robocup_real/_radar_test/radar_avoid.py       （已含 --online-map）
#   ~/robocup_real/_radar_test/online_planner.py    （同目录）
#   ~/robocup_real/_radar_test/occupancy_online.py  （同目录）
#   ~/robocup_real/_demo2026/astar_plan.py
#
# 用法：GEN=~/robocup_real/_genvm/try1 bash fly_online.sh
HERE="$HOME/robocup_real"
GEN="${GEN:-$HOME/robocup_real/_genvm/try1}"
WORLD="$GEN/robocup.world"
X="${X:-0.0}"; Y="${Y:--3.0}"; GX="${GX:-110.0}"; GY="${GY:--35.0}"
ALT="${ALT:-2.8}"; SPEED="${SPEED:-1.2}"
PERIOD="${PERIOD:-5.0}"
# 动态目标跟随（10-03）：接感知侧目标流（PoseStamped 世界系）。
# 不设 TARGET_TOPIC ⇒ 跟随关闭，行为与旧版完全一致。
TARGET_TOPIC="${TARGET_TOPIC:-}"
TARGET_ENTER="${TARGET_ENTER:-12.0}"
TARGET_RESUME="${TARGET_RESUME:-10.0}"
RT="${RT:-_radar_test}"

echo "=== 在线建图模式：机上零文件，雷达自己建图 ==="
echo "=== 场地（只给 gzserver）: $WORLD ==="
[ -s "$WORLD" ] || { echo "WORLD_EMPTY / 缺失"; exit 1; }

# --- 前置检查：三个模块必须同目录 ---
for f in radar_avoid.py online_planner.py occupancy_online.py check_occ_topic.py; do
  [ -f "$HERE/$RT/$f" ] || { echo "缺文件: $HERE/$RT/$f"; exit 2; }
done
grep -q "online-map" "$HERE/$RT/radar_avoid.py" || {
  echo "radar_avoid.py 里没有 --online-map（版本太旧，需重新同步）"; exit 2; }
[ -f "$HERE/_demo2026/astar_plan.py" ] || [ -n "$RADAR_DEMO_DIR" ] || {
  echo "缺 $HERE/_demo2026/astar_plan.py（或设 RADAR_DEMO_DIR）"; exit 2; }

pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[g]zclient' 2>/dev/null
pkill -9 -f '[b]in/px4' 2>/dev/null; pkill -9 -f '[m]avros_node' 2>/dev/null
pkill -9 -f '[r]osmaster' 2>/dev/null; pkill -9 -f '[r]oslaunch' 2>/dev/null
pkill -9 -f '[m]ultirotor_communication' 2>/dev/null
sleep 5
source "$HERE/env_robocup.sh" >/dev/null 2>&1

echo "--- 起场地 ---"
setsid nohup roslaunch "$HERE/world_only.launch" world:="$WORLD" gui:=false \
    > /tmp/fo_world.log 2>&1 </dev/null &
disown
for i in $(seq 1 80); do
  if pgrep -f '[g]zserver' >/dev/null 2>&1 && \
     timeout 10 rosservice list 2>/dev/null | grep -q '/gazebo/get_model_state'; then
    echo "gzserver 就绪（第 $((i*3)) 秒）"; break
  fi
  sleep 3
done
pgrep -f '[g]zserver' >/dev/null || { echo "WORLD_FAIL"; tail -10 /tmp/fo_world.log; exit 1; }
GZPID=$(pgrep -f '[g]zserver' | head -1)
[ -n "$GZPID" ] && taskset -pc 0-5 "$GZPID" >/dev/null 2>&1

echo "--- 起 PX4+MAVROS 出生点=($X,$Y) ---"
setsid nohup roslaunch "$HERE/uav_lidar_clean.launch" x:="$X" y:="$Y" \
    > /tmp/fo_uav.log 2>&1 </dev/null &
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

# 🔴 10-03 WSL 实测：MAVROS connected ≠ PX4 可解锁。起场后 PX4 需要
#   ≳2 分钟才接受 OFFBOARD（实测抢跑：arm 被接受但 OFFBOARD 被拒，
#   回落 AUTO.LOITER 后自动上锁，飞机全程没离地 ⇒ 假成功飞航线）。
echo "--- 等 PX4 就绪（120s；实测 <2min 抢跑 OFFBOARD 会被拒）---"
sleep 120

# 观察在线图话题（判活用；不影响算法）
# ⚠ `rostopic hz/echo` 在 use_sim_time=true 下**抓不到数据**（会假报
#   "no new messages"）⇒ 必须用 Python 订阅。这也是协同层拿图的通道，
#   所以要实测，不能靠推理。
rm -f /tmp/fo_topic.log
setsid nohup python3 check_occ_topic.py /typhoon_h480_0/online_map/grid 600 \
  > /tmp/fo_topic.log 2>&1 </dev/null &
HZ_PID=$!
sleep 2

FOLLOW_ARGS=()
if [ -n "$TARGET_TOPIC" ]; then
  FOLLOW_ARGS+=(--target-topic "$TARGET_TOPIC"
                --target-enter "$TARGET_ENTER"
                --target-resume "$TARGET_RESUME")
fi

echo "--- 实飞（在线建图：只给起点+目标，alt=$ALT，重规划周期 ${PERIOD}s）---"
if [ -n "$TARGET_TOPIC" ]; then
  echo "--- 目标跟随：topic=$TARGET_TOPIC enter=${TARGET_ENTER}m resume=${TARGET_RESUME}s ---"
fi
cd "$HERE/$RT"
timeout 3600 python3 -u radar_avoid.py --uav typhoon_h480_0 --pose-source gazebo \
  --start "$X" "$Y" --goal "$GX" "$GY" --online-map \
  --replan-period "$PERIOD" --alt "$ALT" --speed "$SPEED" \
  "${FOLLOW_ARGS[@]}" \
  --verbose --hold 10 > /home/guo/rf_online.log 2>&1
echo "ONLINE_FLIGHT_DONE rc=$?"
kill $HZ_PID 2>/dev/null; sleep 1

echo "--- ① 确认零文件（不得出现任何文件障碍源）---"
echo -n "  障碍源日志行数(应为0): "; grep -c '障碍源' /home/guo/rf_online.log
grep -o 'A\* 规划.*' /home/guo/rf_online.log | head -2
echo -n "  在线建图模式确认: "; grep -c '在线建图模式' /home/guo/rf_online.log
echo -n "  初始航点数(应为1=只有目标点): "; grep -o '航点数 = [0-9]*' /home/guo/rf_online.log | head -1

echo "--- ② 在线建图工作证据 ---"
echo -n "  在线图状态行数: "; grep -c 'ONLINE-OCC' /home/guo/rf_online.log
grep -o 'ONLINE-OCC.*' /home/guo/rf_online.log | tail -3 | sed 's/^/  /'
echo -n "  重规划触发次数: "; grep -c '在线重规划' /home/guo/rf_online.log
grep -o 'REPLAN.*' /home/guo/rf_online.log | tail -2 | sed 's/^/  /'
echo "  重规划明细（前 6 条）:"
grep -o '在线重规划：.*' /home/guo/rf_online.log | head -6 | sed 's/^/    /'

echo "--- ③ 飞行结果（只读飞行日志，不读任何地图文件）---"
GX="$GX" GY="$GY" python3 - <<'PY'
import math, re, os
gx, gy = float(os.environ['GX']), float(os.environ['GY'])
pts = []
for ln in open('/home/guo/rf_online.log', errors='ignore'):
    m = re.search(r'pos=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)', ln)
    if m:
        pts.append(tuple(float(m.group(i)) for i in (1, 2, 3)))
if len(pts) < 2:
    print('  样本不足 (%d)' % len(pts)); raise SystemExit
sx, sy, _ = pts[0]; ex, ey, ez = pts[-1]
print('  样本 %d 点' % len(pts))
print('  起点=(%.2f,%.2f) 终点=(%.2f,%.2f)' % (sx, sy, ex, ey))
print('  终点距目标=%.2f m' % math.hypot(ex - gx, ey - gy))
print('  最低高度=%.2f m  最高高度=%.2f m' % (min(p[2] for p in pts), max(p[2] for p in pts)))
print('  航程(累计)=%.2f m  直线距离=%.2f m'
      % (sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
             for i in range(len(pts)-1)), math.hypot(gx-sx, gy-sy)))
stall, run = 0, 0
for i in range(1, len(pts)):
    if math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) < 0.01:
        run += 1
        if run == 20: stall += 1
    else:
        run = 0
print('  停滞事件(连续20帧位移<1cm)=%d' % stall)
print('  ★ 高度红线检查: %s' % ('PASS（全程 <6.0m）'
      if max(p[2] for p in pts) < 6.0 else '★FAIL 超过 6.0m ⇒ 官方记 0 分'))
PY
echo "--- ④ 到达与避障层统计 ---"
if [ -n "$TARGET_TOPIC" ]; then
  echo -n "  目标接管/恢复次数: "
  grep -o '目标接管 state=[A-Z]*\|目标丢失超时' /home/guo/rf_online.log | sort | uniq -c
fi
grep -o '航点 [0-9]* 完成\|全部到达\|超时' /home/guo/rf_online.log | sort | uniq -c
grep -o 'blocked=[A-Za-z]*' /home/guo/rf_online.log | sort | uniq -c
grep -o 'obs=[0-9]*' /home/guo/rf_online.log | sort -t= -k2 -n | uniq -c | tail -3
echo "--- ⑤ 在线图话题（协同层接口）实测 ---"
grep -a '\[chk\]' /tmp/fo_topic.log 2>/dev/null | tail -8 | sed 's/^/  /'
echo "--- ⑥ 判活心跳（每 10s 应有一行 ONLINE-OCC）---"
grep -o '\[ONLINE-OCC\] 帧=[0-9]*' /home/guo/rf_online.log | sed 's/^/  /' | tail -5
