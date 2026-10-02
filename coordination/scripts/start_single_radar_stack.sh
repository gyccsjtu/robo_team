#!/bin/bash
# Reused verified WorkBuddy/Codex environment launcher. No control algorithm.
# Single aircraft prototype; generated SDF and libraries are not competition-certified.
# WorkBuddy 隔离栈 v3：typhoon_h480 + hokuyo(2D 激光) + PX4 SITL + MAVROS
# v2 → v3 修正（2026-10-03，Codex 核验后）：
#   (a) **去掉宽泛 pkill** —— 原 `pkill -f '[g]zserver'` 会杀掉用户/Codex 正在跑的仿真。
#       改为：专用 ROS master 端口 + 只清理"本脚本自己记录过的 PID"。
#   (b) 专用 ROS_MASTER_URI（默认 11345），与用户的 11311 隔离，互不影响。
#   (c) OUT 落在带 RUN_ID 的新目录，不再覆盖固定目录。
set +u
EVAL=${EVAL:-/root/vendor_eval/d43bac6}
RUNTIME=${RUNTIME:-/root/robocup_runtime/stereo_20261002T140417Z}
XT=${XT:-/root/third_party_official/XTDrone_official}
PX4=${PX4:-/root/third_party_official/PX4-Autopilot}
SRC_SDF=$EVAL/models/typhoon_h480_lidar/typhoon_h480_lidar.sdf
GPS_OVERLAY=$RUNTIME/gps
WORLD=${WORLD:-$EVAL/robocup_real/_genvm/m1/robocup.world}
GZPLUG=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)_$$
OUT=${RADAR_STACK_OUT:?Set RADAR_STACK_OUT to a new absolute directory}/$RUN_ID
MASTER_PORT=${MASTER_PORT:-11345}
GAZEBO_PORT=${GAZEBO_PORT:-11346}
export GAZEBO_MASTER_URI="http://127.0.0.1:$GAZEBO_PORT"
X=0.0; Y=-3.0
mkdir -p "$OUT"
echo "$OUT" > "$RADAR_STACK_OUT/LATEST"
echo "$RUN_ID" > "$OUT/run_id"

# Cleanup belongs to the owning Python runner; never inspect or kill another run.
: > "$OUT/pids.txt"
sleep 2

if [ "$MASTER_PORT" = "$GAZEBO_PORT" ]; then echo "ROS/Gazebo ports must differ"; exit 2; fi
if ss -H -lntu | awk '{print $5}' | grep -Eq ':(4560|14540|14560|14580)$'; then
  echo "Single-aircraft PX4/MAVROS port occupied; refusing to attach to another flight"; exit 4
fi
if ss -ltn | grep -q ":$GAZEBO_PORT "; then echo "Gazebo port occupied"; exit 2; fi

echo "########## 1. 环境 ##########"
source /opt/ros/noetic/setup.bash
source /root/robocup/robocup_ws/devel/setup.bash
export LD_LIBRARY_PATH="$RUNTIME/gazebo:$GPS_OVERLAY:$RUNTIME/actor_lib:$GZPLUG:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$RUNTIME/gazebo:$GPS_OVERLAY:/root/robocup/robocup_ws/devel/lib:$GZPLUG:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$EVAL/models:$XT/sitl_config/models:$PX4/Tools/sitl_gazebo/models:/root/robocup_resources/vm_gazebo_cache_20261002/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_PACKAGE_PATH="$PX4:$ROS_PACKAGE_PATH"
# ★ 关键：XTDrone 机型是"模型级" GPS 插件（不是嵌套 gps0 模型）。legacy mavlink 接口
#   只有在 ROBOCUP_LEGACY_GPS_MODEL=1 且模型带 <gpsSubTopic> 时才订阅 ~/<model>/gps；
#   否则退回 kDefaultGPSModelNaming（找嵌套模型）⇒ 收不到 GPS ⇒ EKF 不初始化 ⇒ 无 local_position。
export ROBOCUP_LEGACY_GPS_MODEL=1
# ★ 专用 master 端口，避免与用户的 11311 会话互相干扰
export ROS_MASTER_URI="http://127.0.0.1:$MASTER_PORT"
echo "  RUN_ID = $RUN_ID"
echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  GAZEBO_PLUGIN_PATH = $GAZEBO_PLUGIN_PATH"
echo "  libRayPlugin 可见性: $(ls $GZPLUG/libRayPlugin.so 2>/dev/null >/dev/null && echo OK || echo MISSING)"
echo "  X socket: $(ls /tmp/.X11-unix/ 2>/dev/null | tr '\n' ' ')"
[ -s "$WORLD" ] || { echo "WORLD 缺失: $WORLD"; exit 1; }
[ -s "$SRC_SDF" ] || { echo "模型缺失: $SRC_SDF"; exit 1; }
# 端口占用检查：占用则说明另一次运行还在（不杀，直接退出并提示）
if (ss -ltn 2>/dev/null | grep -q ":$MASTER_PORT ") ; then
  echo "端口 $MASTER_PORT 已被占用 —— 可能另有一次运行在跑。"
  echo "  先执行 wb_stop.sh 或在命令行指定 MASTER_PORT=其它端口，本脚本不做宽泛清理。"
  exit 4
fi

echo
echo "########## 2. roscore（专用端口）##########"
setsid nohup roscore -p $MASTER_PORT > $OUT/roscore.log 2>&1 </dev/null &
echo $! >> "$OUT/pids.txt"
disown
for i in $(seq 1 40); do timeout 3 rosparam list >/dev/null 2>&1 && { echo "roscore OK (port $MASTER_PORT)"; break; }; sleep 1; done

timeout 5 rosparam set /use_sim_time true

echo
echo "########## 3. gzserver ##########"
setsid nohup gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD" \
  > $OUT/gzserver.log 2>&1 </dev/null &
echo $! >> "$OUT/pids.txt"   # 记录 PID，供下一轮按键清理
disown
for i in $(seq 1 60); do
  timeout 5 rosservice list 2>/dev/null | grep -q '/gazebo/spawn_sdf_model' && { echo "gzserver 就绪（${i}s）"; break; }
  sleep 2
done
timeout 5 rosservice list 2>/dev/null | grep -q '/gazebo/spawn_sdf_model' || { echo "GZSERVER_FAIL"; tail -15 $OUT/gzserver.log; exit 1; }

echo
echo "########## 4. 适配 + spawn ##########"
SRC_SDF="$SRC_SDF" OUT="$OUT" GPS_OVERLAY="$GPS_OVERLAY" X="$X" Y="$Y" python3 - <<'PY'
import os, sys, xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

src, out, overlay = os.environ['SRC_SDF'], os.environ['OUT'], os.environ['GPS_OVERLAY']
x, y = float(os.environ['X']), float(os.environ['Y'])
text = open(src, encoding='utf-8').read()
notes = []
for old, new, tag in (
    ('libgazebo_gps_plugin.so', overlay + '/librobocup_legacy_model_gps_plugin.so', 'gps_plugin'),
    ('libgazebo_mavlink_interface.so', overlay + '/librobocup_legacy_gps_mavlink_interface.so', 'mavlink_interface'),
):
    if old in text: text = text.replace(old, new); notes.append('plugin %s -> legacy overlay' % tag)
if '<parent>typhoon_h480::base_link</parent>' in text:
    text = text.replace('<parent>typhoon_h480::base_link</parent>', '<parent>base_link</parent>')
    notes.append('parent typhoon_h480::base_link -> base_link')
for old, new, tag in (('<samples>640</samples>', '<samples>512</samples>', 'samples 640->512'),
                      ('<min>0.08</min>', '<min>0.5</min>', 'range_min 0.08->0.5'),
                      ('<max>10</max>', '<max>20</max>', 'range_max 10->20')):
    if old in text: text = text.replace(old, new); notes.append('lidar ' + tag)

# 相机：sensor 在 link 下（不是 model 下）——正确遍历
root = ET.fromstring(text)
m = root.find('model')
has_x = os.path.exists('/tmp/.X11-unix/X0')
ncam = 0
if not has_x:
    for link in m.findall('link'):
        for s in list(link.findall('sensor')):
            if s.get('type') in ('camera', 'depth', 'multicamera'):
                link.remove(s); ncam += 1
    if ncam: notes.append('无 X ⇒ 剥掉相机传感器 %d 个' % ncam)
text = ET.tostring(root, encoding='unicode')
open(out + '/wb_model.sdf', 'w', encoding='utf-8').write(text)
print('  适配:'); [print('   -', n) for n in notes]
print('  模型: %s/wb_model.sdf (%d B)' % (out, len(text)))

rospy.init_node('wb_spawn', anonymous=True, disable_signals=True)
rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=60)
pose = Pose(); pose.position.x = x; pose.position.y = y; pose.position.z = 0.2
pose.orientation.w = 1.0
r = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)('typhoon_h480_0', text, 'typhoon_h480_0', pose, 'world')
print('  spawn success=%s status=%s' % (r.success, r.status_message))
sys.exit(0 if r.success else 3)
PY
[ $? -eq 0 ] || { echo "SPAWN_FAIL"; tail -20 $OUT/gzserver.log; exit 1; }

echo
echo "########## 5. PX4 SITL ##########"
PX4BUILD=$PX4/build/px4_sitl_default
mkdir -p "$OUT/px4_work"
PX4_SIM_MODEL=typhoon_h480 PATH="$PX4BUILD/bin:$PATH" \
setsid nohup "$PX4BUILD/bin/px4" -d "$PX4BUILD/etc" -s etc/init.d-posix/rcS -i 0 -w $OUT/px4_work \
  > $OUT/px4.log 2>&1 </dev/null &
echo $! >> "$OUT/pids.txt"   # 记录 PID，供下一轮按键清理
disown
sleep 15
echo "  --- mavlink 实例 ---"; grep -o "mode: [A-Za-z]*,.*udp port [0-9]* remote port [0-9]*" $OUT/px4.log | head -6
# 取 4MB/s 那条 Onboard（=标准 onboard 口，留给 MAVROS）
FCU=$(grep -o "mode: Onboard, data rate: 4000000 B/s on udp port [0-9]* remote port [0-9]*" $OUT/px4.log | head -1)
LOCAL=$(echo "$FCU" | sed -n 's/.*udp port \([0-9]*\) remote port [0-9]*/\1/p')
REMOTE=$(echo "$FCU" | sed -n 's/.*remote port \([0-9]*\)$/\1/p')
LOCAL=${LOCAL:-14580}; REMOTE=${REMOTE:-14540}
URL="udp://:${REMOTE}@127.0.0.1:${LOCAL}"
echo "  探测: PX4 onboard local=$LOCAL remote=$REMOTE ⇒ fcu_url=$URL"
echo "$URL" > $OUT/fcu_url.txt

echo
echo "########## 6. MAVROS（自建 group-ns launch）##########"
cat > $OUT/wb_mavros.launch <<XML
<launch>
  <arg name="fcu_url" default="$URL"/>
  <group ns="typhoon_h480_0">
    <include file="\$(find mavros)/launch/px4.launch">
      <arg name="fcu_url" value="\$(arg fcu_url)"/>
      <arg name="gcs_url" value=""/>
      <arg name="tgt_system" value="1"/>
      <arg name="tgt_component" value="1"/>
    </include>
  </group>
</launch>
XML
setsid nohup roslaunch $OUT/wb_mavros.launch > $OUT/mavros.log 2>&1 </dev/null &
echo $! >> "$OUT/pids.txt"   # 记录 PID，供下一轮按键清理
disown
ok=0
for i in $(seq 1 30); do
  S=$(timeout 6 rostopic echo -n1 /typhoon_h480_0/mavros/state 2>/dev/null)
  if echo "$S" | grep -q 'connected: True'; then ok=1; echo "  ✅ MAVROS connected（第 ${i} 次探测）"; break; fi
  sleep 3
done
[ "$ok" = 1 ] || { echo "  ✗ MAVROS 未连上"; tail -12 $OUT/mavros.log; exit 5; }
timeout 8 rostopic echo -n1 /typhoon_h480_0/mavros/state 2>/dev/null | grep -E "armed|mode|connected"

echo
echo "########## 7. 关键话题 ##########"
timeout 8 rostopic list 2>/dev/null | grep -E "typhoon_h480_0/(scan|mavros/state)|gazebo/model_states"
echo "  插件加载错误复查:"; grep -c "Failed to load plugin" $OUT/gzserver.log
echo "  --- EKF 链路（关键：GPS fix 与 local_position）---"
for t in /typhoon_h480_0/mavros/global_position/raw/fix /typhoon_h480_0/mavros/local_position/pose /typhoon_h480_0/mavros/imu/data; do
  printf "  %-50s " "$t"; timeout 8 rostopic hz $t 2>&1 | grep -E "average rate|no new" | head -1
done

echo
echo "########## 8. 雷达体检 ##########"
timeout 40 python3 - <<'PY'
import math, time
import rospy
from sensor_msgs.msg import LaserScan
rospy.init_node('wb_scan_check', anonymous=True, disable_signals=True)
got = []
def cb(m):
    if len(got) < 12: got.append(m)
rospy.Subscriber('/typhoon_h480_0/scan', LaserScan, cb, queue_size=10)
t = time.time() + 15
while time.time() < t and len(got) < 8: time.sleep(0.2)
print("  收到帧数:", len(got))
if got:
    m = got[-1]
    fin = [r for r in m.ranges if r == r and 1e-6 < r < 1e9]
    print("  beams=%d fov=%.1f° range=[%.2f,%.2f] frame=%s"
          % (len(m.ranges), math.degrees(m.angle_increment) * len(m.ranges), m.range_min, m.range_max, m.header.frame_id))
    if fin:
        rs = sorted(fin)
        print("  有效回波 %d/%d min=%.2f median=%.2f max=%.2f" % (len(rs), len(m.ranges), rs[0], rs[len(rs)//2], rs[-1]))
        for thr in (1.0, 2.0, 5.0):
            print("    < %.1f m: %d 束" % (thr, sum(1 for r in rs if r < thr)))
    else:
        print("  ⚠ 无有效回波")
PY
echo
echo "STACK2_READY $(date -u +%Y-%m-%dT%H:%M:%SZ)"
