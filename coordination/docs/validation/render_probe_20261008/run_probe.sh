#!/usr/bin/env bash
# Short render-path probe: real city world + one official-spec camera, no PX4,
# no coordination stack. Usage: run_probe.sh <soft|gpu> [seconds]
set +u
BACKEND=${1:-soft}
RUN_SECONDS=${2:-20}
OUT=/root/render_ab/out_$BACKEND
mkdir -p "$OUT"

export ROS_MASTER_URI=http://127.0.0.1:11411
export GAZEBO_MASTER_URI=http://127.0.0.1:11412
export DISPLAY=${DISPLAY:-:0}
export PYTHONUNBUFFERED=1
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=/root/vendor_eval/d43bac6/models:/root/third_party_official/XTDrone_official/sitl_config/models:/root/third_party_official/PX4-Autopilot/Tools/sitl_gazebo/models:/root/robocup_resources/vm_gazebo_cache_20261002/models
RUNTIME=/root/robocup_runtime/stereo_20261002T140417Z
export LD_LIBRARY_PATH=$RUNTIME/gazebo:$RUNTIME/gps:$RUNTIME/actor_lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/root/robo_team_build/codex_authority_v2/devel/lib:/opt/ros/noetic/lib
export GAZEBO_PLUGIN_PATH=$LD_LIBRARY_PATH
export PATH=/opt/ros/noetic/bin:$PATH
source /opt/ros/noetic/setup.bash >/dev/null 2>&1

if [ "$BACKEND" = "gpu" ]; then
  unset LIBGL_ALWAYS_SOFTWARE
else
  export LIBGL_ALWAYS_SOFTWARE=1
fi
echo "=== backend=$BACKEND  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>} ==="

echo "--- starting roscore on 11411 ---"
roscore -p 11411 > "$OUT/roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 6
rosparam set /use_sim_time true 2>/dev/null && echo "use_sim_time=true"

echo "--- starting gzserver (probe.world) ---"
WORLD=${PROBE_WORLD:-/root/render_ab/probe.world}
echo "world=$WORLD"
gzserver --verbose -s libgazebo_ros_api_plugin.so "$WORLD" \
  > "$OUT/gzserver.log" 2>&1 &

# Optional CPU contention, to emulate the full stack (6x PX4 + 6x perception
# + observers) competing with gzserver. PROBE_LOAD=<n> burns n cores.
LOAD_PIDS=""
LOAD=${PROBE_LOAD:-0}
if [ "$LOAD" != "0" ]; then
  echo "--- starting $LOAD cpu-load processes ---"
  for _ in $(seq 1 "$LOAD"); do
    python3 -c "while True: pass" &
    LOAD_PIDS="$LOAD_PIDS $!"
  done
  echo "load pids:$LOAD_PIDS"
fi
GZ_PID=$!

echo "--- waiting for /probe/camera/image_raw ---"
UP=no
for i in $(seq 1 90); do
  if rostopic list 2>/dev/null | grep -q "probe/camera/image_raw"; then
    echo "topic up after $((i*2))s"; UP=yes; break
  fi
  if ! kill -0 $GZ_PID 2>/dev/null; then echo "gzserver died early"; break; fi
  sleep 2
done
rostopic list > "$OUT/topics.txt" 2>/dev/null
if [ "$UP" = "no" ]; then
  echo "!! camera topic never appeared; gzserver log tail:"
  tail -25 "$OUT/gzserver.log"
  kill -TERM $GZ_PID 2>/dev/null; kill -TERM $ROSCORE_PID 2>/dev/null
  exit 1
fi

sleep 5
echo "--- measuring ${RUN_SECONDS}s ---"
python3 /mnt/d/a/.robocup/tmp/render_ab/measure.py --out "$OUT" --seconds "$RUN_SECONDS"

echo "--- cleanup ---"
kill -TERM $GZ_PID 2>/dev/null; sleep 4; kill -KILL $GZ_PID 2>/dev/null
pkill -f "gzserver.*probe.world" 2>/dev/null
for p in $LOAD_PIDS; do kill -KILL "$p" 2>/dev/null; done
pkill -f "while True: pass" 2>/dev/null
kill -TERM $ROSCORE_PID 2>/dev/null; sleep 2; kill -KILL $ROSCORE_PID 2>/dev/null
sleep 2
echo "remaining probe procs: $(pgrep -fc 'probe.world' || echo 0)"
echo "=== done $BACKEND ==="
