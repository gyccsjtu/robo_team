#!/usr/bin/env bash
# Team-only ROS environment. Do not source Gazebo/actor startup scripts here.
PLAYER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOCUP_TEAM_ONLY=1 ROBOCUP_PLAYER_IMAGE=1
export ROBOCUP_ROS_SETUP=/opt/ros/noetic/setup.bash
export ROBOCUP_WS_SETUP="$PLAYER_ROOT/coordination/devel/setup.bash"
export COORD_WS_SETUP="$ROBOCUP_WS_SETUP"
export PR_WEIGHTS="$PLAYER_ROOT/weights/best_yolo11n_bino_v3_clean.pt"
export PR_DISTRACTOR_WEIGHTS="$PLAYER_ROOT/weights/best_yolo11n_bino_sculpture_v3.pt"
export PR_DISTRACTOR_MODE="${PR_DISTRACTOR_MODE:-enforce}"
export RUN_DIR="${RUN_DIR:-/var/log/robocup}"
export ROBOCUP_LOG_DIR="${ROBOCUP_LOG_DIR:-$RUN_DIR/csv}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$RUN_DIR/ultralytics}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$RUN_DIR/matplotlib}"
export YOLO_AUTOINSTALL=false PYTHONDONTWRITEBYTECODE=1
export TEAM_TIME_SOURCE=ros_clock CLOCK_PERMISSION=allowed
export CLOCK_PERMISSION_REFERENCE='用户确认 /clock 可用'
export ROBOCUP_RUNTIME_BASELINE=e77c3a2cd72ff25c237a12728f29b852f5c14d0e
export ROBOCUP_SOURCE_COMMIT="$(cat "$PLAYER_ROOT/deploy/source_commit.txt")"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
for PLAYER_SETUP in "$ROBOCUP_ROS_SETUP" "$ROBOCUP_WS_SETUP"; do
  if [ ! -f "$PLAYER_SETUP" ]; then
    printf '[player] ROS setup missing: %s\n' "$PLAYER_SETUP" >&2
    return 2 2>/dev/null || exit 2
  fi
  PLAYER_NOUNSET=0
  case $- in *u*) PLAYER_NOUNSET=1; set +u ;; esac
  source "$PLAYER_SETUP"
  [ "$PLAYER_NOUNSET" != 1 ] || set -u
done
export PYTHONPATH="$PLAYER_ROOT/perception:$PLAYER_ROOT/coordination/src/robocup_swarm/scripts:$PLAYER_ROOT/coordination/src/robocup_navigation/src:${PYTHONPATH:-}"
mkdir -p "$RUN_DIR" "$ROBOCUP_LOG_DIR" "$YOLO_CONFIG_DIR" "$MPLCONFIGDIR"
