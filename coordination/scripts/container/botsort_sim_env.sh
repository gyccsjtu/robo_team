#!/usr/bin/env bash
# Development comparison environment only. Sourcing this file starts no simulator.
# All existing official trees and verified vision environments remain read-only.
_botsort_repo_default="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export ROBOCUP_TRACKING_WORKTREE="${ROBOCUP_TRACKING_WORKTREE:-$_botsort_repo_default}"
export ROBOCUP_ROS_SETUP="/opt/ros/noetic/setup.bash"
export ROBOCUP_WS_SETUP="/root/robocup/robocup_ws/devel/setup.bash"
export COORD_WS_SETUP="${COORD_WS_SETUP:-/root/robo_team_build/botsort_messages_20261011/devel/setup.bash}"
export PX4_ROOT="${PX4_ROOT:-/root/robo_team_build/botsort_sim_20261011/environment_v2/PX4_Firmware}"
export XTDRONE_DIR="${XTDRONE_DIR:-/root/robo_team_build/botsort_sim_20261011/environment_v2/XTDrone}"
export ROBOCUP_SIM_RUNTIME="/root/robocup_runtime/stereo_20261002T140417Z"
export ROBOCUP_ACTOR_PLUGIN="$ROBOCUP_SIM_RUNTIME/actor_lib/libros_actor_cmd_pose_plugin.so"
export PATH="/root/robo_team_build/botsort_env_20261011/bin:$PATH"
export PR_WEIGHTS="$ROBOCUP_TRACKING_WORKTREE/weights/best_yolo11n_bino_v1.pt"
export PR_DEVICE="${PR_DEVICE:-cpu}"
export PR_CUDA_VISIBLE_DEVICES="${PR_CUDA_VISIBLE_DEVICES:-}"
export YOLO_AUTOINSTALL="false"
export ROBOCUP_LEGACY_GPS_MODEL="1"
export GAZEBO_MODEL_DATABASE_URI=""
export DISPLAY="${DISPLAY:-:0}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11345}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export RUN_DIR="${RUN_DIR:-/root/robo_team_build/botsort_sim_20261011/logs}"
export GAZEBO_GUI="${GAZEBO_GUI:-true}"

# No set -u while sourcing ROS setup scripts.
_botsort_nounset=0
case "$-" in *u*) _botsort_nounset=1; set +u;; esac
source "$ROBOCUP_ROS_SETUP"
source "$ROBOCUP_WS_SETUP"
if [ -f "$COORD_WS_SETUP" ]; then source "$COORD_WS_SETUP"; fi
export PYTHONPATH="$(dirname "$COORD_WS_SETUP")/lib/python3/dist-packages:$(dirname "$ROBOCUP_WS_SETUP")/lib/python3/dist-packages:$ROBOCUP_TRACKING_WORKTREE/coordination/src/robocup_navigation/src:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$ROBOCUP_TRACKING_WORKTREE/launch:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="$XTDRONE_DIR/sitl_config/models:$PX4_ROOT/Tools/sitl_gazebo/models:/root/robocup_resources/vm_gazebo_cache_20261002/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$ROBOCUP_SIM_RUNTIME/gazebo:$ROBOCUP_SIM_RUNTIME/gps:$ROBOCUP_SIM_RUNTIME/actor_lib:/root/robocup/robocup_ws/devel/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$PX4_ROOT/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$ROBOCUP_SIM_RUNTIME/gazebo:$ROBOCUP_SIM_RUNTIME/gps:$ROBOCUP_SIM_RUNTIME/actor_lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
if [ "$_botsort_nounset" = 1 ]; then set -u; fi
unset _botsort_nounset _botsort_repo_default

if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
    case "${1:-}" in
        preflight)
            # This is the existing strict checker; no formal check is bypassed.
            exec bash "$ROBOCUP_TRACKING_WORKTREE/run_match.sh" preflight
            ;;
        probe_paths)
            source "$ROBOCUP_TRACKING_WORKTREE/perception/scripts/env_robocup.sh"
            source "$COORD_WS_SETUP"
            # Match run_match.ros_env's final package-root restoration after
            # all setup scripts have been sourced.
            export ROS_PACKAGE_PATH="$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo:$ROBOCUP_TRACKING_WORKTREE/launch:${ROS_PACKAGE_PATH:-}"
            echo "PX4 package: $(rospack find px4)"
            echo "Gazebo package: $(rospack find mavlink_sitl_gazebo)"
            echo "Python: $(command -v python3)"
            echo "Actor runtime: $ROBOCUP_ACTOR_PLUGIN"
            ;;
        *)
            echo 'Source this file to activate the development environment, or pass preflight.'
            ;;
    esac
fi
