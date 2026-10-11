#!/usr/bin/env bash
# Development simulation; observer truth is never connected to the controller.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ "${BOTSORT_TRACKING_DEV:-0}" = 1 ]; then
    export PX4_ROOT=/root/robo_team_build/botsort_sim_20261011/environment_tracking_dev/PX4_Firmware
    export XTDRONE_DIR=/root/robo_team_build/botsort_sim_20261011/environment_tracking_dev/XTDrone
    export RUN_DIR=/root/robo_team_build/botsort_sim_20261011/logs_tracking_dev
    export ROBOCUP_TRIAL_KIND=tracking_without_actor_collisions
fi
if [ "${BOTSORT_DISABLE_SHADOWS:-0}" = 1 ]; then
    export PX4_ROOT=/root/robo_team_build/botsort_sim_20261011/environment_tracking_no_shadows/PX4_Firmware
    export XTDRONE_DIR=/root/robo_team_build/botsort_sim_20261011/environment_tracking_no_shadows/XTDrone
    export RUN_DIR=/root/robo_team_build/botsort_sim_20261011/logs_tracking_no_shadows
    export ROBOCUP_TRIAL_KIND=tracking_no_shadows
fi
source "$ROOT/coordination/scripts/container/botsort_sim_env.sh"
export GAZEBO_GUI=false
export PR_IMAGE_TRACKER=botsort
export ROBOCUP_LOG_DIR="$RUN_DIR/csv"
mkdir -p "$RUN_DIR"
case "${1:-start}" in
    start)
        bash "$ROOT/run_match.sh" start --no-gui
        ;;
    stop)
        bash "$ROOT/run_match.sh" stop
        ;;
    status)
        bash "$ROOT/run_match.sh" status
        ;;
    observer)
        python -u "$ROOT/tools/record_botsort_trial.py" \
            --out "$(readlink -f "$RUN_DIR/latest")/observer" --master-port 11345 --duration "${BOTSORT_TRIAL_WALL_S:-900}" \
            --algorithm-only --image-period 2.0 \
            --burst-image-period 0.2 --burst-duration 60 --burst-on-report \
            --max-burst-frames 350 --max-image-mb 512
        ;;
    *) echo 'start | stop | status | observer' >&2; exit 2;;
esac
