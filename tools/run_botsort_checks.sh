#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/coordination/scripts/container/botsort_sim_env.sh"
export ROBOCUP_REAL_MOT=1
export ROBOCUP_LOG_DIR=/root/robo_team_build/botsort_sim_20261011/check_logs
export YOLO_CONFIG_DIR=/root/robo_team_build/botsort_sim_20261011/yolo_config
export YOLO_AUTOINSTALL=false
mkdir -p "$ROBOCUP_LOG_DIR" "$YOLO_CONFIG_DIR"
cd "$ROOT"
python -B -m unittest \
    tools.test_image_mot tools.test_tracking_identity_recovery \
    tools.test_target_motion_time tools.test_candidate_bridge \
    tools.test_candidate_perception tools.test_camera_source_continuity \
    tools.test_coordinate_time tools.test_robust_coordinate_report \
    tools.test_bridge_judge_independence tools.test_tracking_view \
    tools.test_red_camera_binding tools.test_detector_class_mapping \
    tools.test_perception_guard tools.test_cpu_inference_threads -v
