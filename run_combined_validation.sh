#!/bin/bash
# Reversible integrated trial. Same official scene/judge/sensors as run_match.
set -euo pipefail
TEAM_TRIAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MOTION_SAFETY="${MOTION_SAFETY:-1}"
export SEARCH_SEGMENTS="${SEARCH_SEGMENTS:-1}"
export SOUTH_PATROL="${SOUTH_PATROL:-1}"
export BRIDGE_REPORT_POLICY="${BRIDGE_REPORT_POLICY:-fresh}"
export CANDIDATE_VIEW_RECOVERY="${CANDIDATE_VIEW_RECOVERY:-1}"
export GIMBAL_AUX_MODE="${GIMBAL_AUX_MODE:-off}"
export GIMBAL_AUX_UAVS="${GIMBAL_AUX_UAVS:-5}"
export PR_WEIGHTS="${PR_WEIGHTS:-$TEAM_TRIAL_ROOT/weights/best_yolo11n_bino_v3_clean.pt}"
export PR_DISTRACTOR_MODE="${PR_DISTRACTOR_MODE:-enforce}"
export PR_DISTRACTOR_WEIGHTS="${PR_DISTRACTOR_WEIGHTS:-$TEAM_TRIAL_ROOT/weights/distractor_verifier_v3.pt}"
exec "$TEAM_TRIAL_ROOT/run_match.sh" "$@"
