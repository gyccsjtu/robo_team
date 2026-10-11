#!/bin/bash
# Reversible integrated trial. Same official scene/judge/sensors as run_match.
set -euo pipefail
TEAM_TRIAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MOTION_SAFETY="${MOTION_SAFETY:-1}"
export SEARCH_SEGMENTS="${SEARCH_SEGMENTS:-1}"
export SOUTH_PATROL="${SOUTH_PATROL:-1}"
export BRIDGE_REPORT_POLICY="${BRIDGE_REPORT_POLICY:-fresh}"
export CANDIDATE_VIEW_RECOVERY="${CANDIDATE_VIEW_RECOVERY:-1}"
export GIMBAL_AUX_MODE="${GIMBAL_AUX_MODE:-handoff}"
export GIMBAL_AUX_UAVS="${GIMBAL_AUX_UAVS:-5}"
export PR_WEIGHTS="${PR_WEIGHTS:-$TEAM_TRIAL_ROOT/artifacts/yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt}"
exec "$TEAM_TRIAL_ROOT/run_match.sh" "$@"
