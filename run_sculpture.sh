#!/usr/bin/env bash
# Retain the accepted c37 trial settings; replace only the primary detector.
# Original five-class preset remains in RoboCup_backups/exact_c37_020158_20261012.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PR_WEIGHTS="$ROOT/weights/best_yolo11n_bino_sculpture_v3.pt"
export SEARCH_SEGMENTS=1 MOTION_SAFETY=1 SOUTH_PATROL=1
export BRIDGE_REPORT_POLICY=fresh
export PR_DISTRACTOR_MODE=off GIMBAL_AUX_MODE=off FAST_VISUAL_YAW=0
export CANDIDATE_VIEW_RECOVERY=0 GAZEBO_GUI=true
EXPECTED='af2aaeab9682ec958e5712f64681e51dd9a41c55d3e766d661339b971bdbf0b7'
ACTUAL=$(sha256sum "$PR_WEIGHTS")
if [ "${ACTUAL%% *}" != "$EXPECTED" ]; then
    echo 'Primary person/sculpture weights do not match the selected model' >&2
    exit 1
fi
cd "$ROOT"
echo '[c37-sculpture] single six-class primary; search_segments=1 motion_safety=1 south_patrol=1 reports=fresh'
exec "$ROOT/run_match.sh" "$@"
