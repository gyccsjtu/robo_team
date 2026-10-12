#!/usr/bin/env bash
# Retain c37 trial settings while keeping v3 actor detection as the primary.
# Original five-class preset remains in RoboCup_backups/exact_c37_020158_20261012.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PR_WEIGHTS="$ROOT/weights/best_yolo11n_bino_v3_clean.pt"
export PR_DISTRACTOR_WEIGHTS="$ROOT/weights/best_yolo11n_bino_sculpture_v3.pt"
export SEARCH_SEGMENTS=1 MOTION_SAFETY=1 SOUTH_PATROL=1
export SOUTH_PATROL_SOUTHEAST="${SOUTH_PATROL_SOUTHEAST:-0}"
export SOUTH_PATROL_NORTHEAST="${SOUTH_PATROL_NORTHEAST:-0}"
export EAST_PATROL="${EAST_PATROL:-1}"
export BRIDGE_REPORT_POLICY=fresh
export PR_DISTRACTOR_MODE=enforce GIMBAL_AUX_MODE=off FAST_VISUAL_YAW=0
export CANDIDATE_VIEW_RECOVERY=0 GAZEBO_GUI=true
# Keep the outer flight envelope; permit only 0.5 m more approach to its edge.
# Set FLIGHT_INSET=2.0 to restore the original internal margin.
export FLIGHT_INSET="${FLIGHT_INSET:-1.5}"
export ORBIT_RADIUS_CLOSE="${ORBIT_RADIUS_CLOSE:-8.0}"
EXPECTED='e5249e3b6310e16ac95ea20c1c39f270d6cf63390450ca1a645d175239a0b27a'
ACTUAL=$(sha256sum "$PR_WEIGHTS")
if [ "${ACTUAL%% *}" != "$EXPECTED" ]; then
    echo 'Primary v3 clean actor weights do not match the selected model' >&2
    exit 1
fi
cd "$ROOT"
echo "[c37-sculpture] v3 actor primary + same-frame sculpture verifier; search_segments=1 motion_safety=1 south_patrol=1 east_persistent=$EAST_PATROL southeast_inner=$SOUTH_PATROL_SOUTHEAST northeast_inner=$SOUTH_PATROL_NORTHEAST reports=fresh flight_inset=$FLIGHT_INSET orbit_radius_close=$ORBIT_RADIUS_CLOSE"
exec "$ROOT/run_match.sh" "$@"
