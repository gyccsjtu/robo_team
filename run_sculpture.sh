#!/usr/bin/env bash
# Retain c37 trial settings while keeping v3 actor detection as the primary.
# Original five-class preset remains in RoboCup_backups/exact_c37_020158_20261012.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PR_WEIGHTS="$ROOT/weights/best_yolo11n_bino_v3_clean.pt"
export PR_DISTRACTOR_WEIGHTS="$ROOT/weights/best_yolo11n_bino_sculpture_v3.pt"
export SEARCH_SEGMENTS=1 MOTION_SAFETY=1
export NORMAL_PARTITION_SEARCH="${NORMAL_PARTITION_SEARCH:-1}"
export SOUTH_PATROL="${SOUTH_PATROL:-0}"
export SOUTH_PATROL_SOUTHEAST="${SOUTH_PATROL_SOUTHEAST:-0}"
export SOUTH_PATROL_NORTHEAST="${SOUTH_PATROL_NORTHEAST:-0}"
export EAST_PATROL="${EAST_PATROL:-0}"
export EAST_SPLIT_PATROL="${EAST_SPLIT_PATROL:-0}"
export EDGE_SEARCH_REVIEW="${EDGE_SEARCH_REVIEW:-0}"
export PRIORITY_CORNER="${PRIORITY_CORNER:-0}" HOT_TARGET_ENABLE=0
export BUILDING_SUBPOINT_REVIEW="${BUILDING_SUBPOINT_REVIEW:-1}"
export EARLY_VIEW_PURSUIT="${EARLY_VIEW_PURSUIT:-1}"
export PIXEL_VIEW_RECOVERY="${PIXEL_VIEW_RECOVERY:-1}"
export PIXEL_RECOVERY_APPROACH="${PIXEL_RECOVERY_APPROACH:-1}"
export TAKEOFF_REQUIRE_CAMERA=1
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
echo "[c37-sculpture] v3 actor primary + same-frame sculpture verifier; normal_partition=$NORMAL_PARTITION_SEARCH search_segments=1 motion_safety=1 south_patrol=$SOUTH_PATROL east_persistent=$EAST_PATROL east_split=${EAST_SPLIT_PATROL:-0} edge_review=${EDGE_SEARCH_REVIEW:-0} building_review=$BUILDING_SUBPOINT_REVIEW pixel_recovery=${PIXEL_VIEW_RECOVERY:-0} pixel_approach=${PIXEL_RECOVERY_APPROACH:-0} southeast_inner=$SOUTH_PATROL_SOUTHEAST northeast_inner=$SOUTH_PATROL_NORTHEAST reports=fresh flight_inset=$FLIGHT_INSET orbit_radius_close=$ORBIT_RADIUS_CLOSE"
exec "$ROOT/run_match.sh" "$@"
