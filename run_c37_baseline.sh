#!/usr/bin/env bash
# Reproduce the recorded 2026-10-12 02:02:11 run configuration.
# Keep the original c37b916 run_match.sh; subsequent fixes have separate commits.
set -euo pipefail
BASELINE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PR_WEIGHTS="$BASELINE_ROOT/artifacts/yolo_remote_audit_20261011/weights/best_yolo11n_bino_v3_clean.pt"
export SEARCH_SEGMENTS=1 MOTION_SAFETY=1 SOUTH_PATROL=1
export BRIDGE_REPORT_POLICY=fresh
export FAST_VISUAL_YAW=0 PR_FAST_TRACKER_SHADOW=0
export PR_DETECT_EVERY=2 PR_COORD_HZ=4 PR_PUB_HZ=10
export PR_HEIGHT_CALIBRATE=1 PR_GROUND_BODY_HEIGHT=0.22
export ORBIT_RADIUS_CLOSE=8.0 CLOSE_TRANSIT_SPEED=0.95
export BRIDGE_PUB_MAX_RANGE=9.5 BRIDGE_PUB_HOLD_RANGE=15.0
export GAZEBO_GUI=true

case "${1:-start}" in
    start|start-team|preflight)
        if [[ ! -f "$PR_WEIGHTS" ]] ||
           [[ "$(sha256sum -- "$PR_WEIGHTS" | cut -d ' ' -f 1)" != e5249e3b6310e16ac95ea20c1c39f270d6cf63390450ca1a645d175239a0b27a ]]; then
            echo '[baseline] v3_clean weight missing or checksum differs from the recorded run.' >&2
            exit 1
        fi
        echo '[baseline] c37b916 run preset: v3_clean, fresh reports, segments + motion safety + south patrol.'
        ;;
esac
exec "$BASELINE_ROOT/run_match.sh" "$@"
