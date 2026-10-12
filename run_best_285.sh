#!/usr/bin/env bash
# Restore parameters recorded by the 2026-10-12 05:13:51 official 6/6 run.
# Runtime source: 193294f; measured completion: 285.872 s, not a guarantee.
set -euo pipefail
BEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PR_WEIGHTS="$BEST_ROOT/weights/best_yolo11n_bino_v3_clean.pt"
export PR_DISTRACTOR_MODE=enforce
export PR_DISTRACTOR_WEIGHTS="$BEST_ROOT/weights/distractor_verifier_v3.pt"
export SEARCH_SEGMENTS=1 MOTION_SAFETY=1 SOUTH_PATROL=1
export BRIDGE_REPORT_POLICY=fresh CANDIDATE_VIEW_RECOVERY=1
export GIMBAL_AUX_MODE=off GIMBAL_AUX_UAVS=5
export GIMBAL_AUX_HZ=3 GIMBAL_AUX_IMGSZ=960
export GAZEBO_GUI=true
case "${1:-start}" in
    start|start-team|preflight)
        for spec in \
            'best_yolo11n_bino_v3_clean.pt:e5249e3b6310e16ac95ea20c1c39f270d6cf63390450ca1a645d175239a0b27a' \
            'distractor_verifier_v3.pt:af2aaeab9682ec958e5712f64681e51dd9a41c55d3e766d661339b971bdbf0b7'; do
            name="${spec%%:*}"; expected="${spec#*:}"
            file="$BEST_ROOT/weights/$name"
            if [[ ! -f "$file" ]] || [[ "$(sha256sum -- "$file" | cut -d ' ' -f 1)" != "$expected" ]]; then
                echo "[best-285] missing or changed recorded weight: $name" >&2
                exit 1
            fi
        done
        echo '[best-285] runtime=193294f; primary=v3_clean verifier=v3; gimbal=off; reports=fresh.'
        ;;
esac
exec "$BEST_ROOT/run_match.sh" "$@"
