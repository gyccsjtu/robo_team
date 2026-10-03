#!/usr/bin/env bash
# Platform city + six physical aircraft + real perception + platform judge.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_file="${COORD_WS_SETUP:-/root/robo_team_build/codex_authority_v2/devel/setup.bash}"
set +u
source "$setup_file"
set -u
export DISPLAY="${DISPLAY:-:0}"
export PYTHONDONTWRITEBYTECODE=1
run_root="${CITY_RUN_ROOT:-/root/robocup_runs/city_$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p "$run_root"
python3 "$repo_dir/coordination/scripts/prepare_competition_scene.py" \
    --output "$run_root/scene" --seed "${CITY_SEED:-17}"
exec python3 "$repo_dir/coordination/scripts/six_radar_connectivity.py" \
    --output "$run_root/flight" --city-scene "$run_root/scene" \
    --flight-seconds "${CITY_SECONDS:-600}" "$@"
