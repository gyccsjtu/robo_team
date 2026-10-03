#!/usr/bin/env bash
# Runnable bounded development product; not an official competition scene.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup_file="${COORD_WS_SETUP:-/root/robo_team_build/codex_authority_v2/devel/setup.bash}"
if [[ ! -f "$setup_file" ]]; then
    echo "Missing catkin setup: $setup_file; run coordination/scripts/build_isolated_swarm.sh" >&2
    exit 2
fi
# ROS setup scripts may reference unset variables.
set +u
source "$setup_file"
set -u
exec python3 "$repo_dir/coordination/scripts/six_radar_connectivity.py" "$@"
