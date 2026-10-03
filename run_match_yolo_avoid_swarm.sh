#!/usr/bin/env bash
# Historical official-scene launcher. Current isolated radar product:
# bash run_radar_swarm.sh --output /tmp/new_run --flight-seconds 60 --obstacle-fixture
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAZEBO_GUI="${GAZEBO_GUI:-true}"
export SKIP_RADAR="${SKIP_RADAR:-0}"
export RADAR_GUARD=1
export YOLO_BRIDGE=1
export D2O_ENABLE=0
export RADAR_HANDOFF=stop
export PR_SENSOR_PROFILE="${PR_SENSOR_PROFILE:-cgo3}"
cd "$repo_dir"
bash ./run_match.sh preflight
exec bash ./run_match.sh start
