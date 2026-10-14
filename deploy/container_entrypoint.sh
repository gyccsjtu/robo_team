#!/usr/bin/env bash
set -Eeuo pipefail
source /opt/robocup/algorithm/deploy/player_env.sh
if [ "${1:-hold}" != hold ]; then exec "$@"; fi
trap '/opt/robocup/algorithm/stop_player.sh || true' TERM INT
sleep infinity &
wait $! || true
