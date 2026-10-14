#!/usr/bin/env bash
set -Eeuo pipefail
PLAYER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$PLAYER_ROOT/deploy/player_env.sh"
exec bash "$PLAYER_ROOT/run_match.sh" stop-team
