#!/usr/bin/env bash
# Baseline search with bounded early takeoff recovery and routine inward viewing.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export VIEW_SPACE_BACKOFF="${VIEW_SPACE_BACKOFF:-0}"
export BRIDGE_FRESH_IMAGE_SLOT="${BRIDGE_FRESH_IMAGE_SLOT:-0}"
export SEARCH_ROAD_FIRST_PASS="${SEARCH_ROAD_FIRST_PASS:-0}"
export SEARCH_DIRECTED_COVERAGE="${SEARCH_DIRECTED_COVERAGE:-0}"
export TAKEOFF_PROGRESS_RECOVERY_EARLY="${TAKEOFF_PROGRESS_RECOVERY_EARLY:-1}"
export ROUTINE_SEARCH_BOUNDS="${ROUTINE_SEARCH_BOUNDS:-1}"
export ROUTINE_OBSERVATION_HEADING="${ROUTINE_OBSERVATION_HEADING:-1}"
exec "$ROOT/run_moving_view_candidate.sh" "$@"
