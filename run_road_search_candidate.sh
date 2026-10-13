#!/usr/bin/env bash
# Reviewed public-road first pass and camera-measured directed coverage.
# Disable either search option explicitly to compare with the selected baseline.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SEARCH_ROAD_FIRST_PASS="${SEARCH_ROAD_FIRST_PASS:-1}"
export SEARCH_DIRECTED_COVERAGE="${SEARCH_DIRECTED_COVERAGE:-1}"
exec "$ROOT/run_tracking_continuity_candidate.sh" "$@"
