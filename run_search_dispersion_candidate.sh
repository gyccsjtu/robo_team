#!/usr/bin/env bash
# Search-only candidate; use SEARCH_DECONFLICTED_ALLOCATION=0 for its baseline.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SEARCH_DECONFLICTED_ALLOCATION="${SEARCH_DECONFLICTED_ALLOCATION:-1}"
exec "$ROOT/run_search_safety_candidate.sh" "$@"
