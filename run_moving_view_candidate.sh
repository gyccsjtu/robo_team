#!/usr/bin/env bash
# Opt-in observation correction with passive matched-report evidence.
# The tested baseline remains available through run_stable_6.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COUPLED_VIEW_GEOMETRY=1 REPORT_FLOW_DIAGNOSTICS=1
exec "$ROOT/run_stable_6.sh" "$@"
