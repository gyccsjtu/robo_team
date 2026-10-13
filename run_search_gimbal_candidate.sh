#!/usr/bin/env bash
# Shared low-priority CGO3 discovery -> fixed-camera handoff; baseline reversible.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COUPLED_VIEW_GEOMETRY=1 REPORT_FLOW_DIAGNOSTICS=1
export SEARCH_REVISIT_EVIDENCE=1
export GIMBAL_AUX_MODE="${GIMBAL_AUX_MODE:-handoff}"
export GIMBAL_AUX_UAVS="${GIMBAL_AUX_UAVS:-2,5}"
export GIMBAL_AUX_HZ="${GIMBAL_AUX_HZ:-3}"
export GIMBAL_AUX_IMGSZ="${GIMBAL_AUX_IMGSZ:-960}"
exec "$ROOT/run_stable_6.sh" "$@"
