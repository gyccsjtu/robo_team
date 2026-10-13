#!/usr/bin/env bash
# Optional full-foot view recovery and qualified fresh-image publication.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export VIEW_SPACE_BACKOFF="${VIEW_SPACE_BACKOFF:-1}"
export BRIDGE_FRESH_IMAGE_SLOT="${BRIDGE_FRESH_IMAGE_SLOT:-1}"
exec "$ROOT/run_moving_view_candidate.sh" "$@"
