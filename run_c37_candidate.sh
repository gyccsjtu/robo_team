#!/usr/bin/env bash
# Controlled sculpture-verifier trial; the baseline primary and speeds stay pinned.
set -euo pipefail
CANDIDATE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PR_DISTRACTOR_MODE="${PR_DISTRACTOR_MODE:-enforce}"
export PR_DISTRACTOR_WEIGHTS="$CANDIDATE_ROOT/weights/distractor_verifier_v7.pt"
case "${1:-start}" in
    start|start-team|preflight)
        if [[ "$PR_DISTRACTOR_MODE" != off ]] && {
            [[ ! -f "$PR_DISTRACTOR_WEIGHTS" ]] ||
            [[ "$(sha256sum -- "$PR_DISTRACTOR_WEIGHTS" | cut -d ' ' -f 1)" != dd5463e0c0fd15791f6e1e49ae5311f81410e47d51d9ff7da84e0185f8be394a ]];
        }; then
            echo '[candidate] sculpture verifier missing or checksum mismatch.' >&2
            exit 1
        fi
        echo "[candidate] primary=v3_clean secondary=v7 mode=$PR_DISTRACTOR_MODE; three-image veto."
        ;;
esac
exec "$CANDIDATE_ROOT/run_c37_baseline.sh" "$@"
