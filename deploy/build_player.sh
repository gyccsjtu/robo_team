#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v docker >/dev/null || { echo 'Docker is not installed; context can still be exported with prepare_context.py' >&2; exit 2; }
docker info >/dev/null
BUILD_DIR=""
if [ -f "$ROOT/deploy/source_commit.txt" ]; then
  # Already a clean exported build directory; no git checkout is required.
  CONTEXT="$ROOT"
else
  BUILD_DIR="$(mktemp -d -t robocup-player.XXXXXX)"
  trap 'rm -rf -- "$BUILD_DIR"' EXIT
  python3 "$ROOT/deploy/prepare_context.py" --output "$BUILD_DIR/context"
  CONTEXT="$BUILD_DIR/context"
fi
SOURCE_COMMIT="$(cat "$CONTEXT/deploy/source_commit.txt")"
IMAGE="${PLAYER_IMAGE:-robocup-player:e77c3a2-${SOURCE_COMMIT:0:12}}"
DOCKER_ARGS=(--platform linux/amd64 -f "$CONTEXT/deploy/Dockerfile.player"
             --build-arg "SOURCE_COMMIT=$SOURCE_COMMIT" -t "$IMAGE")
if [ -n "${CODEX_PROXY_CERT:-}" ]; then
  DOCKER_ARGS+=(--secret "id=proxy_ca,src=$CODEX_PROXY_CERT")
fi
DOCKER_BUILDKIT=1 docker build "${DOCKER_ARGS[@]}" "$CONTEXT"
echo "Built: $IMAGE"
