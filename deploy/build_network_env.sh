#!/usr/bin/env bash
# BuildKit mounts the managed environment CA only for the current RUN.
if [[ -f /run/secrets/proxy_ca ]]; then
  cat /etc/ssl/certs/ca-certificates.crt /run/secrets/proxy_ca > /tmp/player-build-ca.pem
  export PIP_CERT=/tmp/player-build-ca.pem SSL_CERT_FILE=/tmp/player-build-ca.pem
  export REQUESTS_CA_BUNDLE=/tmp/player-build-ca.pem
  PLAYER_APT_CA=(-o Acquire::https::CaInfo=/tmp/player-build-ca.pem)
  trap 'rm -f /tmp/player-build-ca.pem' EXIT
else
  PLAYER_APT_CA=()
fi
