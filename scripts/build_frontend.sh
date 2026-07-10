#!/usr/bin/env bash
# Build the Vue frontend into frontend/dist. Used both interactively and as the
# systemd ExecStartPre for g1d-slam-dashboard.service so the served dist/ is
# always rebuilt from the current source on (re)start.
#
# systemd runs services with a bare environment (no nvm, minimal PATH), so we
# source nvm here to resolve node/npm. If node is unavailable or the build fails
# we exit non-zero; the service unit prefixes this with '-' so startup still
# proceeds and serves the previously built dist/.
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO_ROOT="$(pwd)"

export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
# shellcheck disable=SC1090
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" >/dev/null 2>&1 || true

if ! command -v npm >/dev/null 2>&1; then
  echo "[build_frontend] ERROR: npm not found (nvm at '$NVM_DIR' not usable)." >&2
  exit 1
fi

echo "[build_frontend] node=$(command -v node) npm=$(command -v npm)"

if [ ! -d "$REPO_ROOT/frontend/node_modules" ]; then
  echo "[build_frontend] installing frontend deps (first run)..."
  ( cd "$REPO_ROOT/frontend" && npm install --no-audit --no-fund )
fi

echo "[build_frontend] building frontend -> frontend/dist ..."
( cd "$REPO_ROOT/frontend" && npm run build )
echo "[build_frontend] done."
