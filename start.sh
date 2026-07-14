#!/usr/bin/env bash
# One-shot launcher for the G1D SLAM dashboard (frontend + backend).
#
#   ./start.sh            # prod: build the Vue app, then run the FastAPI backend
#                         #       which serves dist/ AND the API on :18083 (single port)
#   ./start.sh dev        # dev:  run backend (:18083) + Vite dev server (:18089)
#                         #       concurrently; Ctrl+C stops both cleanly
#
# Env overrides: BACKEND_PORT (18083), FRONTEND_PORT (18089), BIND (0.0.0.0)
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
REPO_ROOT="$(pwd)"

MODE="${1:-prod}"
BIND="${BIND:-0.0.0.0}"
BACKEND_PORT="${BACKEND_PORT:-18083}"
FRONTEND_PORT="${FRONTEND_PORT:-18089}"

# Make nvm's node/npm available (frontend build/dev needs it).
export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
# shellcheck disable=SC1090
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" >/dev/null 2>&1 || true

require_node() {
  if ! command -v npm >/dev/null 2>&1; then
    echo "[start] ERROR: npm not found. Install Node.js (or fix nvm) and retry." >&2
    exit 1
  fi
}

ensure_deps() {
  if [ ! -d "$REPO_ROOT/frontend/node_modules" ]; then
    echo "[start] installing frontend deps (first run)..."
    ( cd "$REPO_ROOT/frontend" && npm install --no-audit --no-fund )
  fi
}

case "$MODE" in
  prod)
    require_node
    ensure_deps
    echo "[start] building frontend -> frontend/dist ..."
    ( cd "$REPO_ROOT/frontend" && npm run build )
    echo "[start] launching backend on ${BIND}:${BACKEND_PORT} (serves dist/ + /api)"
    exec bash "$REPO_ROOT/backend/run.sh" --bind "$BIND" --port "$BACKEND_PORT"
    ;;

  dev)
    require_node
    ensure_deps
    pids=()
    cleanup() {
      echo
      echo "[start] stopping..."
      for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
      wait 2>/dev/null || true
    }
    trap cleanup INT TERM EXIT

    echo "[start] backend  -> ${BIND}:${BACKEND_PORT}"
    bash "$REPO_ROOT/backend/run.sh" --bind "$BIND" --port "$BACKEND_PORT" &
    pids+=($!)

    echo "[start] frontend -> ${BIND}:${FRONTEND_PORT} (proxies /api to backend)"
    ( cd "$REPO_ROOT/frontend" && npm run dev -- --host "$BIND" --port "$FRONTEND_PORT" ) &
    pids+=($!)

    echo "[start] both running. Ctrl+C to stop."
    # Block until one child exits; the EXIT trap then tears the other down.
    wait -n 2>/dev/null || true
    ;;

  *)
    echo "usage: $0 [prod|dev]" >&2
    exit 2
    ;;
esac
