#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
exec python3 scripts/x_nav_http_compat_adapter.py --bind 0.0.0.0 --port 9000 "$@"
