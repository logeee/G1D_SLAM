#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
exec python3 scripts/x_nav_ros1_compat_node.py "$@"
