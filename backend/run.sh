#!/usr/bin/env bash
set -eo pipefail

# Run from the repo root so relative data/ paths (maps, nav points, fault logs)
# resolve exactly like the original scripts/base_sensor_visual_server.py.
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/foxy/setup.bash
source /unitree/module/slamware_service_pc4/install/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# IMPORTANT: use the system python3.8 that owns the Foxy rclpy C-extension.
# (The interactive shell's `python3` may be a conda env without rclpy._rclpy.)
# fastapi/uvicorn/pydantic are installed for this interpreter via `pip install --user`.
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

exec "$PYTHON_BIN" -m backend.app.main --bind 0.0.0.0 --port 18090 "$@"
