#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

source /opt/ros/foxy/setup.bash
source /unitree/module/slamware_service_pc4/install/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec python3 scripts/base_sensor_visual_server.py --bind 0.0.0.0 --port 18083 "$@"
