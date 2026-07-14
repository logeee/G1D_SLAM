#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

export PYTHONUNBUFFERED=1
if [ -x /home/unitree/miniconda3/envs/tv/bin/python ]; then
  PYTHON_BIN=/home/unitree/miniconda3/envs/tv/bin/python
elif [ -x /home/unitree/venvs/tv_gpu/bin/python ]; then
  PYTHON_BIN=/home/unitree/venvs/tv_gpu/bin/python
else
  PYTHON_BIN=python3
fi

exec "$PYTHON_BIN" scripts/g1d_lift_height_service.py "$@"
