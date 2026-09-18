#!/bin/bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ./ros_env.sh
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
PYTHON_BIN="${RK3588_PYTHON:-/usr/bin/python3}"
if [ ! -x "$PYTHON_BIN" ]; then PYTHON_BIN=/usr/bin/python3; fi
exec "$PYTHON_BIN" -u radar_web_server.py "$@"
