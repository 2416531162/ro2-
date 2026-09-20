#!/bin/bash
set -e
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/ros_env.sh"
cd -- "$DIR"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec "$RK3588_PYTHON" -u radar_web_server.py "$@"
