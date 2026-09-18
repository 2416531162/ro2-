#!/bin/bash
# RGB-D frames are used only for person tracking; no point-cloud generation.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ./ros_env.sh
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec ros2 launch openni2_camera camera_only.launch.py "$@"
