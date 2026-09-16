#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export SENSOR_TF_CALIBRATED="${SENSOR_TF_CALIBRATED:-1}"
export DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth_raw/image}"
export DEPTH_INFO_TOPIC="${DEPTH_INFO_TOPIC:-/camera/depth_raw/camera_info}"
exec /usr/bin/python3 -u cloud_web.py "$@"
