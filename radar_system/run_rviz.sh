#!/bin/bash
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec rviz2 -d "${ROOT}/config/rk3588_3d.rviz" "$@"
