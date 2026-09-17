#!/bin/bash
# RGB-D frames are used only for person tracking; no point-cloud generation.
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec ros2 launch openni2_camera camera_only.launch.py "$@"
