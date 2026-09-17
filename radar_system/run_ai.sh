#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
exec /usr/bin/python3 -u person_pose_node.py "$@"
