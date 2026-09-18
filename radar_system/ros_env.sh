#!/usr/bin/env bash
# Source the ROS 2 installation selected by ROS_DISTRO, or the first supported
# distribution present on the board. Jetson Ubuntu 22.04 normally uses Humble;
# the RK3588 Ubuntu 24.04 image uses Jazzy.
if [ -n "${ROS_DISTRO:-}" ] && [ -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  _robot_ros_distro="$ROS_DISTRO"
else
  _robot_ros_distro=""
  for _candidate in humble jazzy; do
    if [ -r "/opt/ros/${_candidate}/setup.bash" ]; then
      _robot_ros_distro="$_candidate"
      break
    fi
  done
fi

if [ -z "$_robot_ros_distro" ]; then
  echo "No supported ROS 2 installation found under /opt/ros (expected Humble or Jazzy)" >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_DISTRO="$_robot_ros_distro"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
unset _candidate _robot_ros_distro

