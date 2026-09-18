#!/usr/bin/env bash
# ROS Jazzy's setup.bash references optional variables while it is being
# sourced.  Enable nounset only after the ROS environment has loaded.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export RK3588_ROBOT_CONFIG="${RK3588_ROBOT_CONFIG:-$ROOT/robot_core/robot.json}"
export RK3588_MANAGED_FOLLOWER=1
case "${1:-}" in
  chassis) exec /usr/bin/python3 -u "$ROOT/wheeltec_protocol/wheeltec_driver.py" --ros-args --params-file "$ROOT/wheeltec_protocol/wheeltec.yaml" ;;
  follower) exec /usr/bin/python3 -u "$ROOT/radar_system/person_follower.py" --passive ;;
  camera|lidar|ai|web|gui) exec bash "$ROOT/radar_system/start_component.sh" "$1" ;;
  *) echo 'Expected chassis|follower|camera|lidar|ai|web|gui' >&2; exit 2 ;;
esac
