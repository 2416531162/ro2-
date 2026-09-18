#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export N10P_PORT="${N10P_PORT:-/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00}"
case "${1:-}" in
  follower) exec /usr/bin/python3 -u person_follower.py --passive ;;
  camera) exec bash ./run_camera.sh ;;
  lidar) exec /usr/bin/python3 -u real_lidar_node.py ;;
  ai) exec bash ./run_ai.sh ;;
  web) exec bash ./run_web.sh ;;
  gui)
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
    until test -r "$XAUTHORITY" && xset q >/dev/null 2>&1; do sleep 1; done
    exec bash ./run_gui.sh
    ;;
  *) echo 'Expected camera|lidar|ai|web|gui|follower' >&2; exit 2 ;;
esac
