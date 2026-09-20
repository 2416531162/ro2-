#!/bin/bash
set -e
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/ros_env.sh"
cd -- "$DIR"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export N10P_PORT="${N10P_PORT:-/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00}"
case "${1:-}" in
  follower) shift; exec "$RK3588_PYTHON" -u person_follower.py --passive "$@" ;;
  camera) shift; exec bash ./run_camera.sh "$@" ;;
  lidar) shift; exec "$RK3588_PYTHON" -u real_lidar_node.py "$@" ;;
  ai) shift; exec bash ./run_ai.sh "$@" ;;
  web) shift; exec bash ./run_web.sh "$@" ;;
  gui)
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
    until test -r "$XAUTHORITY" && xset q >/dev/null 2>&1; do sleep 1; done
    shift; exec bash ./run_gui.sh "$@"
    ;;
  *) echo 'Expected camera|lidar|ai|web|gui|follower' >&2; exit 2 ;;
esac
