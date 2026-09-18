#!/bin/bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source ./ros_env.sh
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export N10P_PORT="${N10P_PORT:-/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00}"
PYTHON_BIN="${RK3588_PYTHON:-/usr/bin/python3}"
if [ ! -x "$PYTHON_BIN" ]; then PYTHON_BIN=/usr/bin/python3; fi
case "${1:-}" in
  follower) exec "$PYTHON_BIN" -u person_follower.py --passive ;;
  camera) exec bash ./run_camera.sh ;;
  lidar) exec "$PYTHON_BIN" -u real_lidar_node.py ;;
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
