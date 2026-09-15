#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
cd /root/radar_system
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export N10P_PORT=/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00
case "${1:-}" in
  camera) exec ros2 launch openni2_camera camera_only.launch.py ;;
  rtk) exec /usr/bin/python3 -u rtk_node.py ;;
  lidar) exec /usr/bin/python3 -u real_lidar_node.py ;;
  ai) exec /root/radar_system/run_ai.sh ;;
  mapping) exec /root/radar_system/run_3d_mapping.sh ;;
  web) exec /usr/bin/python3 -u radar_web_server.py ;;
  gui)
    export DISPLAY=:0
    export XAUTHORITY=/run/user/1000/gdm/Xauthority
    until test -r "$XAUTHORITY" && xset q >/dev/null 2>&1; do sleep 1; done
    exec /root/radar_system/run_gui.sh
    ;;
  *) echo 'Expected camera|rtk|lidar|ai|mapping|web|gui' >&2; exit 2 ;;
esac
