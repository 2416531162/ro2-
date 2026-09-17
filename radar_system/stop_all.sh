#!/bin/bash
set -e
# Stop the follower gracefully so its shutdown handler publishes stop/disarm.
# Stop services first, otherwise systemd may restart killed processes.
systemctl stop rk3588-perception.target
systemctl stop rk3588-perception@{camera,lidar,ai,web,gui}.service
pkill -INT -f '[p]erson_follower.py' || true
for process in person_pose_node.py real_lidar_node.py radar_web_server.py board_radar_gui.py; do
  pkill -INT -f "[p]ython.*${process}" || true
done
echo '雷达与摄像头跟踪系统已停止。'
