#!/usr/bin/env bash
set -euo pipefail
# 与 deployment/manage.py 的 PROFILES 一致。默认 rtk：只有 RTK 自动驾驶和手动开车，不起相机、雷达、人体识别、跟随。
# 底盘要不要雷达是激活时按配置写进服务单元的；换配置请用 manage.py activate，不要只用这里多起几个服务。
PROFILE_NAME="${1:-rtk}"
case "$PROFILE_NAME" in
  rtk) COMPONENTS=(web rtk line imu jobrec) ;;
  follow) COMPONENTS=(camera lidar ai web gui follower rtk line imu jobrec) ;;
  headless) COMPONENTS=(camera lidar web follower rtk line imu jobrec) ;;
  perception) COMPONENTS=(camera lidar ai web) ;;
  *) echo 'Usage: start_all.sh [rtk|follow|headless|perception]' >&2; exit 2 ;;
esac
systemctl start rk3588-wheeltec.service rk3588-perception.target
for component in "${COMPONENTS[@]}"; do
  systemctl start "rk3588-perception@${component}.service"
done
systemctl --no-pager --plain list-units 'rk3588-perception@*.service'
echo 'Web: :8088; follower stays passive until explicitly selected.'
