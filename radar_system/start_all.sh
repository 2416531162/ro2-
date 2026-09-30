#!/usr/bin/env bash
set -euo pipefail
# 手动起车上的服务（正常由 deployment/manage.py activate 装成开机自启，不用这个）。与 manage.py 的 PROFILES['rtk'] 一致。
COMPONENTS=(web rtk line imu jobrec)
systemctl start rk3588-can.service rk3588-wheeltec.service rk3588-perception.target
for component in "${COMPONENTS[@]}"; do
  systemctl start "rk3588-perception@${component}.service"
done
systemctl --no-pager --plain list-units 'rk3588-*'
echo 'Web: :8088'
