#!/bin/bash
set -e
# Each component has its own process group, logs and restart policy.
# Disable retired units before starting the reduced tracking stack.
# This also prevents old enabled mapping units from returning at the next boot.
for component in mapping mapping3d rtk foxglove; do
  unit="rk3588-perception@${component}.service"
  if systemctl cat "$unit" >/dev/null 2>&1; then
    systemctl disable --now "$unit" 2>/dev/null || true
  fi
done
systemctl start rk3588-perception.target
systemctl start rk3588-perception@{camera,lidar,ai,web,gui}.service
systemctl --no-pager --plain list-units 'rk3588-perception@*.service'
echo 'Web dashboard: http://192.168.2.173:8088/'
