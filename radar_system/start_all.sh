#!/usr/bin/env bash
set -euo pipefail
PROFILE_NAME="${1:-follow}"
case "$PROFILE_NAME" in
  follow) COMPONENTS=(camera lidar ai web gui follower) ;;
  headless) COMPONENTS=(camera lidar ai web follower) ;;
  perception) COMPONENTS=(camera lidar ai web) ;;
  *) echo 'Usage: start_all.sh [follow|headless|perception]' >&2; exit 2 ;;
esac
systemctl start rk3588-wheeltec.service rk3588-perception.target
for component in "${COMPONENTS[@]}"; do
  systemctl start "rk3588-perception@${component}.service"
done
systemctl --no-pager --plain list-units 'rk3588-perception@*.service'
echo 'Web: :8088; follower stays passive until explicitly selected.'
