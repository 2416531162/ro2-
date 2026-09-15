#!/bin/sh
set -eu
TASK_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "${1:-}" in
  --board)
    adb -s 03801aa8f417ee51 shell 'set -e
      test -f /root/radar_system/homepage_restore_20260915/BASELINE.sh
      test -f /root/radar_system/homepage_restore_20260915/lidar.before.py
      systemctl disable --now rk3588-perception.target
      cp -p /root/radar_system/homepage_restore_20260915/BASELINE.sh /root/radar_system/start_all.sh
      cp -p /root/radar_system/homepage_restore_20260915/lidar.before.py /root/radar_system/real_lidar_node.py
      rm -f /root/radar_system/start_component.sh /etc/systemd/system/rk3588-perception@.service /etc/systemd/system/rk3588-perception.target
      systemctl daemon-reload
      sha256sum /root/radar_system/start_all.sh'
    ;;
  "") echo 'Usage: ROLLBACK.sh /absolute/path/to/copy.sh | --board' >&2; exit 2 ;;
  *)
    test -f "$1"
    case "$1" in "$TASK_DIR/BASELINE.sh"|"$TASK_DIR/MODIFIED_FILE.sh") exit 2;; esac
    cp -p "$TASK_DIR/BASELINE.sh" "$1"
    cmp -s "$TASK_DIR/BASELINE.sh" "$1"
    echo 'ROLLBACK_RESTORED_BASELINE'
    ;;
esac
