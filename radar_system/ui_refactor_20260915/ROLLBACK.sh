#!/bin/sh
set -eu
TASK_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "${1:-}" in
  --board)
    adb -s 03801aa8f417ee51 shell 'set -e
      test -f /root/radar_system/ui_refactor_20260915/BASELINE.py
      cp -p /root/radar_system/ui_refactor_20260915/BASELINE.py /root/radar_system/board_radar_gui.py
      systemctl restart rk3588-perception@gui.service
      sha256sum /root/radar_system/board_radar_gui.py'
    ;;
  "") echo 'Usage: ROLLBACK.sh /absolute/path/to/copy.py | --board' >&2; exit 2 ;;
  *)
    test -f "$1"
    case "$1" in "$TASK_DIR/BASELINE.py"|"$TASK_DIR/MODIFIED_FILE.py") exit 2;; esac
    cp -p "$TASK_DIR/BASELINE.py" "$1"
    cmp -s "$TASK_DIR/BASELINE.py" "$1"
    echo 'ROLLBACK_RESTORED_BASELINE'
    ;;
esac
