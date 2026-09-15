#!/bin/sh
set -eu
TASK_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "${1:-}" in
  --board)
    adb -s 03801aa8f417ee51 shell 'set -e
      test -f /root/wheeltec/ackermann_adaptation_20260915/BASELINE.py
      test -f /root/wheeltec/ackermann_adaptation_20260915/run_driver.before.sh
      systemctl disable --now rk3588-wheeltec.service
      cp -p /root/wheeltec/ackermann_adaptation_20260915/BASELINE.py /root/wheeltec/wheeltec_driver.py
      cp -p /root/wheeltec/ackermann_adaptation_20260915/run_driver.before.sh /root/wheeltec/run_driver.sh
      cp -p /root/wheeltec/ackermann_adaptation_20260915/before-estop.py /root/wheeltec/estop.py
      cp -p /root/wheeltec/ackermann_adaptation_20260915/before-wheeltec_monitor.py /root/wheeltec/wheeltec_monitor.py
      cp -p /root/wheeltec/ackermann_adaptation_20260915/before-README.md /root/wheeltec/README.md
      rm -f /etc/systemd/system/rk3588-wheeltec.service /root/wheeltec/wheeltec.yaml /root/wheeltec/control.py
      systemctl daemon-reload
      sha256sum /root/wheeltec/wheeltec_driver.py'
    ;;
  "")
    echo 'Usage: ROLLBACK.sh /absolute/path/to/driver.py | --board' >&2
    exit 2
    ;;
  *)
    test -f "$1"
    case "$1" in "$TASK_DIR/BASELINE.py"|"$TASK_DIR/MODIFIED_FILE.py") exit 2;; esac
    cp -p "$TASK_DIR/BASELINE.py" "$1"
    echo 'ROLLBACK_RESTORED_BASELINE'
    ;;
esac
