#!/bin/bash
# Independent stop-only cleanup after the bounded transient service exits.
set +e
python3 -u /root/wheeltec/estop.py 3
STOP_STATUS=$?
systemctl start rk3588-wheeltec.service
systemctl is-active rk3588-wheeltec.service
echo "STOP_CLEANUP_EXIT=$STOP_STATUS"
exit "$STOP_STATUS"
