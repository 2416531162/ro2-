#!/bin/sh
set -eu
TASK_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
EXPECTED=15e9af5b3fb1e443b1a8f760be6a9735b2bdff20fa0282bb638a3d6a04d2030c
python3 - "$TASK_DIR/BASELINE.py" "$EXPECTED" <<'PY'
import hashlib,sys
assert hashlib.sha256(open(sys.argv[1],'rb').read()).hexdigest()==sys.argv[2]
PY
case "${1:-}" in
  --board)
    adb -s 03801aa8f417ee51 push "$TASK_DIR/BASELINE.py" /root/radar_system/yolo_integration_20260915/restore.py
    adb -s 03801aa8f417ee51 shell 'set -e
      install -m 755 /root/radar_system/yolo_integration_20260915/restore.py /root/radar_system/ai_3d_detector.py
      systemctl restart rk3588-perception@ai.service
      systemctl is-active rk3588-perception@ai.service
      sha256sum /root/radar_system/ai_3d_detector.py'
    ;;
  "") echo 'Usage: ROLLBACK.sh /absolute/path/to/copy.py | --board' >&2; exit 2 ;;
  *)
    python3 - "$TASK_DIR" "$1" <<'PY'
from pathlib import Path
import shutil,sys
p=Path(sys.argv[1]);target=Path(sys.argv[2]).resolve()
assert target.is_file() and target not in [(p/'BASELINE.py').resolve(),(p/'MODIFIED_FILE.py').resolve()]
shutil.copy2(p/'BASELINE.py',target)
assert target.read_bytes()==(p/'BASELINE.py').read_bytes()
print('ROLLBACK_RESTORED_BASELINE')
PY
    ;;
esac
