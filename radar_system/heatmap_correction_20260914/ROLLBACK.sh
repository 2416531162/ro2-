#!/bin/bash
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
if [ "${1:-}" = --board ]; then
  adb shell 'cd /root/radar_system && sha256sum board_radar_gui.py' | grep -q 66da92851d6d749c6c9f70c1fd59c0ffdabd4ca2c81fe9950f01fe0bb87caa78
  adb push "$D/BASELINE.py" /root/radar_system/board_radar_gui.py.heat-restore >/dev/null
  adb shell 'mv /root/radar_system/board_radar_gui.py.heat-restore /root/radar_system/board_radar_gui.py'
  adb push "$D/restart_gui.py" /tmp/heatmap_restart_gui.py >/dev/null
  adb shell 'python3 /tmp/heatmap_restart_gui.py'
  adb shell 'nohup /root/radar_system/run_gui.sh --cam-mode=depth >/tmp/radar_gui.log 2>&1 </dev/null &'
  echo 'board_restored=baseline gui_restart_requested=1'
  exit 0
fi
TARGET="${1:-$(dirname "$D")/board_radar_gui.py}"
python3 - "$D" "$TARGET" <<'PY'
import sys,pathlib,hashlib,shutil
base,target=pathlib.Path(sys.argv[1]),pathlib.Path(sys.argv[2])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert target.resolve() not in [(base/'BASELINE.py').resolve(),(base/'MODIFIED_FILE.py').resolve()]
assert sha(target)==sha(base/'MODIFIED_FILE.py'),'target has subsequent edits'
shutil.copy2(base/'BASELINE.py',target)
assert sha(target)==sha(base/'BASELINE.py')
print('restored_sha256='+sha(target))
PY
