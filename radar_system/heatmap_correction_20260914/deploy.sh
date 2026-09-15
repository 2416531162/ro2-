#!/bin/bash
set -euo pipefail
cd /root/radar_system
EXPECTED=6d0d8c9b45df829c5373264af2f7715377db075c09ddac6930218fb12ad8ecf6
[ "$(sha256sum board_radar_gui.py | cut -d' ' -f1)" = "$EXPECTED" ]
cp -p board_radar_gui.py board_radar_gui.py.before-pure-depth-20260914
cp /tmp/heatmap-correction/MODIFIED_FILE.py board_radar_gui.py.pure-new
python3 -m py_compile board_radar_gui.py.pure-new
mv board_radar_gui.py.pure-new board_radar_gui.py
python3 - <<'PY'
import os,signal,time
pids=[]
for pid in os.listdir('/proc'):
    if not pid.isdigit():continue
    try:
        a=open('/proc/'+pid+'/cmdline','rb').read().split(b'\0')
        if a and b'python3' in a[0] and any(x in (b'board_radar_gui.py',b'/root/radar_system/board_radar_gui.py') for x in a[1:]):
            os.kill(int(pid),signal.SIGTERM);pids.append(int(pid))
    except (FileNotFoundError,ProcessLookupError):pass
time.sleep(1)
for pid in pids:
    try:os.kill(pid,signal.SIGKILL)
    except ProcessLookupError:pass
PY
nohup /root/radar_system/run_gui.sh --cam-mode=depth >/tmp/radar_gui.log 2>&1 </dev/null &
sleep 2
sha256sum board_radar_gui.py
