#!/bin/bash
set -euo pipefail
cd /root/radar_system
B=.n10p-before-20260914
[ -f "$B/board_radar_gui.py" ] && [ -f "$B/real_lidar_node.py" ]
# Verify this release before replacing files; preserve unrelated later work.
sha256sum -c .n10p-release-20260914.sha256
cp "$B/board_radar_gui.py" board_radar_gui.py
cp "$B/real_lidar_node.py" real_lidar_node.py
rm n10p_pipeline.py
python3 - <<'PY'
import os,signal,time
pids=[]
for pid in os.listdir('/proc'):
    if not pid.isdigit(): continue
    try:
        a=open('/proc/'+pid+'/cmdline','rb').read().split(b'\0')
        if a and b'python3' in a[0] and any(x in (b'real_lidar_node.py',b'/root/radar_system/real_lidar_node.py',b'board_radar_gui.py',b'/root/radar_system/board_radar_gui.py') for x in a[1:]):
            os.kill(int(pid),signal.SIGTERM); pids.append(int(pid))
    except (FileNotFoundError,ProcessLookupError): pass
time.sleep(1)
for pid in pids:
    try: os.kill(pid,signal.SIGKILL)
    except ProcessLookupError: pass
PY
set +u
source /opt/ros/jazzy/setup.bash
set -u
nohup python3 -u real_lidar_node.py >/tmp/real_lidar.log 2>&1 </dev/null &
nohup /root/radar_system/run_gui.sh >/tmp/radar_gui.log 2>&1 </dev/null &
echo 'board_restored=baseline lidar_gui_restarted=1'
