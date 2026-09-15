#!/bin/bash
set -euo pipefail
cd /root/radar_system
[ "$(sha256sum board_radar_gui.py | cut -d' ' -f1)" = 2b4d0cb5eacbfb94ed68859a0f9edc6c8a4bd4a3d86c0621623cbbd895b2277b ]
[ "$(sha256sum real_lidar_node.py | cut -d' ' -f1)" = 279d960ca0cd2cc5d9164be1372b6ace57e9b1ff6ec74d5ac119c9f11a3956b3 ]
BACKUP=/root/radar_system/.n10p-before-20260914
mkdir "$BACKUP"
cp -p board_radar_gui.py real_lidar_node.py "$BACKUP/"
for f in n10p_pipeline.py real_lidar_node.py board_radar_gui.py; do
  cp "/tmp/n10p-opt/$f" "$f.n10p-new"
  mv "$f.n10p-new" "$f"
done
python3 -m py_compile n10p_pipeline.py real_lidar_node.py board_radar_gui.py
python3 - <<'PY'
import os,signal
for pid in os.listdir('/proc'):
    if not pid.isdigit(): continue
    try:
        args=open('/proc/'+pid+'/cmdline','rb').read().split(b'\0')
        if args and b'python3' in args[0] and any(x in (b'real_lidar_node.py',b'/root/radar_system/real_lidar_node.py',b'/root/radar_system/board_radar_gui.py',b'board_radar_gui.py') for x in args[1:]):
            os.kill(int(pid),signal.SIGTERM)
    except (FileNotFoundError,ProcessLookupError,PermissionError): pass
PY
sleep 1
set +u
source /opt/ros/jazzy/setup.bash
set -u
nohup python3 -u real_lidar_node.py >/tmp/real_lidar.log 2>&1 </dev/null &
nohup /root/radar_system/run_gui.sh >/tmp/radar_gui.log 2>&1 </dev/null &
sleep 2
sha256sum n10p_pipeline.py real_lidar_node.py board_radar_gui.py
