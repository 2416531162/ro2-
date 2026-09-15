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
