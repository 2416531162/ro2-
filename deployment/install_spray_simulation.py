#!/usr/bin/env python3
"""安装纯模拟服务；不激活车辆发布版、不重启运动/水泵服务。"""
import argparse
import os
from pathlib import Path
import re
import subprocess
from manage import verify


def unit(release):
    if not re.fullmatch(r'/opt/rk3588/releases/[0-9a-f]{16}',str(release)):
        raise ValueError('模拟服务须从已验证的版本目录运行')
    return f'''[Unit]
Description=RK3588 喷线计算（不下发控制）
After=network.target

[Service]
Type=simple
DynamicUser=true
StateDirectory=rk3588-spray-simulation
WorkingDirectory={release}
Environment=PYTHONDONTWRITEBYTECODE=1
Environment=PYTHONPATH={release}
ExecStart=/usr/bin/python3 -B {release}/radar_system/spray_sim_server.py --root /var/lib/rk3588-spray-simulation --port 19088 --observe-only
Restart=on-failure
RestartSec=3
Nice=10
CPUQuota=35%
MemoryMax=768M
TasksMax=32
NoNewPrivileges=true
PrivateDevices=true
DevicePolicy=closed
ProtectSystem=strict
ProtectHome=true
ProtectKernelTunables=true
ProtectKernelModules=true
ProtectControlGroups=true
RestrictAddressFamilies=AF_INET AF_INET6 AF_UNIX
RestrictRealtime=true
RestrictSUIDSGID=true
LockPersonality=true
UMask=0077

[Install]
WantedBy=multi-user.target
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release',type=Path)
    parser.add_argument('--expected-current',required=True,help='核对时车辆发布 ID；变化时须重新对齐')
    args=parser.parse_args()
    if os.geteuid()!=0:raise SystemExit('需以 root 安装模拟服务')
    current=Path('/opt/rk3588/current').resolve().name
    if current!=args.expected_current:raise SystemExit('车辆版本已改变，请重新对齐后安装')
    release=args.release.resolve();manifest=verify(release)
    path=Path('/etc/systemd/system/rk3588-spray-simulation.service')
    text=unit(release)
    if path.exists():path.with_suffix('.service.previous').write_text(path.read_text())
    temp=path.with_suffix('.tmp');temp.write_text(text);temp.replace(path)
    subprocess.run(['systemctl','daemon-reload'],check=True)
    subprocess.run(['systemctl','enable','--now','rk3588-spray-simulation.service'],check=True)
    subprocess.run(['systemctl','restart','rk3588-spray-simulation.service'],check=True)
    print('模拟服务来源',manifest['release_id'],manifest.get('source_commit'))
    print('车辆发布保持',current)


if __name__=='__main__':main()
