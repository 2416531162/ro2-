#!/bin/bash
set -e
# Each component has its own process group, logs and restart policy.
systemctl start rk3588-perception.target
systemctl start rk3588-perception@{camera,rtk,lidar,ai,mapping,web,gui}.service
systemctl --no-pager --plain list-units 'rk3588-perception@*.service'
echo 'Web dashboard: http://192.168.2.173:8088/'
