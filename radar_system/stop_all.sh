#!/bin/bash
set -e
# 停车上的服务：先停服务单元，不然 systemd 会把杀掉的进程再拉起来；底盘驱动的指令超时也会把车停住。
systemctl stop rk3588-perception.target 2>/dev/null || true
systemctl stop rk3588-perception@{web,rtk,line,imu,jobrec}.service 2>/dev/null || true
systemctl stop rk3588-wheeltec.service 2>/dev/null || true
echo '车上服务已停止。'
