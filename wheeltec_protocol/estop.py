#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮趣底盘 【紧急停车】 脚本 —— 出事了就立刻跑这个

用法（板子上）：
    python3 -u estop.py                 # 默认连续发 10 秒停车帧
    python3 -u estop.py 30              # 发 30 秒
    python3 -u estop.py 10 /dev/ttyACM1 # 指定串口

为什么需要"持续"发：
    实测发现该底盘存在 1~2 秒命令延迟，且停止发送指令后底盘会继续执行
    之前的指令。所以停车必须【持续高频发零帧】，不能只发一次。

零帧固定为： 7b 00 00 00 00 00 00 00 00 7b 7d
    [0]=0x7B 帧头
    [1..8]=0     (模式 / 预留 / vx / vy / wz 全零)
    [9]=0x7B    BCC = XOR(bytes 0..8)
    [10]=0x7D   帧尾
"""

import sys
import time
import serial
import math
import subprocess
from functools import reduce

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
DEFAULT_PORT = "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00"
BAUD = 115200


def build_zero_frame() -> bytes:
    """构造停车帧（全零速度）。"""
    tx = bytearray(11)
    tx[0] = FRAME_HEADER
    # tx[1]..tx[8] 保持 0
    tx[9] = reduce(lambda a, b: a ^ b, tx[0:9], 0)   # BCC
    tx[10] = FRAME_TAIL
    return bytes(tx)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    port = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PORT
    if not math.isfinite(duration) or not 0 < duration <= 60:
        print("停车持续时间应在 0 到 60 秒之间")
        return 2
    # A single process owns the serial port. Ask the live driver to latch STOP.
    if subprocess.run(['systemctl', 'is-active', '--quiet', 'rk3588-wheeltec.service']).returncode == 0:
        return subprocess.run([
            '/bin/bash', '-c',
            'source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; '
            'exec python3 /root/wheeltec/control.py stop'
        ]).returncode

    frame = build_zero_frame()
    print(f"停车帧: {' '.join(f'{b:02x}' for b in frame)}")
    print(f"打开 {port} @ {BAUD}，持续发 {duration} 秒 ...")

    try:
        s = serial.Serial(port, BAUD, timeout=0.02, write_timeout=0.1, exclusive=True)
    except Exception as e:
        print(f"❌ 打不开串口: {e}")
        return 1

    n = 0
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            s.write(frame)
            n += 1
            time.sleep(0.02)          # 50 Hz
    except KeyboardInterrupt:
        pass
    finally:
        # 收尾再多发 50 帧
        try:
            for _ in range(50):
                s.write(frame)
                n += 1
                time.sleep(0.02)
        except Exception:
            pass
        s.close()

    print(f"✅ 共发送 {n} 个停车帧（{time.time()-t0:.1f} 秒）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
