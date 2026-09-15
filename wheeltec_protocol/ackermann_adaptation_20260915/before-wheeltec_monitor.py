#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮趣底盘串口监视器（不依赖 ROS2）—— 用于快速验证协议和硬件

用法：
    python3 wheeltec_monitor.py                 # 自动找串口
    python3 wheeltec_monitor.py /dev/ttyACM1    # 指定串口
    python3 wheeltec_monitor.py --raw           # 顺便打印原始 hex
"""

import os
import sys
import time

import serial
from serial.tools import list_ports

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
SIZE = 24
GYRO_RATIO = 0.00026644
ACCEL_RATIO = 1671.84


def bcc(data):
    c = 0
    for b in data:
        c ^= b
    return c


def s16(h, l):
    v = (h << 8) | l
    return v - 0x10000 if v >= 0x8000 else v


def u16(h, l):
    return (h << 8) | l


def find_port():
    by_id = "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00"
    if os.path.exists(by_id):
        return by_id
    for p in list_ports.comports():
        if (p.vid, p.pid) == (0x1A86, 0x55D4) and (p.serial_number or "").endswith("0002"):
            return p.device
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    show_raw = "--raw" in sys.argv

    port = args[0] if args else find_port()
    if not port:
        print("找不到轮趣底盘串口，请显式指定，例如 /dev/ttyACM1")
        return 1

    print(f"打开 {port} @ 115200 8N1 ...")
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.reset_input_buffer()

    buf = bytearray()
    ok = bad = 0
    t0 = time.time()
    last = 0.0

    hdr = (f"{'#':>6} {'X速度':>8} {'Y速度':>8} {'Z角速度':>9} "
           f"{'AccX':>7} {'AccY':>7} {'AccZ':>7} "
           f"{'GyrX':>7} {'GyrY':>7} {'GyrZ':>7} {'电压V':>7} {'BCC':>4}")
    print(hdr)
    print("-" * len(hdr))

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            while True:
                if not buf:
                    break
                if buf[0] != FRAME_HEADER:
                    i = buf.find(bytes([FRAME_HEADER]))
                    if i < 0:
                        buf.clear()
                        break
                    del buf[:i]
                if len(buf) < SIZE:
                    break

                f = bytes(buf[:SIZE])
                if f[23] != FRAME_TAIL or bcc(f[0:22]) != f[22]:
                    bad += 1
                    del buf[:1]
                    continue
                del buf[:SIZE]
                ok += 1

                vx = s16(f[2], f[3]) / 1000.0
                vy = s16(f[4], f[5]) / 1000.0
                vz = s16(f[6], f[7]) / 1000.0
                ax = s16(f[8], f[9]) / ACCEL_RATIO
                ay = s16(f[10], f[11]) / ACCEL_RATIO
                az = s16(f[12], f[13]) / ACCEL_RATIO
                gx = s16(f[14], f[15]) * GYRO_RATIO
                gy = s16(f[16], f[17]) * GYRO_RATIO
                gz = s16(f[18], f[19]) * GYRO_RATIO
                volt = u16(f[20], f[21]) / 1000.0

                if ok % 5 == 1:
                    print(f"{ok:>6} {vx:>8.3f} {vy:>8.3f} {vz:>9.3f} "
                          f"{ax:>7.3f} {ay:>7.3f} {az:>7.3f} "
                          f"{gx:>7.4f} {gy:>7.4f} {gz:>7.4f} {volt:>7.3f} {f[22]:>4}")
                    if show_raw:
                        print("       raw: " + " ".join(f"{b:02x}" for b in f))

            if time.time() - last > 2.0:
                last = time.time()
                el = time.time() - t0
                print(f"--- 统计: 有效 {ok}  校验失败 {bad}  帧率 {ok/el:.2f} Hz ---")

    except KeyboardInterrupt:
        el = time.time() - t0
        print(f"\n共 {ok} 帧有效 / {bad} 帧校验失败，平均 {ok/el:.2f} Hz")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
