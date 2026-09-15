#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轮趣底盘 【命令延迟量化测试】 —— 用来查清第 6 节那个未解问题

⚠️ 运行前必读：
  1. 先把小车【架起来】，四个轮子完全离地！
  2. 确认物理断电开关在手边
  3. 一次只测一个方向，不要连续测多个方向（会放大失控风险）

用法：
    python3 -u latency_test.py                      # 默认测前进
    python3 -u latency_test.py fwd 3.0              # 前进，持续 3 秒
    python3 -u latency_test.py back 3.0
    python3 -u latency_test.py rotl 3.0
    python3 -u latency_test.py rotr 3.0

它会：
  - 以 50 Hz 持续发送指定指令，同时以尽量高频率读取遥测
  - 每 0.2 秒打印一次"指令 vs 反馈"
  - 结束后强制发 3 秒停车帧
  - 输出：首次响应延迟、峰值速度、稳态速度、停车后的残余速度
"""

import sys
import time
import serial
from functools import reduce

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
PORT = "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00"
BAUD = 115200

MODES = {
    #        名称            vx     vy     wz
    "fwd":  ("前进 +0.15 m/s",  0.15,  0.0,  0.0),
    "back": ("后退 -0.15 m/s", -0.15,  0.0,  0.0),
    "rotl": ("原地左转 +0.4",   0.0,   0.0,  0.4),
    "rotr": ("原地右转 -0.4",   0.0,   0.0, -0.4),
}


def bcc(data) -> int:
    return reduce(lambda a, b: a ^ b, data, 0)


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v


def build(vx, vy, wz) -> bytes:
    tx = bytearray(11)
    tx[0] = FRAME_HEADER
    for off, val in ((3, vx), (5, vy), (7, wz)):
        v = int(val * 1000)
        v = max(-32768, min(32767, v)) & 0xFFFF
        tx[off] = (v >> 8) & 0xFF
        tx[off + 1] = v & 0xFF
    tx[9] = bcc(tx[0:9])
    tx[10] = FRAME_TAIL
    return bytes(tx)


class Runner:
    def __init__(self):
        self.s = serial.Serial(PORT, BAUD, timeout=0.02)
        self.s.reset_input_buffer()
        self.buf = bytearray()
        self.cmd = (0.0, 0.0, 0.0)
        self.fb = (0.0, 0.0, 0.0)
        self.fb_time = None

    def spin(self, dur, label, print_period=0.2):
        t0 = time.time()
        nxt = 0.0
        while True:
            el = time.time() - t0
            if el >= dur:
                break
            self.buf.extend(self.s.read(512))
            self._parse()
            if el >= nxt:
                v = self.fb
                print(f"    [{label} t={el:5.2f}s] 指令=({self.cmd[0]:+.2f},{self.cmd[1]:+.2f},{self.cmd[2]:+.2f})"
                      f"  反馈 vx={v[0]:+.4f} vy={v[1]:+.4f} wz={v[2]:+.4f}")
                nxt += print_period
            self.s.write(build(*self.cmd))
            time.sleep(0.02)

    def _parse(self):
        while True:
            if not self.buf:
                return
            if self.buf[0] != FRAME_HEADER:
                i = self.buf.find(bytes([FRAME_HEADER]))
                if i < 0:
                    self.buf.clear()
                    return
                del self.buf[:i]
            if len(self.buf) < 24:
                return
            f = bytes(self.buf[:24])
            del self.buf[:24]
            if f[23] != FRAME_TAIL or bcc(f[0:22]) != f[22]:
                continue
            self.fb = (s16(f[2], f[3]) / 1000.0,
                       s16(f[4], f[5]) / 1000.0,
                       s16(f[6], f[7]) / 1000.0)
            self.fb_time = time.time()

    def estop(self, duration=3.0):
        self.cmd = (0.0, 0.0, 0.0)
        print(f"    ---- 强制停车 {duration}s ----")
        self.spin(duration, "EST")

    def close(self):
        self.estop(3.0)
        self.s.close()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "fwd"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    if mode not in MODES:
        print(f"未知模式 {mode}，可选: {list(MODES)}")
        return 1
    name, vx, vy, wz = MODES[mode]

    print("=" * 70)
    print("⚠️  确认：小车已架空？物理断电开关在手边？")
    print("=" * 70)
    print(f"测试: {name}   持续 {dur}s   串口 {PORT}\n")

    r = Runner()
    try:
        print("--- 静止基线 1.5s ---")
        r.spin(1.5, "BASE")

        print(f"\n--- 下发 {name} 持续 {dur}s ---")
        r.cmd = (vx, vy, wz)
        r.spin(dur, mode.upper())

        r.estop(3.0)

        print("\n--- 停车后观察 1.5s ---")
        r.spin(1.5, "AFTER")
    except KeyboardInterrupt:
        print("\n⚠️ 中断，立即停车")
    finally:
        r.close()

    print("\n✅ 完成。请对照上面的时间曲线判断：")
    print("   - 从下发到首次出现反馈，延迟多少秒？")
    print("   - 反馈峰值是多少？是否接近指令值？")
    print("   - 停车后是否还有残余速度？多久归零？")
    print("   - 反馈符号是否与指令同号？（转向确认，上车前必查）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
