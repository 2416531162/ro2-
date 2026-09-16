#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达自反射诊断 (Scan Doctor)。

用途:找出激光雷达扫到**车自己结构件**的角度区间。

为什么需要
----------
雷达装在车上,周围有相机支架、天线杆、传感器盒、车架立柱。这些东西会被
雷达扫成"障碍物",而且距离恒定、永不消失,表现为:

  - AEB 一直触发,但人眼看前方明明空无一物
  - 显示的"正前测距"和 AEB 判据对不上(两者用的扇区不同)
  - 加了车体足迹检查之后更糟:自反射点落在车体轮廓内,净空直接判 0,车永久停住

用法
----
把车**停在空旷处**(前后左右至少 2 米没东西),然后:

    python3 scan_doctor.py              # 采 3 秒,打印全周扫描剖面
    python3 scan_doctor.py --seconds 10 # 采久一点更稳
    python3 scan_doctor.py --near 0.8   # 把"可疑"门限放宽到 0.8m

空旷处仍然稳定报出近距离回波的角度,就是车自己。
输出末尾会直接给出可以粘进配置的屏蔽区间。
"""

import argparse
import math
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanDoctor(Node):

    def __init__(self, seconds, near_m, bin_deg):
        super().__init__('scan_doctor')
        self.seconds = seconds
        self.near_m = near_m
        self.bin_deg = bin_deg
        self.n_bins = int(round(360.0 / bin_deg))
        self.samples = defaultdict(list)     # bin -> [距离...]
        self.frames = 0
        self.started = time.monotonic()
        self.meta = None
        self.create_subscription(
            LaserScan, '/scan', self.on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        print(f"采集 {seconds} 秒... 请确保车停在空旷处,周围 2 米内没有物体。")

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        if self.meta is None:
            self.meta = dict(n=n, amin=msg.angle_min, ainc=msg.angle_increment,
                             rmin=msg.range_min, rmax=msg.range_max)
        self.frames += 1
        ainc = msg.angle_increment or (2 * math.pi / n)
        amin = msg.angle_min if msg.angle_increment else -math.pi
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0:
                continue
            if not (msg.range_min <= r <= msg.range_max):
                continue
            deg = math.degrees(amin + i * ainc) % 360.0
            self.samples[int(deg / self.bin_deg) % self.n_bins].append(r)

    def done(self):
        return time.monotonic() - self.started >= self.seconds

    def report(self):
        if not self.frames:
            print("\n没有收到任何 /scan 消息。确认雷达节点在跑,话题名是 /scan。")
            return 1

        m = self.meta
        amin_deg = math.degrees(m['amin'])
        print(f"\n收到 {self.frames} 帧,每帧 {m['n']} 点,"
              f"量程 {m['rmin']:.2f}~{m['rmax']:.2f} m,"
              f"角分辨率 {math.degrees(m['ainc']):.2f}°")
        print(f"angle_min = {amin_deg:+.1f}°  "
              f"(第 0 个光束指向这个方位,不是 0°)\n")

        # 这是实车上最容易踩、后果最严重的一个坑,直接替用户查出来
        if abs(amin_deg) > 1.0:
            shift = -amin_deg % 360.0
            print("\033[1;41;37m 警告:angle_min 不是 0,按序号当角度用会整体旋转 \033[0m")
            print(f"  写成 `deg = i * 360/n` 的代码,会把真实方位整体偏移 {shift:.0f}°。")
            print(f"  也就是说:它以为的「正前方」其实是真实方位 "
                  f"{(amin_deg) % 360.0 - (360.0 if amin_deg % 360.0 > 180 else 0):+.0f}°。")
            if abs(abs(amin_deg) - 180.0) < 5.0:
                print("  \033[1;31m这里正好差 180° —— 前后完全颠倒。\033[0m")
                print("  雷达装在车头时,它会把车尾扫到的自己当成正前方的障碍物,")
                print("  AEB 于是一直硬刹停,而人眼看前方明明空无一物。")
            print(f"  正确写法: deg = degrees(msg.angle_min + i * msg.angle_increment)\n")

        print("角度区间      最近    中位    出现率   判定")
        print("-" * 58)
        suspects = []
        for b in range(self.n_bins):
            vals = self.samples.get(b)
            lo = b * self.bin_deg
            hi = lo + self.bin_deg
            label = f"{lo:5.1f}~{hi:5.1f}°"
            if not vals:
                print(f"{label}      --      --       --    (无回波)")
                continue
            vals_sorted = sorted(vals)
            nearest = vals_sorted[0]
            median = vals_sorted[len(vals_sorted) // 2]
            # 出现率:该扇区平均每帧有多少个近距离点
            close = [v for v in vals if v < self.near_m]
            rate = len(close) / self.frames

            verdict = ""
            if median < self.near_m and rate >= 0.8:
                # 距离稳定且几乎每帧都有 -> 固定物体 -> 空旷处只可能是车自己
                spread = vals_sorted[-1] - vals_sorted[0]
                if spread < 0.10:
                    verdict = "\033[1;31m★ 车自己(距离恒定)\033[0m"
                    suspects.append((lo, hi, median))
                else:
                    verdict = "\033[1;33m? 可疑(距离有波动)\033[0m"
            elif close:
                verdict = "偶发近距离回波"
            print(f"{label}  {nearest:6.2f}  {median:6.2f}   {rate:6.2f}   {verdict}")

        print()
        if not suspects:
            print("✅ 没有发现固定的近距离回波,雷达视野是干净的。")
            print(f"   如果 AEB 仍然误触发,把 --near 调大再看一遍"
                  f"(当前门限 {self.near_m:.2f} m)。")
            return 0

        # 合并相邻的可疑扇区
        merged = []
        for lo, hi, d in suspects:
            if merged and abs(lo - merged[-1][1]) < 1e-6:
                merged[-1] = (merged[-1][0], hi, min(merged[-1][2], d))
            else:
                merged.append((lo, hi, d))

        print(f"\033[1;31m发现 {len(merged)} 个自反射区间:\033[0m\n")
        for lo, hi, d in merged:
            slo = lo - 360 if lo > 180 else lo
            shi = hi - 360 if hi > 180 else hi
            print(f"  {slo:+7.1f}° ~ {shi:+7.1f}°   距离约 {d:.2f} m")

        nearest_self = min(d for _, _, d in merged)
        # 自反射在车头还是车尾,决定了它会不会真的挡住前向探测
        front_side = [(lo, hi, d) for lo, hi, d in merged
                      if min(abs(((lo + hi) / 2 + 180) % 360 - 180),
                             abs(((lo + hi) / 2 - 360 + 180) % 360 - 180)) <= 60]
        print(f"\n最近的自反射在 {nearest_self:.2f} m。")
        if front_side:
            print("\033[1;31m其中有落在前向 ±60° 内的 —— 会直接遮挡前方探测,必须屏蔽。\033[0m")
        else:
            print("\033[1;32m全部在车侧后方,前向视野是干净的。\033[0m")
            print("  如果 AEB 仍然误报前方障碍,那不是雷达的问题,")
            print("  而是**代码把角度算错了** —— 看上面的 angle_min 警告。")
        if not front_side:
            # 侧后方的自反射会被 drop_self_hits() 的车体轮廓过滤自动丢掉,
            # 不需要任何手工配置。此时调 scan_min_valid_m 是有害的 ——
            # 那是全向门限,会让**所有方向**都看不见那么近的东西。
            print("\n\033[1;32m不需要改任何配置。\033[0m")
            print("  这些点落在车体轮廓内,drop_self_hits() 会自动丢掉。")
            print("  \033[1;33m特别提醒:不要调 scan_min_valid_m。\033[0m")
            print(f"  它是全向门限,调到 {nearest_self + 0.05:.2f} 会让所有方向都")
            print(f"  看不见 {nearest_self + 0.05:.2f}m 以内的东西 —— 包括真正挡在车头前的障碍物。")
            return 0

        print(f"\n前向自反射需要按方位屏蔽,填进 FollowerConfig:\n")
        print("    scan_blind_sectors_deg = (")
        for lo, hi, d in front_side:
            slo = lo - 360 if lo > 180 else lo
            shi = hi - 360 if hi > 180 else hi
            print(f"        ({slo:.1f}, {shi:.1f}),   # 约 {d:.2f}m 处的车体结构")
        print("    )")
        print("\n用角度屏蔽而不是调 scan_min_valid_m:后者是全向门限,")
        print("会牺牲所有方向的近距离探测能力。")
        return 0


def main():
    p = argparse.ArgumentParser(description="雷达自反射诊断")
    p.add_argument('--seconds', type=float, default=3.0, help='采集时长')
    p.add_argument('--near', type=float, default=0.6, dest='near_m',
                   help='判为"近距离"的门限(米),默认 0.6')
    p.add_argument('--bin-deg', type=float, default=5.0, help='角度分箱宽度')
    args, _ = p.parse_known_args()

    rclpy.init()
    node = ScanDoctor(args.seconds, args.near_m, args.bin_deg)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
