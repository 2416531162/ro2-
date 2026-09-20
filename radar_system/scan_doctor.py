#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读雷达覆盖与车身扫掠诊断 (Scan Doctor)。

用途:定位雷达未观测的车身扫掠区域，并列出可能的近距固定回波。

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
    python3 scan_doctor.py --path-only  # 非空旷场地仅诊断直行扫掠
    python3 scan_doctor.py --path-only --steer-deg -10  # 按实际右转角诊断

稳定近距回波只是疑点；即使在空旷处也必须现场核对车身结构。
屏蔽扇区只能标记未知区域，不能让车越过没有观测证据的位置。
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

    def __init__(self, seconds, near_m, bin_deg, path_only=False, steer_deg=0.0):
        super().__init__('scan_doctor')
        self.seconds = seconds
        self.near_m = near_m
        self.bin_deg = bin_deg
        self.path_only = path_only
        self.steer_deg = steer_deg
        self.n_bins = int(round(360.0 / bin_deg))
        self.samples = defaultdict(list)     # bin -> [距离...]
        # Resampled bins that were never fired must not count as missing echoes.
        self.missing = defaultdict(int)       # 原始光束序号 -> 已采样但无效的帧数
        self.sampled_count = defaultdict(int)
        self.frames = 0
        self.started = time.monotonic()
        self.meta = None
        self.latest_scan = None
        self.create_subscription(
            LaserScan, '/scan', self.on_scan,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        if path_only:
            print(f"采集 {seconds} 秒... 只诊断当前前向扫掠,不推断近距回波属于车身。")
        else:
            print(f"采集 {seconds} 秒... 请确保车停在空旷处,周围 2 米内没有物体。")

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        self.latest_scan = msg
        if self.meta is None:
            self.meta = dict(n=n, amin=msg.angle_min, ainc=msg.angle_increment,
                             rmin=msg.range_min, rmax=msg.range_max)
        self.frames += 1
        ainc = msg.angle_increment or (2 * math.pi / n)
        amin = msg.angle_min if msg.angle_increment else -math.pi
        intensities = msg.intensities
        has_sampling_mask = len(intensities) == n and any(q == -1.0 for q in intensities)
        for i, r in enumerate(msg.ranges):
            if has_sampling_mask and intensities[i] == -1.0:
                continue
            self.sampled_count[i] += 1
            if not math.isfinite(r) or r <= 0 or not (msg.range_min <= r <= msg.range_max):
                self.missing[i] += 1
            if not math.isfinite(r) or r <= 0:
                continue
            if not (msg.range_min <= r <= msg.range_max):
                continue
            deg = math.degrees(amin + i * ainc) % 360.0
            self.samples[int(deg / self.bin_deg) % self.n_bins].append(r)

    def report_front_shadows(self, half_fov_deg=90.0, min_ratio=0.9):
        """车头 ±90° 内几乎每帧都没有有效回波的方向。"""
        m = self.meta
        n = m['n']
        ainc = m['ainc'] or (2 * math.pi / n)
        amin = m['amin'] if m['ainc'] else -math.pi
        bad = []
        for i in range(n):
            deg = (math.degrees(amin + i * ainc) + 180.0) % 360.0 - 180.0
            sampled = self.sampled_count[i]
            if (abs(deg) <= half_fov_deg and sampled >= max(3, self.frames * .3)
                    and self.missing[i] / sampled >= min_ratio):
                bad.append(deg)
        # N10P 每圈约 450 点分进 720 格,孤立的空格是正常的;只报连续 >= 1.5° 的缝
        bad.sort()
        runs = []
        for deg in bad:
            if runs and deg - runs[-1][1] <= math.degrees(ainc) * 1.5:
                runs[-1][1] = deg
            else:
                runs.append([deg, deg])
        runs = [r for r in runs if r[1] - r[0] >= 1.5]
        if not runs:
            print("✅ 车头 ±90° 内没有持续无回波的方向。\n")
            return
        print("\033[1;33m车头 ±90° 内持续没有有效回波的方向(须检查采样标记和实际遮挡):\033[0m")
        for lo, hi in runs:
            width = hi - lo
            print(f"  {lo:+6.1f}° ~ {hi:+6.1f}°  宽 {width:.1f}°")
        print("  未采样、无回波和被车身挡住不是同一种情况；不能靠扩大屏蔽扇区放行。\n")

    def report_forward_sweep(self):
        """Use the follower's laser-only path check without commanding motion."""
        from follower_config import FollowerConfig
        from follower_recovery import LocalRecovery, ScanEvidence

        msg = self.latest_scan
        if msg is None:
            return
        cfg = FollowerConfig()
        intensities = msg.intensities
        sampled = (tuple(q != -1.0 for q in intensities)
                   if len(intensities) == len(msg.ranges) and any(q == -1.0 for q in intensities)
                   else None)
        scan = ScanEvidence(msg.ranges, msg.angle_min, msg.angle_increment,
                            max(msg.range_min, cfg.scan_min_valid_m), msg.range_max,
                            cfg.lidar_mount, cfg.footprint, cfg.scan_blind_sectors_deg,
                            cfg.self_hit_skin_m, sampled=sampled)
        recovery = LocalRecovery(cfg.footprint, cfg.geometry, cfg.obstacle_profile, cfg.recovery)
        steer = math.radians(self.steer_deg)
        if abs(steer) > cfg.max_steer_rad:
            raise ValueError(f'转角 {self.steer_deg:+.1f}° 超过配置的前轮转角限位')
        clearance = recovery.clearance(scan, steer, current_steer=steer, horizon=0.6)
        print(f"前向 0.6m 扫掠诊断 (假设前轮转角 {self.steer_deg:+.1f}°；"
              "不包含相机补盲，不发送运动指令):")
        if recovery.last_block is None:
            print("  本帧雷达覆盖这段路径；不代表可启动车辆或其他转向安全。\n")
            return
        kind, x, y, _ = recovery.last_block
        if kind == 'no_scan':
            print("  没有可用的雷达扫描；不能推断前方通路。\n")
            return
        angle = math.degrees(math.atan2(y-cfg.lidar_offset_y_m, x-cfg.lidar_offset_x_m)
                             - cfg.lidar_yaw_rad)
        angle = (angle+180) % 360-180
        print(f"  首个{'障碍' if kind == 'obstacle' else '未知区域'}: "
              f"后轴前方 {x:.3f}m, {'左' if y >= 0 else '右'} {abs(y):.3f}m, "
              f"雷达方位 {angle:+.1f}°, 该转角可证实距离 {clearance:.2f}m")
        nearby = scan.explain(x, y)
        print("  附近光束: " + ", ".join(f"{a:+.1f}° {r}m {cause}" for a, r, cause in nearby))
        print("  未知区域须先现场检查遮挡并用有效雷达/已标定深度补足；不能屏蔽后强行前进。\n")

    def done(self):
        return time.monotonic() - self.started >= self.seconds

    def report(self):
        if not self.frames:
            print("\n没有收到任何 /scan 消息。确认雷达节点在跑,话题名是 /scan。")
            return 1
        if self.path_only:
            self.report_forward_sweep()
            return 0

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

        self.report_front_shadows()
        self.report_forward_sweep()

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
                # 距离稳定且几乎每帧都有 -> 固定物体，但不证明属于车身
                spread = vals_sorted[-1] - vals_sorted[0]
                if spread < 0.10:
                    verdict = "\033[1;31m★ 固定近距回波(须现场确认)\033[0m"
                    suspects.append((lo, hi, median))
                else:
                    verdict = "\033[1;33m? 可疑(距离有波动)\033[0m"
            elif close:
                verdict = "偶发近距离回波"
            print(f"{label}  {nearest:6.2f}  {median:6.2f}   {rate:6.2f}   {verdict}")

        print()
        if not suspects:
            print("没有发现持续近距回波；仍须检查上面的前向扫掠与无回波光束。")
            return 0

        # 合并相邻的可疑扇区
        merged = []
        for lo, hi, d in suspects:
            if merged and abs(lo - merged[-1][1]) < 1e-6:
                merged[-1] = (merged[-1][0], hi, min(merged[-1][2], d))
            else:
                merged.append((lo, hi, d))

        print(f"\033[1;31m发现 {len(merged)} 个固定近距回波区间 (不等于车身自反射):\033[0m\n")
        for lo, hi, d in merged:
            slo = lo - 360 if lo > 180 else lo
            shi = hi - 360 if hi > 180 else hi
            print(f"  {slo:+7.1f}° ~ {shi:+7.1f}°   距离约 {d:.2f} m")

        nearest_self = min(d for _, _, d in merged)
        # 固定近距回波在车头还是车尾,决定了需要先检查哪处
        front_side = [(lo, hi, d) for lo, hi, d in merged
                      if min(abs(((lo + hi) / 2 + 180) % 360 - 180),
                             abs(((lo + hi) / 2 - 360 + 180) % 360 - 180)) <= 60]
        print(f"\n最近的固定近距回波在 {nearest_self:.2f} m。")
        if front_side:
            print("\033[1;31m其中有落在前向 ±60° 内的；请现场排查车体遮挡或真实障碍。\033[0m")
        else:
            print("固定近距回波均在车侧后方；仍须检查前向未知区域和传感器时间戳。")
        if not front_side:
            print("\n仅已确认位于车体轮廓内的回波可由自反射过滤；轮廓外仍视为障碍。")
            print("  \033[1;33m特别提醒:不要调 scan_min_valid_m。\033[0m")
            print(f"  它是全向门限,调到 {nearest_self + 0.05:.2f} 会让所有方向都")
            print(f"  看不见 {nearest_self + 0.05:.2f}m 以内的东西 —— 包括真正挡在车头前的障碍物。")
            return 0

        print("\n如果确为车体遮挡，应移开结构件、重新安装雷达，或补充经标定且能看到近距的传感器。")
        print("屏蔽扇区只能标记不可见光束，不能证明遮挡后方空闲；不要调大量程下限。")
        return 0


def main():
    p = argparse.ArgumentParser(description="雷达自反射诊断")
    p.add_argument('--seconds', type=float, default=3.0, help='采集时长')
    p.add_argument('--near', type=float, default=0.6, dest='near_m',
                   help='判为"近距离"的门限(米),默认 0.6')
    p.add_argument('--bin-deg', type=float, default=5.0, help='角度分箱宽度')
    p.add_argument('--path-only', action='store_true', help='只读检查当前前向扫掠,无需空旷场地')
    p.add_argument('--steer-deg', type=float, default=0.0, help='实际前轮转角(左正右负),用于前向扫掠')
    args, _ = p.parse_known_args()
    from follower_config import FollowerConfig
    if not math.isfinite(args.steer_deg) or abs(math.radians(args.steer_deg)) > FollowerConfig().max_steer_rad:
        p.error('--steer-deg 必须在配置的前轮转角限位以内')

    rclpy.init()
    node = ScanDoctor(args.seconds, args.near_m, args.bin_deg,
                      path_only=args.path_only, steer_deg=args.steer_deg)
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
