#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机 / 激光雷达外参检查与粗标定(水平面内)。

为什么需要
----------
跟踪器把相机检测和雷达腿部点簇当作同一个人的两路观测做关联。两者的安装位置
(外参)对不上时:
  - 相机看到的人和雷达看到的腿差几十厘米,关联门控把雷达观测拒掉,
    人一出相机画面就跟丢(面板「雷达轨迹:未认领到腿」);
  - 相机偏航角差几度,远处的人横向就偏出十几厘米,车跟着歪。

做法
----
让一个人在车前**站定**,依次站到几个位置(左 / 中 / 右 × 近 / 远,每处 3 秒)。
本工具同时记录相机算出的人的位置和雷达上最近的腿部点簇,用二维刚体最小二乘
(Umeyama)求出「相机结果应当怎样平移、旋转才能和雷达对齐」,换算成
FollowerConfig 里的 camera_offset_x_m / camera_offset_y_m / camera_yaw_rad。

雷达的安装位置(lidar_offset_x_m)是基准,本工具不改它;先用卷尺量准。

用法
----
    python3 radar_system/calib_check.py --seconds 40

只检查不改任何文件。输出的建议值需要人工填进 person_follower.py。
"""

import argparse
import json
import math
import sys
import time

__all__ = ["fit_rigid_2d", "CalibCollector", "suggest_mount"]


def fit_rigid_2d(src, dst):
    """求 R(θ)、t 使 dst ≈ R·src + t(最小二乘)。返回 (θ, tx, ty, rms)。

    >>> import math
    >>> src = [(2.0, 0.0), (2.0, 1.0), (3.0, -1.0), (1.5, 0.5)]
    >>> th = math.radians(3)
    >>> dst = [(math.cos(th)*x - math.sin(th)*y + 0.05, math.sin(th)*x + math.cos(th)*y - 0.02)
    ...        for x, y in src]
    >>> t, tx, ty, rms = fit_rigid_2d(src, dst)
    >>> round(math.degrees(t), 3), round(tx, 3), round(ty, 3), rms < 1e-9
    (3.0, 0.05, -0.02, True)
    """
    n = len(src)
    if n < 2 or n != len(dst):
        raise ValueError("至少需要两对点")
    sx = sum(p[0] for p in src) / n
    sy = sum(p[1] for p in src) / n
    dx = sum(p[0] for p in dst) / n
    dy = sum(p[1] for p in dst) / n
    h00 = h01 = h10 = h11 = 0.0
    for (ax, ay), (bx, by) in zip(src, dst):
        ax, ay, bx, by = ax - sx, ay - sy, bx - dx, by - dy
        h00 += ax * bx
        h01 += ax * by
        h10 += ay * bx
        h11 += ay * by
    theta = math.atan2(h01 - h10, h00 + h11)
    c, s = math.cos(theta), math.sin(theta)
    tx = dx - (c * sx - s * sy)
    ty = dy - (s * sx + c * sy)
    err = 0.0
    for (ax, ay), (bx, by) in zip(src, dst):
        ex = c * ax - s * ay + tx - bx
        ey = s * ax + c * ay + ty - by
        err += ex * ex + ey * ey
    return theta, tx, ty, math.sqrt(err / n)


def suggest_mount(x_m, y_m, yaw_rad, theta, tx, ty):
    """把「相机结果的修正变换」换算成新的相机安装参数。

    相机结果 p = m + R0·v;修正后 p' = R(θ)·p + t = (R(θ)·m + t) + R(θ)R0·v
    """
    c, s = math.cos(theta), math.sin(theta)
    return (c * x_m - s * y_m + tx, s * x_m + c * y_m + ty, yaw_rad + theta)


class CalibCollector:
    """收集「相机里的人」与「雷达上的腿」的成对样本。纯 Python,可离线测试。"""

    def __init__(self, max_pair_dist_m=0.8, max_dt_s=0.15, min_conf=0.6):
        self.max_pair_dist_m = max_pair_dist_m
        self.max_dt_s = max_dt_s
        self.min_conf = min_conf
        self.pairs = []
        self.last_clusters = None
        self.last_clusters_t = None
        self.skipped_multi = 0
        self.skipped_nolidar = 0

    def add_lidar(self, clusters, t):
        self.last_clusters = list(clusters)
        self.last_clusters_t = t

    def add_camera(self, people, t):
        """people: [(x, y, conf)] 车体系(已用当前相机参数换算)。"""
        good = [p for p in people if p[2] >= self.min_conf]
        if len(good) != 1:
            if len(good) > 1:
                self.skipped_multi += 1        # 画面里多个人,无法确定对应关系
            return False
        if self.last_clusters is None or abs(t - self.last_clusters_t) > self.max_dt_s:
            self.skipped_nolidar += 1
            return False
        cx, cy, _ = good[0]
        best, best_d = None, self.max_pair_dist_m
        for lx, ly in self.last_clusters:
            d = math.hypot(lx - cx, ly - cy)
            if d <= best_d:
                best, best_d = (lx, ly), d
        if best is None:
            self.skipped_nolidar += 1
            return False
        self.pairs.append(((cx, cy), best))
        return True

    def solve(self, iterations=3):
        """稳健拟合:剔除残差大于 3 倍 RMS(至少 10cm)的样本后重算。"""
        pairs = list(self.pairs)
        if len(pairs) < 10:
            return None
        result = None
        for _ in range(iterations):
            src = [p[0] for p in pairs]
            dst = [p[1] for p in pairs]
            theta, tx, ty, rms = fit_rigid_2d(src, dst)
            result = dict(theta=theta, tx=tx, ty=ty, rms=rms, used=len(pairs),
                          total=len(self.pairs))
            c, s = math.cos(theta), math.sin(theta)
            limit = max(0.10, 3 * rms)
            kept = [((ax, ay), (bx, by)) for (ax, ay), (bx, by) in pairs
                    if math.hypot(c * ax - s * ay + tx - bx, s * ax + c * ay + ty - by) <= limit]
            if len(kept) == len(pairs) or len(kept) < 10:
                break
            pairs = kept
        xs = [p[0][0] for p in pairs]
        ys = [p[0][1] for p in pairs]
        result["range_spread_m"] = max(xs) - min(xs)
        result["lateral_spread_m"] = max(ys) - min(ys)
        # 残差本身(修正前):相机与雷达平均差多少
        raw = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in self.pairs]
        result["raw_mean_error_m"] = sum(raw) / len(raw)
        return result


def report(result, cfg, collector):
    if result is None:
        print(f"\n样本不足({len(collector.pairs)} 对,至少 10 对)。")
        print(f"  画面里多人被跳过 {collector.skipped_multi} 次,"
              f"找不到对应雷达点簇 {collector.skipped_nolidar} 次。")
        print("  确认:只有一个人在车前、站在 0.9~3.5m 内、雷达能扫到腿。")
        return 1
    theta, tx, ty = result["theta"], result["tx"], result["ty"]
    print(f"\n样本 {result['used']}/{result['total']} 对;"
          f"纵向跨度 {result['range_spread_m']:.2f} m,横向跨度 {result['lateral_spread_m']:.2f} m")
    print(f"修正前相机与雷达平均相差 {result['raw_mean_error_m']*100:.1f} cm")
    print(f"修正变换:平移 ({tx*100:+.1f}, {ty*100:+.1f}) cm,旋转 {math.degrees(theta):+.2f}°,"
          f"修正后残差 {result['rms']*100:.1f} cm")
    if result["lateral_spread_m"] < 0.8 or result["range_spread_m"] < 1.0:
        print("\033[1;33m站位覆盖不够(横向 < 0.8m 或纵向 < 1m),旋转角不可靠;"
              "请左/中/右、近/远都站一遍。\033[0m")
    small = result["raw_mean_error_m"] < 0.08 and abs(math.degrees(theta)) < 1.0
    if small:
        print("\033[1;32m外参基本一致,不需要修改。\033[0m")
        return 0
    x, y, yaw = suggest_mount(cfg.camera_offset_x_m, cfg.camera_offset_y_m,
                              cfg.camera_yaw_rad, theta, tx, ty)
    print("\n建议写入 person_follower.py 的 FollowerConfig:")
    print(f"    camera_offset_x_m: float = {x:.3f}")
    print(f"    camera_offset_y_m: float = {y:.3f}")
    print(f"    camera_yaw_rad: float = {yaw:.4f}   # {math.degrees(yaw):+.2f}°")
    if x > cfg.footprint_front_m:
        print("\033[1;31m换算出的相机位置在车头之前,多半是深度测距整体偏差而不是安装位置,"
              "请先检查相机俯角 camera_pitch_rad。\033[0m")
    print("\n改完后再运行一次本工具,修正前误差应降到 5cm 左右。")
    return 0


def main():
    p = argparse.ArgumentParser(description="相机/雷达外参检查")
    p.add_argument('--seconds', type=float, default=40.0)
    p.add_argument('--json', action='store_true', help='以 JSON 输出结果')
    args = p.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from person_follower import FollowerConfig
    from footprint import optical_to_vehicle, scan_to_vehicle_frame, drop_self_hits
    from lidar_track import cluster_points

    cfg = FollowerConfig()
    collector = CalibCollector()

    class CalibNode(Node):
        def __init__(self):
            super().__init__('calib_check')
            self.create_subscription(String, '/camera/ai_detection/targets', self.on_targets, 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def on_scan(self, msg):
            pts = []
            for i, r in enumerate(msg.ranges):
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    pts.append((msg.angle_min + i * msg.angle_increment, r))
            vehicle = scan_to_vehicle_frame(pts, cfg.lidar_mount,
                                            blind_sectors_deg=cfg.scan_blind_sectors_deg)
            vehicle, _ = drop_self_hits(vehicle, cfg.footprint, cfg.self_hit_skin_m)
            clusters = cluster_points(vehicle, origin=(cfg.lidar_mount.x_m, cfg.lidar_mount.y_m))
            collector.add_lidar([(c.x, c.y) for c in clusters], time.monotonic())

        def on_targets(self, msg):
            try:
                items = json.loads(msg.data)
            except ValueError:
                return
            people = []
            for it in items if isinstance(items, list) else []:
                if str(it.get('label', '')).lower() != 'person' or not it.get('range_valid'):
                    continue
                x, y, _z = optical_to_vehicle(float(it['x']), float(it.get('y', 0.0)),
                                              float(it['z']), cfg.camera_mount,
                                              cfg.camera_pitch_rad)
                if 0.9 <= x - cfg.camera_offset_x_m <= 3.5:
                    people.append((x, y, float(it.get('conf', 0.0))))
            if collector.add_camera(people, time.monotonic()) and len(collector.pairs) % 20 == 0:
                print(f"  已采集 {len(collector.pairs)} 对样本", flush=True)

    rclpy.init()
    node = CalibNode()
    print(f"采集 {args.seconds:.0f} 秒:请一个人依次站到车前 左/中/右 × 近(1.2m)/远(2.5m),"
          "每处站定 3 秒。", flush=True)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result = collector.solve()
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result else 1
    return report(result, cfg, collector)


if __name__ == '__main__':
    sys.exit(main())
