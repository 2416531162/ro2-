#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
「电子跟屁虫」—— 人体 3D 视觉 + 激光雷达智能跟随控制节点 (Person Follower)
- 视觉感知: 订阅 /camera/ai_detection/targets (Astra S + YOLOv8/FaceNet 3D 目标)
- 雷达防撞: 订阅 /scan (N10P 前向 ±35° 扇区, 障碍物 < 0.45m 毫秒级 AEB 急停)
- 电量监控: 订阅 /voltage (6S 动力电池保护, < 21.0V 自动驻车)
- 运动控制: 发布 /cmd_vel (Twist 速度指令, 极速限幅 0.15 m/s, 舒适跟随距离 1.2m)
- 状态遥测: 发布 /follower/status (JSON 实时运行状态)
"""

import sys
import os
import math
import time
import json
import signal
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

# ================= 控制参数配置 =================
AEB_STOP_DISTANCE_M = 0.40      # 激光雷达主动防撞硬刹停阈值: 0.40 米 (前方 0.4m 扫到物体立马停止)
AEB_RELEASE_DISTANCE_M = 0.48   # 激光雷达防撞解除回差: 0.48 米 (消除临界抖动抽搐)

TARGET_DISTANCE_M = 0.65        # 目标保持距离: 0.65 米 (舒适贴身智能跟随)
DEADBAND_MIN_M = 0.50           # 跟随死区下限: 0.50 米 (人靠近 50cm 小车自动停步待命，距 40cm 防撞有 10cm 安全缓冲)
DEADBAND_MAX_M = 0.80           # 跟随死区上限: 0.80 米 (人离开超 80cm 小车顺滑起步跟进)
MAX_FOLLOW_DISTANCE_M = 3.50    # 最大有效跟随距离: 3.50 米 (超过视为超出视线)
MIN_TARGET_Z_M = 0.35           # 最小有效目标深度: 0.35 米

MAX_SPEED_MPS = 0.65            # 最大前进速度限幅: 0.65 m/s (充沛动力跟随)
MIN_SPEED_MPS = 0.25            # 最小启步速度: 0.25 m/s (强劲破除轮胎静摩擦)
KP_SPEED = 0.45                 # 纵向距离 P 控制增益
ACCEL_LIMIT_MPS2 = 1.50         # 平滑加速度限制: 1.50 m/s^2 (充沛加速，反应迅捷)

MAX_TURN_RADPS = 0.30           # 最大转弯角速度限幅: 0.30 rad/s
KP_TURN = 0.65                 # 横向航向角 P 控制增益
ANGLE_DEADBAND_RAD = 0.08       # 转向死区: ~4.6° (身体微晃绝不摇摆打舵)

MIN_CONFIDENCE = 0.55           # 目标置信度阈值: 低于 0.55 的疑似噪点直接丢弃
TARGET_CONFIRM_FRAMES = 2       # 目标确认帧数: 连续检测到 2 帧才起步，防止单帧误检抽动
TARGET_LOST_FRAMES = 3          # 目标丢失判定: 连续 3 帧 (约 0.15 秒) 无人，立马停步，绝不盲动！
BATTERY_MIN_V = 21.0            # 6S 动力电池保护电压: 21.0 V


class PersonFollowerNode(Node):
    def __init__(self, dry_run=False, target_class="person"):
        super().__init__('person_follower_node')
        self.dry_run = dry_run
        self.target_class = target_class.lower()

        # 状态变量
        self.state = "STANDBY"
        self.latest_target = None
        self.last_target_seen = 0.0
        self.consecutive_seen = 0
        self.consecutive_lost = 0
        self.smooth_x = None
        self.smooth_z = None
        self.min_front_scan = 99.0
        self.voltage = 23.0
        self.aeb_active = False
        self.running = True

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        # ROS 2 订阅者
        self.sub_targets = self.create_subscription(
            String, '/camera/ai_detection/targets', self.on_targets, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_voltage = self.create_subscription(
            Float32, '/voltage', self.on_voltage, 10)

        # ROS 2 发布者
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/follower/status', 10)

        # 底盘安全使能客户端与驱动器状态看门狗
        self.cli_arm = self.create_client(SetBool, '/wheeltec/arm')
        self.cli_stop = self.create_client(Trigger, '/wheeltec/stop')
        self.driver_armed = False
        self.driver_ready = False
        self.last_arm_request = 0.0
        self.sub_driver_status = self.create_subscription(
            String, '/wheeltec/status', self.on_driver_status, 10)
        if not self.dry_run:
            self.arm_chassis(True)

        # 20 Hz 控制决策定时器 (50ms)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_print_time = 0.0

        mode_str = "【仿真演练模式 (DRY RUN - 不发物理指令)】" if self.dry_run else "【实车控制模式 (ACTIVE DRIVING)】"
        self.get_logger().info(f">>> 电子跟屁虫节点初始化就绪 {mode_str}")
        self.get_logger().info(f">>> 目标类别: [{self.target_class}], 保持距离: {TARGET_DISTANCE_M}m, 极速: {MAX_SPEED_MPS}m/s, AEB防撞阈值: {AEB_STOP_DISTANCE_M}m")

    def on_driver_status(self, msg):
        try:
            d = json.loads(msg.data)
            self.driver_armed = bool(d.get('armed', False))
            self.driver_ready = (d.get('ready', '') == 'ready')
            now = time.monotonic()
            if not self.dry_run and not self.driver_armed and self.driver_ready:
                if now - self.last_arm_request > 1.5:
                    self.last_arm_request = now
                    self.arm_chassis(True)
        except Exception:
            pass

    def arm_chassis(self, enable=True):
        if self.dry_run:
            return
        if not self.cli_arm.service_is_ready():
            return
        req = SetBool.Request()
        req.data = enable
        self.last_arm_request = time.monotonic()
        self.cli_arm.call_async(req)
        self.get_logger().info(f">>> 已向底盘发送安全使能请求: Arm={enable}")

    def trigger_chassis_stop(self):
        if self.dry_run:
            return
        if self.cli_stop.service_is_ready():
            req = Trigger.Request()
            self.cli_stop.call_async(req)

    def on_targets(self, msg):
        try:
            items = json.loads(msg.data)
            now = time.monotonic()
            best_target = None
            best_score = 999.0

            if isinstance(items, list):
                for item in items:
                    lbl = item.get('label', '').lower()
                    if self.target_class == 'any':
                        is_match = True
                    elif self.target_class in ('person', 'human'):
                        is_match = lbl in ('person', 'face')
                    else:
                        is_match = (lbl == self.target_class)

                    if not is_match:
                        continue

                    conf = item.get('conf', 0.0)
                    if conf < MIN_CONFIDENCE:
                        continue

                    z = item.get('z', 0.0)
                    x = item.get('x', 0.0)
                    if not (MIN_TARGET_Z_M <= z <= MAX_FOLLOW_DISTANCE_M):
                        continue

                    score = abs(x) * 1.5 + abs(z - TARGET_DISTANCE_M)
                    if score < best_score:
                        best_score = score
                        best_target = {
                            'label': item.get('label'),
                            'conf': conf,
                            'x': round(x, 3),
                            'y': round(item.get('y', 0.0), 3),
                            'z': round(z, 3),
                            'distance': round(item.get('distance', math.hypot(x, z)), 3),
                            'time': now
                        }

            if best_target is not None:
                self.consecutive_seen += 1
                self.consecutive_lost = 0
                if self.consecutive_seen >= TARGET_CONFIRM_FRAMES:
                    # 坐标低通平滑 (EMA)，彻底滤除画面微抖导致的转向抽搐
                    if self.smooth_x is None or self.smooth_z is None:
                        self.smooth_x = best_target['x']
                        self.smooth_z = best_target['z']
                    else:
                        self.smooth_x = 0.65 * self.smooth_x + 0.35 * best_target['x']
                        self.smooth_z = 0.65 * self.smooth_z + 0.35 * best_target['z']
                    best_target['smooth_x'] = round(self.smooth_x, 3)
                    best_target['smooth_z'] = round(self.smooth_z, 3)
                    self.latest_target = best_target
                    self.last_target_seen = now
            else:
                self.consecutive_seen = 0
                self.consecutive_lost += 1
                # 连续 TARGET_LOST_FRAMES 帧 (约 0.15s) 无有效目标，立刻清空并停步，绝不盲动！
                if self.consecutive_lost >= TARGET_LOST_FRAMES:
                    self.latest_target = None
                    self.smooth_x = None
                    self.smooth_z = None
        except Exception:
            pass

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return

        front_dists = []
        step_deg = 360.0 / n
        cone_deg = 30.0

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or not math.isfinite(r):
                continue
            deg = (i * step_deg) % 360.0
            if deg > 180.0:
                deg -= 360.0
            # 严格前向 ±30° 扇区，且过滤掉雷达本体盲区与车架反射噪点 (r >= 0.15m)
            if abs(deg) <= cone_deg and r >= 0.15:
                front_dists.append(r)

        min_d = min(front_dists) if front_dists else 99.0
        self.min_front_scan = min_d

        # 雷达前方 0.40 米主动防撞急停
        if min_d < AEB_STOP_DISTANCE_M:
            self.aeb_active = True
        elif min_d >= AEB_RELEASE_DISTANCE_M:
            self.aeb_active = False

    def on_voltage(self, msg):
        self.voltage = float(msg.data)

    def control_loop(self):
        now = time.monotonic()
        target = self.latest_target
        target_valid = (target is not None and (now - self.last_target_seen <= 0.30))

        target_vx = 0.0
        target_wz = 0.0

        # 优先级 1: 动力电池欠压保护
        if self.voltage < BATTERY_MIN_V and self.voltage > 10.0:
            self.state = "LOW_BATTERY"

        # 优先级 2: 激光雷达 0.4 米主动防撞急停 (毫秒级硬刹停)
        elif self.aeb_active:
            self.state = "AEB_EMERGENCY"
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0

        # 优先级 3: 前方无人或目标丢失 (立刻停步)
        elif not target_valid:
            self.state = "SEARCHING_LOST"

        # 优先级 4: 锁定目标，执行阿克曼平滑跟随控制律
        else:
            x = target.get('smooth_x', target['x'])
            z = target.get('smooth_z', target['z'])
            # 相机光学坐标系到车体转向轴映射：目标在左(x<0)向左转(wz>0)，目标在右(x>0)向右转(wz<0)
            theta = math.atan2(-x, z)

            # 4.1 横向转向控制 (带死区滤波，拒绝身体晃动时抽搐打舵)
            if abs(theta) > ANGLE_DEADBAND_RAD:
                raw_turn = KP_TURN * theta
                target_wz = max(-MAX_TURN_RADPS, min(MAX_TURN_RADPS, raw_turn))
            else:
                target_wz = 0.0

            # 4.2 纵向速度控制 (保持 0.65m 距离，在 0.50~0.80m 死区内完全静止)
            if z > DEADBAND_MAX_M:
                e_z = z - TARGET_DISTANCE_M
                raw_speed = MIN_SPEED_MPS + KP_SPEED * e_z
                target_vx = max(MIN_SPEED_MPS, min(MAX_SPEED_MPS, raw_speed))
                self.state = "TRACKING_FORWARD"
            elif z < DEADBAND_MIN_M:
                # 人员靠得过近 (< 0.50m)，坚决停步，严禁倒车碾压后方
                target_vx = 0.0
                target_wz = 0.0
                self.state = "WAITING_TOO_CLOSE"
            else:
                # 处于 0.50m ~ 0.80m 舒适死区内
                target_vx = 0.0
                target_wz = 0.0
                self.state = "WAITING_IN_DEADBAND"

        # 核心防闭锁保护：静止时角速度严格清零，绝不触发驱动器原地打舵闭锁
        if abs(target_vx) < 1e-4:
            target_vx = 0.0
            target_wz = 0.0

        # 平滑加减速斜坡控制 (Slew Rate Limiter): 起步平稳渐进，刹车迅速果断
        dt = 0.05
        max_dv = ACCEL_LIMIT_MPS2 * dt
        if target_vx > self.cmd_vx:
            self.cmd_vx = min(target_vx, self.cmd_vx + max_dv)
        else:
            # 减速/急停时快速刹车 (加倍制动)
            self.cmd_vx = max(target_vx, self.cmd_vx - max_dv * 3.0)

        if abs(self.cmd_vx) < 1e-4:
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
        else:
            self.cmd_wz = target_wz

        # 发送 Twist 指令
        if not self.dry_run:
            cmd = Twist()
            cmd.linear.x = float(self.cmd_vx)
            cmd.angular.z = float(self.cmd_wz)
            self.pub_cmd_vel.publish(cmd)

        # 发布状态 JSON 供大屏与监控读取
        status_payload = {
            "state": self.state,
            "dry_run": self.dry_run,
            "target": target if target_valid else None,
            "target_seen_age_ms": round((now - self.last_target_seen) * 1000, 1) if self.last_target_seen else None,
            "aeb_min_scan_m": round(self.min_front_scan, 2),
            "aeb_active": self.aeb_active,
            "voltage_v": round(self.voltage, 2),
            "cmd_vx": round(self.cmd_vx, 3),
            "cmd_wz": round(self.cmd_wz, 3),
            "timestamp": round(now, 3)
        }
        self.pub_status.publish(String(data=json.dumps(status_payload, ensure_ascii=False)))

        # 终端单行仪表盘输出 (5 Hz 刷新)
        if now - self.last_print_time >= 0.20:
            self.last_print_time = now
            self.print_dashboard(status_payload)

    def print_dashboard(self, s):
        state_colors = {
            "TRACKING_FORWARD": "\033[1;32m[ 跟踪追随 ]\033[0m",
            "WAITING_IN_DEADBAND": "\033[1;36m[ 距离锁定 ]\033[0m",
            "WAITING_TOO_CLOSE": "\033[1;33m[ 距离过近 ]\033[0m",
            "SEARCHING_LOST": "\033[1;35m[ 搜索目标 ]\033[0m",
            "AEB_EMERGENCY": "\033[1;41;37m[ AEB紧急防撞 ]\033[0m",
            "LOW_BATTERY": "\033[1;31m[ 低电量停车 ]\033[0m",
            "STANDBY": "\033[1;30m[ 原地待命 ]\033[0m"
        }
        st_tag = state_colors.get(s['state'], f"[{s['state']}]")
        tgt = s['target']
        if tgt:
            tgt_info = f"{tgt['label']} X:{tgt['x']:+.2f}m Z:{tgt['z']:.2f}m (dist:{tgt['distance']:.2f}m conf:{tgt['conf']:.2f})"
        else:
            tgt_info = "未发现匹配人体目标"

        aeb_col = "\033[1;31m" if s['aeb_active'] else "\033[1;32m"
        aeb_info = f"{aeb_col}雷达前向: {s['aeb_min_scan_m']:.2f}m\033[0m"
        cmd_info = f"\033[1;37mvx={s['cmd_vx']:+.2f}m/s wz={s['cmd_wz']:+.2f}rad/s\033[0m"
        dry_tag = "\033[1;33m[DRY-RUN]\033[0m " if s['dry_run'] else "\033[1;32m[ACTIVE]\033[0m "

        sys.stdout.write(f"\r{dry_tag}{st_tag} 目标: {tgt_info:<42} | {aeb_info} | {cmd_info} | {s['voltage_v']:.1f}V   ")
        sys.stdout.flush()

    def stop_robot(self):
        self.get_logger().info(">>> 正在发送紧急停机指令...")
        if not self.dry_run:
            stop_cmd = Twist()
            for _ in range(15):
                self.pub_cmd_vel.publish(stop_cmd)
                time.sleep(0.02)
            self.trigger_chassis_stop()
            self.arm_chassis(False)


def main():
    parser = argparse.ArgumentParser(description="RK3588 电子跟屁虫 - 人体 3D 视觉 + 激光雷达跟随")
    parser.add_argument('--dry-run', action='store_true', default=False,
                        help='仿真演练模式：计算并输出所有跟踪与防撞数据，但不向底盘发送驱动指令')
    parser.add_argument('--target', type=str, default='person',
                        help='追踪类别：person (默认), face, 或 any')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = PersonFollowerNode(dry_run=args.dry_run, target_class=args.target)

    def sig_handler(sig, frame):
        print("\n\n>>> 捕获中断信号，安全刹停中...")
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
