#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真用的传感器模型：原始传感器 → 真实融合（gnss_fusion），走车上同一条数据链。

只用于离线调参和回归，不代表实车验收。
    底盘真值 → 合成 UM982 日志（BESTPOSA/BESTNAVHA/HEADINGA/BESTVELA，带 CRC，20 Hz，输出延迟 22–32 ms）
    → um982.Um982Stream（GPS 时间换算）→ LineMission 的输入转换（和 line_node 同一段代码）→ GnssFusion。
    IMU 有零偏、噪声、量化和偶发尖峰，轮速有 2% 比例误差和 1 mm/s 量化，两者都按收到时刻打时间戳（line_node 就是这么做的）。
Receiver / MotionSensors / FusedSource 给 path_sim（Gen5 路线闭环）和 cockpit 的 tools/car_sim_gen5 等用。
2026-11-23 删了阿克曼车：原来的走直线闭环（LineTracker + 阿克曼底盘 Chassis，run / run_fused）一起删了。
"""
import heapq
import json
import math
import os
import random
import sys
from collections import deque
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from robot_core.config import CONTROL_DT  # noqa: E402,F401  （path_sim 从这里取）

DT = 0.01


class GaussMarkov:
    def __init__(self, sigma, tau_s, rng):
        self.sigma, self.tau, self.rng = sigma, tau_s, rng
        self.value = rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def step(self, dt):
        if self.sigma <= 0:
            return 0.0
        a = math.exp(-dt / self.tau)
        self.value = a * self.value + math.sqrt(1 - a * a) * self.rng.gauss(0.0, self.sigma)
        return self.value


# =============================================================================
# 原始传感器 + 真实融合
# =============================================================================

ORIGIN = (30.0, 120.0, 10.0)            # 虚构的作业原点（北纬 30°、东经 120°，椭球高 10 m）
GPS_T0 = 2437 * 604800.0 + 443000.0     # 仿真 t=0 对应的 GPS 时间（任取）
ANTENNA = (0.345, 0.165)                # ANT1 在车体系的位置（后轴中心为原点），与 FusionConfig 默认一致
SAMPLE_DT = 0.05                        # 接收机、IMU、轮速都是 20 Hz
NO_POSE = SimpleNamespace(x=0.0, y=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0, usable_for_control=False,
                          quality='INVALID', sigma_xy=math.nan, sigma_yaw=math.nan)


def _inside(t, window):
    return window is not None and window[0] <= t < window[1]


def unicore_log(name, t_gps, body):
    """拼一行带 CRC 的 Unicore ASCII 日志；头部只有周和周内秒参与解析。"""
    import um982
    week = int(t_gps // um982.GPS_WEEK_S)
    text = '%s,COM2,0,80.0,FINE,%d,%.3f,0,20,18;%s' % (name, week, t_gps - week * um982.GPS_WEEK_S, body)
    return '#%s*%08x\r\n' % (text, um982.crc32(text.encode()))


def secondary_log(t_gps, body):
    """副天线 BESTNAVHA：N4 原生日志头，周数早一列、周内时间单位毫秒（与车上接收机相同）。"""
    import um982
    week = int(t_gps // um982.GPS_WEEK_S)
    text = 'BESTNAVHA,97,GPS,FINE,%d,%d,0,0,18,13;%s' % (week, round((t_gps - week * um982.GPS_WEEK_S) * 1000), body)
    return '#%s*%08x\r\n' % (text, um982.crc32(text.encode()))


class Receiver:
    """按底盘真值合成 UM982 COM2 的四条 20 Hz 日志，格式与车上接收机相同。

    副天线（BESTNAVHA）位置 = ANT1 测量位置 + 基线向量（ANT1→ANT2，方向同 HEADING）+ 白噪声，
    rtk 节点按两天线固定解的实时距离核对测向，仿真里也要有它。

    ANT1 位置 = 真值 + 慢漂（τ 3 s）+ 白噪声；航向 = 真值 + 天线安装偏差 + 慢漂（τ 5 s）+ 白噪声，
    按 ANT1→ANT2、正北顺时针报出；速度是 ANT1 的对地速度（含 ω×杆臂）。
    输出时刻 = 测量时刻 + 解算 20 ms + 串口/调度 2–12 ms。outage 内 RTK 失锁（位置单点、航向无解）。
    """

    def __init__(self, rng, *, pos_drift=0.006, pos_white=0.003, hdg_drift_deg=0.1, hdg_white_deg=0.25,
                 vel_noise=0.02, mount_deg=1.5, baseline_m=0.331, outage=None, heading_outage=None):
        import gnss_fusion
        self.rng = rng
        self.frame = gnss_fusion.GnssFusion()       # 只借用它的经纬度换算
        self.frame.set_origin(*ORIGIN)
        self.drift = [GaussMarkov(pos_drift, 3.0, rng), GaussMarkov(pos_drift, 3.0, rng),
                      GaussMarkov(hdg_drift_deg, 5.0, rng)]
        self.pos_white, self.hdg_white, self.vel_noise = pos_white, hdg_white_deg, vel_noise
        self.mount, self.baseline = mount_deg, baseline_m
        self.outage, self.heading_outage = outage, heading_outage
        # 副天线噪声用单独的随机源（种子取自主随机源的状态但不消耗它），不打乱其余噪声序列
        self.rng2 = random.Random(rng.getstate()[1][0])

    def measure(self, t, car):
        """测量时刻 t 的 BESTPOSA/HEADINGA/BESTVELA；返回 [(到达时刻, 字节)]。"""
        rng = self.rng
        for g in self.drift:
            g.step(SAMPLE_DT)
        c, s = math.cos(car.yaw), math.sin(car.yaw)
        ax, ay = ANTENNA
        lost = _inside(t, self.outage)
        east = car.x + c * ax - s * ay + self.drift[0].value + rng.gauss(0.0, self.pos_white)
        north = car.y + s * ax + c * ay + self.drift[1].value + rng.gauss(0.0, self.pos_white)
        lat, lon = self.frame.to_geodetic(east, north)
        pos = ('SOL_COMPUTED,SINGLE,%.11f,%.11f,%.4f,0.0000,WGS84,1.2000,1.2000,2.5000,"0",0.000,0.000,40,32,32,0,1,12,11,51'
               if lost else
               'SOL_COMPUTED,NARROW_INT,%.11f,%.11f,%.4f,0.0000,WGS84,0.0080,0.0080,0.0150,"2001",1.000,0.000,40,32,32,0,1,12,11,51'
               ) % (lat, lon, ORIGIN[2])
        heading = (180.0 - math.degrees(car.yaw) + self.mount + self.drift[2].value
                   + rng.gauss(0.0, self.hdg_white)) % 360.0
        hdg = 'SOL_COMPUTED,%s,%.4f,%.4f,0.1000,0.0000,0.2800,0.5100,"999",37,32,32,0,3,01,1,41' % (
            'NONE' if lost or _inside(t, self.heading_outage) else 'L1_INT',
            self.baseline + rng.gauss(0.0, 0.001), heading)
        b = math.radians(90.0 - heading)            # ANT1→ANT2 的 ENU 方向
        lat2, lon2 = self.frame.to_geodetic(east + self.baseline * math.cos(b) + self.rng2.gauss(0.0, self.pos_white),
                                            north + self.baseline * math.sin(b) + self.rng2.gauss(0.0, self.pos_white))
        pos2 = ('SOL_COMPUTED,%s,%.11f,%.11f,%.4f,0.0000,WGS84,%s,"0",1.000,0.000,38,30,0,0,0,0,0,0,'
                'SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0,0,0,0') % (
                    'SINGLE' if lost else 'NARROW_INT', lat2, lon2, ORIGIN[2],
                    '1.2000,1.2000,2.5000' if lost else '0.0080,0.0080,0.0150')
        wz = car.wz + getattr(car, 'extra_wz', 0.0)
        vx = car.v - wz * ay + rng.gauss(0.0, self.vel_noise)
        vy = wz * ax + rng.gauss(0.0, self.vel_noise)
        track = (90.0 - math.degrees(car.yaw + math.atan2(vy, vx))) % 360.0
        vel = 'SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,%.4f,%.6f,0.0000,0' % (math.hypot(vx, vy), track)
        t_gps = GPS_T0 + t
        t_rx = t + 0.020 + rng.uniform(0.002, 0.012)
        logs = [unicore_log('BESTPOSA', t_gps, pos), secondary_log(t_gps, pos2), unicore_log('HEADINGA', t_gps, hdg),
                unicore_log('BESTVELA', t_gps, vel)]
        return [(t_rx + 0.001 * k, line.encode()) for k, line in enumerate(logs)]


class MotionSensors:
    """IMU（z 轴陀螺）与轮速：按收到时刻打时间戳，读数是 delay 之前的真值（驱动和 ROS 传输延迟）。"""

    GYRO_LSB = 0.00026644                  # rad/s，六轴芯片 ±500°/s 量程的分辨率

    def __init__(self, rng, *, gyro_bias_dps=0.1, gyro_noise_dps=0.034, spikes=True, wheel_error=0.02,
                 imu_delay=0.02, wheel_delay=0.03):
        self.rng = rng
        self.bias, self.noise = math.radians(gyro_bias_dps), math.radians(gyro_noise_dps)
        self.spikes, self.wheel_error = spikes, wheel_error
        self.imu_delay, self.wheel_delay = imu_delay, wheel_delay
        self.hist = deque()                # (t, v, 舵角算出的角速度, 车身实际角速度)
        self.count = 0

    def record(self, t, car):
        # 路面扰动（坑洼、打滑）带来的转动陀螺测得到，底盘按舵角算的里程计角速度测不到
        self.hist.append((t, car.v, car.wz, car.wz + getattr(car, 'extra_wz', 0.0)))
        while len(self.hist) > 2 and self.hist[1][0] <= t - 0.5:
            self.hist.popleft()

    def _at(self, t):
        best = self.hist[0]
        for h in self.hist:
            if h[0] > t + 1e-9:
                break
            best = h
        return best

    def imu(self, t):
        self.count += 1
        gz = self._at(t - self.imu_delay)[3] + self.bias + self.rng.gauss(0.0, self.noise)
        if self.spikes and self.count % 800 == 311:
            gz -= math.radians(3.8)        # 实测偶发的单点尖峰
        return round(gz / self.GYRO_LSB) * self.GYRO_LSB

    def wheel(self, t):
        _, v, wz, _ = self._at(t - self.wheel_delay)
        return round(v * (1.0 + self.wheel_error), 3), round(wz, 3)


class FusedSource:
    """原始传感器经 Um982Stream、LineMission 的输入转换送进真实融合；pose(t) 就是 line_node 拿到的位姿。"""

    def __init__(self, seed, fusion_config, receiver_kw, motion_kw):
        import gnss_fusion
        import um982
        from line_mission import LineMission
        rng = random.Random(seed + 7777)
        self.fusion = gnss_fusion.GnssFusion(fusion_config)
        self.fusion.set_origin(*ORIGIN)
        self.stream = um982.Um982Stream()
        self.inputs = LineMission(self.fusion, gnss_fusion)     # 只用它的 on_gnss / on_imu / on_wheel
        self.receiver = Receiver(rng, **receiver_kw)
        self.motion = MotionSensors(rng, **motion_kw)
        self.pending, self.seq, self.next_sample = [], 0, 0.0

    def step(self, t, car):
        self.motion.record(t, car)
        if t >= self.next_sample - 1e-9:
            self.next_sample += SAMPLE_DT
            self.inputs.on_imu(t, self.motion.imu(t))
            self.inputs.on_wheel(t, *self.motion.wheel(t))
            for t_rx, data in self.receiver.measure(t, car):
                heapq.heappush(self.pending, (t_rx, self.seq, data))
                self.seq += 1
        while self.pending and self.pending[0][0] <= t:
            t_rx, _, data = heapq.heappop(self.pending)
            for m in self.stream.feed(data, t_rx):
                # 经过一次 JSON 往返，和 /gnss/raw 话题上的消息一样
                self.inputs.on_gnss(json.loads(json.dumps(m, allow_nan=False)))

    def pose(self, t):
        p = self.fusion.pose(t)
        return NO_POSE if p is None else p
