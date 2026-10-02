#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线闭环仿真：真实 LineTracker + 阿克曼底盘模型（Chassis）。

只用于离线调参和回归，不代表实车验收。定位有两种来源：

run()       "真值 + 噪声 + 残余延迟"，直接当作融合输出，用来单独看控制器：
            - 位置误差 = 慢变（一阶高斯-马尔可夫，模拟 RTK 固定解的缓慢漂移）+ 白噪声；
            - 航向误差 = 白噪声 + 固定偏差（安装角没标准的残差）；
            - 融合已用陀螺补偿了 GNSS 延迟，这里只留 30 ms 残余。
run_fused() 原始传感器 → 真实融合模块（gnss_fusion），走车上同一条数据链：
            底盘真值 → 合成 UM982 日志（BESTPOSA/BESTNAVHA/HEADINGA/BESTVELA，带 CRC，20 Hz，输出延迟 22–32 ms）
            → um982.Um982Stream（GPS 时间换算）→ LineMission 的输入转换（和 line_node 同一段代码）
            → GnssFusion → LineTracker。IMU 有零偏、噪声、量化和偶发尖峰，轮速有 2% 比例误差和 1 mm/s
            量化，两者都按收到时刻打时间戳（line_node 就是这么做的）。开始前先静止 3 s 让融合估零偏。
两种来源共用的底盘：指令延迟 60 ms、驱动加速斜坡、舵机速率，可加舵机零位偏差和转角量化；
路面可加随机曲率扰动（坑洼、打滑）。

    python3 radar_system/tools/line_sim.py              # 全部场景（真值 + 噪声）
    python3 radar_system/tools/line_sim.py --fused      # 全部场景（原始传感器 + 真实融合）
    python3 radar_system/tools/line_sim.py trim_2deg --trace
"""
import argparse
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

from line_tracker import LineConfig, LineTracker  # noqa: E402
from robot_core.config import CONTROL_DT  # noqa: E402
from robot_core.kinematics import ChassisGeometry, steer_from_yaw, yaw_from_steer  # noqa: E402
from runtime_config import PROFILE  # noqa: E402

DT = 0.01


class Chassis:
    """阿克曼底盘模型：指令延迟、驱动加速斜坡、舵机限速、电机一阶响应、舵机零位偏差与分辨率。"""

    def __init__(self, delay_s=0.06, accel=PROFILE['driver']['acceleration_m_s2'],
                 decel=1.5, servo_radps=3.0, tau_s=0.10, trim_rad=0.0, steer_quant_rad=0.0):
        self.geo = ChassisGeometry()
        # 舵机零位偏差（打直时车轮实际偏的角度）和舵机分辨率，走直线仿真用
        self.trim, self.quant = trim_rad, steer_quant_rad
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.steer = 0.0
        self.delay_s, self.accel, self.decel = delay_s, accel, decel
        self.servo, self.tau = servo_radps, tau_s
        self.queue = deque()
        self.cmd = (0.0, 0.0)
        self.driver_v = 0.0
        self.wz = 0.0

    def command(self, t, vx, wz):
        self.queue.append((t + self.delay_s, vx, wz))

    def step(self, t, dt):
        while self.queue and self.queue[0][0] <= t:
            _, vx, wz = self.queue.popleft()
            self.cmd = (vx, wz)
        vx, wz = self.cmd
        # 驱动:零/换向立即清零,否则按加速度斜坡
        if vx == 0 or self.driver_v * vx < 0:
            self.driver_v = 0.0
        else:
            step = PROFILE['driver']['acceleration_m_s2'] * dt
            self.driver_v += max(-step, min(step, vx - self.driver_v))
        if abs(self.driver_v) > 0.02:
            target_steer = steer_from_yaw(self.driver_v, wz, self.geo)
        else:
            # 速度为 0 时 Twist 协议无法表达转角,固件保持当前舵角
            target_steer = self.steer
        ds = self.servo * dt
        self.steer += max(-ds, min(ds, target_steer - self.steer))
        # 电机:一阶响应,加速/刹车能力分别受限
        want = self.driver_v
        dv = (want - self.v) * min(1.0, dt / self.tau)
        speeding_up = abs(want) > abs(self.v) and want * self.v >= 0
        lim = (self.accel if speeding_up else self.decel) * dt
        self.v += max(-lim, min(lim, dv))
        if abs(self.v) < 1e-4 and want == 0:
            self.v = 0.0
        wheel = self.steer
        if self.quant > 0:
            wheel = round(wheel / self.quant) * self.quant
        self.wz = yaw_from_steer(self.v, wheel + self.trim, self.geo)
        self.x += self.v * dt * math.cos(self.yaw + self.wz * dt / 2)
        self.y += self.v * dt * math.sin(self.yaw + self.wz * dt / 2)
        self.yaw += self.wz * dt

    def to_body(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = wx - self.x, wy - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def to_world(self, bx, by):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * bx - s * by, self.y + s * bx + c * by


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


def run(name='nominal', *, length=20.0, cruise=0.5, start=(-1.5, 0.0, 0.0), trim_deg=0.0,
        quant_deg=0.2, pos_drift=0.006, pos_white=0.003, yaw_white_deg=0.1, yaw_bias_deg=0.0,
        residual_latency=0.03, outage=None, terrain=0.0, config=None, seed=1, trace=False,
        max_time=None, integral0=None):
    """start=(沿线, 横偏, 车头误差°) 相对 A 点；返回指标字典。"""
    rng = random.Random(seed)
    cfg = config or LineConfig(cruise_mps=cruise)
    tracker = LineTracker(cfg)
    a, b = (0.0, 0.0), (length, 0.0)
    tracker.set_line(a, b)
    car = Chassis(trim_rad=math.radians(trim_deg), steer_quant_rad=math.radians(quant_deg))
    car.x, car.y, car.yaw = start[0], start[1], math.radians(start[2])
    drift_x, drift_y = GaussMarkov(pos_drift, 3.0, rng), GaussMarkov(pos_drift, 3.0, rng)
    bumps = GaussMarkov(terrain, 0.5, rng)
    history = deque()

    def measured(t):
        # 残余延迟后的真值 + 误差
        while len(history) > 1 and history[1][0] <= t - residual_latency:
            history.popleft()
        _, x, y, yaw, v, wz = history[0]
        usable = not (outage and outage[0] <= t < outage[1])
        return SimpleNamespace(
            x=x + drift_x.value + rng.gauss(0, pos_white), y=y + drift_y.value + rng.gauss(0, pos_white),
            yaw=yaw + math.radians(yaw_bias_deg + rng.gauss(0, yaw_white_deg)),
            speed=v, yaw_rate=wz + rng.gauss(0, math.radians(0.03)), usable_for_control=usable)

    t, next_ctrl = 0.0, 0.0
    started = False
    rows = []
    prev_steer, steer_rates = car.steer, []
    stopped_at = outage_x = None
    max_time = max_time or (length / cruise + 25.0)
    while t < max_time:
        history.append((t, car.x, car.y, car.yaw, car.v, car.wz))
        if outage and outage_x is None and t >= outage[0]:
            outage_x = car.x
        drift_x.step(DT)
        drift_y.step(DT)
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = measured(t)
            if not started:
                started = tracker.start(pose, integral=integral0)
                if not started:
                    break
            vx, wz = tracker.update(pose, CONTROL_DT)
            car.command(t, vx, wz)
            st = tracker.st
            rows.append(dict(t=t, x=car.x, y=car.y, e_true=car.y, e_ctrl=st.e, state=st.state,
                             v=car.v, steer_deg=math.degrees(car.steer), v_cmd=vx))
            if st.state in ('DONE', 'ABORTED') and abs(car.v) < 1e-3:
                break
            if st.state == 'PAUSED' and stopped_at is None and abs(car.v) < 1e-3:
                stopped_at = t
        car.step(t, DT)
        if terrain > 0:
            car.yaw += bumps.step(DT) * car.v * DT
        steer_rates.append(abs(car.steer - prev_steer) / DT)
        prev_steer = car.steer
        t += DT

    return _summary(name, rows, tracker, car, length, t, steer_rates, started=started, stopped_at=stopped_at,
                    outage_x=outage_x, trace=trace)


def _summary(name, rows, tracker, car, length, t, steer_rates, *, started, stopped_at, outage_x, trace):
    """横偏（真值）统计：助跑后 2 m 起算，settled 从 6 m 起算，都到 B 前 0.5 m 为止。"""
    track = [r for r in rows if 2.0 <= r['x'] <= length - 0.5 and r['state'] == 'RUNNING']
    settled = [r for r in rows if 6.0 <= r['x'] <= length - 0.5 and r['state'] == 'RUNNING']

    def stat(sel, key):
        vals = [r[key] for r in sel]
        if not vals:
            return None, None
        return math.sqrt(sum(v * v for v in vals) / len(vals)), max(abs(v) for v in vals)

    rms, peak = stat(track, 'e_true')
    rms6, peak6 = stat(settled, 'e_true')
    rms_ctrl, _ = stat(track, 'e_ctrl')
    result = dict(
        name=name, state=tracker.st.state, reason=tracker.st.reason, started=started,
        rms_cm=None if rms is None else round(rms * 100, 2),
        max_cm=None if peak is None else round(peak * 100, 2),
        settled_max_cm=None if peak6 is None else round(peak6 * 100, 2),
        settled_rms_cm=None if rms6 is None else round(rms6 * 100, 2),
        ctrl_rms_cm=None if rms_ctrl is None else round(rms_ctrl * 100, 2),
        end_err_cm=round((car.x - length) * 100, 1),
        steer_rate_rms_dps=round(math.degrees(math.sqrt(sum(r * r for r in steer_rates)
                                                      / max(1, len(steer_rates)))), 2),
        time_s=round(t, 2), paused_stop_s=stopped_at, integral=tracker.st.integral,
        stop_x=round(car.x, 3), outage_x=outage_x,
        trim_learned_deg=round(math.degrees(math.atan(tracker.status()['trim_kappa']
                                                      * tracker.geo.wheelbase_m)), 2))
    if trace:
        result['rows'] = rows
    return result


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


def run_fused(name='nominal', *, length=20.0, cruise=0.5, start=(-1.5, 0.0, 0.0), trim_deg=0.0, quant_deg=0.2,
              pos_drift=0.006, pos_white=0.003, hdg_drift_deg=0.1, hdg_white_deg=0.25, vel_noise=0.02,
              mount_deg=1.5, mount_err_deg=0.0, wheel_error=0.02, gyro_bias_dps=0.1, gyro_noise_dps=0.034,
              spikes=True, imu_delay=0.02, wheel_delay=0.03, outage=None, heading_outage=None, terrain=0.0,
              config=None, seed=1, trace=False, max_time=None, integral0=None, warmup_s=3.0):
    """原始传感器 + 真实融合的闭环。时间轴含开头 warmup_s 秒静止，outage 等时刻按这条时间轴给。

    mount_deg 是天线实际的安装偏差，mount_err_deg 是融合配置里没标定准的部分。
    """
    import gnss_fusion
    rng = random.Random(seed)
    cfg = config or LineConfig(cruise_mps=cruise)
    tracker = LineTracker(cfg)
    tracker.set_line((0.0, 0.0), (length, 0.0))
    car = Chassis(trim_rad=math.radians(trim_deg), steer_quant_rad=math.radians(quant_deg))
    car.x, car.y, car.yaw = start[0], start[1], math.radians(start[2])
    source = FusedSource(
        seed, gnss_fusion.FusionConfig(mount_offset_deg=mount_deg - mount_err_deg),
        dict(pos_drift=pos_drift, pos_white=pos_white, hdg_drift_deg=hdg_drift_deg, hdg_white_deg=hdg_white_deg,
             vel_noise=vel_noise, mount_deg=mount_deg, outage=outage, heading_outage=heading_outage),
        dict(gyro_bias_dps=gyro_bias_dps, gyro_noise_dps=gyro_noise_dps, spikes=spikes, wheel_error=wheel_error,
             imu_delay=imu_delay, wheel_delay=wheel_delay))
    bumps = GaussMarkov(terrain, 0.5, rng)

    t, next_ctrl = 0.0, 0.0
    started = False
    rows = []
    prev_steer, steer_rates = car.steer, []
    stopped_at = outage_x = None
    max_time = max_time or (warmup_s + length / cruise + 25.0)
    while t < max_time:
        source.step(t, car)
        if outage and outage_x is None and t >= outage[0]:
            outage_x = car.x
        if t >= next_ctrl - 1e-9:
            next_ctrl += CONTROL_DT
            pose = source.pose(t)
            if not started and t >= warmup_s:
                started = tracker.start(pose, integral=integral0)
                if not started and t > warmup_s + 5.0:
                    break                   # 融合一直不可用
            if started:
                vx, wz = tracker.update(pose, CONTROL_DT)
                car.command(t, vx, wz)
                st = tracker.st
                rows.append(dict(t=t, x=car.x, y=car.y, e_true=car.y, e_ctrl=st.e, state=st.state,
                                 v=car.v, steer_deg=math.degrees(car.steer), v_cmd=vx,
                                 quality=getattr(pose, 'quality', ''), usable=pose.usable_for_control,
                                 pos_err=math.hypot(pose.x - car.x, pose.y - car.y) if pose is not NO_POSE else None,
                                 yaw_err_deg=math.degrees(math.atan2(math.sin(pose.yaw - car.yaw),
                                                                     math.cos(pose.yaw - car.yaw)))))
                if st.state in ('DONE', 'ABORTED') and abs(car.v) < 1e-3:
                    break
                if st.state == 'PAUSED' and stopped_at is None and abs(car.v) < 1e-3:
                    stopped_at = t
        car.step(t, DT)
        if terrain > 0:
            car.extra_wz = bumps.step(DT) * car.v
            car.yaw += car.extra_wz * DT
        steer_rates.append(abs(car.steer - prev_steer) / DT)
        prev_steer = car.steer
        t += DT

    result = _summary(name, rows, tracker, car, length, t, steer_rates, started=started, stopped_at=stopped_at,
                      outage_x=outage_x, trace=trace)
    usable = [r for r in rows if r['usable'] and r['pos_err'] is not None]
    result['fusion_pos_max_cm'] = round(max(r['pos_err'] for r in usable) * 100, 2) if usable else None
    result['fusion_yaw_max_deg'] = round(max(abs(r['yaw_err_deg']) for r in usable), 3) if usable else None
    result['calibration'] = source.fusion.calibration()
    return result


FUSED_SCENARIOS = {
    'nominal': dict(),
    'offset_start': dict(start=(-2.5, 0.30, 10.0)),
    'trim_2deg': dict(trim_deg=2.0),
    'fast': dict(cruise=0.8),
    'noisy': dict(pos_drift=0.012, pos_white=0.006, hdg_drift_deg=0.2, hdg_white_deg=0.5),
    'outage': dict(outage=(15.0, 16.0)),                    # 起步后约 12 s RTK 失锁 1 s
    'rough': dict(terrain=0.03),
    'ideal': dict(pos_drift=0.0, pos_white=0.0, hdg_drift_deg=0.0, hdg_white_deg=0.0, vel_noise=0.0,
                  quant_deg=0.0, wheel_error=0.0, gyro_bias_dps=0.0, gyro_noise_dps=0.0, spikes=False,
                  imu_delay=0.0, wheel_delay=0.0),
    'mount_1deg': dict(mount_err_deg=1.0),                  # 天线安装偏差没标定准 1°
    'heading_loss': dict(heading_outage=(8.0, 18.0)),       # 双天线航向丢 10 s，靠陀螺和 RTK 轨迹
}


SCENARIOS = {
    'nominal': dict(),
    'offset_start': dict(start=(-2.5, 0.30, 10.0)),
    'trim_2deg': dict(trim_deg=2.0),
    'fast': dict(cruise=0.8),
    'noisy': dict(pos_drift=0.012, pos_white=0.006, yaw_white_deg=0.3),
    'outage': dict(outage=(12.0, 13.0)),
    'rough': dict(terrain=0.03),
    'ideal': dict(pos_drift=0.0, pos_white=0.0, yaw_white_deg=0.0, quant_deg=0.0),
}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('names', nargs='*')
    p.add_argument('--trace', action='store_true')
    p.add_argument('--fused', action='store_true', help='原始传感器 + 真实融合模块（gnss_fusion）')
    p.add_argument('--seed', type=int, default=1)
    args = p.parse_args()
    table, runner = (FUSED_SCENARIOS, run_fused) if args.fused else (SCENARIOS, run)
    names = args.names or list(table)
    keys = ('state', 'rms_cm', 'max_cm', 'settled_max_cm', 'ctrl_rms_cm', 'end_err_cm',
            'steer_rate_rms_dps', 'trim_learned_deg', 'time_s')
    if args.fused:
        keys += ('fusion_pos_max_cm', 'fusion_yaw_max_deg')
    print('%-13s' % 'scenario' + ''.join('%17s' % k for k in keys))
    for n in names:
        r = runner(n, trace=args.trace, seed=args.seed, **table[n])
        print('%-13s' % n + ''.join('%17s' % (r[k],) for k in keys))
        if args.trace:
            for row in r['rows'][::10]:
                print('  t=%5.2f x=%6.2f e=%+6.2fcm steer=%+5.2f° v=%.2f %s' % (
                    row['t'], row['x'], row['e_true'] * 100, row['steer_deg'], row['v'], row['state']))


if __name__ == '__main__':
    main()
