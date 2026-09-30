#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IMU 定向直行：app 点「开始」，车以当时的 IMU 偏航为目标、按固定车速一直往前开，点「停止」才停。

RTK 不参与：方向只看 /imu/data 的偏航角（Gen5 是 HDA536T 自己算的角度，上电为 0 的相对值；旧车是
imu_filter_madgwick），同一条消息的陀螺 z 用来估持续偏转。不依赖 ROS，radar_web_server 和 app 演示服务器共用。

控制（每收到一条 app 指令、以及 50 Hz 手动发布循环各算一次，积分按真实时间间隔累加）：
    e      = 偏航 − 目标（左偏为正，−180…180°），取最近一帧 IMU（时刻 t_imu）
    已发未转 = ∫ wz 指令 dt（t_imu − delay_s → 现在）：已经发出、车头还没转出来的角度（Smith 预估）
    残差   = 陀螺z − delay_s 前的 wz 指令：指令解释不了的转动（两轮不一致、万向轮、地面）
    偏转   = I + kd·残差低通（drift_tc）：持续偏转的估计
    e_pred = e + 已发未转 + (delay_s + 现在 − t_imu)·偏转      —— 指令生效时车头会偏多少
    wz     = −(kp·e_pred + I) − kd·残差，限幅 max_wz；I += ki·e_pred·dt
限幅时、车没走起来时（起步斜坡前半段；有里程计且车速不到指令一半，即轮子卡住）不累积 I，松开后不猛甩。
为什么要预估：2026-09-30 两次实车直行，陀螺 ≈ 0.44 s 前的指令 × 1.02–1.04；旧控制器（kp 3、ki 1.2、kd 0.2
乘陀螺，按 0.1–0.3 s 延迟设计）在 0.44 s 下纠偏过头，中途持续摆 0.54–0.62 Hz、来回 1.8–2.6°。
kp 2.2、ki 1.0、kd 0.4 是在延迟 0.3–0.7 s、增益 0.9–1.2、偏转 ±0.02 rad/s、万向轮低频偏转（来回 2°）上
仿真挑的（tests/test_heading_hold.py）：0.1 Hz 以下的慢摆标称来回约 0.44°、网格都在 ±1° 内；按实测谱
（到 0.32 Hz）约 1.2°——那部分受 0.44 s 延迟所限，任何参数都压不到 0.5°。延迟 0.1–0.3 s 的底盘也稳。
起步车速按 accel 斜坡升到 speed，减小两轮起步不同步甩出去的角度。

哪些情况停车（这一次作业结束，要再点开始；再点开始按那时的偏航重新定目标）：
- 开始时没有 IMU 航向（no_imu）、途中 IMU 超过 imu_stale_s 没更新（imu_stale）。调用方自己卡住（两次计算隔
  超过 stall_s，2026-09-30 实车 web 进程每隔几秒卡 0.2–0.4 s，IMU 其实一直 50 Hz）时 IMU 回调还压在队列里，
  卡完先给 imu_grace_s 让它追上，追不上再停；
- 偏差超过 abort_deg（deviation）：被推歪、IMU 重启归零、方向装反（正反馈会越拐越偏），不能再开；
- 其余由调用方结束：app 点停止（user）、指令中断（command_timeout）、急停（estop）、
  摇杆或网页接管（manual）、平板断线（disconnected）、底盘不收指令（chassis）。

保证的是「IMU 读数」稳在目标 ±1° 内。IMU 本身的零偏漂移会让真实车头跟着偏同样的角度：
2026-09-30 实车 HDA536T 静止 84 s 只漂了 0.02°（约 0.03°/分钟），开起来的漂移还没实测。
"""
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

DEG = math.pi / 180.0

# 停车原因（app 按这些代码显示中文）
REASONS = ('user', 'no_imu', 'imu_stale', 'deviation', 'command_timeout', 'estop', 'manual', 'disconnected',
           'chassis')


@dataclass(frozen=True)
class HoldConfig:
    speed: float = 0.8              # 固定车速，m/s（用户定的：2026-09-30 下午由 0.4 提到 0.8）
    kp: float = 2.2                 # 预估偏差 → 角速度，1/s
    ki: float = 1.0                 # 积分，1/s²；消掉两轮不一致、万向轮带来的持续偏转
    kd: float = 0.4                 # 预估偏差变化率（陀螺 − delay_s 前的指令）→ 角速度，s
    delay_s: float = 0.44           # 底盘转向延迟（指令 → 陀螺），s；0 = 不做延迟补偿
    drift_tc: float = 1.5           # 陀螺残差（持续偏转）低通时间常数，s
    max_wz: float = 0.35            # 纠偏角速度上限，rad/s
    i_max: float = 0.15             # 积分项上限，rad/s
    moving_ratio: float = 0.5       # 指令车速不到 speed 的这个比例（起步）、里程计不到指令的这个比例（卡住）不积分
    odom_stale_s: float = 0.3       # 里程计这么久没更新就不看它（只按起步斜坡判断）
    accel: float = 0.5              # 起步车速斜坡，m/s²
    min_speed: float = 0.05         # 起步第一帧的车速，m/s（0 会被底盘当成停车）
    imu_stale_s: float = 0.2        # IMU 这么久没更新就停
    stall_s: float = 0.1            # 两次计算隔这么久算调用方自己卡了（正常 20 ms 一次）
    imu_grace_s: float = 0.15       # 卡完给 IMU 回调追上的时间，这段时间 IMU 旧了先不停
    abort_deg: float = 5.0          # 偏差超过它就停
    tolerance_deg: float = 1.0      # 目标精度，只用于统计（线内时间占比）


def wrap(a):
    """弧度归到 (−π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    """四元数 → 偏航（弧度，REP-103 左转为正）；四元数无效返回 None。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class HeadingHold:
    """一次一次的「直行」作业：run 是 app 每点一次开始换的编号，同一个 run 的指令接着开。线程安全。"""

    def __init__(self, config=None, clock=time.monotonic):
        self.cfg = config or HoldConfig()
        self.clock = clock
        self._lock = threading.Lock()
        self._imu = None            # (收到时刻, 偏航 rad, 陀螺 z rad/s)
        self.run = None
        self.state = 'idle'         # idle 从没开过 / running / stopped
        self.reason = None
        self.target = None
        self._started = None
        self._last_step = None
        self._grace = None          # (宽限到何时, 卡之前那次计算的时刻)
        self._integ = 0.0
        self._sent = deque()        # 这次作业发出的 (时刻, wz)，每个值一直作用到下一条；只留延迟补偿要用的
        self._pred = None
        self._drift_lp = 0.0
        self._odom = None           # (收到时刻, 车速 m/s)，调用方给了才用
        self._err = None
        self._max_err = 0.0
        self._in_tol_s = 0.0
        self._run_s = 0.0
        self._out = (0.0, 0.0)

    def on_imu(self, yaw, rate, now=None):
        """一帧 IMU：偏航（rad）、陀螺 z（rad/s，没有时给 0）。非有限值丢掉。"""
        if yaw is None or not math.isfinite(yaw):
            return
        rate = rate if rate is not None and math.isfinite(rate) else 0.0
        with self._lock:
            self._imu = (self.clock() if now is None else float(now), float(yaw), float(rate))

    def on_odom(self, speed, now=None):
        """一帧里程计车速（m/s）。可选：给了就用来判断「车没走起来」（轮子卡住），那时不累积积分。"""
        if speed is None or not math.isfinite(speed):
            return
        with self._lock:
            self._odom = (self.clock() if now is None else float(now), float(speed))

    def _fresh_imu(self, now):
        imu = self._imu
        if imu is None or not 0.0 <= now - imu[0] <= self.cfg.imu_stale_s:
            return None
        return imu

    def _sent_since(self, t0, now):
        """(t0 到 now 之间发出去的转向角度 ∫wz dt, t0 那一刻在发的 wz)。前者车头还没转出来，后者是此刻
        陀螺里应有的指令转动。顺手丢掉 t0 以前用不到的。"""
        sent = self._sent
        while len(sent) > 1 and sent[1][0] <= t0:
            sent.popleft()
        total, at_t0 = 0.0, 0.0
        for i, (ts, wz) in enumerate(sent):
            te = sent[i + 1][0] if i + 1 < len(sent) else now
            if ts <= t0:
                at_t0 = wz
            if te > t0:
                total += wz * (te - max(ts, t0))
        return total, at_t0

    def _moving(self, now, vx):
        """车走起来了没有：起步斜坡前半段（指令车速还不到 moving_ratio × speed）不算；有新鲜里程计时车速还不到
        指令的 moving_ratio（轮子卡住）也不算。"""
        c = self.cfg
        if vx < c.moving_ratio * c.speed:
            return False
        odom = self._odom
        if odom is None or not 0.0 <= now - odom[0] <= c.odom_stale_s:
            return True
        return odom[1] >= c.moving_ratio * vx

    def step(self, run, now=None, target=None, vx=None):
        """run 这次作业此刻该发的 (vx, wz)；新 run 先按当前偏航定目标。停了返回 (0, 0)。

        target：调用方每步给的目标偏航（IMU 系，rad），给了就用它，不再锁定开始时的偏航（定点画线外环用）；
        vx：调用方自己定的车速（m/s），给了就原样输出、也按它判断「车走起来了没有」，不走本类的起步斜坡。
        两个都不给时与原来完全一样（IMU 直行）。"""
        now = self.clock() if now is None else float(now)
        c = self.cfg
        with self._lock:
            if run != self.run:
                self._begin(run, now)
            if self.state != 'running':
                return 0.0, 0.0
            if target is not None:
                self.target = float(target)
            prev, self._last_step = self._last_step, now
            if now - prev > c.stall_s:
                # 两次计算隔了这么久，是调用方自己卡了（web 进程执行器、GIL），IMU 回调可能还压在队列里
                self._grace = (now + c.imu_grace_s, prev)
            imu = self._fresh_imu(now)
            if imu is None:
                grace = self._grace
                if (grace is not None and now <= grace[0] and self._imu is not None
                        and grace[1] - self._imu[0] <= c.stall_s):
                    return self._out        # 卡之前 IMU 在正常来：等它的回调追上，先按卡之前的输出开
                return self._stop('imu_stale')
            t_imu, yaw, rate = imu
            e = wrap(yaw - self.target)
            self._err = e
            if abs(e) > c.abort_deg * DEG:
                return self._stop('deviation')
            dt = min(max(now - prev, 0.0), 0.1)
            if vx is None:
                vx = min(c.speed, c.min_speed + c.accel * (now - self._started))
            else:
                vx = max(0.0, float(vx))
            # 延迟补偿：t_imu − delay_s 以后发的指令，到这帧 IMU 时还没转出来；持续偏转（≈积分项）
            # 在「这帧 IMU 到指令生效」这段时间里还会接着偏
            if c.delay_s > 0:
                pending, commanded = self._sent_since(t_imu - c.delay_s, now)
                horizon = c.delay_s + (now - t_imu)
            else:
                pending, commanded, horizon = 0.0, 0.0, 0.0
            drift = rate - commanded        # 指令解释不了的转动（预估偏差的变化率）
            moving = self._moving(now, vx)
            if moving:                      # 轮子卡住时陀螺是 0、指令不是 0，不能当成偏转攒起来
                self._drift_lp += min(1.0, dt / c.drift_tc) * (drift - self._drift_lp)
            # 持续偏转的估计 = 积分项 + kd × 残差的慢变部分（稳态时两者合起来正好抵消偏转）；
            # 少了后一项稳态会留 kd × 延迟 × 偏转（0.02 rad/s 时约 0.2°）
            pred = e + pending + horizon * (self._integ + c.kd * self._drift_lp)
            wz = -(c.kp * pred + self._integ) - c.kd * drift
            if abs(wz) < c.max_wz and moving:       # 限幅、没走起来时不积分，松开后不猛甩
                self._integ = max(-c.i_max, min(c.i_max, self._integ + c.ki * pred * dt))
                pred = e + pending + horizon * (self._integ + c.kd * self._drift_lp)
                wz = -(c.kp * pred + self._integ) - c.kd * drift
            wz = max(-c.max_wz, min(c.max_wz, wz))
            self._pred = pred
            if c.delay_s > 0:
                self._sent.append((now, wz))
            self._max_err = max(self._max_err, abs(e))
            self._run_s += dt
            if abs(e) <= c.tolerance_deg * DEG:
                self._in_tol_s += dt
            self._out = (vx, wz)
            return self._out

    def _begin(self, run, now):
        self.run, self._started, self._last_step, self._grace = run, now, now, None
        self._integ, self._err, self._max_err, self._in_tol_s, self._run_s = 0.0, None, 0.0, 0.0, 0.0
        self._sent.clear()
        self._drift_lp = 0.0
        self._pred = None
        self._out = (0.0, 0.0)
        imu = self._fresh_imu(now)
        if imu is None:
            self.target = None
            self.state, self.reason = 'stopped', 'no_imu'
        else:
            self.target = imu[1]
            self.state, self.reason = 'running', None

    def _stop(self, reason):
        self.state, self.reason = 'stopped', reason
        self._out = (0.0, 0.0)
        return self._out

    @property
    def running(self):
        with self._lock:
            return self.state == 'running'

    def end(self, reason):
        """调用方结束正在跑的这次作业（停止、超时、急停、接管、断线）；没在跑时什么都不做。返回是否结束了一次。"""
        with self._lock:
            if self.state != 'running':
                return False
            self._stop(reason)
            return True

    def status(self, now=None):
        """给 app 的状态（角度为度）。从没开过、也没有 IMU 时只有 imu_ok。"""
        now = self.clock() if now is None else float(now)
        with self._lock:
            imu = self._fresh_imu(now)
            out = {'imu_ok': imu is not None,
                   'yaw_deg': None if imu is None else round(math.degrees(imu[1]), 2)}
            if self.run is None:
                return dict(out, state='idle')
            deg = lambda a: None if a is None else round(math.degrees(a), 2)  # noqa: E731
            running = self.state == 'running'
            out.update(run=self.run, state=self.state, reason=self.reason, target_deg=deg(self.target),
                       error_deg=deg(wrap(imu[1] - self.target)) if imu is not None and self.target is not None
                       else deg(self._err),
                       max_error_deg=deg(self._max_err),
                       within_pct=round(100.0 * self._in_tol_s / self._run_s, 1) if self._run_s > 0 else None,
                       elapsed_s=round(self._run_s, 1),
                       speed=round(self._out[0], 3) if running else 0.0,
                       wz=round(self._out[1], 3) if running else 0.0)
            return out
