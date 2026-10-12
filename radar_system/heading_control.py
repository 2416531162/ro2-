#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享航向控制内核：PID、已发未执行转角和陀螺残差，纯计算、不依赖 ROS。

误差由调用方给出，正值表示车头在线路目标左侧；本模块只输出角速度。
车速、IMU 新鲜度、停车/中止、路线状态和定位许可均由调用方管理。
所有时间使用同一单调时钟。过渡时应 record=False 计算，再记录真实下发的混合指令。
"""
import math
from collections import deque
from dataclasses import dataclass
from turn_response import TurnResponse


@dataclass(frozen=True)
class HeadingControlConfig:
    kp: float = 2.2
    ki: float = 1.0
    kd: float = 0.4
    delay_s: float = 0.44
    drift_tc: float = 1.5
    max_wz: float = 0.35
    i_max: float = 0.15
    direct_rate_damping: bool = False
    adaptive_response: bool = False

    def __post_init__(self):
        if type(self.direct_rate_damping) is not bool:
            raise ValueError('direct_rate_damping 必须是布尔值')
        if type(self.adaptive_response) is not bool:
            raise ValueError('adaptive_response 必须是布尔值')
        for name in ('kp', 'ki', 'kd', 'delay_s', 'i_max'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + ' 必须是非负有限值')
        for name in ('drift_tc', 'max_wz'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' 必须是正有限值')


class HeadingControl:
    """非线程安全的纯控制内核；锁和安全许可留在手动/路线调用方。"""

    def __init__(self, config=None):
        self.cfg = config or HeadingControlConfig()
        self.sent = deque()
        self.reset()

    @staticmethod
    def _finite(**values):
        if any(not math.isfinite(value) for value in values.values()):
            raise ValueError('航向控制输入必须是有限值')

    def reset(self, now=None, integral=0.0, drift_lp=0.0, sent=(), output=0.0):
        """重置一次控制；sent 是真实已下发的 (单调时刻, wz) 历史，保留边界前最后一条。"""
        self._finite(integral=integral, drift_lp=drift_lp, output=output)
        if now is not None:
            self._finite(now=now)
        history = sorted(list(sent))  # sent 可以就是本对象自己的队列。
        for t, wz in history:
            self._finite(t=t, wz=wz)
            if now is not None and t > now:
                raise ValueError('已下发历史不能来自未来')
        self.sent.clear()
        self.response=TurnResponse() if self.cfg.adaptive_response else None
        self.response_scale=1.
        for t, wz in history:
            self.record_sent(t, wz)
        self.last_step = now
        self.integral = max(-self.cfg.i_max, min(self.cfg.i_max, integral))
        self.drift_lp = float(drift_lp)
        self.pred = None
        self.output = max(-self.cfg.max_wz, min(self.cfg.max_wz, output))
        self.pending = self.commanded = self.residual = self.horizon = 0.0

    def record_sent(self, now, wz):
        """记录真正下发的指令；相同时刻用最终值覆盖，避免过渡计算值污染预估。"""
        self._finite(now=now, wz=wz)
        if self.sent and now < self.sent[-1][0]:
            raise ValueError('已下发指令的时刻不能倒退')
        if self.sent and now == self.sent[-1][0]:
            self.sent[-1] = (float(now), float(wz))
        else:
            self.sent.append((float(now), float(wz)))
        if self.response is not None:self.response.record_sent(now,wz)

    def sent_since(self, t0, now):
        """返回 t0...now 的指令转角积分，以及 t0 时生效的指令角速度。"""
        self._finite(t0=t0, now=now)
        if t0 > now:
            raise ValueError('积分区间开始不能晚于结束')
        while len(self.sent) > 1 and self.sent[1][0] <= t0:
            self.sent.popleft()
        total, at_t0 = 0.0, 0.0
        for i, (ts, wz) in enumerate(self.sent):
            if ts > now:
                break
            te = min(self.sent[i + 1][0], now) if i + 1 < len(self.sent) else now
            if ts <= t0:
                at_t0 = wz
            if te > t0:
                total += wz * (te - max(ts, t0))
        return total, at_t0

    def _terms(self, now, error, rate, sample_t, target_rate):
        self._finite(now=now, error=error, rate=rate, sample_t=sample_t, target_rate=target_rate)
        if sample_t > now:
            raise ValueError('航向样本不能来自未来')
        c = self.cfg
        if c.delay_s > 0:
            pending, commanded = self.sent_since(sample_t - c.delay_s, now)
            horizon = c.delay_s + (now - sample_t)
        else:
            pending, commanded, horizon = 0.0, target_rate, 0.0
        if self.response is not None:
            pending/=self.response_scale
            commanded/=self.response_scale
        return pending, commanded, horizon, rate - commanded

    def prime(self, now, error, rate, *, sample_t=None, sent=(), output=0.0, target_rate=0.0):
        """承接旧控制的历史与输出，反解有界积分；目标仍须调用方保证连续。

        返回在积分上限约束下可承接的输出，调用方可用它决定过渡混合。
        output 视为 now 时已经使用的真实旧输出；下一步 record_sent 可覆盖同一时刻的最终值。
        """
        sample_t = now if sample_t is None else sample_t
        self.reset(now=now, sent=sent, output=output)
        pending, commanded, horizon, drift = self._terms(now, error, rate, sample_t, target_rate)
        c = self.cfg
        base = error + pending - horizon * target_rate
        damping = rate-target_rate if c.direct_rate_damping else drift
        integral = (target_rate - self.output - c.kp * base - c.kd * damping) / (1.0 + c.kp * horizon)
        self.integral = max(-c.i_max, min(c.i_max, integral))
        self.pending, self.commanded, self.horizon, self.residual = pending, commanded, horizon, drift
        self.pred = base + horizon * self.integral
        achievable = target_rate - (c.kp * self.pred + self.integral) - c.kd * damping
        self.record_sent(now, output)
        return max(-c.max_wz, min(c.max_wz, achievable))

    def step(self, now, error, rate, *, sample_t=None, moving=True, target_rate=0.0,
             reference_lead_s=0.0, dt=None, record=True, adaptation_ok=False):
        """计算角速度，error 为 sample_t 同时刻的航向减目标；默认固定目标。

        target_rate 为目标的连续变化率；reference_lead_s 表示调用方已经预测过的目标时域，避免重复前推。
        过渡模式 record=False 时，计算输出不会写入历史；
        必须由调用方随后 record_sent(now, 实际输出)。dt 可由安全层传入以保留其卡顿处理语义。
        """
        sample_t = now if sample_t is None else sample_t
        self._finite(now=now,error=error,rate=rate,sample_t=sample_t,target_rate=target_rate)
        if sample_t>now:raise ValueError('航向样本不能来自未来')
        self._finite(reference_lead_s=reference_lead_s)
        if reference_lead_s < 0:
            raise ValueError('目标预瞄时域必须非负')
        if dt is None:
            dt = 0.0 if self.last_step is None else now - self.last_step
        self._finite(dt=dt)
        dt = min(max(dt, 0.0), 0.1)
        self.last_step = now
        c = self.cfg
        if self.response is not None:
            self.response.observe(now,sample_t,rate,eligible=bool(moving and adaptation_ok))
            self.response_scale=self.response.advance(now,dt)
        pending, commanded, horizon, drift = self._terms(now, error, rate, sample_t, target_rate)
        if moving:
            self.drift_lp += min(1.0, dt / c.drift_tc) * (drift - self.drift_lp)
        base = error + pending - max(0.0, horizon-reference_lead_s) * target_rate
        # 直接角速度阻尼用于模型失配对照；稳态时D为0，此时恒定扰动只由I估计。
        damping = rate-target_rate if c.direct_rate_damping else drift
        bias_term = 0.0 if c.direct_rate_damping else c.kd*self.drift_lp
        pred = base + horizon * (self.integral + bias_term)
        wz = target_rate - (c.kp * pred + self.integral) - c.kd * damping
        if abs(wz*self.response_scale) < c.max_wz and moving:
            self.integral = max(-c.i_max, min(c.i_max, self.integral + c.ki * pred * dt))
            pred = base + horizon * (self.integral + bias_term)
            wz = target_rate - (c.kp * pred + self.integral) - c.kd * damping
        self.pred = pred
        self.pending, self.commanded, self.horizon, self.residual = pending, commanded, horizon, drift
        self.output = max(-c.max_wz, min(c.max_wz, wz*self.response_scale))
        if record and (c.delay_s > 0 or self.response is not None):
            self.record_sent(now, self.output)
        return self.output

    def status(self):
        return dict(integral=self.integral, pred=self.pred, drift_lp=self.drift_lp, output=self.output,
                    pending=self.pending, commanded=self.commanded, residual=self.residual, horizon=self.horizon,
                    direct_rate_damping=self.cfg.direct_rate_damping,adaptive_response=self.response is not None,
                    **({} if self.response is None else self.response.status()))
