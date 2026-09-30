#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速车（Gen5）定点画线（A→B 直线）：IMU 航向保持做内环 + RTK 横偏修正做外环。纯计算，不依赖 ROS。

为什么不用 LineTracker：它按曲率控制、只往前外推 0.1 s；Gen5 底盘转向晚约 0.44 s（2026-09-30 实测：陀螺 ≈
0.44 s 前的角速度指令 × 1.02–1.06），0.8 m/s 时来回大摆（仿真喷头横偏 95% 约 25 cm）。IMU 直行
（heading_hold.HeadingHold，带 Smith 预估）实车 0.8 m/s 车头稳在目标 ±0.8° 内，这里直接拿它当内环。

外环（每个控制周期，用 RTK 融合位姿）：
    e  = 喷头（车体 x = nozzle_x_m）相对 A→B 线的横偏，线左为正
    δ  = −atan(k_e·e) − k_i·∫e ds，限幅 ±max_bias_deg        目标航向偏置：偏左就朝右偏一点开回来
    ∫e ds 只在前进、|e| < integ_band_m 时累积，并限幅（积分项贡献不超过 integ_max_deg）
坐标换算（内环在 IMU 自己的偏航系里跑：HDA536T 上电为 0，会慢慢漂）：
    偏移 = 融合航向 − IMU 偏航，低通（时间常数 offset_tau_s；刚开始按样本平均，攒够一个时间常数后按低通）
    目标 = 线方向 − 偏移 + δ（IMU 系）；偏移的残差（IMU 漂移跟不上、双天线安装残差）由外环积分吃掉
    起步 / 继续时目标从当时的 IMU 偏航出发，按 turn_rate_deg_s 限速转到上式：内环偏离目标超过 5° 就停车，
    不能一步跳过去。还没转到位（差 align_deg 以上）时车速不超过 align_radius_m × turn_rate，边慢慢走边转。
车速（规则同 LineTracker）：起步斜坡、横偏超过 slow_offset_m 降速、离 B 按减速度刹停。底盘车速也晚约 0.4 s，
离 B 的剩余距离扣掉已发出、还没执行的车速指令走的路（speed_delay_s），免得冲过 B。

s（沿线距离）、起止和起步检查按记 A、B 用的同一个点（track_x_m，与 LineTracker 相同），横偏 e 按喷头量。
航向跳变（2026-09-30 用户定的规则：IMU 航向 5 s 内肯定没问题）：
    参考航向 = IMU 偏航 + 偏移。喷头位置、车头误差都按参考航向算：融合用自己的航向从天线倒推后轴中心，
    航向错 Δ 后轴就横着偏 0.24·Δ（天线在后轴前 0.24 m），所以先按融合航向还原出天线位置，再用参考航向推回后轴
    和喷头（天线到喷头只差前 0.06 m，全用融合航向时喷头也只偏 0.06·Δ，跳 3° 约 3 mm）。
    跳变真正带歪车的是偏移：一直照学，就把跳变慢慢学进目标航向，车真的按错的航向开（仿真：一直错 3°，喷头偏到 6 cm）。
    融合航向与参考航向差超过 jump_deg 算跳变：偏移不学，车照常按 IMU 开（mission 报「航向跳变，按 IMU 维持」）；
    差回到 recover_deg 以内算恢复，重新计时。跳变超过 jump_hold_s（5 s）还没恢复 → PAUSED(heading_jump) 停车，
    航向恢复后由 LineMission 的自动继续接着走（用户选的：停车等，不认新航向、也不一直按 IMU 开）。
    起步不判跳变（用户：起步是万向轮，不管 IMU 的变化）：每次开始 / 继续后，车头对准、车速斜坡走完再过
    start_settle_s 才开始判；起步期间偏移照学，起步转完那一刻偏移从头重新平均——起步转弯时 IMU 的刻度误差
    （转 30°、误差 2% 就差 0.6°）、万向轮甩尾时融合与 IMU 的时间差都不会被当成跳变。
停车 / 中止与 LineTracker 相同：定位不可用 → PAUSED；|e| > 0.5 m 或车头偏 > 45° → ABORTED(off_line)；
内环自己停车映射成中止：IMU 偏离目标超过 5°（imu_deviation：被推歪、IMU 重启归零、方向装反）、
IMU 中断（imu_stale）；开始时没有 IMU 航向不能起步（no_imu）。

参数来源：外环 k_e 0.6 1/m、k_i 0.05 1/m²、±3° 与内环现参数（kp 2.2 / ki 1.0 / kd 0.4，延迟 0.44 s）是在实测底盘
模型上仿真挑的（tests/test_line_cascade.py）；内环换成更温和的参数反而变差。
"""
import dataclasses
import math
from collections import deque
from dataclasses import dataclass, field

from heading_hold import HeadingHold, HoldConfig
from line_tracker import LineState, wrap
from robot_core.kinematics import ChassisGeometry

DEG = math.pi / 180.0

# 中止 / 起步不了的原因（mission 的 message 用；off_line 等与 LineTracker 相同的代码由 app 翻译）
REASON_TEXT = {
    'imu_deviation': 'IMU 航向偏离目标超过 5°（被推歪、IMU 重启归零或装反）',
    'imu_stale': 'IMU 数据中断',
    'no_imu': '没有 IMU 航向（/imu/data）',
    'heading_jump': '航向跳变（双天线航向与 IMU 不一致），等航向恢复；要按现在的航向重来，先停止再开始',
}
# 跳变时 mission 的 message（跳变结束后 mission 改回「行驶中」）
NOTICE_HOLD = '航向跳变，按 IMU 维持'
NOTICE_PAUSED = '航向跳变超过 5 s，已停车，航向恢复后自动继续'
NOTICES = (NOTICE_HOLD, NOTICE_PAUSED)


@dataclass(frozen=True)
class CascadeConfig:
    cruise_mps: float = 0.8
    # 以下几项与 LineConfig 同名同义，LineMission 从 LineConfig 抄过来（SHARED_FIELDS）
    accel_mps2: float = 0.5
    decel_mps2: float = 0.5
    min_mps: float = 0.06            # 终点前的蠕行速度，保证真的走到 B
    track_x_m: float = 0.0           # 沿线距离 s、起止按这个点（车体系 x），与记 A、B 的点相同
    start_max_offset_m: float = 0.5
    start_max_heading_rad: float = math.radians(30.0)
    start_before_a_m: float = 3.0    # 允许从 A 点后方最多这么远起步（助跑）
    abort_offset_m: float = 0.5
    abort_heading_rad: float = math.radians(45.0)
    slow_offset_m: float = 0.10      # 横偏超过它开始降速
    stop_tol_m: float = 0.02
    min_line_m: float = 1.0
    # 外环
    nozzle_x_m: float = 0.30         # 横偏量喷头：后轴前多少米
    k_e: float = 0.6                 # 横偏 → 目标航向偏置，1/m
    k_i: float = 0.05                # 横偏积分，1/m²
    max_bias_deg: float = 3.0        # 目标航向偏置限幅
    integ_band_m: float = 0.10       # |e| 小于它才积分（切入阶段不积）
    integ_max_deg: float = 2.0       # 积分项贡献的航向偏置上限
    offset_tau_s: float = 10.0       # 融合航向 − IMU 偏航的低通时间常数
    # 航向跳变：5 s 内 IMU 漂不到 0.1°、平时融合航向与参考航向差 0.1–0.2°，0.5° 对应喷头约 0.5 cm
    jump_deg: float = 0.5            # 融合航向与参考航向差超过它算跳变
    recover_deg: float = 0.3         # 回到它以内算恢复
    jump_hold_s: float = 5.0         # 跳变最多按 IMU 维持这么久，还没恢复就停车
    start_settle_s: float = 2.0      # 起步对准、加速完再过这么久才判跳变（底盘延迟 0.4 s、万向轮甩尾约 0.6–0.8 s）
    # 目标限速转向（起步 / 继续时从当时的车头转到线方向）
    turn_rate_deg_s: float = 3.0     # 慢走时内环不积分，跟 r 的斜坡落后约 r/kp + r × 延迟 ≈ 2.7°（6°/s 会过 5° 停车）
    align_deg: float = 1.0           # 目标还差这么多没转到位就算「在对准」
    align_radius_m: float = 1.0      # 对准时车速上限 = 这个半径 × turn_rate（约 0.1 m/s）
    speed_delay_s: float = 0.40      # 底盘车速响应延迟（2026-09-30 实测约 0.4 s），到 B 的刹停按它预估
    hold: HoldConfig = field(default_factory=HoldConfig)    # 内环参数（与 IMU 直行相同）


# 与 LineConfig 同名、按同一规则用的字段：LineMission 建差速车定点画线时从当前 LineConfig 抄
SHARED_FIELDS = ('accel_mps2', 'decel_mps2', 'min_mps', 'track_x_m', 'start_max_offset_m', 'start_max_heading_rad',
                 'start_before_a_m', 'abort_offset_m', 'abort_heading_rad', 'slow_offset_m', 'stop_tol_m',
                 'min_line_m')


class ImuLineTracker:
    """接口与 LineTracker 相同（set_line / check_start / start / stop / update / status、st），另加 sync：
    每个控制周期（和起步检查前）由 LineMission 给此刻的单调时钟和最近一帧 IMU (t, 偏航 rad, 陀螺 z rad/s)。"""

    def __init__(self, config=CascadeConfig(), geometry=None):
        self.cfg = config
        self.geo = geometry or ChassisGeometry()
        # 内环的 speed 只用来判断「车走起来了没有」（指令车速不到一半不积分），按巡航速度算
        self.hold_cfg = dataclasses.replace(config.hold, speed=config.cruise_mps)
        self.hold = HeadingHold(self.hold_cfg)
        self.a = self.b = None
        self.st = LineState()
        self._now = None
        self._imu = None
        self._lever = None           # 融合用的主天线杠杆臂（车体系前、左，m），mission 每周期给
        self._diff = None            # 融合航向 − 参考航向，rad
        self._jump_since = None      # 这次跳变从什么时候开始（没在跳变为 None）
        self._settled_since = None   # 这次起步什么时候对准、加速完（之前是起步阶段，不判跳变）
        self._run = 0                # 每次起步 / 继续换一个，内环按新的一次作业重新开始
        self._offset = None          # 融合航向 − IMU 偏航（低通），rad
        self._offset_n = 0
        self._target = None          # 发给内环的目标（IMU 系，限速后），rad
        self._bias = 0.0             # δ，rad
        self._aligning = False
        self._wz = 0.0
        self._last_s = None
        self._v_prev = 0.0
        self._v_sent = deque()       # 发出的 (时刻, 车速)，每个值一直作用到下一条；只留预估要用的
        self._sum2 = self._n = 0.0
        self._max = 0.0

    # ---- 输入 ----
    def sync(self, now, imu, lever=None):
        """此刻的时钟、最近一帧 IMU（没有时 None）、融合正在用的主天线杠杆臂（没有时 None：位置不改算）。"""
        self._now = float(now)
        self._lever = lever
        if imu is not None and imu != self._imu:
            self._imu = imu
            self.hold.on_imu(imu[1], imu[2], now=imu[0])

    def _imu_now(self):
        """(IMU 偏航按陀螺推到此刻, 陀螺)；IMU 太旧或没有时 None。"""
        imu, now = self._imu, self._now
        if imu is None or now is None:
            return None
        age = now - imu[0]
        if not 0.0 <= age <= self.hold_cfg.imu_stale_s:
            return None
        return wrap(imu[1] + imu[2] * age), imu[2]

    def _reference(self, pose):
        """参考航向（ENU）= IMU 偏航 + 偏移，顺手记下融合航向与它的差；还没有偏移或 IMU 太旧时 None。"""
        imu = self._imu_now()
        if imu is None or self._offset is None:
            self._diff = None
            return None
        ref = wrap(imu[0] + self._offset)
        self._diff = wrap(pose.yaw - ref)
        return ref

    def _position(self, pose, heading):
        """后轴中心按 heading 改算：融合位置 + R(融合航向)·杠杆臂 = 天线位置，再减 R(heading)·杠杆臂。"""
        if self._lever is None or heading == pose.yaw:
            return pose.x, pose.y
        ax, ay = self._lever
        cf, sf, cr, sr = math.cos(pose.yaw), math.sin(pose.yaw), math.cos(heading), math.sin(heading)
        return (pose.x + (cf - cr) * ax - (sf - sr) * ay, pose.y + (sf - sr) * ax + (cf - cr) * ay)

    def _start_phase(self):
        """还在起步阶段（没对准、没加速完，或完了还不到 start_settle_s）：不判航向跳变。"""
        return self._settled_since is None or self._now - self._settled_since < self.cfg.start_settle_s

    def notice(self):
        """航向跳变时给 mission 显示的话；没有时 None。"""
        if self.st.state == 'PAUSED' and self.st.reason == 'heading_jump':
            return NOTICE_PAUSED
        if self.st.state == 'RUNNING' and self._jump_since is not None:
            return NOTICE_HOLD
        return None

    # ---- 任务 ----
    def set_line(self, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < self.cfg.min_line_m:
            raise ValueError('A、B 两点太近（%.2f m）' % length)
        self.a, self.b = tuple(a), tuple(b)
        self.u = (dx / length, dy / length)
        self.n = (-self.u[1], self.u[0])
        self.line_yaw = math.atan2(dy, dx)
        self.st = LineState(length=length)

    def _along_across(self, x, y, yaw, forward):
        px, py = x + forward * math.cos(yaw), y + forward * math.sin(yaw)
        rx, ry = px - self.a[0], py - self.a[1]
        return rx * self.u[0] + ry * self.u[1], rx * self.n[0] + ry * self.n[1]

    def geometry_of(self, x, y, yaw):
        """(喷头横偏, 车头误差, 记点那个点的沿线距离)。"""
        _, e = self._along_across(x, y, yaw, self.cfg.nozzle_x_m)
        s, _ = self._along_across(x, y, yaw, self.cfg.track_x_m)
        return e, wrap(yaw - self.line_yaw), s

    def check_start(self, pose):
        """能否从当前位姿开始；返回 (可以, 原因)。条件同 LineTracker，另要有新鲜的 IMU 航向。"""
        if self.a is None:
            return False, 'no_line'
        if not pose.usable_for_control:
            return False, 'pose_unusable'
        if self.st.state == 'PAUSED' and self.st.reason == 'heading_jump':
            # 因航向跳变停的车：融合航向回到参考航向 recover_deg 以内才走（参考航向没了也不走）
            if self._reference(pose) is None or abs(self._diff) > self.cfg.recover_deg * DEG:
                return False, 'heading_jump'
        e, he, s = self.geometry_of(pose.x, pose.y, pose.yaw)
        if abs(e) > self.cfg.start_max_offset_m:
            return False, 'too_far_from_line'
        if abs(he) > self.cfg.start_max_heading_rad:
            return False, 'heading_not_aligned'
        if s < -self.cfg.start_before_a_m:
            return False, 'too_far_behind_a'
        if s > self.st.length - 0.5:
            return False, 'already_past_b'
        if self._imu_now() is None:
            return False, 'no_imu'
        return True, ''

    def start(self, pose, integral=None):
        """开始或在暂停后继续；integral 不给时沿用当前值。内环重新开始，目标从此刻的 IMU 偏航出发。"""
        ok, why = self.check_start(pose)
        if not ok:
            if self.st.state != 'PAUSED':
                self.st.state = 'IDLE'
            self.st.reason = why
            return False
        self.st.state, self.st.reason = 'RUNNING', ''
        if integral is not None:
            self.st.integral = integral
        imu_yaw, _ = self._imu_now()
        if self._offset is None:
            self._offset, self._offset_n = wrap(pose.yaw - imu_yaw), 1
        self._target = imu_yaw
        self._jump_since = self._settled_since = None
        self._run += 1
        self._last_s = None
        self._v_prev = max(0.0, pose.speed)
        self._v_sent.clear()
        self._sum2 = self._n = self._max = 0.0
        return True

    def stop(self, reason='operator'):
        if self.st.state in ('RUNNING', 'PAUSED'):
            self.st.state, self.st.reason = 'ABORTED', reason
        self.hold.end('user')
        return self._halt()

    def _halt(self, state=None, reason=None):
        if state is not None:
            self.st.state, self.st.reason = state, reason
        self.st.v_cmd, self._v_prev, self._wz = 0.0, 0.0, 0.0
        if self._now is not None:
            self._v_sent.append((self._now, 0.0))
        return 0.0, 0.0

    def _pending_m(self, now):
        """已经发出、底盘还没执行（speed_delay_s 内）的车速指令还要走的路，m。"""
        t0, sent = now - self.cfg.speed_delay_s, self._v_sent
        while len(sent) > 1 and sent[1][0] <= t0:
            sent.popleft()
        total = 0.0
        for i, (ts, v) in enumerate(sent):
            te = sent[i + 1][0] if i + 1 < len(sent) else now
            if te > t0:
                total += v * (te - max(ts, t0))
        return total

    # ---- 每个控制周期 ----
    def update(self, pose, dt):
        """返回 (vx, wz)。pose 需有 x, y, yaw, speed, usable_for_control；调用前先 sync。"""
        st, cfg = self.st, self.cfg
        if st.state not in ('RUNNING', 'PAUSED'):
            st.v_cmd = 0.0
            return 0.0, 0.0
        if not pose.usable_for_control:
            # 定位不可用：立即停，恢复后由操作员（或 LineMission 的自动继续）重新开始
            self.hold.end('pose')
            self._jump_since = None
            return self._halt('PAUSED', 'pose_unusable')
        ref = self._reference(pose)
        if st.state == 'PAUSED':
            if st.reason != 'heading_jump':
                st.reason = 'pose_recovered_waiting_operator'
            return self._halt()
        now = self._now

        # 航向跳变：融合航向离参考航向太远就不学偏移、位置按参考航向改算，照常开；5 s 还没恢复就停车。
        # 起步阶段不判（万向轮），偏移照学
        if ref is not None:
            limit = self.cfg.recover_deg if self._jump_since is not None else self.cfg.jump_deg
            if not self._start_phase() and abs(self._diff) > limit * DEG:
                if self._jump_since is None:
                    self._jump_since = now
                elif now - self._jump_since > cfg.jump_hold_s:
                    self._jump_since = None
                    self.hold.end('heading_jump')
                    return self._halt('PAUSED', 'heading_jump')
            else:
                self._jump_since = None
                # 偏移 = 融合航向 − IMU 偏航（同一时刻）：先按样本平均，攒够一个时间常数后按低通
                self._offset_n += 1
                alpha = max(min(1.0, dt / cfg.offset_tau_s), 1.0 / self._offset_n)
                self._offset = wrap(self._offset + alpha * self._diff)
                ref = wrap(ref + alpha * self._diff)
        heading = pose.yaw if ref is None else ref
        x, y = self._position(pose, heading)
        e, he, s = self.geometry_of(x, y, heading)
        st.e, st.heading_err, st.s = e, he, s
        if abs(e) > cfg.abort_offset_m or abs(he) > cfg.abort_heading_rad:
            self.hold.end('off_line')
            return self._halt('ABORTED', 'off_line')
        remaining = st.length - s - self._pending_m(now)
        if remaining <= cfg.stop_tol_m:
            self.hold.end('user')
            return self._halt('DONE', '')

        # 积分只在前进且贴近直线时累积，并限幅
        ds = 0.0 if self._last_s is None else max(0.0, s - self._last_s)
        self._last_s = s
        if abs(e) < cfg.integ_band_m:
            lim = cfg.integ_max_deg * DEG / cfg.k_i
            st.integral = max(-lim, min(lim, st.integral + e * ds))

        bias = -math.atan(cfg.k_e * e) - cfg.k_i * st.integral
        lim = cfg.max_bias_deg * DEG
        self._bias = bias = max(-lim, min(lim, bias))
        desired = wrap(self.line_yaw - self._offset + bias)
        step = cfg.turn_rate_deg_s * DEG * dt
        self._target = wrap(self._target + max(-step, min(step, wrap(desired - self._target))))
        self._aligning = abs(wrap(desired - self._target)) > cfg.align_deg * DEG

        # 车速：巡航、偏差降速、终点前按减速度刹停、对准时慢走、起步斜坡
        v = cfg.cruise_mps
        if abs(e) > cfg.slow_offset_m:
            v *= max(0.4, 1.0 - (abs(e) - cfg.slow_offset_m) / 0.3)
        v = min(v, max(cfg.min_mps, math.sqrt(2.0 * cfg.decel_mps2 * max(0.0, remaining - cfg.stop_tol_m))))
        if self._aligning:
            v = min(v, max(cfg.min_mps, cfg.align_radius_m * cfg.turn_rate_deg_s * DEG))
        ramping = v > self._v_prev + cfg.accel_mps2 * dt
        v = min(v, self._v_prev + cfg.accel_mps2 * dt)
        if self._settled_since is None and not self._aligning and not ramping:
            # 起步转完、加速完：偏移从这里重新平均（起步转弯里 IMU 刻度误差攒下的差不带进来）
            self._settled_since = now
            self._offset_n = 0

        self.hold.on_odom(pose.speed, now=now)
        _, wz = self.hold.step(self._run, now, target=self._target, vx=v)
        if self.hold.state != 'running':
            reason = 'imu_deviation' if self.hold.reason == 'deviation' else (self.hold.reason or 'imu_stale')
            return self._halt('ABORTED', reason)
        self._v_prev = st.v_cmd = v
        self._wz = wz
        self._v_sent.append((now, v))
        st.kappa = wz / v if v >= 0.05 else 0.0

        if s >= 0.0:
            self._n += 1
            self._sum2 += e * e
            self._max = max(self._max, abs(e))
            st.stats = dict(rms_m=math.sqrt(self._sum2 / self._n), max_m=self._max, samples=int(self._n))
        return v, wz

    def status(self):
        st = self.st
        deg = lambda a: None if a is None else round(math.degrees(a), 3)  # noqa: E731
        imu = self._imu_now()
        hold_err = None if imu is None or self._target is None else wrap(imu[0] - self._target)
        # 键与 LineTracker 相同（app 不用改）；hold 是内外环的中间量，作业记录里用来查问题
        return dict(state=st.state, reason=st.reason, e=st.e, heading_err_deg=math.degrees(st.heading_err),
                    s=st.s, length=st.length, v_cmd=st.v_cmd, kappa=st.kappa, integral=st.integral,
                    trim_kappa=None, stats=dict(st.stats),
                    hold=dict(target_deg=deg(self._target), offset_deg=deg(self._offset), bias_deg=deg(self._bias),
                              integ_bias_deg=deg(-self.cfg.k_i * st.integral), error_deg=deg(hold_err),
                              aligning=self._aligning, wz=round(self._wz, 4), state=self.hold.state,
                              reason=self.hold.reason, heading_diff_deg=deg(self._diff),
                              start_phase=self.st.state == 'RUNNING' and self._start_phase(),
                              heading_jump_s=None if self._jump_since is None or self._now is None
                              else round(self._now - self._jump_since, 2)))
