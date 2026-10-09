"""左右轮接收时刻对齐与轮程区间；纯计算，不读总线、不发送运动命令。

驱动器没有硬件采样时钟，因此这里明确使用主机接收时刻并保留时间余量。
只在两侧已有数据的共同区间插值，不把某侧旧值重新标成当前测量。
"""
import math
from collections import deque
from robot_core.wheel_speed import HallSpeed
from robot_core.standstill import HALL_STILL_MAX_MPS

# 累计角度来自霍尔：每个电角度周期 6 格（Gen5 20 对极每圈 120 格、轮上约 1.0 cm 一格；2026-10-03 实车读回增量
# 都在 3° 的整数倍附近）。20 ms 一帧里没走满一格或多走一格，平均速度就差约 0.5 m/s，和回读转速比不出真假。
HALL_STEPS_PER_ELECTRICAL_TURN = 6


class WheelMotionAligner:
    def __init__(self, wheels, max_age_s=.2, max_gap_s=.2, time_uncertainty_s=.01, accumulate_s=.1, feedback_delay_s=0., reset_gap_s=1.):
        self.wheels = wheels
        self.max_age, self.max_gap, self.time_padding = max_age_s, max_gap_s, time_uncertainty_s
        # 2026-10-09：累计角度反馈间隔超过 1 s 才重建。与插值区间和当前反馈新鲜度分开，
        # 避免放宽重建阈值后生成 WheelInput 不接受的长区间，或把迟到数据当作当前测量。
        if not self._finite(reset_gap_s) or reset_gap_s <= 0:
            raise ValueError('轮程重建间隔必须是有限正数')
        self.reset_gap = reset_gap_s
        # accumulate_s 保留旧调用兼容；新霍尔区间不再等回读转速一致后才发送。
        self.step = (wheels.centideg_to_m(36000.0 / (HALL_STEPS_PER_ELECTRICAL_TURN * wheels.pole_pairs))
                     if wheels.configured else 0.0)
        if not self._finite(feedback_delay_s) or not 0 <= feedback_delay_s <= .1:
            raise ValueError('霍尔反馈补偿必须在 0–0.1 秒内')
        # 标定未能证明存在固定延迟前保持 0，不用回读转速的滤波滞后来补霍尔时刻。
        self.feedback_delay = feedback_delay_s
        self.hall = {m: HallSpeed(self.step) for m in (1,2)}
        self.speed = {m: deque(maxlen=128) for m in (1, 2)}
        self.position = {m: deque(maxlen=128) for m in (1, 2)}
        self.raw_position = {}
        self.anchor = None
        self.last_t = -math.inf
        self.seq = 0
        self.resets = 0

    @staticmethod
    def _finite(*values):
        return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)

    def _at(self, rows, t):
        for a, b in zip(rows, list(rows)[1:]):
            if a[0] <= t <= b[0] and 0 < b[0]-a[0] <= self.max_gap:
                fraction = (t-a[0])/(b[0]-a[0])
                return a[1] + fraction*(b[1]-a[1])
        if rows and abs(rows[-1][0]-t) <= 1e-9:
            return rows[-1][1]
        return None

    def _travel(self, rows, t0, t1):
        """回读转速在 t0→t1 的积分（按相邻回复线性插值）；区间没被回复覆盖时返回 None。"""
        v0, v1 = self._at(rows, t0), self._at(rows, t1)
        if v0 is None or v1 is None:
            return None
        points = [(t0, v0)] + [r for r in rows if t0 < r[0] < t1] + [(t1, v1)]
        return sum(.5*(a[1]+b[1])*(b[0]-a[0]) for a, b in zip(points, points[1:]))

    def stationary(self, now, states):
        evidence = []
        for m in (1, 2):
            state = states[m]
            try:
                position_t = self.position[m][-1][0]
            except IndexError:  # I/O 线程刚清空重置的区间，不能据此确认静止。
                evidence.append(None)
                continue
            speed_t, erpm = state.t_speed, state.speed_erpm
            if (not 0 <= now-position_t <= self.max_age
                    or speed_t is None or not 0 <= now-speed_t <= self.max_age
                    or erpm is None or not math.isfinite(erpm)):
                evidence.append(None)
                continue
            hall = self.hall[m].at(position_t, use_window=False)
            still = hall[2] if hall is not None else None
            # 新鲜回读明显运动时不能把卡住的计数当作静止。
            if still is True and abs(self.wheels.to_mps(erpm)) > HALL_STILL_MAX_MPS:
                still = False
            evidence.append(still)
        return True if all(v is True for v in evidence) else False if any(v is False for v in evidence) else None

    def update(self, now, states):
        wl = self.wheels
        base = dict(version=1, time_basis='host_receive', t_rx=now, seq=self.seq,
                    quality='pending', reason='waiting_for_pair', resets=self.resets)
        if not self._finite(now) or not wl.configured:
            return dict(base, quality='invalid', reason='unconfigured')
        reset = gap = False
        for motor, sign in ((1, wl.left_sign), (2, wl.right_sign)):
            state = states[motor]
            t, value = state.t_speed, state.speed_erpm
            rows = self.speed[motor]
            if self._finite(t, value) and t <= now and (not rows or t > rows[-1][0]):
                speed = sign*wl.to_mps(value)
                if abs(speed) <= 1.5*wl.max_wheel_mps:
                    rows.append((t, speed))
            t, raw = state.t_position, state.position_cdeg
            if self._finite(t):
                t -= self.feedback_delay
            previous = self.raw_position.get(motor)
            if (not self._finite(t, raw) or int(raw) != raw or not -2**31 <= raw < 2**31
                    or t > now or (previous and t <= previous[0])):
                continue
            rows = self.position[motor]
            if previous is None:
                rows.append((t, 0.0))
            else:
                dt = t-previous[0]
                delta = (int(raw)-previous[1]+2**31) % 2**32 - 2**31
                distance = sign*wl.centideg_to_m(delta)
                # 霍尔刚跨格时，两次很近的回复也可能相差一整格，不能按连续角度误判计数器重置。
                if dt > self.reset_gap+1e-9 or abs(distance) > 1.5*wl.max_wheel_mps*dt + self.step + .003:
                    rows.clear()
                    self.hall[motor].reset()
                    rows.append((t, 0.0))
                    self.resets += 1
                    reset = True
                else:
                    rows.append((t, rows[-1][1]+distance))
                    gap = gap or dt > self.max_gap
            self.hall[motor].add(t, rows[-1][1])
            self.raw_position[motor] = (t, int(raw))
        if reset:
            self.anchor = None
        base['resets'] = self.resets
        speed_ok = all(self.speed[m] and now-self.speed[m][-1][0] <= self.max_age for m in (1,2))
        positions_ok = all(self.position[m] and now-self.position[m][-1][0] <= self.max_age for m in (1,2))
        if not speed_ok and not positions_ok:
            self.anchor = None
            return dict(base,quality='invalid',reason='wheel_feedback_stale')
        speeds_t = [self.speed[m][-1][0] if self.speed[m] else None for m in (1,2)]
        positions_t = [self.position[m][-1][0] if self.position[m] else None for m in (1,2)]
        # 有霍尔位置时按位置回复对齐，不能让较慢的 5001 回复拖住累计轮程。
        times = positions_t if positions_ok else speeds_t
        common = min(times)
        if common <= self.last_t+1e-9:
            return base
        velocities = [self._at(self.speed[m],common) for m in (1,2)] if speed_ok else [None,None]
        if speed_ok:
            # 回读仅供诊断或明确降级；无法在霍尔时刻插值时保留已收到的最新值和它自己的时刻。
            velocities = [v if v is not None else self.speed[m][-1][1]
                          for m,v in zip((1,2),velocities)]
        positions = [self._at(self.position[m],common) for m in (1,2)] if positions_ok else [None,None]
        v,w = (sum(velocities)/2,(velocities[1]-velocities[0])/wl.track_m) if speed_ok else (0.,0.)
        uncertainty = self.time_padding+(max(times)-min(times))
        encoder = None
        if None not in positions:
            if self.anchor is not None and 0 < common-self.anchor[0] <= self.max_gap and not reset:
                start, pl, pr = self.anchor
                dl, dr = positions[0]-pl, positions[1]-pr
                dt = common-start
                if max(abs(dl), abs(dr)) <= 1.5*wl.max_wheel_mps*dt + self.step + .003:
                    # 累计位置是主要轮程证据；回读转速偏小或滞后不能否决有效的霍尔区间。
                    encoder = dict(quality='encoder',reason='',start=start,
                                   distance=(dl+dr)/2,turn=(dr-dl)/wl.track_m,
                                   left_distance=dl,right_distance=dr,
                                   sigma_distance_m=max(self.step/math.sqrt(6),wl.accel_mps2*dt*uncertainty))
        # 跨格事件使用各轮本次已经收到的最新回复。若以较早的共同时间裁掉另一轮刚来的事件，
        # 会漏判起步，并把连续转动误当作太久没跨格而衰减；时刻差完整计入时间余量。
        hall = [self.hall[m].at(positions_t[m-1],use_window=False) for m in (1,2)] if positions_ok else [None,None]
        # 长间隔可保留累计角度，但恢复这一拍不把长跨格周期当即时霍尔速度。
        # 先用新鲜转速回读，后续短区间再恢复轮程；不放宽融合端的不确定度校验。
        hall_ok = all(h is not None and h[0] is not None for h in hall) and not (reset or gap)
        diagnostics = []
        if hall_ok:
            estimated = [h[0] for h in hall]
            sigmas=[h[1] for h in hall]
            for i,m in enumerate((1,2)):
                if abs(estimated[i]) >= .35:
                    window=self.hall[m].window
                    before=self._at(self.position[m],common-window)
                    after=self._at(self.position[m],common)
                    if before is not None and after is not None:
                        # 两个端点都用本周期已收到的位置在共同时间窗内对齐。
                        # 不裁成随通信节拍变化的短窗口，避免异步 CAN 回复造成取样偏差。
                        estimated[i]=(after-before)/window
                        sigmas[i]=max(.015,self.step/(math.sqrt(6)*window))
                        # 窗口均速的测量中心比区间终点早半窗，速度时刻余量须明确保留。
                        uncertainty=max(uncertainty,self.time_padding+max(times)-min(times)+window/2)
            v,w = sum(estimated)/2,(estimated[1]-estimated[0])/wl.track_m
            sigma = math.hypot(*sigmas)/2
            for m in (1,2):
                before = self._at(self.position[m],common-1.)
                after = self._at(self.position[m],common)
                raw_travel = self._travel(self.speed[m],common-1.,common)
                if before is not None and after is not None and raw_travel is not None:
                    # 按完整一秒轮程核对，避免低速周期的量化抖动反复清掉诊断计时。
                    distance = after-before
                    if abs(distance-raw_travel)>max(.015,.2*abs(distance)):
                        diagnostics.append('readback_hall_disagreement_'+str(m))
        else:
            if not speed_ok:
                return dict(base,reason='hall_speed_initializing')
            estimated = velocities
            uncertainty=max(uncertainty,self.time_padding+max(abs(t-common) for t in speeds_t))
            sigma = max(.15,wl.accel_mps2*uncertainty)
        self.seq += 1
        frame = dict(base, seq=self.seq, quality='speed', reason='encoder_initializing', t=common,
                     speed=v, yaw_rate=w, wheel_speed=velocities, speed_t=speeds_t,
                     position_t=positions_t, time_uncertainty_s=uncertainty,
                     sigma_speed_mps=max(.015,sigma,max(wl.accel_mps2,wl.decel_mps2)*uncertainty),
                     sigma_yaw_rate_rps=2*sigma/wl.track_m,
                     sigma_turn_rad=self.step*math.sqrt(2/3)/wl.track_m,
                     speed_source='hall' if hall_ok else 'readback_fallback',
                     hall_still=self.stationary(now, states),
                     hall_motion_age_s=[h[3] if h else None for h in hall],
                     hall_wheel_speed=estimated if hall_ok else None,diagnostics=diagnostics,
                     feedback_delay_s=self.feedback_delay)
        if None not in positions:
            frame.update(encoder or {})
            self.anchor = (common, *positions)
        else:
            self.anchor = None
            frame['reason'] = 'encoder_unavailable'
        if not hall_ok and positions_ok:
            frame['reason'] = ('encoder_reset' if reset else 'encoder_gap' if gap else 'hall_speed_initializing')
        self.last_t = common
        return frame
