"""Gen5 差速底盘与传感器的实车标定模型（车端代码仿真用）。

只模拟「车」和「传感器」：规划、控制、融合全部是车端仓库的真实代码。参数来源：
- 轮速环：2026-10-02 17:09–17:18 六段实车作业记录，每个轮子 下发 erpm_sent → 读回 erpm_read 拟合：纯延迟 0.08 s
  + 一阶滞后 0.12 s（有效约 0.2 s），驱动器内部无额外斜坡、无共振；残差 0.044 m/s（信号 0.52 m/s）。
- 前万向轮（CAD：转轴在后轴前 0.724 m、拖距 0.0625 m）：无侧滑运动学 + 轴承静摩擦（卡住角 6°），转动时的力矩折成两轮
  转向扰动。按同批 16 次起步拟合：倒车后向前走 0.18 m 才翻、峰值 0.19 rad/s、车头甩约 2°；原地转后倒车车头顺转向甩 2°。
- 低速轮速不平顺：0.08–0.2 m/s 两轮转向残差 σ 0.07 rad/s（1–3 Hz），0.5 m/s 以上不加。
- 传感器：主 / 副天线固定解 20 Hz（慢漂 4 mm / 5 s + 白噪声 1 mm，延迟 24 / 26 ms）、双天线测向 20 Hz（每趟偏差
  ±0.3°、慢漂 0.3°、白噪声 0.3°，延迟 45 ms）、BESTVEL、IMU 50 Hz（陀螺 + 航向角 0.01° 量化）、轮速。车端有详细轮程
  （robot_core.wheel_motion、wheel_input）且融合配置打开 wheel_increment_enabled 时，走驱动器累计角 → 对齐 → WheelInput 的
  真实链路，否则 /odom。
- 闭环核对（同代码 6d53366、同一条 23.6 m 直线，与实车控制器横偏比）：助跑 0.9 / 实车 1.4、线头 0.6 / 0.5、
  进线 0–1 m 0.7 / 0.7、1–3 m 1.0 / 0.8、巡航 1.1 / 0.8 cm（巡航偏悲观约 0.3 cm）。
"""
import heapq
import itertools
import json
import math
import os
import random
import re
import tempfile

# 车上 fusion.json（10-02 17:16 作业记录里的标定）
CALIBRATION_DEFAULT = dict(antenna_xy=[0.24, 0.2075], baseline_yaw_body_deg=-90.0, mount_offset_deg=0.0,
                           antenna_height_m=0.53)
ORIGIN = (31.3675, 120.7358, 12.85)
BASELINE_M = 0.415
TRACK_M, WHEEL_R = 0.675, 0.2046
PX, TRAIL = 0.724, 0.0625                       # 万向轮转轴在后轴前、拖距
MODEL_INFO = dict(
    chassis='Gen5 实车拟合：两轮速度环 延迟 0.08 s + 一阶 0.12 s；前万向轮（转轴 0.724 m、拖距 0.0625 m、卡住 6°）；'
            '低速轮速不平顺；轮子每圈起伏 0.5 %；地面扰动按路程',
    sensors='主 / 副天线 20 Hz、双天线测向 20 Hz、BESTVEL、IMU 航向角 50 Hz、轮速（详细轮程或 /odom）',
    source='2026-10-02 轮速/传感器拟合；2026-10-04 角速度二阶响应拟合（1.1 Hz、阻尼 .30）')
_RV = (0.0, 0.03, 0.08, 0.14, 0.25, 0.35, 0.5)
_RS = (0.0, 0.045, 0.065, 0.065, 0.03, 0.008, 0.0)


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return ys[-1]


def ripple_sigma(v):
    """低速轮速不平顺，两轮转向扰动的 σ（rad/s）。"""
    return _interp(abs(v), _RV, _RS)


class Gen5Chassis:
    """真值底盘。command(t, vx, wz) 收车端下发；step(t, dt) 推进。x / y / yaw / v / wz 是车身真实运动，
    wheel_v / wheel_w 是两轮实际转出来的（轮速读回、/odom、累计角看得到），enc 是两轮累计路程（按标称半径）。"""

    def __init__(self, seed, *, delay_s=0.08, wheel_lag_s=0.12, ground=0.004, wheel_scale=1.0, caster=True, angular_hz=1.1, angular_damping=.30):
        self.angular_hz,self.angular_damping=angular_hz,angular_damping
        self.angular_rate=self.angular_speed=0.
        self.rng = random.Random(seed * 7919 + 1)          # 地面、轮子起伏
        self.rng2 = random.Random(seed * 104729 + 7)       # 万向轮、低速不平顺：开关对比时不改动其余噪声
        self.delay_s, self.wheel_lag_s, self.ground, self.wheel_scale = delay_s, wheel_lag_s, ground, wheel_scale
        self.caster = caster
        self.x = self.y = self.yaw = 0.0
        self.v = self.wz = 0.0
        self.vl = self.vr = 0.0
        self.wheel_v = self.wheel_w = 0.0
        self.enc = [0.0, 0.0]
        self.buf = []
        self.eps = [[(0.005, self.rng.uniform(0, 2 * math.pi)), (0.004, self.rng.uniform(0, 2 * math.pi))] for _ in range(2)]
        self.th = [self.rng.uniform(0, 2 * math.pi), self.rng.uniform(0, 2 * math.pi)]
        self.g = 0.0
        self.phi = self.rng2.gauss(0.0, math.radians(5.0))  # 万向轮相对车身转角（0 = 车向前时的稳定位置）
        self.wc = self.ripple = 0.0
        self._moving = False

    def command(self, t, vx, wz):
        self.buf.append((t, vx, wz))

    def _cmd_at(self, t):
        v = w = 0.0
        for tt, vv, ww in self.buf:
            if tt <= t:
                v, w = vv, ww
            else:
                break
        return v, w

    def _caster(self, dt, v, w):
        vpx, vpy = v, w * PX
        sp = math.hypot(vpx, vpy)
        moving = sp > 0.005
        if moving and not self._moving:                     # 每次从停车起步随机抖一下，决定翻转方向
            self.phi += self.rng2.gauss(0.0, math.radians(5.0))
        self._moving = moving
        s, c = math.sin(self.phi), math.cos(self.phi)
        v_perp, v_roll = -vpx * s + vpy * c, vpx * c + vpy * s
        phidot = v_perp / TRAIL - w
        stuck = sp > 1e-4 and v_roll > 0 and abs(v_perp) < sp * math.sin(math.radians(6.0))
        if stuck or sp <= 1e-4:
            phidot = 0.0
        self.phi = math.remainder(self.phi + phidot * dt, 2 * math.pi)
        swivel = math.tanh(phidot / 3.0) * (1.0 - PX / TRAIL * c) / (PX / TRAIL)
        drag = 0.0 if stuck else -math.tanh(v_roll / 0.02) * s
        self.wc += (0.06 * swivel + 0.05 * drag - self.wc) * dt / 0.08
        sig = ripple_sigma(v)
        self.ripple += -self.ripple * dt / 0.1 + sig * math.sqrt(2 * dt / 0.1) * self.rng2.gauss(0.0, 1.0)
        return self.wc + self.ripple

    def step(self, t, dt):
        while len(self.buf) > 2 and self.buf[1][0] <= t - self.delay_s - 0.5:
            self.buf.pop(0)
        vc, wc = self._cmd_at(t - self.delay_s)
        a = dt / (self.wheel_lag_s + dt)
        self.vl += (vc - wc * TRACK_M / 2 - self.vl) * a
        self.vr += (vc + wc * TRACK_M / 2 - self.vr) * a
        v, w = (self.vl + self.vr) / 2, (self.vr - self.vl) / TRACK_M
        if self.angular_hz is not None:
            # 当前板子 284bbaac 的入弧整形针对实测二阶角速度环；模拟保留同一共振。
            _, target_w = self._cmd_at(t - .05)
            wn = 2 * math.pi * self.angular_hz
            self.angular_rate += (wn*wn*(1.05*target_w-self.angular_speed)-2*self.angular_damping*wn*self.angular_rate)*dt
            self.angular_speed += self.angular_rate*dt
            w = self.angular_speed
        dis = self._caster(dt, v, w) if self.caster else 0.0
        self.wheel_v, self.wheel_w = v, w + dis
        wl, wr = v - self.wheel_w * TRACK_M / 2, v + self.wheel_w * TRACK_M / 2
        self.enc[0] += wl * dt
        self.enc[1] += wr * dt
        ripple = [sum(amp * math.cos((h + 1) * self.th[i] + ph) for h, (amp, ph) in enumerate(self.eps[i])) for i in range(2)]
        gl, gr = wl * (1 + ripple[0]) * self.wheel_scale, wr * (1 + ripple[1]) * self.wheel_scale
        ds, L = abs(v) * dt, 1.6
        self.g += -self.g * ds / L + (self.ground / 0.8) * math.sqrt(2 * ds / L) * self.rng.gauss(0.0, 1.0)
        self.th[0] += wl * dt / WHEEL_R
        self.th[1] += wr * dt / WHEEL_R
        self.v, self.wz = (gl + gr) / 2, (gr - gl) / TRACK_M + self.g * abs(v)
        mid = self.yaw + self.wz * dt / 2
        self.x += self.v * dt * math.cos(mid)
        self.y += self.v * dt * math.sin(mid)
        self.yaw += self.wz * dt

    def describe(self):
        return dict(model='gen5', delay_s=self.delay_s, wheel_lag_s=self.wheel_lag_s, ground_rad_s=self.ground,
                    caster=self.caster, wheel_scale=self.wheel_scale, track_m=TRACK_M,
                    angular_hz=self.angular_hz,angular_damping=self.angular_damping,angular_delay_s=.05,angular_gain=1.05)


class IdealChassis:
    """旧模型：车端 path_sim.DifferentialChassis（纯延迟 0.28 s + 轮加速度 0.6 m/s²，无万向轮、扰动）。"""

    def __init__(self, path_sim):
        self.c = path_sim.DifferentialChassis()
        self.enc = [0.0, 0.0]

    x = property(lambda s: s.c.x, lambda s, v: setattr(s.c, 'x', v))
    y = property(lambda s: s.c.y, lambda s, v: setattr(s.c, 'y', v))
    yaw = property(lambda s: s.c.yaw, lambda s, v: setattr(s.c, 'yaw', v))
    v = property(lambda s: s.c.v)
    wz = property(lambda s: s.c.wz)
    wheel_v = property(lambda s: s.c.v)
    wheel_w = property(lambda s: s.c.wz)

    def command(self, t, vx, wz):
        self.c.command(t, vx, wz)

    def step(self, t, dt):
        self.c.step(t, dt)
        self.enc[0] += self.c.left * dt
        self.enc[1] += self.c.right * dt

    def describe(self):
        return dict(model='ideal', chassis='path_sim.DifferentialChassis', delay_s=self.c.delay_s,
                    wheel_accel=self.c.wheel_accel, max_wheel=self.c.max_wheel, track_m=self.c.TRACK_M)


def check_calibration(value):
    cal = dict(CALIBRATION_DEFAULT, **(value or {}))
    xy = cal['antenna_xy']
    if not (isinstance(xy, (list, tuple)) and len(xy) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                                                   and math.isfinite(v) and -3 <= v <= 3 for v in xy)):
        raise ValueError('主天线位置须为两个 −3–3 m 的数')
    for key, low, high in (('baseline_yaw_body_deg', -360, 360), ('mount_offset_deg', -30, 30), ('antenna_height_m', 0, 3)):
        v = cal[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(f'{key}须在 {low}–{high} 之间')
    unknown = set(cal) - set(CALIBRATION_DEFAULT)
    if unknown:
        raise ValueError('未知标定项：' + '、'.join(sorted(unknown)))
    return dict(cal, antenna_xy=[float(xy[0]), float(xy[1])])


def load_fusion_config(source, gnss_fusion, calibration):
    """车上 line_node.load_fusion_config 原样执行（line_node 依赖 rclpy，只取出这个函数和模块级常量），
    读的 fusion.json 换成给定标定。返回 (FusionConfig, 说明)。"""
    text = open(os.path.join(source, 'radar_system', 'line_node.py'), encoding='utf-8').read()
    start = text.index('def load_fusion_config')
    end = text.index('\ndef ', start + 10)
    ns = dict(os=os, json=json, math=math)
    try:
        from line_mission import RTK_HOLD_S
        ns['RTK_HOLD_S'] = RTK_HOLD_S
    except ImportError:
        pass
    for line in text.splitlines():                    # 模块级大写常量（OBSOLETE_FUSION_KEYS 等）
        if re.match(r'^[A-Z][A-Z0-9_]* *=', line):
            try:
                exec(line, ns)
            except Exception:  # noqa: BLE001  依赖 rclpy 等的常量跳过
                pass
    ns.setdefault('OBSOLETE_FUSION_KEYS', ('baseline_m', 'baseline_tol_m', 'baseline_check'))
    exec(text[start:end], ns)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'fusion.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(calibration, fh)
        return ns['load_fusion_config'](gnss_fusion, path)


class Gen5Sensors:
    """把真值底盘变成车上收到的测量，按到达时刻送给 LineMission（同 line_node 的话题回调）。"""

    def __init__(self, seed, fusion, mission, car, calibration, *, t0=0.0, wheel_detail=True):
        self.rng = random.Random(seed * 65537 + 3)
        self.fusion, self.mission, self.car = fusion, mission, car
        cal = calibration
        self.ant1 = tuple(cal['antenna_xy'])
        b = math.radians(cal['baseline_yaw_body_deg'])
        self.ant2 = (self.ant1[0] + BASELINE_M * math.cos(b), self.ant1[1] + BASELINE_M * math.sin(b))
        self.baseline = b
        self.bias = math.radians(random.Random(10000 + seed).uniform(-0.3, 0.3))   # 每趟测向偏差（实车反推 ±0.3°）
        self.rtk = [0.0, 0.0]
        self.hdg_slow = self.rng.gauss(0.0, math.radians(0.3))
        self.imu_off = self.rng.uniform(-math.pi, math.pi)
        self.imu_drift = math.radians(self.rng.gauss(0.0, 0.003))
        self.queue, self.seq = [], itertools.count()
        self.next = dict(gnss=t0, imu=t0)
        self.wheel = None
        if wheel_detail:
            self.wheel = self._wheel_path(t0)
        self.wheel_source = 'encoder' if self.wheel else 'odom'

    def _wheel_path(self, t0):
        cfg = getattr(self.fusion, 'config', None)
        if not getattr(cfg, 'wheel_increment_enabled', False):
            return None
        try:
            from oid_link import WheelConfig
            from robot_core.wheel_motion import WheelMotionAligner
            from wheel_input import WheelInput
        except ImportError:
            return None
        wcfg = WheelConfig(pole_pairs=20, left_sign=1, right_sign=1)
        from types import SimpleNamespace
        return dict(cfg=wcfg, aligner=WheelMotionAligner(wcfg), input=WheelInput(t0, getattr(cfg, 'sensor_timeout_s', .2)),
                    states={1: SimpleNamespace(t_speed=None, speed_erpm=None, t_position=None, position_cdeg=None),
                            2: SimpleNamespace(t_speed=None, speed_erpm=None, t_position=None, position_cdeg=None)},
                    next=[t0, t0 + 0.007])

    def _later(self, t, fn):
        heapq.heappush(self.queue, (t, next(self.seq), fn))

    def _body(self, xy):
        c, s = math.cos(self.car.yaw), math.sin(self.car.yaw)
        return self.car.x + xy[0] * c - xy[1] * s, self.car.y + xy[0] * s + xy[1] * c

    def step(self, t):
        car, m, rng = self.car, self.mission, self.rng
        if t >= self.next['gnss'] - 1e-9:
            self.next['gnss'] += 0.05
            a = math.exp(-0.05 / 5.0)
            self.rtk = [a * v + math.sqrt(1 - a * a) * 0.004 * rng.gauss(0.0, 1.0) for v in self.rtk]
            for kind, ant, lat_s in (('fix', self.ant1, 0.024), ('fix2', self.ant2, 0.026)):
                ax, ay = self._body(ant)
                lat, lon = self.fusion.to_geodetic(ax + self.rtk[0] + rng.gauss(0.0, 0.001), ay + self.rtk[1] + rng.gauss(0.0, 0.001))
                msg = dict(kind=kind, t=t, lat=lat, lon=lon, height=ORIGIN[2], sol='NARROW_INT', sigma_n=0.0083,
                           sigma_e=0.0082, sigma_u=0.0233, diff_age=0.7, sats=40)
                self._later(t + lat_s + rng.uniform(0, 0.006), lambda msg=msg: m.on_gnss(msg))
            ah = math.exp(-0.05 / 3.0)
            self.hdg_slow = ah * self.hdg_slow + math.sqrt(1 - ah * ah) * math.radians(0.3) * rng.gauss(0.0, 1.0)
            yaw_m = car.yaw + self.bias + self.hdg_slow + rng.gauss(0.0, math.radians(0.3))
            heading = (90.0 - math.degrees(yaw_m + self.baseline)) % 360.0
            hmsg = dict(kind='heading', t=t, heading=heading, pitch=0.0, length=BASELINE_M + rng.gauss(0.0, 0.002),
                        sol='NARROW_INT', sigma_heading=0.7, sats=39, valid=True)
            self._later(t + 0.045 + rng.uniform(0, 0.01), lambda msg=hmsg: m.on_gnss(msg))
            track = car.yaw if car.v >= 0 else car.yaw + math.pi
            vmsg = dict(kind='vel', t=t, speed=abs(car.v), track=(90.0 - math.degrees(track)) % 360.0, sol='DOPPLER_VELOCITY')
            self._later(t + 0.029, lambda msg=vmsg: m.on_gnss(msg))
        while self.queue and self.queue[0][0] <= t + 1e-12:
            heapq.heappop(self.queue)[2]()
        if t >= self.next['imu'] - 1e-9:
            self.next['imu'] += 0.02
            yi = car.yaw + self.imu_off + self.imu_drift * t
            yaw_i = math.radians(round(math.degrees(math.atan2(math.sin(yi), math.cos(yi))), 2))
            m.on_imu(t, car.wz + rng.gauss(0.0, math.radians(0.05)), yaw_i, roll=0.0, pitch=0.0)
            if self.wheel:
                self.wheel['input'].legacy(t, car.wheel_v, car.wheel_w)
            else:
                m.on_wheel(t, car.wheel_v, car.wheel_w)
        if self.wheel:
            self._wheel_feedback(t)

    def _wheel_feedback(self, t):
        w, car = self.wheel, self.car
        speeds = (car.wheel_v - car.wheel_w * TRACK_M / 2, car.wheel_v + car.wheel_w * TRACK_M / 2)
        for k, motor in enumerate((1, 2)):
            if t < w['next'][k] - 1e-9:
                continue
            w['next'][k] += 0.02                      # 左右驱动器各约 50 Hz 回一次（不同总线，接收时刻错开带抖动）
            rx = t + self.rng.uniform(0, 0.002)
            st = w['states'][motor]
            st.t_speed = st.t_position = rx
            st.speed_erpm = float(w['cfg'].to_erpm(speeds[k]))
            st.position_cdeg = int(round(math.degrees(car.enc[k] / w['cfg'].wheel_radius_m) * 100))
            frame = w['aligner'].update(rx, w['states'])
            try:
                sample = w['input'].decode(json.loads(json.dumps(dict(frame, epoch='sim'))), rx)
            except ValueError:
                sample = None
            if sample is not None:
                self.mission.on_wheel_motion(sample)

    def control_tick(self, t):
        """line_node.control 开头：详细轮程中断时用 /odom 兜底。"""
        if self.wheel:
            sample = self.wheel['input'].fallback(t)
            if sample is not None:
                self.mission.on_wheel_motion(sample)

    def describe(self):
        return dict(model='gen5', antenna_main=list(self.ant1), antenna_aux=[round(v, 4) for v in self.ant2],
                    heading_bias_deg=round(math.degrees(self.bias), 3), wheel_source=self.wheel_source,
                    rtk_drift_m=0.004, rtk_drift_tau_s=5.0, heading_white_deg=0.3, heading_slow_deg=0.3)
