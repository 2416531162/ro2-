"""坡度开关与永久保存的 IMU 水平零点（四向原地旋转标定）；只在停稳、无任务时改变定位参考。"""
import dataclasses
import json
import math
import os
import tempfile

# 单天线接收机自己按杆长做倾斜补偿（line_node.single_antenna_config），车端坡度补偿不能再开，免得补两次
SINGLE_TILT_BLOCKED = '单天线接收机自己做坡度补偿（接收机上设杆长），车端不再补'
# HDA536T 水平零点（2026-10-11 用户：「先把横滚约 −0.57°、俯仰约 −0.55° 补偿进去，等我再一次手动清零再自动清除」）。
# 车上 imu_zero.json 是 10-06 以前在不平处清的零（横滚 −0.25°、俯仰 +0.41°）。10-06–10-10 五天 24 次大角度原地转前后
# 静止读数取平均（坡度正反抵消）、同一条路来回反向的直线、整圆三种算法都反推出水平时 IMU 读数约为横滚 −0.57°、俯仰 −0.55°
# （差 0.3° / 0.9°，天线高 0.53 m → 左右 0.3 cm、前后 0.9 cm），五天里读数稳定，不是传感器漂移。
# 文件还是旧零点时用反推值；用户再标定后文件写的是新值，照用（同 line_node 天线手量值换实测值）。不改车上文件，回滚即恢复
HDA536T_ZERO_OLD_DEG = (-0.25, 0.41)
HDA536T_ZERO_MEASURED_DEG = (-0.57, -0.55)

# 四向停稳原地旋转标定（0° → 90° → 180° → 270° → 回 360° 原朝向）参数
SELECT_TIMEOUT_S = 2.0
SETTLE_0_S = 0.1
SETTLE_S = 0.4
RETURN_SETTLE_S = 0.2
SAMPLE_S = 0.8
MIN_SAMPLES = 5
MAX_STOP_SPREAD_DEG = 0.8
MAX_PAIR_DIFF_DEG = 0.6
MAX_ZERO_DEG = 15.0
MAX_CAL_WZ = 0.6
MIN_CAL_WZ = 0.08
ACCEL_WZ = 1.0
STOP_TOL_DEG = 0.8
LEAD_S = 0.08
TURN_TIMEOUT_S = 12.0
TOTAL_TIMEOUT_S = 45.0
SENSOR_STALE_S = 0.3


def _wrap_deg(angle):
    """角度归到 (−180, 180]。"""
    return (angle + 180.0) % 360.0 - 180.0


def _solve3(matrix, rhs):
    """高斯消元解 3×3 线性方程组 M·x = rhs；奇异时抛 ValueError。"""
    a = [list(matrix[i]) + [float(rhs[i])] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-9:
            raise ValueError('四向采样角度分布异常')
        a[col], a[pivot] = a[pivot], a[col]
        div = a[col][col]
        for j in range(col, 4):
            a[col][j] /= div
        for r in range(3):
            if r != col and abs(a[r][col]) > 0.0:
                factor = a[r][col]
                for j in range(col, 4):
                    a[r][j] -= factor * a[col][j]
    return a[0][3], a[1][3], a[2][3]


def solve_four_way_zero(stops):
    """由 0°/90°/180°/270° 四个停稳方向的采样均值拟合 IMU 安装零点与地面坡度。

    在相对起始车头转角 Δψ 下：
        roll_raw(Δψ)  = r0 + a_r·cos(Δψ) + b_r·sin(Δψ)
        pitch_raw(Δψ) = p0 + a_p·cos(Δψ) + b_p·sin(Δψ)
    按实测停稳转角 Δψ 最小二乘解出 (r0, a_r, b_r) 与 (p0, a_p, b_p)，既彻底抵消地面坡度，
    也不要求停车角度严丝合缝卡在 90° 整数倍；剩余 1 个自由度即 (0°+180°)/2 与 (90°+270°)/2
    的正交对偶残差，用来交叉校验四个方向是否处于同一平整坡面（识别轮子压坑或凸包）。
    """
    if len(stops) != 4:
        raise ValueError('四向采样未完成')
    rows = []
    for k, st in enumerate(stops):
        turned = float(st['turned_deg'])
        if abs(turned - 90.0 * k) > 15.0:
            raise ValueError('第 %d 向转角偏差过大（%.1f°）' % (k + 1, turned))
        rad = math.radians(turned)
        rows.append((1.0, math.cos(rad), math.sin(rad)))
    m = [[sum(rows[k][i] * rows[k][j] for k in range(4)) for j in range(3)] for i in range(3)]
    rolls = [float(st['roll_deg']) for st in stops]
    pitches = [float(st['pitch_deg']) for st in stops]
    rhs_r = [sum(rows[k][i] * rolls[k] for k in range(4)) for i in range(3)]
    rhs_p = [sum(rows[k][i] * pitches[k] for k in range(4)) for i in range(3)]
    r0, ar, br = _solve3(m, rhs_r)
    p0, ap, bp = _solve3(m, rhs_p)
    res_r = [rolls[k] - (r0 + ar * rows[k][1] + br * rows[k][2]) for k in range(4)]
    res_p = [pitches[k] - (p0 + ap * rows[k][1] + bp * rows[k][2]) for k in range(4)]
    pair_diff = 2.0 * max(max(abs(e) for e in res_r), max(abs(e) for e in res_p))
    if pair_diff > MAX_PAIR_DIFF_DEG:
        raise ValueError('四向倾角不一致（相差 %.2f°），地面不平或轮子压坑，请换平缓处重试' % pair_diff)
    if abs(r0) > MAX_ZERO_DEG or abs(p0) > MAX_ZERO_DEG:
        raise ValueError('标定零点超出合理范围（横滚 %+.2f°、俯仰 %+.2f°），请检查传感器安装' % (r0, p0))
    ground_slope_deg = 0.5 * (math.hypot(ar, br) + math.hypot(ap, bp))
    return r0, p0, ground_slope_deg, pair_diff


class AttitudeControls:
    def __init__(self, mission, fusion_path):
        self.mission, self.fusion_path = mission, fusion_path
        self.still_since = None
        self.results = {}
        self.wheel = None
        self.zero_path = os.path.join(os.path.dirname(fusion_path), 'imu_zero.json') if fusion_path else None
        self.default_zero_path = self.zero_path
        self.zero_save_error = None
        self._cal = None
        self._release_pending = False
        self._completed_result = None
        self._restore_zero()

    @property
    def calibrating(self):
        return self._cal is not None

    @property
    def active(self):
        return self._cal is not None or self._release_pending

    def pop_result(self):
        res, self._completed_result = self._completed_result, None
        return res

    def select_imu_source(self, source):
        """每一路使用独立水平零点；旧 HDA536T 零点文件保持兼容。换源时融合已经重建。"""
        if self._cal is not None:
            self.stop('切换 IMU 来源，旋转标定已停止')
        self.zero_path = (os.path.join(os.path.dirname(self.default_zero_path), 'imu_zero-imu762.json')
                          if source == 'imu762' and self.default_zero_path else self.default_zero_path)
        self.zero_save_error = None
        # 开机恢复到另一来源且它还没有零点时，也不能借用旧 IMU 的水平零点。
        self.mission.fusion._tilt_zero = (0.0, 0.0)
        self.mission.fusion._tilt_zeroed = False
        self._restore_zero()

    def observe_wheel(self, t, speed, rate, hall_still=None):
        """只用实际底盘观测，不依赖 RTK 固定解或融合位姿初始化。"""
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in (t, speed, rate))
                or hall_still is not None and type(hall_still) is not bool):
            return
        if self.wheel is None or t > self.wheel[0]:
            self.wheel = (t, speed, rate, hall_still)
            if abs(speed) > .02 or abs(rate) > .05 or hall_still is False:
                self.still_since = None

    def observe_sample(self, sample):
        self.observe_wheel(sample.end_t if sample.end_t is not None else sample.t,
            sample.end_speed if sample.end_t is not None else sample.speed,
            sample.end_yaw_rate if sample.end_t is not None else sample.yaw_rate,
            sample.hall_still)

    def _wheel_still(self):
        w = self.wheel
        return w is not None and abs(w[1]) <= .02 and abs(w[2]) <= .05 and w[3] is not False

    def blocker(self, now, motion):
        if self._cal is not None:
            return '正在执行 IMU 旋转标定'
        m = self.mission
        if m.phase != 'IDLE' or m.operations.held or m.braking:
            self.still_since = None
            return '请先结束当前作业，再操作姿态设置'
        if not isinstance(motion, dict) or motion.get('mode') not in ('IDLE', 'MANUAL') or not motion.get('healthy'):
            self.still_since = None
            return '等待车辆空闲且状态正常'
        command = motion.get('command', {})
        values = (command.get('vx'), command.get('wz'))
        if (not all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 1e-6 for v in values)
                or motion.get('waiting_stationary')):
            self.still_since = None
            return '请先停车并等待停稳'
        wheel = self.wheel
        if wheel is None or not 0 <= now - wheel[0] <= .2:
            self.still_since = None
            return '等待新鲜底盘轮速反馈'
        if abs(wheel[1]) > .02 or abs(wheel[2]) > .05 or wheel[3] is False:
            self.still_since = None
            return '请先停车并等待停稳'
        if self.still_since is None:
            self.still_since = now
        return '等待车辆停稳' if now - self.still_since < .3 - 1e-9 else None

    def status(self, now, motion, attitude):
        blocked = self.blocker(now, motion)
        fresh = (attitude is not None and 0 <= now - attitude[0] <= .2
                 and all(math.isfinite(v) for v in attitude[1:3]))
        has_yaw = fresh and len(attitude) >= 4 and type(attitude[3]) in (int, float) and math.isfinite(attitude[3])
        diff_ok = getattr(self.mission, 'differential', True) is not False
        # blocked 管坡度开关、zero_blocked 管 IMU 旋转标定；单天线只挡坡度开关（标定存的零点双天线照用）
        single = getattr(self.mission, 'antenna_mode', 'dual') == 'single'
        zero_blocked = (blocked
                        or (None if diff_ok else '当前底盘不支持原地旋转标定')
                        or (None if fresh else '等待新鲜 IMU 姿态数据')
                        or (None if has_yaw else '等待 IMU 航向数据'))
        return dict(blocked=SINGLE_TILT_BLOCKED if single else blocked,
                    zero_blocked=zero_blocked,
                    zero_persistent=self.zero_path is not None, zero_save_error=self.zero_save_error,
                    calibrating=self.calibrating,
                    calibration_progress=self._cal['progress'] if self._cal is not None else None)

    def stop(self, reason='旋转标定已停止', request_id=None):
        cal = self._cal
        if cal is None:
            return True, '未在标定'
        cal_id = request_id or cal['id']
        self._cal = None
        self._release_pending = True
        self.still_since = None
        result = (False, reason)
        if cal_id:
            self._remember_result(cal_id, result)
        self._completed_result = dict(ok=False, message=reason, id=cal_id, cmd='imu_zero')
        return True, reason

    def _remember_result(self, request_id, result):
        if request_id:
            self.results[request_id] = result
            if len(self.results) > 32:
                del self.results[next(iter(self.results))]

    def _start_calibration(self, request_id, now, attitude):
        progress = '正在旋转标定：第 1/4 向（0°）停稳采集中…'
        self._cal = dict(
            id=request_id,
            started=now,
            select_since=now,
            driving=False,
            stage='select',
            step=0,
            start_yaw_deg=float(attitude[3]),
            last_yaw_deg=float(attitude[3]),
            turned_deg=0.0,
            target_deg=0.0,
            turn_since=now,
            settle_since=now,
            samples=[],
            stops=[],
            wz=0.0,
            braking=False,
            brake_since=0.0,
            last_tick=now,
            last_sample_t=None,
            progress=progress,
        )
        self._release_pending = False
        self._completed_result = None
        return True, progress

    def _finish_calibration(self, ok, message, keep_still=False):
        cal = self._cal
        cal_id = cal['id'] if cal is not None else None
        settle_since = cal.get('settle_since') if cal is not None else None
        self._cal = None
        self._release_pending = False
        self.still_since = settle_since if (ok and keep_still) else None
        result = (ok, message)
        self._remember_result(cal_id, result)
        self._completed_result = dict(ok=ok, message=message, id=cal_id, cmd='imu_zero')

    def tick(self, now, motion_mode, attitude):
        """50 Hz 控制周期：驱动四向停稳原地旋转（0°→90°→180°→270°→360°），返回 (vx, wz, select)。"""
        cal = self._cal
        if cal is None:
            if self._release_pending:
                self._release_pending = False
                return 0.0, 0.0, False
            return 0.0, 0.0, None
        dt = max(0.0, min(0.1, now - cal['last_tick']))
        cal['last_tick'] = now
        m = self.mission
        if m.phase != 'IDLE' or m.operations.held or m.braking:
            self._finish_calibration(False, '姿态设置未完成：作业已启动，旋转标定已取消')
            return 0.0, 0.0, False
        if now - cal['started'] > TOTAL_TIMEOUT_S:
            self._finish_calibration(False, '姿态设置未完成：旋转标定超时，已停止')
            return 0.0, 0.0, False
        if self.wheel is None or not 0 <= now - self.wheel[0] <= SENSOR_STALE_S:
            self._finish_calibration(False, '姿态设置未完成：底盘轮速反馈中断，旋转标定已停止')
            return 0.0, 0.0, False
        if (attitude is None or not 0 <= now - attitude[0] <= SENSOR_STALE_S
                or len(attitude) < 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in attitude[1:4])):
            self._finish_calibration(False, '姿态设置未完成：IMU 姿态数据中断，旋转标定已停止')
            return 0.0, 0.0, False

        dyaw = _wrap_deg(float(attitude[3]) - cal['last_yaw_deg'])
        cal['last_yaw_deg'] = float(attitude[3])
        cal['turned_deg'] += dyaw

        if cal['stage'] == 'select':
            if motion_mode == 'NAVIGATION':
                cal['driving'] = True
                cal['stage'] = 'sample'
                cal['settle_since'] = now
                cal['turn_since'] = now
                cal['samples'].clear()
            elif now - cal['select_since'] > SELECT_TIMEOUT_S:
                self._finish_calibration(False, '姿态设置未完成：底盘控制权申请失败（当前 %s）' % motion_mode)
                return 0.0, 0.0, False
            else:
                cal['progress'] = '正在旋转标定：准备开始…'
                return 0.0, 0.0, True

        if motion_mode != 'NAVIGATION':
            self._finish_calibration(False, '姿态设置未完成：控制权被切走（%s），旋转标定已停止' % motion_mode)
            return 0.0, 0.0, False

        still = self._wheel_still() and abs(dyaw) <= max(0.35, 3.5 * dt)

        if cal['stage'] == 'sample':
            k = cal['step']
            target_label = k * 90
            cal['wz'] = 0.0
            if not still:
                cal['settle_since'] = now
                cal['samples'].clear()
                if now - cal['turn_since'] > TURN_TIMEOUT_S:
                    self._finish_calibration(False, '姿态设置未完成：等待车辆停稳超时，旋转标定已停止')
                    return 0.0, 0.0, False
                cal['progress'] = '正在旋转标定：第 %d/4 向（%d°）等待停稳…' % (k + 1, target_label)
                return 0.0, 0.0, None
            settle_need = SETTLE_0_S if k == 0 else SETTLE_S
            if now - cal['settle_since'] < settle_need - 1e-9:
                cal['progress'] = '正在旋转标定：第 %d/4 向（%d°）停稳中…' % (k + 1, target_label)
                return 0.0, 0.0, None
            if attitude[0] != cal['last_sample_t']:
                cal['last_sample_t'] = attitude[0]
                cal['samples'].append((float(attitude[0]), float(attitude[1]), float(attitude[2]), cal['turned_deg']))
            samples = cal['samples']
            elapsed = (samples[-1][0] - samples[0][0]) if len(samples) >= 2 else 0.0
            cal['progress'] = '正在旋转标定：第 %d/4 向（%d°）停稳采集中…' % (k + 1, target_label)
            if len(samples) >= MIN_SAMPLES and elapsed >= SAMPLE_S - 1e-9:
                rolls = [s[1] for s in samples]
                pitches = [s[2] for s in samples]
                if max(rolls) - min(rolls) > MAX_STOP_SPREAD_DEG or max(pitches) - min(pitches) > MAX_STOP_SPREAD_DEG:
                    self._finish_calibration(False, '姿态设置未完成：采样期间车身晃动过大，请停稳后重试')
                    return 0.0, 0.0, False
                n = len(samples)
                cal['stops'].append(dict(
                    step=k,
                    roll_deg=sum(rolls) / n,
                    pitch_deg=sum(pitches) / n,
                    turned_deg=sum(s[3] for s in samples) / n,
                    n=n,
                ))
                cal['step'] = k + 1
                cal['target_deg'] = (k + 1) * 90.0
                cal['stage'] = 'turn'
                cal['turn_since'] = now
                cal['braking'] = False
                cal['brake_since'] = 0.0
                cal['samples'].clear()
            return 0.0, 0.0, None

        if cal['stage'] == 'turn':
            if now - cal['turn_since'] > TURN_TIMEOUT_S:
                self._finish_calibration(False, '姿态设置未完成：原地旋转超时（轮子可能受阻），旋转标定已停止')
                return 0.0, 0.0, False
            k = cal['step']
            target_deg = cal['target_deg']
            remaining_deg = target_deg - cal['turned_deg']
            geo = getattr(self.mission, 'geo', None)
            max_wz = min(MAX_CAL_WZ, float(getattr(geo, 'pivot_rate_rad_s', MAX_CAL_WZ) or MAX_CAL_WZ))
            accel_wz = min(ACCEL_WZ, float(getattr(geo, 'pivot_accel_rad_s2', ACCEL_WZ) or ACCEL_WZ))
            floor_wz = min(MIN_CAL_WZ, max_wz)
            wheel_wz = self.wheel[2] if self.wheel is not None else cal['wz']
            lead_deg = math.degrees(max(0.0, wheel_wz) * LEAD_S)
            if k < 4:
                cal['progress'] = '正在旋转标定：转向第 %d/4 向（%d°）· 已转 %.0f°' % (
                    k + 1, int(target_deg), max(0.0, cal['turned_deg']))
            else:
                cal['progress'] = '正在旋转标定：转回起始朝向（360°）· 已转 %.0f°' % max(0.0, cal['turned_deg'])

            if not cal['braking'] and remaining_deg - lead_deg <= STOP_TOL_DEG:
                cal['braking'] = True
                cal['brake_since'] = now

            if cal['braking']:
                step_w = accel_wz * dt if dt > 0 else accel_wz * 0.02
                if abs(cal['wz']) <= step_w or remaining_deg <= STOP_TOL_DEG:
                    cal['wz'] = 0.0
                else:
                    cal['wz'] = max(0.0, cal['wz'] - step_w)
                if abs(cal['wz']) < 1e-6 and still:
                    if remaining_deg > 5.0 and now - cal['brake_since'] >= 0.2:
                        # 草地阻力提前停住（欠转超过 5°）：恢复低速推到目标窗口
                        cal['braking'] = False
                    else:
                        if k < 4:
                            cal['stage'] = 'sample'
                            cal['settle_since'] = now
                            cal['turn_since'] = now
                            cal['samples'].clear()
                        else:
                            cal['stage'] = 'return_settle'
                            cal['settle_since'] = now
                            cal['turn_since'] = now
                        return 0.0, 0.0, None
                return 0.0, cal['wz'], None

            rem_rad = math.radians(max(0.0, remaining_deg - lead_deg))
            want_wz = max(floor_wz, min(max_wz, math.sqrt(2.0 * accel_wz * rem_rad), 1.8 * rem_rad))
            step_w = accel_wz * dt if dt > 0 else accel_wz * 0.02
            cal['wz'] += max(-step_w, min(step_w, want_wz - cal['wz']))
            return 0.0, cal['wz'], None

        if cal['stage'] == 'return_settle':
            cal['wz'] = 0.0
            cal['progress'] = '正在旋转标定：停稳保存中…'
            if not still:
                cal['settle_since'] = now
                if now - cal['turn_since'] > TURN_TIMEOUT_S:
                    self._finish_calibration(False, '姿态设置未完成：等待车辆停稳超时，旋转标定已停止')
                    return 0.0, 0.0, False
                return 0.0, 0.0, None
            if now - cal['settle_since'] < RETURN_SETTLE_S - 1e-9:
                return 0.0, 0.0, None
            try:
                r0, p0, slope_deg, _ = solve_four_way_zero(cal['stops'])
                self._save_zero(r0, p0)
                self.mission.fusion.zero_tilt(math.radians(r0), math.radians(p0))
                self.zero_save_error = None
                msg = '旋转标定完成：零点 横滚 %+.2f°、俯仰 %+.2f°（地面坡度 %.2f°），重启后保留' % (r0, p0, slope_deg)
                self._finish_calibration(True, msg, keep_still=True)
            except (ValueError, TypeError, OSError) as exc:
                self._finish_calibration(False, '姿态设置未完成：' + str(exc))
            return 0.0, 0.0, False

        return 0.0, 0.0, None

    def command(self, cmd, now, motion, attitude):
        request_id = cmd.get('id')
        kind = cmd.get('cmd')
        if kind == 'imu_zero' and cmd.get('action') == 'stop':
            return self.stop('旋转标定已停止', request_id)
        # 同一标定指令即使重复转发，也不能重新触发。
        if request_id and request_id in self.results:
            return self.results[request_id]
        if kind == 'imu_zero' and self._cal is not None and request_id and request_id == self._cal['id']:
            return True, self._cal['progress']
        st = self.status(now, motion, attitude)
        problem = st['zero_blocked'] if kind == 'imu_zero' else st['blocked']
        try:
            if problem:
                raise ValueError(problem)
            fusion = self.mission.fusion
            if kind == 'tilt':
                enabled = cmd.get('enabled')
                if type(enabled) is not bool:
                    raise ValueError('坡度开关必须是布尔值')
                if enabled and fusion.config.antenna_height_m <= 0:
                    raise ValueError('请先设置天线离地高度')
                cfg = dataclasses.replace(fusion.config, tilt_compensation_enabled=enabled)
                self._save(enabled)
                fusion.config = cfg
                fusion.tilt_reference_changed()
                result = True, '坡度补偿已开启' if enabled else '坡度补偿已关闭'
            elif kind == 'imu_zero':
                if not request_id:
                    raise ValueError('IMU 标定必须带指令 id')
                return self._start_calibration(request_id, now, attitude)
            else:
                raise ValueError('不支持的姿态设置')
        except (ValueError, TypeError, OSError) as exc:
            result = False, '姿态设置未完成：' + str(exc)
        self._remember_result(request_id, result)
        return result

    def _save(self, enabled):
        if not self.fusion_path:
            raise ValueError('车辆未配置融合标定文件路径')
        path = self.fusion_path
        data = {}
        if os.path.exists(path):
            with open(path, encoding='utf-8') as fh:
                data = json.load(fh)
            if not isinstance(data, dict):
                raise ValueError('原标定文件格式错误')
        data['tilt_compensation_enabled'] = enabled
        self._write_json(path, data)

    @staticmethod
    def _write_json(path, data):
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix='.attitude-', suffix='.tmp', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2, allow_nan=False)
                fh.write('\n'); fh.flush(); os.fsync(fh.fileno())
            os.replace(tmp, path)
            directory_fd = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    @staticmethod
    def _zero_values(data):
        if not isinstance(data, dict) or type(data.get('version')) is not int or data['version'] != 1:
            raise ValueError('IMU 零点文件版本无效')
        roll, pitch = data.get('roll_deg'), data.get('pitch_deg')
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in (roll, pitch))
                or abs(roll) > 180 or abs(pitch) > 90):
            raise ValueError('IMU 零点角度无效')
        return roll, pitch

    def _save_zero(self, roll, pitch):
        if not self.zero_path:
            raise ValueError('车辆未配置 IMU 零点保存路径')
        data = dict(version=1, roll_deg=roll, pitch_deg=pitch)
        self._zero_values(data)
        self._write_json(self.zero_path, data)

    def _restore_zero(self):
        if not self.zero_path or not os.path.exists(self.zero_path):
            return
        try:
            with open(self.zero_path, encoding='utf-8') as fh:
                roll, pitch = self._zero_values(json.load(fh))
            if (self.zero_path == self.default_zero_path
                    and all(abs(v - old) < .005 for v, old in zip((roll, pitch), HDA536T_ZERO_OLD_DEG))):
                roll, pitch = HDA536T_ZERO_MEASURED_DEG
            self.mission.fusion.zero_tilt(math.radians(roll), math.radians(pitch))
        except (ValueError, TypeError, OSError) as exc:
            self.zero_save_error = '已保存的 IMU 零点无法读取，请重新标定：' + str(exc)
