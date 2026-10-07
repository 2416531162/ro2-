"""坡度开关与永久保存的 IMU 水平零点；只在停稳、无任务时改变定位参考。"""
import dataclasses
import json
import math
import os
import tempfile

# 单天线接收机自己按杆长做倾斜补偿（line_node.single_antenna_config），车端坡度补偿不能再开，免得补两次
SINGLE_TILT_BLOCKED = '单天线接收机自己做坡度补偿（接收机上设杆长），车端不再补'


class AttitudeControls:
    def __init__(self, mission, fusion_path):
        self.mission, self.fusion_path = mission, fusion_path
        self.still_since = None
        self.results = {}
        self.wheel = None
        self.zero_path = os.path.join(os.path.dirname(fusion_path), 'imu_zero.json') if fusion_path else None
        self.default_zero_path = self.zero_path
        self.zero_save_error = None
        self._restore_zero()

    def select_imu_source(self, source):
        """每一路使用独立水平零点；旧 HDA536T 零点文件保持兼容。换源时融合已经重建。"""
        self.zero_path = (os.path.join(os.path.dirname(self.default_zero_path),'imu_zero-imu762.json')
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

    def blocker(self, now, motion):
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
        # blocked 管坡度开关、zero_blocked 管 IMU 清零；单天线只挡坡度开关（清零存的零点双天线照用）
        single = getattr(self.mission, 'antenna_mode', 'dual') == 'single'
        return dict(blocked=SINGLE_TILT_BLOCKED if single else blocked,
                    zero_blocked=blocked or (None if fresh else '等待新鲜 IMU 姿态数据'),
                    zero_persistent=self.zero_path is not None, zero_save_error=self.zero_save_error)

    def command(self, cmd, now, motion, attitude):
        request_id = cmd.get('id')
        # 同一清零指令即使重复转发，也不能拿后来的角度重新清零。
        if request_id and request_id in self.results:
            return self.results[request_id]
        kind = cmd.get('cmd')
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
                    raise ValueError('IMU 清零必须带指令 id')
                self._save_zero(attitude[1], attitude[2])
                fusion.zero_tilt(math.radians(attitude[1]), math.radians(attitude[2]))
                self.zero_save_error = None
                result = True, '横滚、俯仰已清零并保存，重启后保留'
            else:
                raise ValueError('不支持的姿态设置')
        except (ValueError, TypeError, OSError) as exc:
            result = False, '姿态设置未完成：' + str(exc)
        if request_id:
            self.results[request_id] = result
            if len(self.results) > 32:
                del self.results[next(iter(self.results))]
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
            self.mission.fusion.zero_tilt(math.radians(roll), math.radians(pitch))
        except (ValueError, TypeError, OSError) as exc:
            self.zero_save_error = '已保存的 IMU 零点无法读取，请重新清零：' + str(exc)
