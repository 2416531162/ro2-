"""两路 IMU 的观测、选源和持久配置；不发布运动或模块设置命令。"""
from collections import deque
import json
import math
from pathlib import Path

from attitude_controls import AttitudeControls

SOURCES = ('hda536t','imu762')
LABELS = {'hda536t':'HDA536T（CAN）','imu762':'IMU762（USB）'}


class ImuSources:
    def __init__(self, supported, path=None):
        self.supported = supported
        self.path = Path(path) if path else None
        self.active = 'hda536t'
        self.revision = 0
        self.error = None
        self.samples = {key:None for key in SOURCES}
        self.times = {key:deque(maxlen=1024) for key in SOURCES}
        self.driver = {key:{} for key in SOURCES}
        self.results = {}
        if self.path and self.path.exists():
            try:
                data = json.loads(self.path.read_text())
                if data.get('version') != 1 or data.get('active') not in SOURCES:
                    raise ValueError('已保存的 IMU 来源无效')
                self.active = data['active']
            except (ValueError,OSError,AttributeError) as exc:
                self.error = 'IMU 选择无法恢复：'+str(exc)

    def observe(self, source, stamp, attitude, gyro):
        values = (stamp,gyro,*(attitude or ()))
        if source not in SOURCES or attitude is None or not all(math.isfinite(v) for v in values):
            return False
        if self.samples[source] is not None and stamp <= self.samples[source][0]:
            return False
        self.samples[source] = (stamp,*attitude,gyro)
        self.times[source].append(stamp)
        return True

    def ready(self, source, now):
        sample = self.samples.get(source)
        return sample is not None and 0 <= now-sample[0] <= .2

    def select(self, source, now):
        if not self.supported:
            raise ValueError('当前车型不支持两路 IMU 切换')
        if source not in SOURCES:
            raise ValueError('不支持的 IMU 来源')
        if not self.ready(source,now):
            raise ValueError('目标 IMU 没有新鲜的姿态和角速度数据')
        if source == self.active:
            return False
        if not self.path:
            raise ValueError('没有配置 IMU 来源保存路径')
        AttitudeControls._write_json(str(self.path),dict(version=1,active=source))
        self.active = source
        self.revision += 1
        self.error = None
        return True

    def status(self, now, blocked):
        sensors = []
        for source in SOURCES:
            sample = self.samples[source]
            times = self.times[source]
            while times and now-times[0] > 2.:times.popleft()
            hz = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
            driver = self.driver[source]
            row = dict(source=source,label=LABELS[source],available=self.ready(source,now),
                       age_s=None if sample is None else round(max(0.,now-sample[0]),3),
                       nominal_hz=400 if source=='imu762' else 50,
                       received_hz=driver.get('rate_hz',driver.get('rates_hz',{}).get('角度',round(hz,1))),feed_hz=round(hz,1),
                       temperature_c=driver.get('temp_c'),bad_checksum=driver.get('bad_checksum',0),
                       sequence_gaps=driver.get('sequence_gaps',0),problem=driver.get('problem'))
            if sample is not None:
                row.update(roll_deg=round(sample[1],3),pitch_deg=round(sample[2],3),
                           yaw_deg=round(sample[3],3),gyro_z_deg_s=round(math.degrees(sample[4]),4))
            sensors.append(row)
        return dict(supported=self.supported,active=self.active,revision=self.revision,
                    blocked=blocked,error=self.error,sensors=sensors)
