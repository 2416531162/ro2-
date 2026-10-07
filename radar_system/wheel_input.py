"""详细轮程输入的校验及源选择；旧 /odom 与同源轮程不能同时进入融合。"""
import math

from gnss_fusion import WheelSample


class WheelInput:
    def __init__(self, started, timeout=.2):
        self.started, self.timeout = started, timeout
        self.seen = False
        self.epoch, self.seq, self.rx = None, -1, -math.inf
        self.pending = None

    def legacy(self, t, speed, rate):
        self.pending = (t, speed, rate)

    def fallback(self, now):
        pending, self.pending = self.pending, None
        if (self.seen or now-self.started < .25 or pending is None
                or not 0 <= now-pending[0] <= self.timeout):
            return None
        return WheelSample(*pending)

    def decode(self, frame, now):
        def number(key, low=-math.inf, high=math.inf):
            value = frame[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('轮程字段无效：'+key)
            return value

        if (not isinstance(frame, dict) or frame.get('version') != 1
                or frame.get('time_basis') != 'host_receive'
                or not isinstance(frame.get('epoch'), str) or not frame['epoch']
                or frame.get('quality') not in ('pending', 'invalid', 'speed', 'encoder')):
            raise ValueError('轮程格式无效')
        rx = number('t_rx', now-1, now)
        seq = number('seq', 0)
        if int(seq) != seq or rx < self.rx:
            return None
        epoch = frame['epoch']
        # 接到有效版本即确认能力。故障心跳不能刷新测量，也不能用旧 /odom 掩盖故障。
        self.seen, self.rx = True, rx
        if self.epoch != epoch:
            self.epoch, self.seq = epoch, -1
        if frame['quality'] in ('pending', 'invalid') or seq <= self.seq:
            return None
        t = number('t', rx-1, rx)
        speed, rate = number('speed', -3, 3), number('yaw_rate', -20, 20)
        timing = number('time_uncertainty_s', 0, self.timeout)
        sigma = number('sigma_speed_mps', 0, 3)
        extra = dict(source=frame['quality'], time_uncertainty_s=timing, sigma_speed_mps=sigma, epoch=epoch)
        if 'speed_source' in frame:
            if frame['speed_source'] not in ('hall','readback_fallback') or (frame.get('hall_still') is not None and type(frame['hall_still']) is not bool):
                raise ValueError('轮速证据格式无效')
            extra.update(speed_basis=frame['speed_source'],hall_still=frame.get('hall_still'),
                         sigma_yaw_rate_rps=number('sigma_yaw_rate_rps',0,30))
        if frame['quality'] == 'encoder':
            start = number('start', t-self.timeout, t)
            if not start < t:
                raise ValueError('轮程区间为空')
            dt = t-start
            distance, turn = number('distance', -3*dt, 3*dt), number('turn', -20*dt, 20*dt)
            # 霍尔一格量化误差可大于短区间最大路程，不能按速度上限截掉标准差。
            sd = number('sigma_distance_m', 0, 3)
            extra.update(end_t=t,end_speed=speed,end_yaw_rate=rate,sigma_speed_mps=max(sigma,sd/dt),end_sigma_speed_mps=sigma)
            if 'sigma_turn_rad' in frame:
                extra['sigma_yaw_rate_rps']=max(extra.get('sigma_yaw_rate_rps',0),number('sigma_turn_rad',0,3)/dt)
            sample = WheelSample(start, distance/dt, turn/dt, **extra)
        else:
            sample = WheelSample(t, speed, rate, **extra)
        self.seq = seq
        return sample
