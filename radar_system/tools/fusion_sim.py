"""可复现的离线传感器仿真；阿克曼真值不使用融合模块的预测器。"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

from gnss_fusion import (DualHeading, FusionConfig, GnssFix, GnssFusion,
                         GnssVelocity, ImuSample, WheelSample)


@dataclass(frozen=True)
class Segment:
    duration: float
    speed: float
    steer: float = 0.0


@dataclass(frozen=True)
class Truth:
    t: float
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float


@dataclass(frozen=True)
class Event:
    arrival: float
    sensor: str
    sample: object


class FusionSimulation:
    def __init__(self, segments=None, *, seed=20260925, delay=.08, jitter=.02,
                 heading_outage=None, fix_outage=None, initial_yaw_deg=20.,
                 noise=True, imu_jitter=True, spikes=True, mount_deg=1.5,
                 wheel_error=.02, bias_deg_s=.1):
        self.segments = tuple(segments or (Segment(3., 0.), Segment(40., .5)))
        self.duration = sum(s.duration for s in self.segments)
        self.seed, self.delay, self.jitter = seed, delay, jitter
        self.heading_outage, self.fix_outage = heading_outage, fix_outage
        self.initial_yaw = math.radians(initial_yaw_deg)
        self.noise, self.imu_jitter, self.spikes = noise, imu_jitter, spikes
        self.mount_deg, self.wheel_error = mount_deg, wheel_error
        self.bias = math.radians(bias_deg_s)
        self.config = FusionConfig(mount_offset_deg=mount_deg)
        self.frame = GnssFusion()
        self.frame.set_origin(31.2304, 121.4737, 12.)

    @staticmethod
    def _rate(segment):
        if abs(segment.steer) < 1e-12 or segment.speed == 0:
            return 0.
        radius = .52 / math.tan(abs(segment.steer)) + .60 / 2
        return segment.speed / radius * (1 if segment.steer > 0 else -1)

    def truth(self, t):
        x = y = 0.
        yaw = self.initial_yaw
        elapsed = 0.
        for segment in self.segments:
            dt = min(max(0., t-elapsed), segment.duration)
            w, v = self._rate(segment), segment.speed
            if abs(w) > 1e-12:
                x += v/w * (math.sin(yaw+w*dt)-math.sin(yaw))
                y += v/w * (math.cos(yaw)-math.cos(yaw+w*dt))
            else:
                x += v*dt*math.cos(yaw)
                y += v*dt*math.sin(yaw)
            yaw += w*dt
            elapsed += segment.duration
            if t < elapsed - 1e-9:
                return Truth(t, x, y, yaw, v, w)
        return Truth(t, x, y, yaw, self.segments[-1].speed, self._rate(self.segments[-1]))

    @staticmethod
    def _out(t, interval):
        return interval is not None and interval[0] <= t < interval[1]

    def _events(self, sensor):
        # 各传感器独立随机流，修改传输延迟不改变物理测量值。
        number = ('wheel', 'imu', 'heading', 'fix', 'velocity').index(sensor)
        rng = np.random.default_rng(self.seed+number)
        transit = np.random.default_rng(self.seed+100+number)
        previous_arrival = -1.
        for i in range(int(self.duration*20)+1):
            t = i*.05
            if sensor == 'imu' and self.imu_jitter and i > 0:
                t += float(np.clip(rng.normal(0., .0013), -.004, .004))
                if i % 199 == 0:
                    t += .01
            if t > self.duration:
                continue
            truth = self.truth(t)
            if sensor == 'imu':
                raw = truth.yaw_rate+self.bias+(rng.normal(0., math.radians(.034)) if self.noise else 0.)
                raw = round(raw/.00026644)*.00026644 if self.noise else raw
                if self.spikes and i > 0 and i % 800 == 311:
                    raw -= math.radians(3.8)
                sample = ImuSample(t, float(raw))
            elif sensor == 'wheel':
                speed = truth.speed*(1+self.wheel_error)
                sample = WheelSample(t, round(speed, 3), round(truth.yaw_rate, 3))
            elif sensor == 'heading':
                # yaw = 90-heading-baseline+mount，反算接收机应报告的值。
                heading = 90.-math.degrees(truth.yaw)-self.config.baseline_yaw_body_deg+self.mount_deg
                heading += float(rng.normal(0., .3)) if self.noise else 0.
                valid = not self._out(t, self.heading_outage) and not self._out(t, self.fix_outage)
                sample = DualHeading(t, heading % 360., 0., .33, 'L1_INT' if valid else 'NONE', .3, 20)
            elif sensor == 'fix':
                ax, ay = self.config.antenna_xy
                cs, sn = math.cos(truth.yaw), math.sin(truth.yaw)
                ex, ey = rng.normal(0., .01, 2) if self.noise else (0., 0.)
                lat, lon = self.frame.to_geodetic(truth.x+cs*ax-sn*ay+ex, truth.y+sn*ax+cs*ay+ey)
                sol = 'NONE' if self._out(t, self.fix_outage) else 'NARROW_INT'
                sample = GnssFix(t, lat, lon, 12., sol, .01, .01, .02, .2, 20)
            else:
                ax, ay = self.config.antenna_xy
                # 天线速度含 ω×杆臂，不能直接拿转弯时的航迹当车头方向。
                vx, vy = truth.speed-truth.yaw_rate*ay, truth.yaw_rate*ax
                if self.noise:
                    vx += float(rng.normal(0., .03))
                    vy += float(rng.normal(0., .03))
                track = 90.-math.degrees(truth.yaw+math.atan2(vy, vx))
                sol = 'NONE' if self._out(t, self.fix_outage) else 'DOPPLER_VELOCITY'
                sample = GnssVelocity(t, math.hypot(vx, vy), track % 360., sol)
            arrival = t
            if sensor not in ('wheel', 'imu'):
                arrival += self.delay+float(transit.uniform(-self.jitter, self.jitter))
            arrival = max(t, arrival, previous_arrival+1e-7)
            previous_arrival = arrival
            yield Event(arrival, sensor, sample)

    def events(self):
        """流式合并，仅保留每个传感器的下一个事件，可用于一小时内存测试。"""
        streams = [self._events(k) for k in ('wheel', 'imu', 'heading', 'fix', 'velocity')]
        return heapq.merge(*streams, key=lambda e: (e.arrival, ('wheel', 'imu', 'heading', 'fix', 'velocity').index(e.sensor)))


def feed(fusion, event):
    getattr(fusion, 'add_'+event.sensor)(event.sample)


def run_simulation(simulation, config=None):
    fusion = GnssFusion(config or simulation.config)
    fusion.set_origin(31.2304, 121.4737, 12.)
    rows = []
    for event in simulation.events():
        feed(fusion, event)
        if event.sensor == 'imu':
            pose = fusion.pose(event.sample.t)
            if pose is not None:
                rows.append((simulation.truth(pose.t), pose))
    return fusion, rows


def metrics(rows, start=3., stop=math.inf):
    selected = [(truth, pose) for truth, pose in rows if start <= truth.t <= stop]
    yaw = [math.degrees(math.atan2(math.sin(p.yaw-t.yaw), math.cos(p.yaw-t.yaw))) for t, p in selected]
    position = [math.hypot(p.x-t.x, p.y-t.y) for t, p in selected]
    return {'samples': len(yaw), 'yaw_rms_deg': float(np.sqrt(np.mean(np.square(yaw)))),
            'yaw_max_deg': max(map(abs, yaw)), 'position_rms_m': float(np.sqrt(np.mean(np.square(position)))),
            'position_max_m': max(position)}


if __name__ == '__main__':
    import json
    fusion, rows = run_simulation(FusionSimulation())
    print(json.dumps({'metrics': metrics(rows), 'calibration': fusion.calibration()}, ensure_ascii=False, indent=2))
