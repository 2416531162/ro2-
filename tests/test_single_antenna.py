#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单天线流动站（2026-10-07）：平板转来的单天线接收机出 $INS.NAVI 就切单天线，它 5 Hz 的 INS.NAVI 当定位（GGA 只给质量），杠杆臂换成右天线
（后轴中心前 0.234 m、右 0.212 m）；没有测向，车头按行驶轨迹定，起步「轮子往前动一下就知道车头」。"""
import json
import math
import os
import random
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)

import ros_stubs  # noqa: E402

ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')

import gnss_fusion as gf  # noqa: E402
import line_mission  # noqa: E402
import line_node  # noqa: E402
import runtime_config  # noqa: E402
import um982  # noqa: E402
from line_mission import LineMission  # noqa: E402


def nmea(body):
    return '$%s*%02X' % (body, um982.nmea_checksum(body))


# 2026-10-07 单天线流动站 S20241460 实抓（室内，GGA 是 Q2 差分）：INS.NAVI 5 Hz、GGA 1 Hz
REAL = [
    '$INS.NAVI,4552.400,31.367864179,120.737001398,22.541874806,0.002085,0.000165,0.003432,-0.013684,-0.074292,'
    '9.999900,-1.000000,0.000000,0.000000,0.000000,-0.000541,-0.001381,0.007070,46.739132,169607169*5E',
    '$GNGGA,004553.00,3122.07183817,N,12044.22006923,E,2,28,0.6,13.7943,M,8.7952,M,2.0,137*65',
    '$GNGSA,M,3,05,15,20,23,24,,,,,,,,1.2,0.6,1.1,1*3C',
    '$GNGSA,M,3,01,02,03,04,06,07,09,10,14,21,22,34,1.2,0.6,1.1,4*31',
    '$INS.NAVI,4553.000,31.367863969,120.737001154,22.589430003,0.001646,-0.000153,0.002961,-0.013674,-0.074294,'
    '9.999900,-1.000000,0.000000,0.000000,0.000000,-0.000542,-0.001380,0.007070,46.739132,169607169*71',
    '$INS.NAVI,4553.200,31.367863902,120.737001074,22.605236731,0.001572,-0.000832,0.002941,-0.013694,-0.074331,'
    '9.999900,-1.000000,0.000000,0.000000,0.000000,-0.000542,-0.001380,0.007070,46.739132,169607169*74',
]


def ins(hms, lat=31.367863969, lon=120.737001154, h=22.589430003):
    return nmea('INS.NAVI,%s,%.9f,%.9f,%.9f,0.001646,-0.000153,0.002961,-0.013674,-0.074294,9.999900,-1.000000,'
                '0.000000,0.000000,0.000000,-0.000542,-0.001380,0.007070,46.739132,169607169' % (hms, lat, lon, h))


def gsa(pdop='1.2'):
    return nmea('GNGSA,M,3,01,02,03,04,06,07,09,10,14,21,22,34,%s,0.6,1.1,4' % pdop)


def gga(hms='004553.00', quality='4', sats='28', age='2.0'):
    return nmea('GNGGA,%s,3122.07183817,N,12044.22006923,E,%s,%s,0.6,13.7943,M,8.7952,M,%s,137' % (hms, quality, sats, age))


def gst(hms='004553.00', lat='0.006', lon='0.004', alt='0.013'):
    return nmea('GNGST,%s,0.8,0.010,0.006,45.0,%s,%s,%s' % (hms, lat, lon, alt))


BESTPOS = ('#BESTPOSA,COM1,0,55.0,FINESTEERING,2437,443243.000,00000000,0000,1114;'
           'SOL_COMPUTED,NARROW_INT,31.61309208,120.72830388,25.15,8.50,WGS84,0.010,0.011,0.020,"0",'
           '1.000,0.000,40,32,32,32,0,00,00,00')
BESTPOS = '%s*%08x' % (BESTPOS, um982.crc32(BESTPOS[1:].encode('ascii')))


def feed(stream, *lines, t=10.0):
    out = []
    for i, line in enumerate(lines):
        out += stream.feed((line + '\r\n').encode('ascii'), t + 0.05 * i)
    return out


class Stream(unittest.TestCase):
    def test_real_capture_fields(self):
        p = um982.parse_ins_navi(REAL[4])                      # 00:45:53.000 那帧
        self.assertAlmostEqual(p['tod'], 45 * 60 + 53.0)
        self.assertEqual((p['lat'], p['lon'], p['height']), (31.367863969, 120.737001154, 22.589430003))
        # 同一秒的 GGA：经纬度、椭球高（海拔 + 大地水准面差距）一致
        g = um982.parse_gga(REAL[1])
        self.assertAlmostEqual(p['lat'], 31 + 22.07183817 / 60, places=8)
        self.assertAlmostEqual(p['lon'], 120 + 44.22006923 / 60, places=8)
        self.assertAlmostEqual(p['height'], float(g[9]) + float(g[11]), places=3)

    def test_ins_navi_is_the_position_and_gga_only_gives_the_quality(self):
        s = um982.Um982Stream()
        out = feed(s, gga(), gsa(), ins('4553.000')) + feed(s, ins('4553.200', lat=31.367863902), t=10.30)
        self.assertEqual(len(out), 2)
        m = out[0]
        self.assertEqual((m['kind'], m['source'], m['antenna_mode'], m['sol']), ('fix', 'INS.NAVI', 'single', 'NARROW_INT'))
        self.assertEqual((m['pdop'], m['pdop_ok']), (1.2, True))
        self.assertEqual((m['lat'], m['lon'], m['height']), (31.367863969, 120.737001154, 22.589430003))
        self.assertEqual((m['sats'], m['diff_age']), (28, 2.0))
        self.assertAlmostEqual(out[1]['t'] - m['t'], 0.2, places=3)       # 按 INS.NAVI 自己的时刻
        st = s.status(10.2)
        self.assertEqual((st['antenna_mode'], st['rtk'], st['antennas']['ant1']['source']), ('single', 'FIXED', 'INS.NAVI'))
        self.assertIsNone(st['sigma_h'])                        # INS.NAVI 不带精度：不把假定值当接收机精度报
        self.assertEqual(st['pdop'], 1.2)
        self.assertFalse(st['antennas']['ant2']['available'])

    def test_gga_is_never_a_position(self):
        s = um982.Um982Stream()
        feed(s, ins('4552.800'))
        self.assertEqual(feed(s, gga('004553.00'), gga('004554.00'), t=11.0), [])

    def test_real_capture_is_differential_not_fixed(self):
        out = feed(um982.Um982Stream(), *REAL)
        self.assertEqual([m['sol'] for m in out], ['NONE', 'PSRDIFF', 'PSRDIFF'])   # 第一帧前面还没有 GGA
        self.assertEqual([m['pdop'] for m in out], [None, 1.2, 1.2])

    def test_quality_only_from_a_recent_gga(self):
        s = um982.Um982Stream()
        out = feed(s, gga('004550.00'), ins('4551.400'), ins('4553.000'))
        self.assertEqual([m['sol'] for m in out], ['NARROW_INT', 'NONE'])
        out = feed(s, gga('004553.00', quality='5'), ins('4553.200'), t=12.0)
        self.assertEqual(out[0]['sol'], 'NARROW_FLOAT')

    def test_pdop_must_be_below_two(self):
        # 用户规则：单天线 PDOP 小于 3 才可信，3 及以上一律不用；解类型照实报
        for lines, ok in (((gsa('2.0'),), True), ((gsa('2.99'),), True), ((gsa('3.0'),), False), ((gsa('3.4'),), False),
                          ((), False)):
            with self.subTest(lines=lines):
                s = um982.Um982Stream()
                m = feed(s, gga(), *lines, ins('4553.000'))[-1]
                self.assertEqual((m['sol'], m['pdop_ok']), ('NARROW_INT', ok))
        s = um982.Um982Stream()
        feed(s, gga(), gsa('1.2'), ins('4553.000'))
        late = feed(s, gga('004556.00'), ins('4556.000'), t=12.5)[-1]     # PDOP 2.5 s 前收到的：过期
        self.assertEqual((late['pdop'], late['pdop_ok']), (None, False))

    def test_gst_gives_the_real_sigma(self):
        # 2026-10-07 接收机开了 GST：水平位置精度用它报的纬、经度 1σ，状态如实报，不再是假定的 1 cm；高程精度不用
        s = um982.Um982Stream()
        m = feed(s, gga(), gsa(), gst(), ins('4553.000'))[-1]
        self.assertEqual((m['sigma_n'], m['sigma_e'], m['sigma_u'], m['sigma_assumed']), (0.006, 0.004, 0.012, False))
        st = s.status(10.2)
        self.assertEqual(st['sigma_h'], 0.006)
        self.assertNotIn('sigma_v', st)
        self.assertEqual(st['pdop'], 1.2)

    def test_gst_only_from_the_same_second(self):
        s = um982.Um982Stream()
        m = feed(s, gga(), gsa(), gst('004550.00'), ins('4553.000'))[-1]   # 3 s 前的 GST：不是这一帧的精度
        self.assertEqual((m['sigma_n'], m['sigma_assumed']), (0.01, True))
        self.assertIsNone(s.status(10.2)['sigma_h'])
        m = feed(s, gst('004553.00', alt=''), ins('4553.200'), t=10.3)[-1]  # 高程精度缺了也照用（本来就不用）
        self.assertEqual((m['sigma_n'], m['sigma_assumed']), (0.006, False))

    def test_gst_parse_rejects_bad_lines(self):
        self.assertEqual(um982.gst_sigma(gst()), dict(tod=2753.0, sigma_n=0.006, sigma_e=0.004))
        self.assertIsNone(um982.gst_sigma(gst(lat='0.000')))                # 没算出精度
        self.assertIsNone(um982.gst_sigma(gst()[:-2] + '00'))               # 校验不对
        self.assertIsNone(um982.gst_sigma(gsa()))

    def test_bad_checksum_counts_and_keeps_mode(self):
        s = um982.Um982Stream()
        feed(s, ins('4552.800'))
        feed(s, REAL[0][:-2] + '00', t=11.0)                   # 校验和改坏
        self.assertEqual(s.crc_errors, 1)
        self.assertEqual(s.antenna_mode, 'single')

    def test_bestpos_switches_back_to_dual(self):
        s = um982.Um982Stream()
        feed(s, gga(), ins('4553.000'))
        out = feed(s, BESTPOS, gga('004554.00'), t=12.0)
        self.assertEqual([(m['source'], m['antenna_mode']) for m in out], [('BESTPOS', 'dual')])
        self.assertEqual(s.status(12.2)['antenna_mode'], 'dual')

    def test_measurement_time_survives_utc_midnight(self):
        s = um982.Um982Stream()
        feed(s, gga('235959.00'))
        a = feed(s, ins('235959.900'), t=20.0)[0]
        b = feed(s, ins('0.100'), t=20.2)[0]                   # 省掉前导 0 的 00:00:00.1
        self.assertAlmostEqual(b['t'] - a['t'], 0.2, places=3)
        self.assertEqual((a['sol'], b['sol']), ('NARROW_INT', 'NARROW_INT'))


GEN5 = dict(antenna_xy=[0.24, 0.2075], baseline_yaw_body_deg=-90.0, mount_offset_deg=0.0,
            antenna_height_m=0.53, tilt_compensation_enabled=True)
DIFF = dict(runtime_config.PROFILE, drive=dict(runtime_config.PROFILE.get('drive') or {}, type='differential'))


def gen5_configs():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'fusion.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(GEN5, fh)
        with mock.patch.object(runtime_config, 'PROFILE', DIFF):
            dual = line_node.load_fusion_config(gf, path)
    return dual, line_node.single_antenna_config(dual)


class Car:
    """直线车：mission 给的车速按 1 m/s² 跟上，车头不变；单天线装在右天线位置，定位 5 Hz（INS.NAVI）、2 mm 抖动。"""
    FRAME = gf._Frame(31.6, 120.7, 30.0)

    def __init__(self, mission, yaw_deg=30.0, mode='single'):
        self.m, self.mode = mission, mode
        self.x = self.y = self.v = self.odo = 0.0
        self.yaw = math.radians(yaw_deg)
        self.imu_yaw = 1.234                       # IMU 航向角是上电时的相对值，和地面方向差一个常数
        self.t, self.motion = 100.0, 'IDLE'
        self.rng = random.Random(7)
        self.selects = []
        self.pdop_ok = True
        self.pdop_missing = False                  # 接收机没出 GSA：没有 PDOP
        self.wobble = 0.0                          # 走着时车身左右摆的幅度（°/s，1 Hz）：实车低速摆 ±1–4°/s
        self.imu_drift = 0.0                       # IMU 航向角漂移（°/s）
        self.wz = 0.0
        self.free_v = None                         # 不经任务、直接按这个车速走（测试作业中途纠偏用）
        self.gnss_lag = 0.0                        # 卫星定位比它标的时间晚多少（秒），测延迟估计用
        self.hist = []

    def antenna(self, lever):
        ax, ay = lever
        x, y, yaw = self.x, self.y, self.yaw
        if self.gnss_lag > 0:
            # 「不晚」时报的 self.x 是上一拍末的位置（hist 里记在上一拍的时刻），晚 N 秒同一口径往前再退 N 秒
            t = self.t - 0.02 - self.gnss_lag
            x, y, yaw = next(((a, b, c) for (s, a, b, c) in reversed(self.hist) if s <= t + 1e-9),
                             self.hist[0][1:] if self.hist else (x, y, yaw))
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * ax - s * ay, y + s * ax + c * ay

    def sense(self):
        self.m.on_imu(self.t, self.wz, self.imu_yaw, roll=0.0, pitch=0.0)
        self.m.on_wheel(self.t, self.v, self.wz)
        if round(self.t * 1000) % (200 if self.mode == 'single' else 50) == 0:
            lever = line_node.SINGLE_ANTENNA_XY if self.mode == 'single' else line_node.GEN5_ANTENNA_MEASURED
            e, n = self.antenna(lever)
            lat, lon = self.FRAME.geodetic(e + self.rng.gauss(0, .002), n + self.rng.gauss(0, .002))
            self.m.on_gnss(dict(kind='fix', t=self.t, t_rx=self.t, lat=lat, lon=lon, height=30.0, sol='NARROW_INT',
                                status='SOL_COMPUTED', sigma_n=.01, sigma_e=.01, sigma_u=.02, diff_age=None,
                                sats=32, antenna=1, source='INS.NAVI' if self.mode == 'single' else 'BESTPOS',
                                antenna_mode=self.mode, pdop=None if self.pdop_missing else 1.2 if self.pdop_ok else 3.4,
                                pdop_ok=self.pdop_ok and not self.pdop_missing))

    def run(self, seconds, dt=0.02):
        """每 20 ms 一拍传感器，每 0.05 s 一拍任务。"""
        for _ in range(int(round(seconds / dt))):
            self.t = round(self.t + dt, 3)
            self.sense()
            if round(self.t * 1000) % 50 == 0:
                vx, wz, select = self.m.tick(self.t, self.motion)
                self.selects.append(select)
                if select is True:
                    self.motion = 'NAVIGATION'
                elif select is False:
                    self.motion = 'IDLE'
                target = vx if self.motion == 'NAVIGATION' else 0.0
                self.cmd = target
            target = getattr(self, 'cmd', 0.0) if self.motion == 'NAVIGATION' else 0.0
            if self.free_v is not None:
                target = self.free_v
            self.v += max(-dt, min(dt, target - self.v))      # 1 m/s²
            self.wz = math.radians(self.wobble) * math.sin(2 * math.pi * self.t) if abs(self.v) > 0.05 else 0.0
            self.yaw += self.wz * dt
            self.imu_yaw += self.wz * dt + math.radians(self.imu_drift) * dt
            step = self.v * dt
            self.x += step * math.cos(self.yaw)
            self.y += step * math.sin(self.yaw)
            self.odo += abs(step)
            if self.gnss_lag > 0:
                self.hist.append((self.t, self.x, self.y, self.yaw))
                del self.hist[:-200]


def mission():
    dual, single = gen5_configs()
    fusion = gf.GnssFusion(dual)
    return LineMission(fusion, gf, single_config=single), fusion, dual, single


class SingleAntennaMission(unittest.TestCase):
    def test_single_fix_switches_lever_to_right_antenna(self):
        m, fusion, dual, single = mission()
        car = Car(m)
        car.run(1.0)
        self.assertEqual(m.antenna_mode, 'single')
        self.assertEqual(fusion.config.antenna_xy, (0.234, -0.212))
        self.assertEqual(m.status()['antenna_mode'], 'single')
        pose = fusion.pose(car.t)
        self.assertIn('heading_uninitialized', pose.reasons)    # 没有测向，停着定不了车头
        car.mode = 'dual'
        car.run(0.2)
        self.assertEqual((m.antenna_mode, fusion.config.antenna_xy), ('dual', line_node.GEN5_ANTENNA_MEASURED))

    def test_orient_drives_forward_a_little_then_stops_with_heading(self):
        m, fusion, _, _ = mission()
        car = Car(m, yaw_deg=30.0)
        car.run(1.0)
        ok, msg = m.command({'cmd': 'route_orient'}, car.t)
        self.assertTrue(ok, msg)
        self.assertEqual(m.phase, 'ORIENTING')
        for _ in range(200):
            car.run(0.1)
            if m.phase == 'IDLE':
                break
        self.assertEqual(m.phase, 'IDLE', m.message)
        self.assertTrue(m.message.startswith('车头已确定'), m.message)
        car.run(0.5)
        self.assertIn(False, car.selects)                      # 控制权交还
        self.assertLess(abs(car.v), 0.01)
        self.assertLess(car.odo, 0.8)                          # 「往前动一下」：不到 0.8 m
        pose = fusion.pose(car.t)
        self.assertNotIn('heading_uninitialized', pose.reasons)
        self.assertLess(abs(math.degrees(gf._wrap(pose.yaw - car.yaw))), 1.0)
        # 后轴中心位置按右天线杠杆臂倒推回来（融合原点是第一帧定位，换算到同一个本地坐标再比）
        x, y = fusion.to_local(*car.FRAME.geodetic(car.x, car.y))
        self.assertLess(math.hypot(pose.x - x, pose.y - y), 0.01)

    def test_orient_works_whatever_way_the_car_faces(self):
        for yaw in (-170.0, -120.0, -10.0, 90.0, 175.0):
            with self.subTest(yaw=yaw):
                m, fusion, _, _ = mission()
                car = Car(m, yaw_deg=yaw)
                car.run(1.0)
                self.assertTrue(m.command({'cmd': 'route_orient'}, car.t)[0])
                for _ in range(200):
                    car.run(0.1)
                    if m.phase == 'IDLE':
                        break
                self.assertTrue(m.message.startswith('车头已确定'), m.message)
                self.assertLess(car.odo, 0.8)
                pose = fusion.pose(car.t)
                self.assertLess(abs(math.degrees(gf._wrap(pose.yaw - car.yaw))), 1.0)

    def test_line_start_orients_first_then_runs(self):
        m, fusion, _, _ = mission()
        car = Car(m, yaw_deg=40.0)
        car.run(1.0)
        c, s = math.cos(car.yaw), math.sin(car.yaw)
        m.points = {k: car.FRAME.geodetic(car.x + d * c, car.y + d * s) for k, d in (('A', 2.0), ('B', 8.0))}
        ok, msg = m.command({'cmd': 'start'}, car.t)
        self.assertEqual((ok, m.phase), (True, 'ORIENTING'), msg)
        phases = ['ORIENTING']
        for _ in range(200):
            car.run(0.05)
            if m.phase != phases[-1]:
                phases.append(m.phase)
            if m.phase == 'RUNNING':
                break
        self.assertEqual(phases, ['ORIENTING', 'SELECTING', 'RUNNING'])
        self.assertNotIn(False, car.selects)                  # 定完车头接着开始，控制权不交还
        self.assertLess(car.odo, 0.8)

    def test_orient_refused_on_dual_antenna_and_stop_ends_it(self):
        m, fusion, _, _ = mission()
        car = Car(m, mode='dual')
        car.run(0.5)
        ok, msg = m.command({'cmd': 'route_orient'}, car.t)
        self.assertFalse(ok)
        car.mode = 'single'
        car.run(0.5)
        self.assertTrue(m.command({'cmd': 'route_orient'}, car.t)[0])
        car.run(0.3)
        self.assertTrue(m.command({'cmd': 'stop'}, car.t)[0])
        self.assertEqual(m.phase, 'IDLE')
        car.run(1.0)
        self.assertLess(abs(car.v), 0.01)

    def test_orient_needs_fixed_position(self):
        m, fusion, _, _ = mission()
        car = Car(m)
        car.run(0.5)
        car.t += 1.0                                          # 1 s 没有固定解
        car.m.on_imu(car.t, 0.0, car.imu_yaw, roll=0.0, pitch=0.0)
        car.m.on_wheel(car.t, 0.0, 0.0)
        ok, msg = m.command({'cmd': 'route_orient'}, car.t)
        self.assertFalse(ok)
        self.assertIn('rtk_hold', msg)

    def test_start_without_points_or_route_does_not_move(self):
        m, fusion, _, _ = mission()
        car = Car(m)
        car.run(0.5)
        self.assertFalse(m.command({'cmd': 'start'}, car.t)[0])
        self.assertFalse(m.command({'cmd': 'route_start'}, car.t)[0])
        self.assertEqual(m.phase, 'IDLE')

    def test_pdop_not_ok_is_not_given_to_fusion(self):
        m, fusion, _, _ = mission()
        car = Car(m)
        car.pdop_ok = False
        car.run(1.0)
        self.assertEqual(m.antenna_mode, 'single')
        self.assertIsNone(fusion.pose(car.t))                 # 一直没有可信的固定解：融合没有位置
        car.pdop_ok = True
        car.run(0.5)
        self.assertIsNotNone(fusion.pose(car.t))

    def test_no_single_config_ignores_single_fixes(self):
        dual, _ = gen5_configs()
        fusion = gf.GnssFusion(dual)
        m = LineMission(fusion, gf)
        car = Car(m)
        car.run(0.5)
        self.assertIsNone(fusion.pose(car.t))                 # 按主天线杠杆臂算会差 0.41 m：不用


def running_line(yaw_deg=40.0):
    """单天线定完车头、沿一条 40 m 的线跑起来（已过 A 点 2 m）。"""
    m, fusion, _, _ = mission()
    car = Car(m, yaw_deg=yaw_deg)
    car.run(1.0)
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    m.points = {k: car.FRAME.geodetic(car.x + d * c, car.y + d * s) for k, d in (('A', 2.0), ('B', 40.0))}
    assert m.command({'cmd': 'start'}, car.t)[0]
    for _ in range(400):
        car.run(0.05)
        if m.phase == 'RUNNING' and m.tracker.st.state == 'RUNNING' and car.odo > 4.0:
            return m, car
    raise AssertionError(m.message)


class PdopTruthfulStatus(unittest.TestCase):
    """PDOP 不合格的帧按没有解交给融合，融合只会说 rtk_hold / rtk_fixed_timeout；App 上必须是真实原因（PDOP），
    不能说成「固定解中断」。"""

    def test_reason_code(self):
        m, _, _, _ = mission()
        m.antenna_mode = 'single'
        m._pdop = (10.0, 3.4, False)
        self.assertEqual(m._pdop_reason(10.5), 'pdop_high')
        self.assertIsNone(m._pdop_reason(12.0))               # 单天线定位 1.5 s 没来：说不清是不是 PDOP
        m._pdop = (10.0, None, False)
        self.assertEqual(m._pdop_reason(10.5), 'pdop_missing')
        m._pdop = (10.0, 1.2, True)
        self.assertIsNone(m._pdop_reason(10.5))
        m._pdop, m.antenna_mode = (10.0, 3.4, False), 'dual'
        self.assertIsNone(m._pdop_reason(10.5))               # 双天线不看 PDOP

    def test_high_pdop_while_running_says_pdop_not_fix_lost(self):
        m, car = running_line()
        car.pdop_ok = False
        car.run(0.3)                                          # 等下一帧单天线定位（5 Hz）带来 PDOP 不合格
        messages, first = [], set()
        for _ in range(100):
            car.run(0.1)
            if not messages or messages[-1] != m.message:
                messages.append(m.message)
            first.add(m.pose.reasons[0])
        self.assertEqual(first, {'pdop_high'})                # 根因排在最前面，App 按它翻译
        self.assertEqual(m.status()['pose']['reasons'][0], 'pdop_high')
        problem = line_mission.pose_problem(m.pose)
        self.assertIn('rtk_fixed_timeout', m.pose.reasons)
        self.assertTrue(problem.startswith('pose_unusable：pdop_high'), problem)
        self.assertNotIn('rtk_fixed_timeout', problem)          # PDOP 带出来的「固定解中断」不说
        self.assertIn('单天线 PDOP 3.4（要小于 3），卫星位置不用，按 IMU 推算继续行驶', messages)
        self.assertIn('单天线 PDOP 3.4（要小于 3），超过 8 s 已停喷，即将停车', messages)
        self.assertEqual(messages[-1], '单天线 PDOP 3.4（要小于 3），卫星位置不用，已停喷并原地停车，恢复后自动继续')
        self.assertFalse([t for t in messages if '固定解中断' in t])
        self.assertLess(abs(car.v), 0.01)
        car.pdop_ok = True                                    # PDOP 恢复：自动接着走，提示不再说 PDOP
        car.run(5.0)
        self.assertEqual((m.phase, m.tracker.st.state), ('RUNNING', 'RUNNING'))
        self.assertNotIn('PDOP', m.message)
        self.assertNotEqual(m.pose.reasons[:1], ('pdop_high',))
        self.assertGreater(car.v, 0.5)

    def test_missing_pdop_is_named(self):
        m, car = running_line()
        car.pdop_missing = True
        car.run(2.0)
        self.assertEqual(m.pose.reasons[0], 'pdop_missing')
        self.assertEqual(m.message, '单天线没有收到 PDOP，卫星位置不用，按 IMU 推算继续行驶')
        car.pdop_missing = False
        car.run(1.0)
        self.assertEqual(m.message, '行驶中')                # 维持期间没暂停：直接改回行驶中

    def test_orient_refused_names_pdop_first(self):
        m, fusion, _, _ = mission()
        car = Car(m)
        car.run(1.0)
        car.pdop_ok = False
        car.run(1.0)
        ok, msg = m.command({'cmd': 'route_orient'}, car.t)
        self.assertFalse(ok)
        self.assertEqual(msg, 'pose_unusable：pdop_high')     # 不再跟着 rtk_hold、fix_unavailable（PDOP 带出来的）
        self.assertEqual(m.phase, 'IDLE')


class ReceiverTilt(unittest.TestCase):
    """2026-10-07 用户：单天线接收机输出的已是按杆长做完倾斜补偿的坐标（杆长在接收机上设），车端不再补，免得补两次。"""

    def test_single_config_leaves_tilt_to_the_receiver(self):
        dual, single = gen5_configs()
        self.assertTrue(dual.tilt_compensation_enabled)
        self.assertFalse(single.tilt_compensation_enabled)
        self.assertEqual(single.antenna_height_m, dual.antenna_height_m)     # 双天线的天线高照留，换回去照常补
        m, fusion, _, _ = mission()
        car = Car(m)
        car.run(1.0)
        m.on_imu(car.t + 0.01, 0.0, car.imu_yaw, roll=math.radians(3.0), pitch=math.radians(-2.0))
        self.assertEqual(fusion.antenna_lever(), line_node.SINGLE_ANTENNA_XY)   # 坡上也按平地杠杆臂：接收机已补
        tilt = m.status()['tilt']
        self.assertEqual((tilt['active'], tilt['reason'], tilt['enabled']), (False, 'receiver', False))
        self.assertNotIn('tilt_uncompensated', fusion.pose(car.t).reasons)

    def test_tilt_switch_refused_on_single_antenna(self):
        from attitude_controls import SINGLE_TILT_BLOCKED, AttitudeControls
        m, fusion, _, _ = mission()
        car = Car(m)
        car.run(0.5)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            controls = AttitudeControls(m, path)
            st = controls.status(car.t, None, None)
            self.assertEqual(st['blocked'], SINGLE_TILT_BLOCKED)
            self.assertNotEqual(st['zero_blocked'], SINGLE_TILT_BLOCKED)        # IMU 清零不因单天线挡
            ok, msg = controls.command(dict(cmd='tilt', id='t1', enabled=True), car.t, None, None)
            self.assertFalse(ok)
            self.assertIn(SINGLE_TILT_BLOCKED, msg)
            self.assertFalse(fusion.config.tilt_compensation_enabled)
            self.assertFalse(os.path.exists(path))                              # fusion.json 没被改


    def test_tilt_switch_in_dual_survives_a_single_antenna_round_trip(self):
        # 双天线时在 App 关掉坡度补偿 → 换单天线 → 换回双天线：还是关着（原来会恢复成开机时的配置）
        from attitude_controls import AttitudeControls
        m, fusion, _, _ = mission()
        car = Car(m, mode='dual')
        car.run(0.5)
        motion = dict(mode='IDLE', healthy=True, waiting_stationary=False, command=dict(vx=0., wz=0.))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'fusion.json')
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(GEN5, fh)
            controls = AttitudeControls(m, path)
            controls.observe_wheel(car.t, 0.0, 0.0)
            controls.status(car.t, motion, None)
            controls.observe_wheel(car.t + 0.4, 0.0, 0.0)
            ok, msg = controls.command(dict(cmd='tilt', id='off', enabled=False), car.t + 0.4, motion, None)
            self.assertTrue(ok, msg)
        self.assertFalse(fusion.config.tilt_compensation_enabled)
        car.mode = 'single'
        car.run(0.5)
        self.assertEqual(fusion.config.antenna_xy, line_node.SINGLE_ANTENNA_XY)
        car.mode = 'dual'
        car.run(0.5)
        self.assertEqual(fusion.config.antenna_xy, line_node.GEN5_ANTENNA_MEASURED)
        self.assertFalse(fusion.config.tilt_compensation_enabled)
        self.assertEqual(fusion.config.antenna_height_m, 0.53)

    def test_car_without_single_config_keeps_it_none(self):
        dual, _ = gen5_configs()
        fusion = gf.GnssFusion(dual)
        m = LineMission(fusion, gf)
        car = Car(m, mode='dual')
        car.run(0.3)
        car.mode = 'single'
        car.run(0.3)
        car.mode = 'dual'
        car.run(0.3)
        self.assertIsNone(m._antenna_configs['single'])       # 没把双天线配置记成单天线的
        car.mode = 'single'
        car.run(0.3)
        self.assertEqual(fusion.config.antenna_xy, line_node.GEN5_ANTENNA_MEASURED)


def orient(m, car):
    assert m.command({'cmd': 'route_orient'}, car.t)[0]
    for _ in range(200):
        car.run(0.1)
        if m.phase == 'IDLE':
            break
    return m.message


def heading_error_deg(fusion, car):
    return abs(math.degrees(gf._wrap(fusion.pose(car.t).yaw - car.yaw)))


class ImuAlignedHeading(unittest.TestCase):
    """单天线车头 = IMU 航向角 + 偏移，偏移按一段卫星位移和 IMU 航向 + 轮程推算位移的夹角定、修（gnss_fusion._imu_course，
    2026-10-07 用户「用 imu 的航向角做航向，偏移来修正」）。原来按「走直」的轨迹回归定，车身一摆就定不出。"""

    def test_wobbling_car_orients_in_half_a_metre(self):
        for yaw in (-150.0, 20.0, 100.0):
            with self.subTest(yaw=yaw):
                m, fusion, _, single = mission()
                car = Car(m, yaw_deg=yaw)
                car.wobble = 3.0
                car.run(1.0)
                msg = orient(m, car)
                self.assertTrue(msg.startswith('车头已确定'), msg)
                self.assertLess(car.odo, 0.7)                  # 定车头走 0.5 m，加渐停
                self.assertGreaterEqual(car.odo, single.course_init_distance_m)
                self.assertLess(heading_error_deg(fusion, car), 0.7)

    def test_old_straight_rule_could_not_orient_a_wobbling_car(self):
        import dataclasses
        dual, single = gen5_configs()
        old = dataclasses.replace(single, course_align_distance_m=0.0, course_init_distance_m=0.3, course_min_speed_mps=0.15)
        m = LineMission(gf.GnssFusion(dual), gf, single_config=old)
        car = Car(m, yaw_deg=20.0)
        car.wobble = 3.0
        car.run(1.0)
        self.assertIn('还没定出车头', orient(m, car))

    def test_offset_keeps_being_corrected_while_driving(self):
        # IMU 航向角一直漂、车身来回摆：每走 1 m 修一次偏移，车头误差一直被拉住、不越漂越大。
        # 0.02°/s 约实测（10-06 晚 0.4°/min）的 3 倍；0.1°/s 是 15 倍
        for drift, bound in ((0.02, 0.5), (0.1, 1.0)):
            with self.subTest(drift=drift):
                m, fusion, _, _ = mission()
                car = Car(m, yaw_deg=60.0)
                car.wobble = 2.0
                car.run(1.0)
                self.assertTrue(orient(m, car).startswith('车头已确定'))
                car.imu_drift = drift
                car.free_v = 0.5
                errors = []
                for _ in range(80):                           # 40 s、约 20 m
                    car.run(0.5)
                    if car.odo > 2.5:
                        errors.append(heading_error_deg(fusion, car))
                self.assertGreater(car.odo, 19.0)
                self.assertLess(max(errors), bound)
                self.assertLess(max(errors[-20:]), bound)     # 后 10 s 也没漂开
                self.assertTrue(fusion.pose(car.t).usable_for_control)

    def test_heading_uncertainty_does_not_run_away_when_parked(self):
        # 单天线估不了 IMU 漂移速度：起始不确定度按实测量级给，停 60 s 车头不确定度也涨不到不能用
        m, fusion, _, _ = mission()
        car = Car(m, yaw_deg=10.0)
        car.run(1.0)
        self.assertTrue(orient(m, car).startswith('车头已确定'))
        car.run(60.0)
        pose = fusion.pose(car.t)
        self.assertTrue(pose.usable_for_control, pose.reasons)
        self.assertLess(math.degrees(pose.sigma_yaw), 2.0)


class WheelsOnlyStandstill(unittest.TestCase):
    """单天线没有卫星地速：停稳只看轮子（霍尔），2026-10-07 用户「只要轮子不动就行」；双天线照旧要两路证据。"""

    def test_single_needs_only_hall(self):
        from robot_core.standstill import stationary
        dual, single = gen5_configs()
        for cfg, still in ((single, True), (dual, False)):
            with self.subTest(single=cfg is single):
                f = gf.GnssFusion(cfg)
                f.set_origin(31.6, 120.7, 30.0)
                t = 10.0
                for i in range(100):
                    t = round(10.0 + i * 0.02, 3)
                    f.add_imu(gf.ImuSample(t, 0.0, 0.5, 0.0, 0.0))
                    f.add_wheel(gf.WheelSample(t, 0.0, 0.0, source='encoder', speed_basis='hall', hall_still=True))
                    if i % 10 == 0:
                        f.add_fix(gf.GnssFix(t, 31.6, 120.7, 30.0, 'NARROW_INT', .005, .005, .01, 1.0, 30))
                pose = f.pose(t)
                self.assertEqual(pose.gnss_still_required, cfg is dual)
                self.assertEqual(stationary(pose), still)
                moving = gf.GnssFusion(cfg)                   # 霍尔说在动：两种都不算停稳
                self.assertFalse(stationary(dataclasses_replace(pose, hall_still=False)))


def dataclasses_replace(obj, **kw):
    import dataclasses
    return dataclasses.replace(obj, **kw)


if __name__ == '__main__':
    unittest.main()
