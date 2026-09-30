#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UM982 解析、GNSS 时间换算和 NTRIP 客户端测试（样例坐标是虚构的）。"""
import math
import os
import socket
import sys
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'radar_system'))

import um982  # noqa: E402


def unicore(name, body, week=2437, tow=443243.7):
    text = '%s,COM2,11826,86.0,FINE,%d,%.3f,253448,20,18;%s' % (name, week, tow, body)
    return '#%s*%08x' % (text, um982.crc32(text.encode()))


def nmea(body):
    return '$%s*%02X' % (body, um982.nmea_checksum(body))


def recrc(line, old, new):
    """替换 Unicore 日志里的一段文字并重算校验。"""
    text = line[1:line.rindex('*')].replace(old, new)
    return '#%s*%08x' % (text, um982.crc32(text.encode()))


BESTPOS_BODY = ('SOL_COMPUTED,NARROW_INT,30.00000012345,120.00000067890,12.3456,8.5000,'
                'WGS84,0.0081,0.0092,0.0150,"2001",1.000,0.000,40,32,32,0,1,12,11,51')
BESTPOS = unicore('BESTPOSA', BESTPOS_BODY)


def secondary_at(north_m, up_m=0.0, tow=443243.7):
    """副天线：相对 BESTPOS 主天线北 north_m、高 up_m 的 NARROW_INT 定位（BESTNAVHA 头是毫秒）。"""
    body = ('BESTNAVHA,97,GPS,FINE,2437,%d,0,0,18,13;SOL_COMPUTED,NARROW_INT,'
            '%.11f,120.00000067890,%.4f,8.5000,WGS84,0.01,0.01,0.02,"0",1,0,38,31,0,0,0,0,0,0,'
            'SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0,0,0,0') % (
                round(tow * 1000), 30.00000012345 + north_m / 110574.0, 12.3456 + up_m)
    return '#%s*%08x' % (body, um982.crc32(body.encode()))


def heading_line(length, sol='NARROW_INT', tow=443243.7):
    return unicore('HEADINGA', 'SOL_COMPUTED,%s,%.4f,91.2500,-0.8000,0.0000,0.2800,0.5100,'
                               '"999",37,32,32,0,3,01,1,41' % (sol, length), tow=tow)


def epoch_lines(n=6, north_m=0.3312, length=0.3312, up_m=0.0, sol='NARROW_INT'):
    """n 个 20 Hz 历元，每个历元一段：主天线、副天线、测向（同一 GPS 时刻），测向有实时基线可核对。"""
    return ['\r\n'.join([unicore('BESTPOSA', BESTPOS_BODY, tow=443243.7 + 0.05 * i),
                          secondary_at(north_m, up_m, 443243.7 + 0.05 * i),
                          heading_line(length, sol, 443243.7 + 0.05 * i)]) for i in range(n)]
HEADING = unicore('HEADINGA', 'SOL_COMPUTED,L1_INT,0.3312,91.2500,-0.8000,0.0000,0.2800,0.5100,'
                              '"999",37,32,32,0,3,01,1,41')
BESTVEL = unicore('BESTVELA', 'SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.5012,1.250000,-0.0022,0')
GGA = nmea('GNGGA,024019.00,3000.00000741,N,12000.00004073,E,4,32,0.6,12.3456,M,8.5000,M,1.0,2001')


def secondary_log(used=19, tracked=27, status='SOL_COMPUTED'):
    # N4 原生 BESTNAVHA 头使用毫秒，副天线计数故意与 BESTPOS、HEADING 不同。
    body = (f'BESTNAVHA,97,GPS,FINE,2437,443243700,0,0,18,13;{status},SINGLE,'
            f'30.0,120.0,12.0,8.5,WGS84,1.1,1.2,2.2,"0",0,0,{tracked},{used},0,0,0,0,0,0,'
            'SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0,0,0,0')
    return '#%s*%08x' % (body, um982.crc32(body.encode()))


class TestParse(unittest.TestCase):

    def test_crc_known_answer(self):
        # 算法已用接收机的真实日志核对过；这里固定一个值防止回归
        self.assertEqual(um982.crc32(b'123456789'), 0x2DFD2D88)

    def test_bestpos(self):
        m = um982.parse_unicore(BESTPOS)
        self.assertIsInstance(m, um982.BestPos)
        self.assertEqual((m.week, m.pos_type), (2437, 'NARROW_INT'))
        self.assertAlmostEqual(m.tow, 443243.7)
        self.assertAlmostEqual(m.lat_deg, 30.00000012345)
        self.assertAlmostEqual(m.height_m, 12.3456 + 8.5)       # 椭球高
        self.assertEqual((m.sigma_lat, m.sigma_lon, m.sigma_hgt), (0.0081, 0.0092, 0.0150))
        self.assertEqual((m.diff_age_s, m.sats_tracked, m.sats_used), (1.0, 40, 32))
        self.assertIn(m.pos_type, um982.RTK_FIXED)

    def test_no_differential_age_is_infinite(self):
        line = unicore('BESTPOSA', 'SOL_COMPUTED,SINGLE,30.0,120.0,12.0,8.5,WGS84,1.1,1.2,2.2,"0",'
                                   '0.000,0.000,39,28,28,0,1,12,11,51')
        self.assertEqual(um982.parse_unicore(line).diff_age_s, math.inf)

    def test_heading(self):
        m = um982.parse_unicore(HEADING)
        self.assertIsInstance(m, um982.Heading)
        self.assertEqual(m.pos_type, 'L1_INT')
        self.assertEqual((m.length_m, m.heading_deg, m.pitch_deg), (0.3312, 91.25, -0.8))
        self.assertEqual((m.sigma_heading_deg, m.sats_tracked, m.sats_used), (0.28, 37, 32))
        self.assertIn(m.pos_type, um982.HEADING_FIXED)

    def test_bestvel(self):
        m = um982.parse_unicore(BESTVEL)
        self.assertIsInstance(m, um982.BestVel)
        self.assertEqual((m.speed_mps, m.track_deg, m.vert_mps), (0.5012, 1.25, -0.0022))

    def test_corrupted_and_foreign_lines(self):
        broken = BESTPOS.replace('NARROW_INT', 'NARROW_FLT')
        with self.assertRaises(um982.ParseError):
            um982.parse_unicore(broken)
        self.assertIsNone(um982.parse_unicore(unicore('BESTPOS2A', 'SOL_COMPUTED,SINGLE')))
        self.assertIsNone(um982.parse_unicore(GGA))
        with self.assertRaises(um982.ParseError):
            um982.parse_unicore(unicore('HEADINGA', 'SOL_COMPUTED,L1_INT,0.33'))   # 字段不全

    def test_gga_and_compact_report(self):
        fields = um982.parse_gga(GGA)
        self.assertEqual(fields[6], '4')
        self.assertIsNone(um982.parse_gga(GGA[:-2] + '00'))
        line = um982.compact_gga(fields)
        self.assertLessEqual(len(line.strip()), 82)
        self.assertIsNotNone(um982.parse_gga(line))
        self.assertIn(',3000.00001,N,12000.00004,E,4,', line)
        no_fix = nmea('GNGGA,024019.00,,,,,0,00,99.9,,M,,M,,')
        self.assertIsNone(um982.compact_gga(um982.parse_gga(no_fix)))


class TestClock(unittest.TestCase):

    def test_minimum_delay_tracks_true_time(self):
        import random
        rng = random.Random(3)
        clock = um982.GnssClock(solution_latency_s=0.02)
        offset = 1000.0 - 2437 * um982.GPS_WEEK_S
        errs = []
        for k in range(400):
            tow = 100.0 + k * 0.05
            true_mono = 2437 * um982.GPS_WEEK_S + tow + offset
            t_rx = true_mono + 0.02 + 0.005 + rng.expovariate(1 / 0.02)   # 解算 20ms + 串口 5ms + 抖动
            t = clock.measurement_time(2437, tow, t_rx)
            if k > 40:
                errs.append(t - true_mono)
        self.assertLess(max(abs(e) for e in errs), 0.008)

    def test_receiver_restart_resets_estimate(self):
        clock = um982.GnssClock()
        for k in range(50):
            clock.measurement_time(2437, 100 + k * 0.05, 5000 + k * 0.05)
        t = clock.measurement_time(2437, 900.0, 5003.0)     # GPS 时间突然跳了 800 s
        self.assertAlmostEqual(t, 5003.0 - clock.latency, places=6)


class TestNtripEnv(unittest.TestCase):

    def test_env_file_rules(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'ntrip.env')
            self.assertIsNone(um982.load_ntrip_env(path))
            with open(path, 'w') as fh:
                fh.write('NTRIP_HOST=h\nNTRIP_PORT=1\nNTRIP_MOUNT=M\nNTRIP_USER=u\nNTRIP_PASS=p\n')
            os.chmod(path, 0o644)
            with self.assertRaises(PermissionError):
                um982.load_ntrip_env(path)
            os.chmod(path, 0o600)
            self.assertEqual(um982.load_ntrip_env(path)['NTRIP_MOUNT'], 'M')
            with open(path, 'w') as fh:
                fh.write('NTRIP_HOST=h\n')
            with self.assertRaises(ValueError):
                um982.load_ntrip_env(path)

    def test_rtcm_frame_split(self):
        frame = b'\xd3\x00\x03' + bytes([0x43, 0xE0, 0x00]) + b'\x00\x00\x00'   # 类型 1086
        types, rest = um982.rtcm_frames(b'junk' + frame + frame[:4])
        self.assertEqual(types, [1086])
        types, rest = um982.rtcm_frames(rest + frame[4:])
        self.assertEqual((types, rest), ([1086], b''))


class TestStream(unittest.TestCase):

    def test_independent_secondary_antenna_and_expiry(self):
        s = um982.Um982Stream()
        m = um982.parse_unicore(secondary_log())
        self.assertEqual((m.antenna, m.week, m.tow, m.sats_used, m.sats_tracked), (2, 2437, 443243.7, 19, 27))
        out = s.feed((BESTPOS + '\r\n' + HEADING + '\r\n' + secondary_log() + '\r\n').encode(), 10.0)
        self.assertEqual([m['kind'] for m in out], ['fix', 'heading', 'fix2'])
        a = s.status(10.2)['antennas']
        self.assertEqual((a['ant1']['sats_used'], a['ant2']['sats_used']), (32, 19))
        self.assertEqual((a['ant2']['source'], a['ant2']['fix']), ('BESTNAVH', 'SINGLE'))
        s.feed((BESTPOS + '\r\n' + HEADING + '\r\n').encode(), 13.0)
        a = s.status(13.1)['antennas']
        self.assertTrue(a['ant1']['available'])
        self.assertFalse(a['ant2']['available'])
        self.assertIsNone(a['ant2']['sats_used'])
        s.feed((secondary_log(0, 12, 'INSUFFICIENT_OBS') + '\r\n').encode(), 14.0)
        self.assertEqual(s.status(14.1)['antennas']['ant2']['fix'], 'NONE')

    secondary = staticmethod(lambda north_m, up_m=0.0, tow=443243.7: secondary_at(north_m, up_m, tow))
    heading = staticmethod(lambda length, sol='NARROW_INT', tow=443243.7: heading_line(length, sol, tow))

    def feed_epochs(self, s, north_m, length, n=6, up_m=0.0, sol='NARROW_INT', t0=10.0):
        out = []
        for i, text in enumerate(epoch_lines(n, north_m, length, up_m, sol)):
            out += s.feed((text + '\r\n').encode(), t0 + 0.05 * i)
        return out

    def test_fixed_heading_needs_baseline_between_30_and_50_cm(self):
        # 用户规则：测向整周固定、基线 0.30–0.50 m 就算有效；两端都算，范围外不算
        for length, ok in ((0.415, True), (0.30, True), (0.50, True), (0.28, False), (0.52, False), (2.6, False)):
            s = um982.Um982Stream()
            out = self.feed_epochs(s, 0.415, length)
            hd = [m for m in out if m['kind'] == 'heading']
            self.assertEqual([m['valid'] for m in hd], [ok] * len(hd), length)   # 第一个历元就下结论
            self.assertEqual(s.status(10.3)['heading_ok'], ok, length)
        # 实时两天线距离照实报，但不参与判定
        self.assertAlmostEqual(s.status(10.3)['heading']['ref_m'], 0.415, places=2)
        self.assertIsNone(s.status(10.3)['antennas']['ant2']['consistent'])

    def test_fixed_heading_is_trusted_without_fixed_antenna_positions_float_is_not(self):
        # 平板转发的接收机两根天线常是 L1_INT（车端归浮点）：测向整周固定、基线在范围内照样有效
        s = um982.Um982Stream()
        for i in range(3):
            s.feed((unicore('BESTPOSA', BESTPOS_BODY.replace('NARROW_INT', 'L1_INT'), tow=443243.7 + 0.05 * i)
                    + '\r\n' + self.heading(0.415, 'L1_INT', 443243.7 + 0.05 * i) + '\r\n').encode(), 10.0 + 0.05 * i)
        st = s.status(10.2)
        self.assertEqual((st['rtk'], st['heading_ok'], st['heading']['ref_m']), ('FLOAT', True, None))
        s = um982.Um982Stream()
        out = self.feed_epochs(s, 0.415, 0.415, sol='NARROW_FLOAT')
        self.assertFalse([m for m in out if m['kind'] == 'heading'][-1]['valid'])

    def test_antenna_spacing_uses_plane_coordinates_only(self):
        # 天线间距不写死、不实测：取两根天线固定解的东、北平面距离；副天线高 0.20 m 不算进去（三维会是 0.36 m）
        s = um982.Um982Stream()
        out = self.feed_epochs(s, 0.30, 0.36, up_m=0.20)
        hd = [m for m in out if m['kind'] == 'heading']
        self.assertTrue(hd[-1]['valid'])
        self.assertAlmostEqual(hd[-1]['baseline_ref'], 0.30, delta=0.005)
        st = s.status(10.3)
        self.assertAlmostEqual(st['heading']['ref_m'], 0.30, delta=0.005)
        self.assertAlmostEqual(st['antennas']['ant2']['offset_m'], 0.30, delta=0.005)

    def test_repeater_like_zero_baseline_is_not_valid(self):
        # 室内转发器：两根天线收到同一路信号，距离和测向都只有几毫米
        s = um982.Um982Stream()
        out = self.feed_epochs(s, 0.001, 0.001)
        self.assertFalse([m for m in out if m['kind'] == 'heading'][-1]['valid'])

    def test_secondary_offset_needs_both_fixed(self):
        s = um982.Um982Stream()
        self.feed_epochs(s, 0.33, 0.33)
        self.assertAlmostEqual(s.status(10.3)['antennas']['ant2']['offset_m'], 0.33, places=2)
        # 主天线只有单点解时没法判断（外场差分重连时实测单点对固定"相距 7.3 m"）
        single = unicore('BESTPOSA', 'SOL_COMPUTED,SINGLE,30.00000012345,120.00000067890,12.3456,8.5000,'
                                     'WGS84,1.2,1.3,2.5,"0",0.000,0.000,40,17,17,0,1,12,11,51')
        s.feed((single + '\r\n' + self.secondary(0.33) + '\r\n').encode(), 12.0)
        self.assertEqual(s.status(12.1)['antennas']['ant1']['fix'], 'SINGLE')   # 确实喂进去了单点解
        a2 = s.status(12.1)['antennas']['ant2']
        self.assertEqual((a2['offset_m'], a2['consistent']), (None, None))

    def test_heading_block_is_reported_separately(self):
        s = um982.Um982Stream()
        s.feed((BESTPOS + '\r\n' + HEADING + '\r\n').encode(), 10.0)
        h = s.status(10.1)['heading']
        self.assertEqual((h['sol'], h['fix'], h['sats_used'], h['sats_tracked']), ('L1_INT', 'FIXED', 32, 37))
        # 没有副天线定位就没有实时两天线距离；测向整周固定、基线 0.331 m 在范围内照样有效
        self.assertEqual((h['length_m'], h['ref_m'], h['valid']), (0.331, None, True))
        self.assertTrue(s.status(10.1)['heading_ok'])
        self.assertIsNone(s.status(12.0)['heading'])
        float_hd = HEADING.replace('L1_INT', 'NARROW_FLOAT')
        float_hd = '#%s*%08x' % (float_hd[1:].rsplit('*', 1)[0], um982.crc32(float_hd[1:].rsplit('*', 1)[0].encode()))
        s.feed((float_hd + '\r\n').encode(), 13.0)
        self.assertEqual(s.status(13.1)['heading']['fix'], 'FLOAT')

    def test_missing_secondary_is_never_filled_from_heading(self):
        s = um982.Um982Stream()
        s.feed((BESTPOS + '\r\n' + HEADING + '\r\n').encode(), 10.0)
        self.assertIsNone(s.status(10.1)['antennas']['ant2']['sats_used'])

    def test_split_lines_crc_errors_and_status(self):
        s = um982.Um982Stream()
        data = ('\r\n'.join([GGA, BESTPOS, HEADING, BESTVEL, BESTPOS.replace('NARROW_INT', 'NARROW_FLT')])
                + '\r\n').encode()
        out = s.feed(data[:37], 10.0) + s.feed(data[37:], 10.01)
        self.assertEqual([m['kind'] for m in out], ['fix', 'heading', 'vel'])
        self.assertEqual(s.crc_errors, 1)
        fix = out[0]
        self.assertEqual((fix['sol'], fix['sats'], fix['diff_age']), ('NARROW_INT', 32, 1.0))
        self.assertLess(fix['t'], fix['t_rx'])
        st = s.status(10.05)
        # 只有主天线和测向、没有副天线定位：测向整周固定、基线在范围内就有效
        self.assertEqual((st['rtk'], st['heading_ok'], st['receiver_ok']), ('FIXED', True, True))
        self.assertAlmostEqual(st['baseline_m'], 0.3312)
        self.assertIsNotNone(s.gga_report())
        self.assertFalse(s.status(12.0)['receiver_ok'])

    def test_rates_and_single_point(self):
        s = um982.Um982Stream()
        single = unicore('BESTPOSA', 'SOL_COMPUTED,SINGLE,30.0,120.0,12.0,8.5,WGS84,1.1,1.2,2.2,"0",'
                                     '0.000,0.000,39,28,28,0,1,12,11,51', tow=0)
        for k in range(40):
            line = single.replace('FINE,2437,0.000', 'FINE,2437,%.3f' % (k * 0.05))
            text = line[1:line.rindex('*')]
            s.feed(('#%s*%08x\r\n' % (text, um982.crc32(text.encode()))).encode(), 100 + k * 0.05)
        st = s.status(102.0)
        self.assertEqual(st['rtk'], 'SINGLE')
        self.assertEqual(st['diff_age'], None)
        self.assertAlmostEqual(st['rates']['fix'], 20.0, delta=0.6)
        self.assertEqual(s.crc_errors, 0)

    def test_unsolved_status_is_reported_as_no_solution(self):
        # 解算状态不是 SOL_COMPUTED 时解类型字段不可信：下游只看 sol，一律给 NONE
        s = um982.Um982Stream()
        lines = [recrc(BESTPOS, 'SOL_COMPUTED', 'INSUFFICIENT_OBS'), recrc(HEADING, 'SOL_COMPUTED', 'COLD_START'),
                 recrc(BESTVEL, 'SOL_COMPUTED', 'INSUFFICIENT_OBS')]
        out = s.feed(('\r\n'.join(lines) + '\r\n').encode(), 10.0)
        self.assertEqual([(m['kind'], m['sol'], m['status']) for m in out],
                         [('fix', 'NONE', 'INSUFFICIENT_OBS'), ('heading', 'NONE', 'COLD_START'),
                          ('vel', 'NONE', 'INSUFFICIENT_OBS')])
        st = s.status(10.05)
        self.assertEqual((st['rtk'], st['heading_ok']), ('NONE', False))

    def test_fixed_level_and_heading_follow_fusion_rules(self):
        # 宽巷/单频固定解融合不用于控制，界面也不能显示"固定"；室内转发器的毫米级基线，航向无效
        s = um982.Um982Stream()
        wide = recrc(BESTPOS, 'NARROW_INT', 'WIDE_INT')
        short = recrc(HEADING, '0.3312', '0.0040')
        s.feed(('%s\r\n%s\r\n' % (wide, short)).encode(), 10.0)
        st = s.status(10.05)
        self.assertEqual((st['rtk'], st['sol'], st['heading_sol']), ('FLOAT', 'WIDE_INT', 'L1_INT'))
        self.assertFalse(st['heading_ok'])
        self.assertAlmostEqual(st['baseline_m'], 0.004)


class FakeCaster:
    """本机假 CORS：只接受 AUTO 接入点和固定账号，收到 GGA 后推一帧 RTCM。"""

    def __init__(self):
        self.srv = socket.socket()
        self.srv.bind(('127.0.0.1', 0))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.requests, self.ggas = [], []
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn):
        req = b''
        while b'\r\n\r\n' not in req:
            req += conn.recv(1024)
        self.requests.append(req.decode())
        if not req.startswith(b'GET /AUTO ') or b'Authorization: Basic dTpw' not in req:
            conn.sendall(b'HTTP/1.1 401 Unauthorized\r\n\r\n')
            conn.close()
            return
        conn.sendall(b'ICY 200 OK\r\n\r\n')
        conn.settimeout(3)
        try:
            gga = conn.recv(1024)
            self.ggas.append(gga.decode())
            conn.sendall(b'\xd3\x00\x03' + bytes([0x43, 0xE0, 0x00]) + b'\x00\x00\x00')
            time.sleep(0.5)
        except OSError:
            pass
        conn.close()


class TestNtripClient(unittest.TestCase):

    def test_falls_back_to_second_mount_and_relays_corrections(self):
        caster = FakeCaster()
        got = []
        env = dict(NTRIP_HOST='127.0.0.1', NTRIP_PORT=str(caster.port), NTRIP_MOUNT='RTCM33_GGB',
                   NTRIP_MOUNT_FALLBACK='AUTO', NTRIP_USER='u', NTRIP_PASS='p')
        gga = um982.compact_gga(um982.parse_gga(GGA))
        client = um982.NtripClient(env, lambda: gga, got.append).start()
        deadline = time.time() + 5
        while not got and time.time() < deadline:
            time.sleep(0.05)
        client.stop()
        status = client.status()
        self.assertTrue(got, status)
        self.assertEqual(status['rejected'], ['RTCM33_GGB'])
        self.assertEqual(status['mount'], 'AUTO')
        self.assertIn(1086, status['rtcm_types'])
        self.assertEqual(caster.ggas[0], gga)
        self.assertNotIn('p', status.get('error') or '')


if __name__ == '__main__':
    unittest.main()
