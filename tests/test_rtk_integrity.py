"""RTK 完好性监测（11-27 用户「这些先加上」，gnss_integrity.RtkIntegrity，user_rules.RTK_INTEGRITY_*）。

实录夹具（c30c07fd，tools/rtk_integrity_replay.py 截的，只留用到的信号）：
- rtk_jump_ea69f975：22:18:28 那条，喷线 14.7 m（tow 446701.8）处 RTK 两根天线一起往左跳 10 cm、之后一直偏着；
  离线 TDCP 那里是平的、IMU 车头没动——跳的是 RTK，要判出来、而且一直判到这段结束；
- rtk_good_9b6c55c6：22:12:17 那条喷线中 20 s，RTK 好（鼓包 1.8 cm），一次都不能判跳。
"""
import base64
import gzip
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import gnss_fusion as gf  # noqa: E402
import gnss_integrity as gi  # noqa: E402
import user_rules as rules  # noqa: E402

FIX = ROOT / 'tests' / 'fixtures'


def replay(name, mon=None):
    """夹具按顺序喂进去：返回 (监测器, [(tow, 状态, 水平差, 原因)])。"""
    data = json.load(gzip.open(FIX / name, 'rt'))
    mon = mon or gi.RtkIntegrity()
    rows, checks = [], 0
    for kind, v in data['events']:
        if kind in ('eph', 'obs'):
            mon.feed_bytes(base64.b64decode(v))
        elif kind == 'fix':
            tow, lat, lon, h, sol = v
            mon.add_fix(tow, lat, lon, h, sol == 'NARROW_INT')
        elif kind == 'heading':
            mon.add_heading(*v)
        if mon.checks != checks:
            checks = mon.checks
            st = mon.status()
            rows.append((mon.last_tow, st['state'], st['delta_m'], st['reason']))
    return mon, rows


class Recorded(unittest.TestCase):
    def test_detects_the_10cm_common_mode_jump_and_holds_it(self):
        mon, rows = replay('rtk_jump_ea69f975.json.gz')
        jumps = [r for r in rows if r[1] == 'jump']
        self.assertTrue(jumps)
        first = jumps[0]
        self.assertAlmostEqual(first[0], 446701.8, delta=0.5)        # 喷线 15.0 m
        self.assertGreater(first[2], .09)
        # 判上以后到这段结束一直当跳了（星少了查不了也不回头信刚跳过的 RTK）
        self.assertTrue(all(r[1] == 'jump' for r in rows if r[0] >= first[0]), [r for r in rows if r[0] >= first[0]][:5])
        self.assertTrue(mon.suspect())
        # 跳之前是好的
        self.assertTrue(all(r[1] != 'jump' for r in rows if r[0] < first[0] - .3))
        self.assertGreaterEqual(sum(1 for r in rows if r[0] < first[0] and r[1] == "ok"), 5)

    def test_good_line_never_flags(self):
        mon, rows = replay('rtk_good_9b6c55c6.json.gz')
        self.assertFalse(any(r[1] == 'jump' for r in rows))
        ok = sum(1 for r in rows if r[1] == 'ok')
        self.assertGreater(ok, .8 * len(rows))
        self.assertLess(max(r[2] for r in rows if r[2] is not None), rules.RTK_JUMP_M)

    def test_disabled_never_flags(self):
        mon, rows = replay('rtk_jump_ea69f975.json.gz', gi.RtkIntegrity(enabled=False))
        self.assertEqual(rows, [])
        self.assertFalse(mon.suspect())
        self.assertFalse(mon.status()['enabled'])


class Pieces(unittest.TestCase):
    def test_rules_table_and_checks(self):
        mon = gi.RtkIntegrity()
        self.assertEqual((mon.window_s, mon.jump_m, mon.persist, mon.recover_m, mon.recover_s, mon.hold_max_s),
                         (rules.RTK_JUMP_WINDOW_S, rules.RTK_JUMP_M, rules.RTK_JUMP_PERSIST, rules.RTK_RECOVER_M,
                          rules.RTK_RECOVER_S, rules.RTK_JUMP_HOLD_MAX_S))
        self.assertEqual(mon.min_sats, rules.TDCP_MIN_SATS)
        for kw in (dict(enabled=1), dict(jump_m=0.), dict(recover_m=.06), dict(min_sats=4), dict(persist=0)):
            with self.assertRaises(ValueError):
                gi.RtkIntegrity(**kw)

    def test_frames_split_across_reads_and_bad_crc(self):
        data = json.load(gzip.open(FIX / 'rtk_good_9b6c55c6.json.gz', 'rt'))
        frames = [base64.b64decode(v) for k, v in data['events'] if k == 'obs'][:3]
        blob = b'junk' + frames[0] + frames[1][:-1] + b'\x00' + frames[2]      # 第二帧 CRC 坏了
        sp = gi.FrameSplitter()
        got = []
        for i in range(0, len(blob), 37):                                        # 37 字节一读，帧被切开
            got += sp.feed(blob[i:i + 37])
        self.assertEqual([g[0] for g in got], [gi.ID_OBSVM, gi.ID_OBSVM])
        self.assertEqual(got[1][3], frames[2][24:-4])

    def test_turning_is_unknown_not_jump(self):
        # 原地转（测向 30°/s）：判「不确定」，不拦 RTK
        mon, rows = replay('rtk_jump_ea69f975.json.gz', None)
        mon2 = gi.RtkIntegrity()
        data = json.load(gzip.open(FIX / 'rtk_jump_ea69f975.json.gz', 'rt'))
        for kind, v in data['events']:
            if kind in ('eph', 'obs'):
                mon2.feed_bytes(base64.b64decode(v))
            elif kind == 'fix':
                mon2.add_fix(v[0], v[1], v[2], v[3], v[4] == 'NARROW_INT')
            elif kind == 'heading':
                mon2.add_heading(v[0], (v[1] + 30. * (v[0] - 446695.)) % 360., True)
        self.assertEqual(mon2.jumps, 0)
        self.assertEqual(mon2.status()['state'], 'unknown')
        self.assertEqual(mon2.status()['reason'], 'turning')


class Mission(unittest.TestCase):
    def mission(self):
        import tempfile
        from line_mission import IdleTracker, LineMission
        from path_tracker import PathConfig, differential_config
        from robot_core.kinematics import ChassisGeometry
        added = []
        fusion = NS(to_local=lambda lat, lon: ((lon - 120.) / 1e-5, (lat - 30.) / 1e-5),
                    to_geodetic=lambda x, y: (30. + y * 1e-5, 120. + x * 1e-5),
                    add_fix=added.append, add_fix2=lambda s: None)
        self.tmp = tempfile.TemporaryDirectory()
        m = LineMission(fusion, gf, IdleTracker(ChassisGeometry(drive='differential', track_m=.675, diff_min_radius_m=.5)),
                        route_path=str(Path(self.tmp.name) / 'route.json'),
                        path_config=differential_config(PathConfig(cruise_mps=.8)), nozzle_x=(.24, -.4575))
        return m, added

    def fix(self, state, kind='fix'):
        return dict(kind=kind, t=10., lat=30.00001, lon=120.00001, height=12., sol='NARROW_INT', sigma_n=.012,
                    sigma_e=.011, sigma_u=.03, diff_age=1., sats=24, antenna_mode='dual',
                    integrity=dict(state=state, delta_m=.11))

    def tearDown(self):
        if hasattr(self, 'tmp'):
            self.tmp.cleanup()

    def test_jump_is_downweighted_not_dropped(self):
        m, added = self.mission()
        m.on_gnss(self.fix('jump'))
        s = added[-1]
        self.assertEqual(s.sol, 'NARROW_INT')                                    # 不丢帧、照常算固定解
        self.assertTrue(s.suspect)
        self.assertEqual(s.position_variance_scale, rules.RTK_JUMP_VARIANCE_SCALE)
        self.assertFalse(m._rtk_quality[4])                                      # 直线纠偏当不是固定解
        self.assertTrue(m._rtk_suspect)
        self.assertEqual(m.status()['rtk_integrity']['state'], 'jump')
        m.on_gnss(dict(self.fix('jump', 'fix2'), t=10.05))
        self.assertFalse(m._fix2[3])

    def test_ok_is_unchanged(self):
        m, added = self.mission()
        m.on_gnss(self.fix('ok'))
        s = added[-1]
        self.assertFalse(s.suspect)
        self.assertEqual(s.position_variance_scale, 1.)
        self.assertTrue(m._rtk_quality[4])
        m.on_gnss(dict(self.fix('ok'), t=10.1, integrity=None))                   # 老 rtk_node 没有 integrity
        self.assertFalse(added[-1].suspect)


class Fusion(unittest.TestCase):
    def test_suspect_fix_is_not_trusted(self):
        from test_secondary_position import ANT, BASE, config
        f = gf.GnssFusion(config())
        f.set_origin(31., 120.)
        for i, suspect in enumerate((False, False, False, True)):
            t = .1 * (i + 1)
            f.add_wheel(gf.WheelSample(t, 0., 0.))
            f.add_imu(gf.ImuSample(t, 0., 0.))
            f.add_heading(gf.DualHeading(t, 180., 0., BASE, 'NARROW_INT', .3, 30))
            lat2, lon2 = f.to_geodetic(ANT[0], ANT[1] - BASE)
            f.add_fix2(gf.GnssFix(t, lat2, lon2, 0., 'NARROW_INT', .01, .01, .02, 1., 30))
            lat, lon = f.to_geodetic(ANT[0], ANT[1])
            f.add_fix(gf.GnssFix(t, lat, lon, 0., 'NARROW_INT', .01, .01, .02, 1., 30, suspect=suspect))
            f.pose(t)
            if not suspect:
                self.assertTrue(f._s.trusted)
        self.assertFalse(f._s.trusted)


if __name__ == '__main__':
    unittest.main()


class Node(unittest.TestCase):
    def node(self):
        import importlib
        import os
        import threading
        sys.path[:0] = [os.path.join(str(ROOT), 'tests')]
        import ros_stubs
        ros_stubs.install()
        for name in ('Imu', 'NavSatFix', 'NavSatStatus'):
            setattr(sys.modules['sensor_msgs.msg'], name, ros_stubs._simple(name))
        rtk_node = importlib.import_module('rtk_node')
        node = rtk_node.RtkNode.__new__(rtk_node.RtkNode)
        node.stream_lock = threading.Lock()
        return node

    def test_fix_messages_carry_the_judgement(self):
        node = self.node()
        msgs = [dict(kind='fix', gps_tow_s=1., lat=31., lon=120., height=10., sol='NARROW_INT'),
                dict(kind='fix2', gps_tow_s=1.), dict(kind='heading', gps_tow_s=1., heading=90., valid=True,
                                                      sol='NARROW_INT', length=.41, sats=30), dict(kind='vel')]
        node._check_integrity(b'', msgs)
        self.assertEqual(msgs[0]['integrity']['state'], 'unknown')
        self.assertIn('integrity', msgs[1])
        self.assertNotIn('integrity', msgs[2])
        self.assertIsNone(node.integrity_error if hasattr(node, 'integrity_error') else None)

    def test_monitor_error_does_not_break_positioning(self):
        node = self.node()

        class Broken:
            def feed_bytes(self, data):
                raise RuntimeError('boom')
        node.integrity = Broken()
        msgs = [dict(kind='fix', gps_tow_s=1., lat=31., lon=120., height=10., sol='NARROW_INT')]
        node._check_integrity(b'x', msgs)                  # 不抛
        self.assertIn('boom', node.integrity_error)
        self.assertIsInstance(node.integrity, gi.RtkIntegrity)   # 换一个新的接着用


class JobSummary(unittest.TestCase):
    def test_summary_counts_states_and_jumps(self):
        import job_recorder as jr
        s = jr.Summary()
        for i, (state, jumps) in enumerate((('ok', 0), ('ok', 0), ('jump', 1), ('jump', 1))):
            s.feed('/gnss/status', dict(rtk='FIXED', heading_ok=True, source='tablet_usb', sats=20, diff_age=1.,
                                        integrity=dict(enabled=True, state=state, jumps=jumps)), i * .5)
        rtk = s.to_dict()['rtk']
        self.assertEqual(rtk['integrity'], dict(share={'ok': .5, 'jump': .5}, rtk_jumps=1))
        s2 = jr.Summary()
        s2.feed('/gnss/status', dict(rtk='FIXED', heading_ok=True, source='can', sats=20, diff_age=1.), 0.)
        self.assertIsNone(s2.to_dict()['rtk']['integrity'])            # 老 rtk_node：没有这项
