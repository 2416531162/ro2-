"""场地作业虚线（app「划线线型」）：车端规划算出每一截在哪开泵，执行时只在这几截内喷。只使用软件底盘。"""
import bisect
import json
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT)]
import route_plan as rp  # noqa: E402
import field_planner as fp  # noqa: E402
import field_rework  # noqa: E402
from path_tracker import Path as Route, Section  # noqa: E402
from route_operations import field_paint_spans, tail  # noqa: E402
import test_continuous_field_arcs as kin  # noqa: E402

CFG = fp.Settings(.24, .4575, (.911, .604, .39), .557)
_, PLANES = fp.polygon([[-10, -10], [30, -10], [30, 30], [-10, 30]])


def nozzle(sec, s):
    return rp.nozzle_at(*sec.car_pose(s), CFG.forward, -CFG.right)


def along(stroke, q):
    """喷头点 q 在原图形上从起点量的米数和离线距离。"""
    if stroke['kind'] == 'line':
        (ax, ay), (bx, by) = stroke['a'], stroke['b']
        L = math.dist(stroke['a'], stroke['b']);ux, uy = (bx - ax) / L, (by - ay) / L
        return (q[0] - ax) * ux + (q[1] - ay) * uy, abs(-(q[0] - ax) * uy + (q[1] - ay) * ux)
    cx, cy = stroke['c'];so = 1 if stroke['sweep'] > 0 else -1
    a = math.atan2(q[1] - cy, q[0] - cx)
    return stroke['r'] * (((a - stroke['start']) * so) % fp.TAU), abs(math.hypot(q[0] - cx, q[1] - cy) - stroke['r'])


def inside(ranges, u, tol=1e-6):
    return any(a - tol <= u <= b + tol for a, b in ranges)


def dashed_job(order='optimized'):
    return dict(execution_order=order, boundary=[[-10, -10], [30, -10], [30, 30], [-10, 30]], elements=[
        dict(id='side', kind='polyline', points=[[0, 0], [10, 0], [10, 6]], dash=dict(mode='length', dash=1.0, gap=.6, count=10)),
        dict(id='solid', kind='polyline', points=[[0, 10], [8, 10]]),
        dict(id='ring', kind='circle', center=[20, 15], radius=3, start_deg=30, dash=dict(mode='count', dash=1, gap=1, count=6)),
        dict(id='bow', kind='arc', center=[5, 20], radius=4, start_deg=180, sweep_deg=-120, dash=dict(mode='length', dash=.8, gap=.8, count=10))])


class DashRanges(unittest.TestCase):
    def test_same_ranges_as_the_app(self):
        self.assertEqual(fp.dash_ranges(dict(mode='length', dash=1.0, gap=1.0), 5.5),
                         [(0, 1), (2, 3), (4, 5)])
        r = fp.dash_ranges(dict(mode='length', dash=1.0, gap=.5), 3.6)     # 最后一截被线尾截短
        self.assertEqual(r, [(0, 1), (1.5, 2.5), (3, 3.6)])
        self.assertEqual(len(fp.dash_ranges(dict(mode='length', dash=1.0, gap=.5), 4.2)), 3)   # 4.0–4.2 是空隙
        r = fp.dash_ranges(dict(mode='count', count=3), 10.0)               # 线、空等长，两端有线
        self.assertEqual([tuple(round(v, 6) for v in p) for p in r], [(0, 2), (4, 6), (8, 10)])
        self.assertIsNone(fp.dash_ranges(None, 3.0))

    def test_bad_styles_are_rejected_with_the_element_name(self):
        for style in ('x', dict(mode='dots'), dict(mode='count', count=1), dict(mode='count', count=2.0),
                      dict(mode='length', dash=.1, gap=1), dict(mode='length', dash=1, gap=True),
                      dict(mode='length', dash=.2, gap=.2, count=2)):
            with self.subTest(style=style):
                length = 500.0 if style == dict(mode='length', dash=.2, gap=.2, count=2) else 5.0
                with self.assertRaises(ValueError) as cm:
                    fp.dash_ranges(style, length, '边线')
                self.assertIn('边线', str(cm.exception))
        with self.assertRaises(ValueError):
            fp.parse_strokes([dict(id='p', kind='polyline', points=[[0, 0], [5, 0]], dash=dict(mode='count', count=0))])

    def test_travel_spans_reverse_and_full_circle(self):
        r = [(0, 1), (2, 3)]
        self.assertEqual(fp.travel_spans(r, 3.5), [(0, 1), (2, 3)])
        self.assertEqual(fp.travel_spans(r, 3.5, True), [(.5, 1.5), (2.5, 3.5)])     # 倒着画：第一截不在线头
        # 整圆 10 m（段数模式：首尾两截在原起点相接，画过原起点时是一截 8→10→2）、从原起点后 2.5 m 起画
        ring = [(0, 2), (4, 6), (8, 10)]
        self.assertEqual([tuple(round(v, 9) for v in p) for p in fp.travel_spans(ring, 10, False, 2.5)],
                         [(1.5, 3.5), (5.5, 9.5)])
        self.assertEqual([tuple(round(v, 9) for v in p) for p in fp.travel_spans(ring, 10, True, 2.5)],
                         [(.5, 4.5), (6.5, 8.5)])
        gapped = [(0, 1), (4, 6), (8, 9)]          # 线长模式：原起点两边都是空隙，跨过原起点那截照样分开
        self.assertEqual([tuple(round(v, 9) for v in p) for p in fp.travel_spans(gapped, 10, False, 8.5)],
                         [(0, .5), (1.5, 2.5), (5.5, 7.5), (9.5, 10)])     # 转回起画点：8–8.5 那半截最后补上
        # 首尾两截在原起点相接（段数模式整圆）：画过原起点时合成一截，不在原起点停喷又开喷
        self.assertEqual([tuple(round(v, 9) for v in p) for p in fp.travel_spans(ring, 10, False, 9.0)],
                         [(0, 3), (5, 7), (9, 10)])
        self.assertEqual(fp.travel_spans(ring, 10, False, 0.0), [(0, 2), (4, 6), (8, 10)])


class DashPlanning(unittest.TestCase):
    def test_every_travel_option_sprays_exactly_the_drawn_dashes(self):
        strokes, _ = fp.parse_strokes(dashed_job()['elements'])
        for stroke in strokes:
            ranges = stroke.get('paint_ranges')
            options = fp.stroke_options(stroke, CFG, PLANES)
            self.assertTrue(options)
            for option in options:
                with self.subTest(stroke=stroke['id'], reverse=option['reverse'], start=round(option['section'].car_pose(0)[2], 3)):
                    offsets = option['paint_offsets']
                    if ranges is None:
                        self.assertIsNone(offsets);continue
                    sec, base = option['section'], option['paint_from']
                    scale = (option['paint_to'] - option['paint_from']) / option['nozzle_length']
                    self.assertAlmostEqual(sum(b - a for a, b in offsets) / scale, sum(b - a for a, b in ranges), places=6)
                    for (a, b), nxt in zip(offsets, offsets[1:] + [None]):
                        for s in (a + 1e-4, (a + b) / 2, b - 1e-4):
                            u, off = along(stroke, nozzle(sec, base + s))
                            self.assertLess(off, 1e-3)
                            self.assertTrue(inside(ranges, u, 1e-3), (u, ranges))
                        if nxt is not None:      # 两截之间的空隙落在图上的空隙里
                            u, _ = along(stroke, nozzle(sec, base + (b + nxt[0]) / 2))
                            self.assertFalse(inside(ranges, u, -1e-3), (u, ranges))

    def test_plan_carries_spans_solid_lines_unchanged_and_survives_json(self):
        for order in ('optimized', 'drawing'):
            with self.subTest(order=order):
                out = fp.plan(dashed_job(order), (-2., -2., 0.), CFG)
                out = json.loads(json.dumps(out))
                paint = [m for m in out['motions'] if m['role'] == 'paint']
                self.assertEqual(len(paint), 5)
                for m in paint:
                    if m['element_id'].startswith('solid'):
                        self.assertIsNone(m['paint_spans']);continue
                    spans = m['paint_spans']
                    self.assertGreater(len(spans), 1)
                    self.assertAlmostEqual(m['paint_from'], spans[0][0])
                    self.assertAlmostEqual(m['paint_to'], spans[-1][1])
                    self.assertTrue(all(a < b <= c for (a, b), (c, _) in zip(spans, spans[1:])))
                for m in out['motions']:
                    if m['role'] == 'transit':self.assertIsNone(m['paint_spans'])
                # 喷涂长度按实际要喷的几截算
                want = sum(sum(b - a for a, b in s['paint_ranges']) if 'paint_ranges' in s else field_rework.length(s)
                           for s in fp.parse_strokes(dashed_job()['elements'])[0])
                self.assertAlmostEqual(out['paint_length_m'], want, places=6)
                preview = [p for p in out['preview'] if p['role'] == 'paint' and p['paint_spans_m']]
                self.assertEqual(len(preview), 4)

    def test_field_start_loads_spans_and_rejects_bad_ones(self):
        sec = Section(1, Route((0, 0, 0), [(10, 0)]));sec.paint_from_m, sec.paint_to_m = 1.0, 8.0
        self.assertEqual(field_paint_spans([[1, 2], [3, 5], [6, 8]], sec), [(1, 2), (3, 5), (6, 8)])
        self.assertIsNone(field_paint_spans(None, sec))
        for bad in ([], [[2, 1]], [[1, 3], [2, 8]], [[1, 2], [3, 9]], [[0, 2], [3, 8]], [[1, 2], [3, 7]],
                    [[1, '2'], [3, 8]], [[1, 2, 3]], 'x', [[1, float('nan')], [3, 8]]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                field_paint_spans(bad, sec)

    def test_resume_tail_keeps_only_the_remaining_dashes(self):
        sec = Section(1, Route((0, 0, 0), [(10, 0)]));sec.paint_from_m, sec.paint_to_m = 1.0, 8.0
        sec.paint_spans = [(1, 2), (3, 5), (6, 8)];sec.field_role = 'paint'
        sec.paint_dashed = True
        rest = tail([sec], 4.0)[0]
        self.assertEqual(rest.paint_spans, [(0, 1), (2, 4)])
        self.assertEqual((rest.paint_from_m, rest.paint_to_m), (0, 4))
        self.assertTrue(rest.paint_dashed)

    def test_rework_crop_keeps_dashes_aligned_with_the_original_line(self):
        stroke = fp.parse_strokes(dashed_job()['elements'])[0][0]      # side · 1：0–10 m，1.0 / 0.6
        part = field_rework.crop(stroke, 2.5, 6.0)
        # 原线 1.0 / 0.6：0–1、1.6–2.6、3.2–4.2、4.8–5.8 … 截 2.5–6.0 平移到 0 起
        self.assertEqual([tuple(round(v, 9) for v in p) for p in part['paint_ranges']],
                         [(0, .1), (.7, 1.7), (2.3, 3.3)])
        wide = dict(stroke, paint_ranges=fp.dash_ranges(dict(mode='length', dash=1.0, gap=2.0), 10.0))
        with self.assertRaises(ValueError) as cm:
            field_rework.crop(wide, 1.2, 2.8)                          # 整段落在空隙里
        self.assertIn('空隙', str(cm.exception))
        ring = fp.parse_strokes(dashed_job()['elements'])[0][-2]
        self.assertEqual(ring['kind'], 'circle')
        arc = field_rework.crop(ring, 0.0, 5.0)
        self.assertEqual(arc['kind'], 'arc')
        self.assertTrue(all(0 <= a < b <= 5.0 for a, b in arc['paint_ranges']))


class DashDriving(unittest.TestCase):
    """软件底盘按规划开一遍，用生产喷涂判据（RouteOperations.paint_status）看哪里开泵。"""
    dashed = True

    def drive(self, job, start, cfg, car_speed=False):
        out = fp.plan(job, start, cfg)
        secs = kin.sections(out)
        for sec, m in zip(secs, out['motions']):
            sec.paint_spans = None if not m.get('paint_spans') else [tuple(p) for p in m['paint_spans']]
            sec.paint_dashed = sec.paint_spans is not None and self.dashed      # 同 route_operations 场地开始作业
        if car_speed:      # 同车上 LineMission：喷线定速（constant_paint_speed）
            tr = kin.SectionTracker(secs, replace(kin.differential_config(kin.PathConfig(cruise_mps=.8)), straight_imu_enabled=False),
                                    kin.GEO, pivot_rate=.8, constant_straight_speed=True, constant_paint_speed=True)
            tr.set_nozzle(.24, -.4575)
        else:
            tr = kin.tracker(secs)
        rows = kin.drive(tr, 1500.)
        self.assertEqual(tr.st.state, 'DONE', tr.status())
        return out, tr, rows

    def check(self, job, rows, tr, out):
        strokes = {s['id']: s for s in fp.parse_strokes(job['elements'])[0]}
        painted = {k: [] for k in strokes}
        for r, nxt in zip(rows, rows[1:]):
            if not r['paint']:continue
            sec = tr.sections[r['index']];m = out['motions'][r['index']]
            self.assertEqual(m['role'], 'paint')
            u, off = along(strokes[m['element_id']], nozzle(sec, r['s']))
            painted[m['element_id']].append(u)
        for key, stroke in strokes.items():
            ranges = stroke.get('paint_ranges') or [(0.0, field_rework.length(stroke))]     # 实线：整条
            with self.subTest(element=key):
                # 停喷照常越线截止（直线多 ≤ 5 cm）/ 弧提前 5 cm、起喷按控制周期：边界各留 8 cm
                self.assertTrue(all(inside(ranges, u, .08) for u in painted[key]),
                                [round(u, 2) for u in painted[key] if not inside(ranges, u, .08)][:5])
                for a, b in ranges:      # 每一截都喷到了
                    hits = [u for u in painted[key] if a <= u <= b]
                    self.assertTrue(hits)
                    self.assertLess(min(hits) - a, .1)
                    self.assertLess(b - max(hits), .1)

    def test_corner_dashes_from_both_ends_with_arc_turns(self):
        job = kin.corner()
        for e in job['elements']:
            e['dash'] = dict(mode='length', dash=.5, gap=.5, count=10)
        cfg = kin.CFG
        for start in ((-.3, .4, 0.), (5.5, 6.3, -math.pi / 2)):
            with self.subTest(start=start):
                out, tr, rows = self.drive(job, start, cfg)
                self.check(job, rows, tr, out)
                self.assertFalse(any(r['switching'] for r in rows))
                # 同一笔里的虚线空隙不停车
                for i, m in enumerate(out['motions']):
                    if m['role'] != 'paint':continue
                    a, b = m['paint_spans'][0][0], m['paint_spans'][-1][1]
                    v = [r['v'] for r in rows if r['index'] == i and a + .5 <= r['s'] <= b - .5]
                    self.assertGreater(min(v), .25)

    def test_mixed_field_with_pivot_turns_circle_and_arc(self):
        job = dashed_job()
        cfg = fp.Settings(.24, .4575, (.911, .604, .39), .557, lateral_accel=.2)
        out, tr, rows = self.drive(job, (-2., -2., 0.), cfg)
        self.check(job, rows, tr, out)
        # 实线照旧整条喷
        solid = next(i for i, m in enumerate(out['motions']) if m['element_id'] == 'solid/0' and m['role'] == 'paint')
        on = [r['s'] for r in rows if r['index'] == solid and r['paint']]
        m = out['motions'][solid]
        self.assertLess(min(on) - m['paint_from'], .05)
        self.assertGreater(max(on), m['paint_to'] - .05)

    def test_dashes_drive_like_the_solid_line(self):
        """虚线只管水泵：车速和同一条实线一样，空隙里不提速、下一截不当新线头重新压回起步速度。"""
        job = dashed_job('drawing')
        job['elements'][0]['dash'] = dict(mode='length', dash=.5, gap=.5, count=10)    # 线头稳行 1 m 内就有空隙
        solid = json.loads(json.dumps(job))
        for e in solid['elements']:
            e.pop('dash', None)
        cfg = fp.Settings(.24, .4575, (.911, .604, .39), .557, lateral_accel=.2)
        out, tr, rows = self.drive(job, (-2., -2., 0.), cfg, car_speed=True)
        out0, _, rows0 = self.drive(solid, (-2., -2., 0.), cfg, car_speed=True)
        self.check(job, rows, tr, out)
        checked = 0
        for i, m in enumerate(out['motions']):
            if m['role'] != 'paint' or not m['paint_spans']:continue
            self.assertEqual(out0['motions'][i]['element_id'], m['element_id'])
            # 喷线段从头到尾在跟踪器里是一整段（喷头只按各截开关）
            self.assertEqual(tr.trackers[i].paint_spans, [(m['paint_from'], m['paint_to'])])
            a, b = m['paint_spans'][0][0], m['paint_spans'][-1][1]
            ref = sorted((r['s'], r['v']) for r in rows0 if r['index'] == i)
            keys = [s for s, _ in ref]
            worst = 0.
            for r in rows:
                if r['index'] != i or not a <= r['s'] <= b:continue
                j = min(max(1, bisect.bisect_left(keys, r['s'])), len(ref) - 1)
                (s0, v0), (s1, v1) = ref[j - 1], ref[j]
                v_solid = v0 + (v1 - v0) * (r['s'] - s0) / (s1 - s0) if s1 > s0 else v1
                worst = max(worst, abs(r['v'] - v_solid))
            with self.subTest(element=m['element_id']):
                self.assertLess(worst, .02)
            checked += 1
        self.assertEqual(checked, 4)


if __name__ == '__main__':
    unittest.main()
