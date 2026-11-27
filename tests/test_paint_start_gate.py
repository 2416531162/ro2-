"""起喷门槛（2026-10-10 用户：「如果起喷是差 5cm 以上，必须停喷」）：每个喷线区间第一次开喷时喷头离线不到 5 cm
才开，没到先不喷、车照走；开喷后中途偏差照旧只报警不停喷。2026-10-11 起作业的第一个线头没过门槛时退回去重新助跑。

复现 10-10 实录 16:05:30：开始作业时给控制的航向错 27°（每秒只追 1°），接入开偏，原来到线头偏 23 cm 照样开喷。
11-27 起开始作业进线只走圆弧（2 m 直线助跑把横偏收回，到线头过得了门槛），门槛本身用 route start（原地转进线）复现。"""
from pathlib import Path
import math
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'radar_system'), str(ROOT), str(ROOT / 'tests')]
import line_mission as lm
import paint_quality as pq
import route_plan as rp
from test_offline_reentry import DT, Car, mission


def run(m, car, seconds, hook=None):
    rows = []
    for i in range(int(seconds / DT)):
        vx, wz, _ = m.tick(i * DT, 'NAVIGATION')
        car.step(vx, wz)
        paint = m.operations.paint_status() if m.phase == 'RUNNING' else {}
        rows.append(dict(on=m.operations._paint_mark_on, est=paint.get('nozzle_deviation_m'),
                         blocked=paint.get('paint_blocked'), message=m.message))
        if hook:
            hook(i, m, car)
        if m.phase == 'IDLE':
            break
    return rows


class PaintStartGate(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.folder.cleanup()

    def test_threshold_is_five_centimetres(self):
        self.assertEqual(pq.PAINT_START_MAX_OFFSET_M, .05)

    # 2026-11-19 进线改成转正后顺线倒回 RUNUP_BACK_M：倒的这 1.5 m 已把 27° 航向错带来的横偏收回，到线头不再过门槛；
    # 这里关掉它，只验门槛本身。开始作业 11-27 起只走圆弧进线，圆弧后 2 m 直线助跑也把横偏收回了：用从图纸起点开始
    # （route start，还是原地转进线）复现到线头偏
    @mock.patch.object(rp, 'RUNUP_BACK_M', 0.)
    def test_wrong_heading_start_backs_up_and_paints_from_the_head(self):
        # 2026-10-11 用户：第一个线头没过门槛不跳过线头往前喷，退回去重新助跑（见 test_launch_twist）
        car = Car(-3., 1.7, math.radians(100.), err_deg=27.)
        m = mission(car, self.folder.name, cmd='route_start')
        rows = run(m, car, 300.)
        first = next(i for i, r in enumerate(rows) if r['on'])
        self.assertTrue(any(r['message'].startswith('到线头时喷头离线') for r in rows[:first]))
        self.assertLess(rows[first]['est'], .05)
        self.assertEqual(m.message, '路线完成')
        self.assertEqual(m.operations.record['reentries'], 1)
        on = [i for i in m.operations._paint_marks if i['on']]
        self.assertEqual(len(on), 1)
        self.assertLess(abs(on[0]['x']), .05)          # 从线头开喷，没有拦掉的一截

    # 同上：关掉 RUNUP_BACK_M、用 route start 复现到线头偏
    @mock.patch.object(rp, 'RUNUP_BACK_M', 0.)
    def test_wrong_heading_start_does_not_paint_until_within_5cm(self):
        # 退回去的次数用完（REENTRY_MAX）：照旧车照走、先不喷，到 5 cm 内才开
        car = Car(-3., 1.7, math.radians(100.), err_deg=27.)
        m = mission(car, self.folder.name, cmd='route_start')
        m.operations.record['reentries'] = lm.REENTRY_MAX
        rows = run(m, car, 200.)
        first = next(i for i, r in enumerate(rows) if r['on'])
        # 开喷前被门槛拦过，开喷那一拍车自己算的喷头离线在 5 cm 内
        self.assertTrue(any(r['blocked'] == 'paint_start_off_line' for r in rows[:first]))
        self.assertTrue(any(r['message'].startswith(lm.PAINT_START_OFF_PREFIX) for r in rows[:first]))
        self.assertLess(rows[first]['est'], .05)
        self.assertTrue(all(r['est'] is None or r['est'] >= .05 for r in rows[:first]
                            if r['blocked'] == 'paint_start_off_line'))
        # 拦掉的那段如实记成停喷区段（App 提示补画），开喷后提示改回
        self.assertIn('停喷区段', m.message)
        self.assertFalse(rows[first + 5]['message'].startswith(lm.PAINT_START_OFF_PREFIX))

    def test_correct_heading_paints_from_the_head_as_before(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        rows = run(m, car, 120.)
        self.assertEqual(m.message, '路线完成')
        self.assertFalse(any(r['blocked'] == 'paint_start_off_line' for r in rows))

    def test_after_paint_started_large_offset_keeps_painting(self):
        car = Car(-3., 1.7, math.radians(100.))
        m = mission(car, self.folder.name)
        state = {}

        def bump(i, m, car):
            # 开喷 2 m 后车被横推 8 cm：中途偏差超过 5 cm 不停喷（只有起喷才拦）
            if 'at' not in state and m.operations._paint_mark_on and m.operations.record['cursor_m'] > 0 \
                    and car.x > 2.:
                car.y += .08
                state['at'] = i
        rows = run(m, car, 120., bump)
        i = state['at']
        after = rows[i + 1:i + 6]
        self.assertTrue(all(r['est'] is not None and r['est'] > .05 for r in after), after)
        self.assertTrue(all(r['on'] for r in after))
        self.assertFalse(any(r['blocked'] == 'paint_start_off_line' for r in rows[i:]))


if __name__ == '__main__':
    unittest.main()
