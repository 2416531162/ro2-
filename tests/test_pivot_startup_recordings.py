"""起步对准真实失败作业 8a100103 / 3fe42370；固定观测回放不预测实车新轨迹。"""
import csv
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import path_tracker as pt
from robot_core.kinematics import ChassisGeometry

FIXTURES=ROOT/'tests/fixtures'
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5,pivot_rate_rad_s=.8,pivot_accel_rad_s2=1.)

def replay(case, module=pt):
    metadata=json.loads((FIXTURES/'pivot_startup_cases.json').read_text())[case]
    with (FIXTURES/f'pivot_startup_{case}.csv').open() as stream:
        rows=[{k:(float(v) if v else None) for k,v in r.items()} for r in csv.DictReader(stream)]
    first=rows[0]
    section=module.Section(1,module.Path((first['x'],first['y'],math.radians(metadata['target_deg'])),[(3.,0.)]))
    tr=module.SectionTracker([section],module.differential_config(module.PathConfig()),GEO,
                             pivot_rate=.8,pivot_accel=1.,settle_s=.2)
    began=False;output=[]
    for row in rows:
        p=SimpleNamespace(**{k:row[k] for k in ('x','y','yaw','speed','yaw_rate')},usable_for_control=bool(row['usable']),
                          motion_evidence=bool(row['motion_evidence']),hall_still=bool(row['hall_still']),
                          gnss_still=None if row['gnss_still'] is None else bool(row['gnss_still']))
        if not began:
            if not row['running']:continue
            assert tr.start(p)
            began=True
        v,w=tr.update(p,row['dt'])
        output.append(dict(row,v=v,w=w,attempts=tr._turns,recovering=getattr(tr,'_pivot_recovering',False),state=tr.st.state))
    return tr,output


class RecordedStartupAlignment(unittest.TestCase):
    def test_both_recorded_startup_rollbacks_keep_driving_in_one_attempt(self):
        for case in ['8a100103','3fe42370']:
            with self.subTest(case=case):
                tr,rows=replay(case)
                recovery=next((r for r in rows if r['recovering']),None)
                self.assertIsNotNone(recovery)
                self.assertTrue(4.3<recovery['elapsed']<4.7,recovery)
                # 原录包在这个时间段撤力并回退，重放修正版必须继续给予同向转向指令。
                rollback=[r for r in rows if 4.7<r['elapsed']<5.1]
                self.assertTrue(all(r['recorded_w']==0. for r in rollback))
                self.assertTrue(all(r['w']>=.1 for r in rollback))
                self.assertEqual(tr._turns,1)
                self.assertEqual(tr.st.state,'RUNNING')
                self.assertTrue(all(r['v']==0. for r in rows))

    def test_initial_alignment_command_is_preserved_until_rollback(self):
        for case in ['8a100103','3fe42370']:
            with self.subTest(case=case):
                _,rows=replay(case)
                initial=[r for r in rows if r['elapsed']<4.05]
                self.assertLess(max(abs(r['w']-r['recorded_w']) for r in initial),.0002)
                self.assertLessEqual(max(abs(r['w']) for r in rows),.8+1e-9)
                for a,b in zip(rows,rows[1:]):
                    self.assertLessEqual(abs(a['w']-b['w']),b['dt']+1e-7)


if __name__=='__main__':unittest.main()
