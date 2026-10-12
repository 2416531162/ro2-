"""用实录姿态和限幅前指令回放起步交接；不将固定输入回放当成新的实车轨迹。"""
import csv,json,math
from pathlib import Path
from types import SimpleNamespace as NS
import sys,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import path_tracker as pt
from robot_core.kinematics import ChassisGeometry

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)
FIXTURES=ROOT/'tests/fixtures'

def replay(case,module=pt):
    meta=json.loads((FIXTURES/'launch_handover_cases.json').read_text())[case]
    with (FIXTURES/f'launch_handover_{case}.csv').open() as stream:
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(stream)]
    reverse=meta['direction']=='reverse'
    sec=module.Section(-1 if reverse else 1,module.Path((0.,0.,0.),[(10.,0.)]))
    sec.field_role='transit' if reverse else 'paint'
    if not reverse:sec.paint_from_m=meta['paint_from_m'];sec.paint_to_m=9.5
    route=module.SectionTracker([sec],module.differential_config(module.PathConfig(cruise_mps=.5)),GEO,
                                reverse_mps=.5,reverse_gain=1.,constant_straight_speed=True,constant_paint_speed=True)
    route.set_nozzle(.24,-.4575)
    tr=route.trackers[0]
    output=[]
    for i,r in enumerate(rows):
        p=NS(x=r['s'],y=r['e'],yaw=math.radians(r['he_deg']),speed=r['speed'],yaw_rate=r['yaw_rate'],
             usable_for_control=True,quality='RTK_FIXED' if r['trusted'] else 'RTK_FLOAT',rtk_trusted=bool(r['trusted']),
             reasons=(),correction_xy=(0.,0.),correction_yaw=0.)
        if i==0:assert tr.start(p)
        # 本回放只比较末级稳定/交接策略：内环请求来自同一条实录，排除重新估计参考航向的差异。
        def request(*args,w=r['w_requested']):
            tr._straight_status=dict(active=True,final_wz=w,inner_wz=w)
            return w
        tr._straight_control=request
        v,w=tr.update(p,r['dt'])
        output.append(dict(r,v=v,w=w,settle=dict(tr._settle or {}),final_wz=tr._straight_status.get('final_wz')))
    return tr,output


class RecordedHandover(unittest.TestCase):
    def test_large_trusted_errors_are_not_stuck_at_two_degrees_per_second(self):
        for case in ['9fbb5dde','be9f8e09','06e063d2']:
            with self.subTest(case=case):
                tr,rows=replay(case)
                clipped=[r for r in rows if abs(abs(r['w_sent'])-math.radians(2.))<.0001
                         and abs(r['w_requested'])>math.radians(4.)]
                self.assertGreater(len(clipped),20)
                self.assertGreater(max(abs(r['w']) for r in clipped),math.radians(3.))
                for r in rows:
                    self.assertLessEqual(abs(r['w']),2*abs(r['v'])/GEO.track_m+1e-9)
                    self.assertAlmostEqual(r['final_wz'],r['w'],places=10)

    def test_recorded_six_cm_error_does_not_release_speed_after_one_metre(self):
        _,rows=replay('9fbb5dde')
        bad=[r for r in rows if 1.1<r['s']<1.45]
        self.assertTrue(bad)
        self.assertGreater(max(r['v_sent'] for r in bad),.49)
        self.assertTrue(all(not r['settle']['done'] for r in bad))
        self.assertLessEqual(max(r['v'] for r in bad),.35+1e-9)

    def test_handover_obeys_zero_speed_cap(self):
        tr,_=replay('9fbb5dde')
        tr.speed_limit_mps=0.
        p=NS(x=3.,y=.06,yaw=.05,speed=.3,yaw_rate=0.,usable_for_control=True,quality='RTK_FIXED',
             rtk_trusted=True,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)
        self.assertEqual(tr.update(p,.02),(0.,0.))


if __name__=='__main__':unittest.main()
