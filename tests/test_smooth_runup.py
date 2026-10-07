"""直线起步连续提速；可信位置拟合不被起步积分冻结或正常平滑校正挡住。"""
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import math,sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route,Section,SectionTracker,PathConfig,differential_config
from robot_core.kinematics import ChassisGeometry
from line_tracker import lateral_integral_allowed

DT=.02
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def make(speed=.8,reverse=False):
    sec=Section(1,Route((0.,0.,0.),[(15.,0.)]))
    sec.field_role='paint';sec.paint_from_m=2.5;sec.paint_to_m=14.5
    config=differential_config(PathConfig(cruise_mps=speed))
    route=SectionTracker([sec],config,GEO,constant_straight_speed=True,constant_paint_speed=True)
    tr=route.trackers[0];tr.set_nozzle(0.,0.)
    tr.reverse_runup=reverse
    if reverse:tr.cfg=replace(tr.cfg,start_hold_m=1.)
    return tr

def pose(x=0.,v=0.,pending=.006):
    return NS(x=x,y=0.,yaw=0.,speed=v,yaw_rate=0.,usable_for_control=True,quality='RTK_FIXED',
              rtk_trusted=True,reasons=(),correction_xy=(pending,0.),correction_yaw=0.)

def drive(speed=.8,reverse=False,broken=None):
    tr=make(speed,reverse);x=v=0.;sent=deque([0.]*15);rows=[]
    assert tr.start(pose())
    for i in range(1200):
        t=(i+1)*DT;p=pose(x,v)
        tr.sync(t,(t,0.,0.))
        if i%10==0:
            tr.set_heading_baseline(t,.412)
            tr.set_rtk_quality(t,.008,32,.005)
        if broken=='float':p.quality='RTK_FLOAT';p.rtk_trusted=False
        if broken=='jump' and x>1.2:p.y=.12
        vc,w=tr.update(p,DT)
        rows.append(dict(t=t,x=x,v=vc,a=(vc-rows[-1]['v'])/DT if rows else vc/DT,
                         ready=tr.paint_start_guard.ready,started=tr.paint_start_guard.started,
                         points=tr._straight_guide.fit_points,weight=tr._straight_weight))
        if tr.st.state!='RUNNING' or x>3.:break
        sent.append(vc);v=sent.popleft();x+=v*DT
    return tr,rows

class SmoothRunup(unittest.TestCase):
    def test_forward_and_reverse_entry_reach_cruise_before_original_line_head(self):
        for reverse in (False,True):
            for speed in (.5,.8):
                with self.subTest(reverse=reverse,speed=speed):
                    tr,rows=drive(speed,reverse)
                    self.assertEqual(tr.st.state,'RUNNING',tr.status())
                    before=[r for r in rows if r['x']<2.5]
                    self.assertGreater(before[-1]['points'],0)
                    self.assertTrue(before[-1]['ready'])
                    self.assertAlmostEqual(before[-1]['v'],speed,places=5)
                    self.assertTrue(tr.paint_start_allowed())
                    self.assertTrue(all(not r['started'] for r in before))
                    self.assertTrue(all(b['v']>=a['v']-1e-8 for a,b in zip(rows,rows[1:])))
                    self.assertLessEqual(max(r['a'] for r in rows),.250001)
                    self.assertLessEqual(max(abs(b['a']-a['a'])/DT for a,b in zip(rows,rows[1:])),.500001)
                    # 不再沿前段连续卡在 0.3 m/s，然后把提速挤到线头前。
                    self.assertLess(sum(abs(r['v']-.3)<.005 for r in before)*DT,.10)

    def test_bad_position_and_real_jump_cannot_start_spray(self):
        for broken in ('float','jump'):
            with self.subTest(broken=broken):
                tr,rows=drive(broken=broken)
                self.assertFalse(tr.paint_start_allowed())
                self.assertEqual(tr.st.reason,'paint_start_unready')
                self.assertLess(rows[-1]['x'],2.5)

    def test_pending_correction_freezes_integral_but_not_trusted_runup_fit(self):
        tr=make();p=pose(.8,.3)
        self.assertFalse(lateral_integral_allowed(p))
        tr.st.integral_frozen=True;tr.st.integral=.012
        g=tr._straight_guide;g.offset=0.;g.locked=True;g.source='learned'
        for i in range(1,161):
            t=i*DT;tr.sync(t,(t,0.,0.));tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
            tr.st.s=.1+.006*i
            tr._straight_first(p,DT,.3,0.,tr.path.pieces[0],0.,0.)
        self.assertGreater(g.fit_points,0)
        self.assertEqual(tr.st.integral,.012)
        for reasons in (('rtk_rejoin',),('position_outlier',),('rtk_hold_pending',)):
            p.reasons=reasons
            self.assertFalse(tr._runup_position_trusted(p))

    def test_releasing_external_cap_restarts_acceleration_gently(self):
        tr=make();tr.paint_start_guard=None
        tr.start(pose(0.,.3));tr._v_prev=.3;tr._accel_prev=.25;tr.speed_limit_mps=.3
        v,_=tr.update(pose(0.,.3),DT)
        self.assertAlmostEqual(v,.3)
        tr.speed_limit_mps=None
        v,_=tr.update(pose(.01,.3),DT)
        self.assertLessEqual(v-.3,.5*DT*DT+1e-9)
        tr.speed_limit_mps=0.
        self.assertEqual(tr.update(pose(.02,.3),DT),(0.,0.))

if __name__=='__main__':unittest.main()
