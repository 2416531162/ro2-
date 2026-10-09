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
    tr._test_route=route
    tr.reverse_runup=reverse
    if reverse:tr.cfg=replace(tr.cfg,start_hold_m=1.)
    return tr

def pose(x=0.,v=0.,pending=.006):
    return NS(x=x,y=0.,yaw=0.,speed=v,yaw_rate=0.,usable_for_control=True,quality='RTK_FIXED',
              rtk_trusted=True,reasons=(),correction_xy=(pending,0.),correction_yaw=0.)

def drive(speed=.8,reverse=False,broken=None,stop_at=6.):
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
                         points=tr._straight_guide.fit_points,weight=tr._straight_weight,
                         paint_ready=tr.paint_speed_ready,expected=tr.paint_expected_mps))
        if tr.st.state!='RUNNING' or x>stop_at:break
        sent.append(vc);v=sent.popleft();x+=v*DT
    return tr,rows

class SmoothRunup(unittest.TestCase):
    def test_short_runup_is_ready_at_head_without_lower_speed_or_faster_handoff(self):
        sec=Section(1,Route((0.,0.,0.),[(7.233333333333,0.)]))
        sec.field_role='paint';sec.paint_from_m=5/6;sec.paint_to_m=sec.path.length-.4
        route=SectionTracker([sec],differential_config(PathConfig(cruise_mps=.8)),GEO,
                             constant_straight_speed=True,constant_paint_speed=True)
        tr=route.trackers[0];tr.set_nozzle(0.,0.);x=0.;v=.35
        self.assertTrue(tr.start(pose(x,v)))
        speeds=[];lost=False
        for i in range(400):
            t=(i+1)*DT;p=pose(x,v);tr.sync(t,(t,0.,0.))
            tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
            if .775<=x<.796:p.rtk_trusted=False;lost=True
            v,_=tr.update(p,DT);speeds.append(v)
            if x>=sec.paint_from_m:
                self.assertTrue(tr.paint_start_allowed(),tr.status())
                break
            self.assertAlmostEqual(v,.35,places=8)
            x+=v*DT
        self.assertTrue(lost)
        self.assertGreaterEqual(x,sec.paint_from_m)
        self.assertEqual(tr.cfg.straight_blend_s,.8)
        self.assertLessEqual(speeds[-1]-.35,.5*DT*DT+1e-8)

    def test_first_metre_is_half_speed_then_ramps_while_spraying(self):
        for reverse in (False,True):
            for speed in (.5,.8,1.):
                with self.subTest(reverse=reverse,speed=speed):
                    tr,rows=drive(speed,reverse)
                    self.assertEqual(tr.st.state,'RUNNING',tr.status())
                    before=[r for r in rows if r['x']<2.5]
                    self.assertGreater(before[-1]['points'],0)
                    self.assertTrue(before[-1]['ready'])
                    self.assertAlmostEqual(before[-1]['v'],min(speed,.5),places=5)
                    first_metre=[r for r in rows if 2.5<=r['x']<3.5]
                    self.assertTrue(first_metre)
                    self.assertTrue(all(abs(r['v']-min(speed,.5))<1e-6 for r in first_metre))
                    painted=[r for r in rows if r['x']>=2.5]
                    self.assertTrue(all(r['paint_ready'] for r in painted))
                    self.assertAlmostEqual(rows[-1]['v'],speed,places=5)
                    self.assertTrue(tr.paint_start_allowed())
                    self.assertTrue(all(not r['started'] for r in before))
                    self.assertTrue(all(b['v']>=a['v']-1e-8 for a,b in zip(rows,rows[1:])))
                    self.assertGreater(rows[0]['v'],0.)
                    self.assertLess(rows[0]['v'],.005)
                    # 新鲜 IMU 下静止起步连续升至 0.35，之后沿原缓慢提速；两阶段都不允许速度阶跃。
                    self.assertLessEqual(max(r['a'] for r in rows),.500001)
                    self.assertLessEqual(max(r['a'] for r in rows if r['v']>.351),.200001)
                    accel=[0.]+[r['a'] for r in rows]
                    self.assertLessEqual(max(abs(b-a)/DT for a,b in zip(accel,accel[1:])),2.000001)
                    # 不再沿前段连续卡在 0.3 m/s，然后把提速挤到线头前。
                    self.assertLess(sum(abs(r['v']-.3)<.005 for r in before)*DT,.10)

    def test_bad_position_and_real_jump_cannot_start_spray(self):
        for broken in ('float','jump'):
            with self.subTest(broken=broken):
                tr,rows=drive(broken=broken)
                self.assertFalse(tr.paint_start_allowed())
                self.assertEqual(tr.st.state,'RUNNING')
                self.assertGreater(rows[-1]['x'],2.5)
                self.assertAlmostEqual(rows[-1]['v'],.5)

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
        self.assertTrue(tr.st.integral_frozen)
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

    def test_start_floor_is_once_only_and_does_not_override_lower_targets(self):
        tr=make();tr.paint_start_guard=None;tr.start(pose())
        tr.speed_limit_mps=0.
        self.assertEqual(tr.update(pose(),DT),(0.,0.))
        tr.speed_limit_mps=None
        self.assertEqual(tr.update(pose(),DT)[0],.35)
        # 已经起步后的保护降速和恢复均不再跳回起步下限。
        tr.speed_limit_mps=.1
        self.assertEqual(tr.update(pose(.01,.35),DT)[0],.1)
        tr.speed_limit_mps=None
        self.assertLessEqual(tr.update(pose(.02,.1),DT)[0],.1+.5*DT*DT+1e-9)
        slow=make(.2);slow.paint_start_guard=None;slow.start(pose())
        self.assertEqual(slow.update(pose(),DT)[0],.2)

    def test_rolling_entry_does_not_jump_to_start_floor(self):
        tr=make();tr.paint_start_guard=None;tr.start(pose(0.,.2));tr._rolling_entry=True
        self.assertLessEqual(tr.update(pose(0.,.2),DT)[0],.2+.5*DT*DT+1e-9)

    def test_runup_uses_normal_steering_eligibility_before_one_metre(self):
        tr=make(reverse=True)
        tr._creep_from=0.;tr.st.s=.2;tr.st.e=.03;tr.st.heading_err=math.radians(2.)
        self.assertEqual(tr._straight_eligible(tr.path.pieces[0],.35),(True,False))
        # IMU 尚未锁定仍不能接管；该判断在 _straight_first 中独立执行。
        self.assertFalse(tr._straight_guide.locked)

    def test_unstable_first_metre_holds_half_speed_instead_of_accelerating(self):
        tr=make();tr.start(pose());tr.st.e=0.;tr.st.heading_err=0.
        self.assertEqual(tr._entry_paint_target(4.,.8),.5)
        for i in range(20):
            tr._entry_speed_guard.update(DT,trusted=True,frame=i,weight=1.,rtk_error=0.,guide_error=0.,heading_error=0.)
        self.assertEqual(tr._entry_paint_target(4.,.8),.8)
        # 已放行的提速不会因单帧证据变化重新跳回 0.5；偏离/定位保护由独立检查处理。
        tr._entry_speed_guard.update(DT,trusted=False,frame=21,weight=1.,rtk_error=0.,guide_error=0.,heading_error=0.)
        self.assertEqual(tr._entry_paint_target(4.01,.8),.8)

    def test_planned_ramp_and_final_output_mismatch_both_allow_spray(self):
        from line_mission import LineMission
        import paint_quality
        tr,rows=drive(.8,stop_at=4.)
        self.assertGreater(rows[-1]['v'],.5);self.assertLess(rows[-1]['v'],.8)
        m=LineMission.__new__(LineMission);m.tracker=tr._test_route;m.task='route'
        m.pose=pose(rows[-1]['x'],rows[-1]['v']);m.differential=True;m.nozzle=(0.,0.)
        m._check_paint_output_speed(rows[-1]['v'])
        self.assertTrue(paint_quality.status(m,{'field_plan':True},True,m.tracker.sections[0],tr)['painting'])
        m._check_paint_output_speed(.3)
        m.pose.speed=.3
        self.assertFalse(tr.paint_speed_ready)
        self.assertTrue(paint_quality.status(m,{'field_plan':True},True,m.tracker.sections[0],tr)['painting'])

    def test_stopped_midline_resume_uses_floor_and_new_stabilization_distance(self):
        tr=make();tr.start(pose());tr.paint_start_guard.started=True
        tr._hint=4.;tr.pause();self.assertTrue(tr.start(pose(4.,0.)))
        self.assertEqual(tr.update(pose(4.,0.),DT)[0],.35)
        self.assertEqual(tr._entry_hold_from,4.)
        self.assertEqual(tr.paint_target_mps,.5)

if __name__=='__main__':unittest.main()
