"""倒车坐标、闭环方向、换向隔离及保护；不连接真实底盘。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest import TestCase

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route,PathConfig,Section,SectionTracker,differential_config,wrap
from robot_core.kinematics import ChassisGeometry
from line_cascade import ImuAlignment

DT=.02
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)

def setup(heading=math.pi,curve=0.,enabled=True):
    sec=Section(-1,Route((0.,0.,heading),[(6.,curve)]));sec.field_role='transit'
    cfg=replace(differential_config(PathConfig(cruise_mps=.5)),straight_imu_enabled=enabled)
    route=SectionTracker([sec],cfg,GEO,reverse_mps=.5,reverse_gain=1.,constant_straight_speed=True)
    route.set_nozzle(.24,-.4575)
    return route

def pose(x=0.,y=0.,yaw=0.,speed=0.,rate=0.):
    return NS(x=x,y=y,yaw=yaw,speed=speed,yaw_rate=rate,usable_for_control=True,
        quality='RTK_FIXED',rtk_trusted=True,reasons=(),correction_xy=(0.,0.),correction_yaw=0.)

def tick(route,t,p,stale=False):
    route.sync(t,(t-.3 if stale else t,wrap(p.yaw+1.2),p.yaw_rate),(.235,.197))
    route.set_heading_baseline(t,.412);route.set_rtk_quality(t,.008,32,.005)
    return route.update(p,DT)

class ReverseImu(TestCase):
    def test_first_sample_uses_imu_without_reversing_gyro_or_wheel_direction(self):
        for angle in (0.,math.pi,math.pi-.005):
            with self.subTest(angle=angle):
                route=setup(angle);p=pose(yaw=wrap(angle-math.pi));route.start(p)
                v,w=tick(route,.02,p);tr=route.trackers[0]
                self.assertLess(v,0.)
                self.assertEqual(tr._straight_status['control_law'],'imu_launch')
                self.assertEqual(tr._straight_nozzle,(0.,0.))
                self.assertEqual(tr._imu_lever,(-.235,-.197))
                p.yaw=wrap(p.yaw+.005);p.yaw_rate=.1
                v,w=tick(route,.04,p)
                self.assertLess(w,0.)
                self.assertEqual(tr._imu[2],.1)
                self.assertLessEqual(v-GEO.track_m*w/2.,0.)
                self.assertLessEqual(v+GEO.track_m*w/2.,0.)

    def test_reverse_control_remains_active_in_the_middle_at_half_metre_per_second(self):
        route=setup();p=pose();route.start(p);v=0.
        for i in range(1,231):
            t=i*DT;p.speed=v
            v,w=tick(route,t,p);p.x+=v*DT
            if t>2.5:
                self.assertAlmostEqual(v,-.5,places=7)
                self.assertEqual(route.trackers[0]._straight_status['control_law'],'imu_first')
                self.assertAlmostEqual(route.trackers[0]._straight_weight,1.)
        self.assertGreater(abs(p.x),1.)
        self.assertEqual(route.st.state,'RUNNING')

    def test_both_reverse_wheel_targets_stay_backward_with_initial_error(self):
        for sign in (-1,1):
            with self.subTest(sign=sign):
                route=setup();p=pose(y=sign*.02,yaw=sign*.03);route.start(p);v=0.
                for i in range(1,60):
                    p.speed=v;v,w=tick(route,i*DT,p)
                    self.assertLessEqual(v-GEO.track_m*w/2.,1e-12)
                    self.assertLessEqual(v+GEO.track_m*w/2.,1e-12)
                    p.x+=v*DT

    def test_reverse_samples_do_not_change_forward_alignment_or_nozzle(self):
        reverse=Section(-1,Route((0.,0.,math.pi),[(2.5,0.)]));reverse.field_role='transit'
        forward=Section(1,Route((-2.5,0.,0.),[(8.,0.)]));forward.field_role='paint'
        route=SectionTracker([reverse,forward],differential_config(PathConfig()),GEO)
        alignment=ImuAlignment();route.share_imu_alignment(alignment);route.set_nozzle(.24,-.4575)
        route.sync(1.,(1.,.5,.2),(.235,.197))
        route.set_secondary_fix(1.,2.,3.,True,(.235,-.213))
        back,front=route.trackers
        self.assertIsNone(back._imu_alignment)
        self.assertIs(front._imu_alignment,alignment)
        self.assertEqual(front._imu,(1.,.5,.2))
        self.assertEqual(front._imu_lever,(.235,.197))
        self.assertEqual(front._straight_nozzle,(.24,-.4575))
        self.assertEqual(back._secondary[4],(-.235,.213))

    def test_stale_imu_and_invalid_sample_fall_back_without_old_integral(self):
        route=setup();p=pose();route.start(p);tick(route,.02,p)
        self.assertGreater(route.trackers[0]._launch_weight,0.)
        v,w=tick(route,.04,p,stale=True)
        tr=route.trackers[0]
        self.assertEqual(tr._launch_weight,0.)
        self.assertEqual(tr._straight_core.integral,0.)
        self.assertLessEqual(v-GEO.track_m*w/2.,1e-12)
        self.assertLessEqual(v+GEO.track_m*w/2.,1e-12)
        for bad in ((.06,float('nan'),0.),(.06,float('inf'),0.),('bad',),None):
            route.sync(.06,bad)

    def test_stop_and_pose_loss_return_zero_in_reverse(self):
        for mode in ('cap','pause','stop','pose'):
            with self.subTest(mode=mode):
                route=setup();p=pose();route.start(p);tick(route,.02,p)
                if mode=='cap':route.speed_limit_mps=0.;out=tick(route,.04,p)
                elif mode=='pose':p.usable_for_control=False;out=tick(route,.04,p)
                else:out=getattr(route,mode)()
                self.assertEqual(out,(0.,0.))
                self.assertEqual(route.trackers[0]._launch_weight,0.)

    def test_reverse_curve_and_disabled_imu_do_not_use_the_straight_controller(self):
        for k,enabled in ((.3,True),(0.,False)):
            with self.subTest(k=k,enabled=enabled):
                route=setup(curve=k,enabled=enabled)
                self.assertFalse(route.trackers[0].cfg.straight_imu_enabled)
                self.assertFalse(route.trackers[0].reverse_straight)

    def test_amplified_delayed_turn_response_does_not_keep_oscillating(self):
        # 实录倒车约 0.2 s 的相位滞后、周期幅值约 3 倍；再用 0.35 s 检查失配余量。
        # 这是保守的闭环压力模型，不是对实车机械参数的唯一辨识。
        for delay in (.2,.35):
            with self.subTest(delay=delay):
                route=setup();p=pose(y=.01,yaw=math.radians(.5));route.start(p)
                queue=[];head=0;cmd=(0.,0.);dt=.01;middle=[];error=[]
                for i in range(1600):
                    t=i*dt
                    if i%2==0:
                        v,w=tick(route,t,p);queue.append((t,v,w))
                    while head<len(queue) and queue[head][0]<=t-delay+1e-9:
                        cmd=queue[head][1:];head+=1
                    p.speed+=(cmd[0]-p.speed)*dt/(.12+dt)
                    kick=.3*math.sin(math.pi*(t-.25)/.5) if .25<t<.75 else 0.
                    p.yaw_rate+=(3.*cmd[1]+kick-p.yaw_rate)*dt/(.06+dt)
                    p.yaw+=p.yaw_rate*dt
                    p.x+=p.speed*math.cos(p.yaw)*dt;p.y+=p.speed*math.sin(p.yaw)*dt
                    if 2.<=t<=6.:middle.append(math.degrees(p.yaw))
                    error.append(abs(p.y))
                    if route.st.state=='DONE' and abs(p.speed)<.001:break
                self.assertEqual(route.st.state,'DONE')
                self.assertLess(max(middle)-min(middle),3.5)
                self.assertLess(max(error),.025)
                self.assertLess(abs(p.y),.01)
