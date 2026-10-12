"""原地转向收尾：坡上回退、短指令推不动、真实堵转。仅运行数学模型，不访问车辆。"""
from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
import path_tracker as pt
from robot_core.kinematics import ChassisGeometry

DT=.02
DEG=math.pi/180.
GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5,pivot_rate_rad_s=.8,pivot_accel_rad_s2=1.)

def pose(yaw=0., rate=0., usable=True):
    return NS(x=0.,y=0.,yaw=yaw,speed=0.,yaw_rate=rate,usable_for_control=usable)

def tracker(angle=90., module=pt):
    cfg=replace(module.differential_config(module.PathConfig()),straight_imu_enabled=False)
    tr=module.SectionTracker([module.Section(1,module.Path((0.,0.,angle*DEG),[(3.,0.)]))],cfg,GEO,
                             pivot_rate=.8,pivot_accel=1.,settle_s=.2)
    assert tr.start(pose())
    return tr


class PivotPlant:
    """延迟 + 一阶速度响应；可附加小速度起动等待和有限回弹。

    起动等待代表坡载/万向轮阻力，必须持续给出非零指令才能开始转；零指令释放有限储能。
    参数用于敏感性回归，不宣称是本次现场坡度的实测标定。
    """
    def __init__(self,delay=.325,gain=1.,breakaway=0.,spring_deg=0.,jammed=False):
        self.delay,self.gain,self.breakaway,self.spring_limit=delay,gain,breakaway,spring_deg*DEG
        self.jammed=jammed
        self.yaw=self.rate=self.clock=self.energized=self.spring=0.
        self.sent=deque([(0.,0.)])

    def step(self,w,dt=DT):
        self.sent.append((self.clock,w))
        due=self.clock-self.delay
        while len(self.sent)>1 and self.sent[1][0]<=due+1e-9:self.sent.popleft()
        cmd=self.sent[0][1]
        driving=abs(cmd)>=.06
        self.energized=self.energized+dt if driving else 0.
        effective=cmd if driving and (abs(cmd)>=.25 or self.energized>=self.breakaway) else 0.
        if self.jammed:effective=0.
        self.rate+=(effective*self.gain-self.rate)*dt/(.05+dt)
        forward=self.rate*dt
        if driving:
            self.spring=max(-self.spring_limit,min(self.spring_limit,self.spring+forward))
        else:
            back=self.spring*dt/(.15+dt)
            self.spring-=back
            forward-=back
        self.yaw+=forward
        self.clock+=dt
        return pose(self.yaw,forward/dt)


def simulate(angle=90.,module=pt,**kwargs):
    tr,car=tracker(angle,module),PivotPlant(**kwargs)
    p=pose();out=[];last=0.;reversals=0
    for _ in range(int(65/DT)):
        v,w=tr.update(p,DT)
        if abs(w)>1e-5:
            sign=math.copysign(1.,w)
            if last and last!=sign:reversals+=1
            last=sign
        out.append((car.clock,p.yaw,p.yaw_rate,w,tr.switching,tr.status()['pivoting']))
        if tr.st.state!='RUNNING' or not tr.switching:break
        p=car.step(w)
    return dict(state=tr.st.state,reason=tr.st.reason,settled=not tr.switching,
                error_deg=math.degrees(pt.wrap(angle*DEG-p.yaw)),seconds=car.clock,
                attempts=tr._turns,reversals=reversals,recoveries=getattr(tr,'_pivot_recoveries',0)),out


class PivotFeedback(unittest.TestCase):
    def test_zero_command_does_not_end_turn_while_vehicle_is_rolling_back(self):
        tr=tracker()
        for _ in range(20):tr.update(pose(),DT)
        tr._pivot_braking=True;tr._w=0.
        tr.update(pose(80*DEG,-.1),DT)
        self.assertTrue(tr.status()['pivoting'])

    def test_rollback_recovers_in_the_same_attempt(self):
        tr=tracker()
        for _ in range(20):tr.update(pose(),DT)
        attempts=tr._turns
        tr._pivot_braking=True;tr._w=0.
        commands=[]
        for i in range(30):commands.append(tr.update(pose((80.-i*.08)*DEG,-.07),DT)[1])
        self.assertGreater(max(commands),.1)
        self.assertEqual(tr._turns,attempts)

    def test_short_rollback_noise_does_not_reenergize(self):
        tr=tracker()
        for _ in range(20):tr.update(pose(),DT)
        tr._pivot_braking=True;tr._w=0.
        commands=[tr.update(pose((89.+(-1)**i*.1)*DEG,(-1)**i*.03),DT)[1] for i in range(10)]
        self.assertEqual(commands,[0.]*10)

    def test_pending_commands_drain_before_accepting_standstill(self):
        tr=tracker()
        for _ in range(20):tr.update(pose(),DT)
        tr._pivot_braking=True;tr._w=0.
        for _ in range(10):tr.update(pose(89.8*DEG,0.),DT)
        self.assertTrue(tr.status()['pivoting'])
        for _ in range(30):tr.update(pose(89.8*DEG,0.),DT)
        self.assertFalse(tr.switching)

    def test_pause_and_bad_pose_always_output_zero(self):
        tr=tracker()
        for _ in range(30):tr.update(pose(),DT)
        self.assertGreater(tr._w,0.)
        tr.pause()
        self.assertEqual(tr.update(pose(),DT),(0.,0.))
        self.assertTrue(tr.start(pose()))
        self.assertEqual(tr.update(pose(usable=False),DT),(0.,0.))
        self.assertEqual(tr.st.state,'PAUSED')


class PivotSimulation(unittest.TestCase):
    def test_slope_load_and_springback_can_finish_without_exhausting_retries(self):
        for angle in (-90.,-10.,10.,90.):
            with self.subTest(angle=angle):
                r,_=simulate(angle,breakaway=.4,spring_deg=6.)
                self.assertTrue(r['settled'],r)
                self.assertLessEqual(abs(r['error_deg']),5.,r)
                self.assertLess(r['seconds'],20.,r)
                self.assertEqual(r['attempts'],1,r)
                self.assertGreater(r['recoveries'],0,r)

    def test_small_rebound_does_not_require_precise_repeated_topups(self):
        r,_=simulate(90.,breakaway=.4,spring_deg=1.2)
        self.assertTrue(r['settled'],r)
        self.assertEqual((r['attempts'],r['recoveries']),(1,0),r)

    def test_flat_delay_and_gain_mismatch_stays_bounded(self):
        for angle in (-180.,-90.,90.,180.):
            for delay in (.3,.45,.6):
                for gain in (.9,1.,1.2):
                    with self.subTest(angle=angle,delay=delay,gain=gain):
                        r,out=simulate(angle,delay=delay,gain=gain)
                        self.assertTrue(r['settled'],r)
                        self.assertLessEqual(abs(r['error_deg']),5.,r)
                        self.assertLess(r['seconds'],12.,r)
                        self.assertLessEqual(max(abs(row[3]) for row in out),.8+1e-9)
                        self.assertTrue(all(abs(b[3]-a[3])<=DT+1e-8 for a,b in zip(out,out[1:])))

    def test_real_jam_still_aborts_with_a_bounded_timeout(self):
        r,_=simulate(90.,jammed=True)
        self.assertEqual((r['state'],r['reason']),('ABORTED','pivot_timeout'))
        self.assertLess(r['seconds'],22.)


if __name__=='__main__':unittest.main()
