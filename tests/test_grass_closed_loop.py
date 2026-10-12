"""Integrated PathTracker + delayed chassis. Scores physical nozzle geometry, not guide.e."""
from collections import deque
from dataclasses import replace
import math,random,statistics,sys,unittest
from pathlib import Path as FilePath
from types import SimpleNamespace as Pose

ROOT=FilePath(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path,PathConfig,PathTracker,differential_config
from robot_core.kinematics import ChassisGeometry

def run(enabled,delay=.44,gain=1.,seed=31,length=35.,slip=.003,cruise=.5,run_out=0.):
    cfg=replace(differential_config(PathConfig(cruise_mps=cruise)),grass_lateral_enabled=enabled)
    geo=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)
    tracker=PathTracker(Path((0.,0.,0.),[(length+3.5+run_out,0.)]),cfg,geo)
    tracker.constant_straight_speed=True;tracker.paint_from_m=3.5
    nx,ny=.24,-.4575;tracker.set_nozzle(nx,ny)
    x,y,yaw,v,w,t=0.,.003,0.,0.,0.,0.
    dt=.02;sent=deque([(-10.,0.,0.)]);rng=random.Random(seed);noise=0.
    rows=[];offset=1.7;bias=math.radians(.35 if seed%2 else -.35)
    def pose():
        return Pose(x=x,y=y+noise,yaw=yaw+bias,speed=v,yaw_rate=w,usable_for_control=True,
                    quality='RTK_FIXED',reasons=(),correction_xy=(0.,0.),correction_yaw=0.,rtk_trusted=True)
    tracker.sync(t,(t,yaw+offset,0.));assert tracker.start(pose())
    while t<max(180.,length/cruise+70.) and tracker.st.state=='RUNNING':
        noise+=.1*(rng.gauss(0,.003)-noise)
        disturbance=.006*math.sin(2*math.pi*.28*t+.3)
        tracker.sync(t,(t,yaw+offset,w+disturbance+.008*math.sin(2*math.pi*3.7*t)))
        tracker.set_heading_baseline(t,.41)
        tracker.set_rtk_quality(t,.009,32,.003,True)
        vc,wc=tracker.update(pose(),dt)
        sent.append((t,vc,wc))
        while len(sent)>1 and sent[1][0]<=t-delay:sent.popleft()
        _,want_v,want_w=sent[0]
        v+=(1-math.exp(-dt/.10))*(want_v-v)
        w+=(1-math.exp(-dt/.12))*(gain*want_w-w)
        y+=(v*math.sin(yaw)+slip*math.sin(2*math.pi*x/18))*dt
        x+=v*math.cos(yaw)*dt;yaw+=(w+disturbance)*dt;t+=dt
        if 3.5<=x<=length+3.5:
            rows.append((x-3.5,y+nx*math.sin(yaw)+ny*(math.cos(yaw)-1),vc,wc))
    es=sorted(abs(r[1]) for r in rows);signed=sorted(r[1] for r in rows)
    metrics=dict(state=tracker.st.state,p95_cm=100*es[int(.95*(len(es)-1))],
                 swing_cm=100*(signed[int(.95*(len(signed)-1))]-signed[int(.05*(len(signed)-1))]),
                 max_cm=100*max(es),damping=tracker._straight_core.cfg.direct_rate_damping)
    return metrics,rows

class IntegratedGrass(unittest.TestCase):
    def test_real_tracker_configuration_and_ground_geometry_improve(self):
        for delay,gain in ((.25,.8),(.44,1.),(.65,1.2)):
            with self.subTest(delay=delay,gain=gain):
                before,_=run(False,delay,gain);after,_=run(True,delay,gain)
                self.assertEqual(before['state'],'DONE');self.assertEqual(after['state'],'DONE')
                self.assertTrue(after['damping']);self.assertFalse(before['damping'])
                self.assertLess(after['p95_cm'],before['p95_cm'])
                self.assertLess(after['swing_cm'],before['swing_cm'])

    def test_higher_speed_keeps_original_trajectory(self):
        before,a=run(False,cruise=.8,length=15.)
        after,b=run(True,cruise=.8,length=15.)
        self.assertEqual(a,b)
        self.assertFalse(after['damping'])

if __name__=='__main__':unittest.main()
