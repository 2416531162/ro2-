"""0.5 m/s closed-loop sensitivity study; model outcomes are not field guarantees."""
from pathlib import Path
import argparse,collections,dataclasses,importlib.util,json,math,sys
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parents[1]
sys.path[:0]=[str(SOURCE/'radar_system'),str(SOURCE)]
import line_cascade as current
from heading_control import HeadingControl,HeadingControlConfig

def run(mod,variant,case):
    cfg=mod.ImuFirstConfig(rtk_trim=False,rtk_fit=True,**({'grass_lateral':True} if variant=='after' else {}))
    if variant=='after':cfg=dataclasses.replace(cfg,**case.get('policy',{}))
    g=mod.ImuFirstGuide(cfg);g.offset=0.;g.locked=True;g.offset_n=50;g.source='learned'
    inner=dict(direct_rate_damping=cfg.grass_lateral) if variant=='after' else {}
    if variant=='after':inner.update(case.get('inner',{}))
    core=HeadingControl(HeadingControlConfig(**inner));core.reset(now=0)
    boost=mod.PullBoost(cfg)
    dt=.02;speed=case.get('speed_mps',.5);step=speed*dt;nx,ny=.24,-.4575
    t=0.;tick=0;s=-3.5;y=case.get('initial_cm',1.)/100;yaw=0.;w=0.;target=0.;aligned=False
    sent=collections.deque([(-10.,0.)]);ys=[];commands=[];reference=[];est=[];target_rates=[];ss=[];scales=[]
    rng=np.random.default_rng(case.get('seed',7));noise=0.
    while s<case.get('length',64.):
        nozzle=y+nx*math.sin(yaw)+ny*(math.cos(yaw)-1)
        noise += .10*(rng.normal(0,case.get('random_sigma',.005))-noise)
        period=case.get('noise_period',6.)
        measured=nozzle+noise+case.get('noise_amp',.003)*math.sin(2*math.pi*s/period)
        good=not(case.get('dropout',False) and 15<s<19)
        if case.get('jump',False) and 20<s<21:measured+=.05
        imu_bias=math.radians(case.get('imu_bias',.4)+case.get('drift',.08)*t/60)
        imu=yaw+imu_bias
        gyro=(w+case.get('turn_disturbance',.006)*math.sin(2*math.pi*.28*t+.3)
              +case.get('gyro_vibration',.008)*math.sin(2*math.pi*3.7*t))
        roll=math.radians(case.get('roll',.5)*math.sin(2*math.pi*s/13))
        g.update(measured,imu,step,dt,(nx,ny),good,fast=s<0,roll=roll)
        entry=mod.runup_weight(cfg,s)
        if entry>0 and g.rtk_accepted:g.pull_to(measured,1-math.exp(-step*entry/cfg.runup_rtk_tau_m))
        pull=mod.paint_pull(cfg,max(0,s));boost.update(g.e,g.rtk_accepted and g.mode=='rtk',step);pull=boost.limit(pull)
        old_target=target
        target,bias,_,aligned=mod.imu_first_target(cfg,g,target,0.,dt,3.,1.,aligned,pull=pull,entry_weight=entry)
        tr=mod.wrap(target-old_target)/dt
        feedforward=tr if variant!='after' or case.get('feedforward',True) else 0.
        cmd=core.step(t,mod.wrap(imu-target),gyro,sample_t=t,moving=True,dt=dt,target_rate=feedforward,
                      adaptation_ok=g.rtk_accepted)
        if variant=='after' and hasattr(mod,'direct_rate'):
            cmd=mod.direct_rate(g,imu,gyro,dt)
        sent.append((t,cmd))
        delay=case.get('delay_after',case['delay']) if s>=case.get('change_at_m',32.) else case['delay']
        while len(sent)>1 and sent[1][0]<=t-delay:sent.popleft()
        gain=case.get('gain_after',case['gain']) if s>=case.get('change_at_m',32.) else case['gain']
        target_w=gain*sent[0][1]
        w+=(1-math.exp(-dt/case.get('tau',.12)))*(target_w-w)
        # Separate physical turning disturbance from sensor vibration.
        physical_w=w+case.get('turn_disturbance',.006)*math.sin(2*math.pi*.28*t+.3)
        lateral=case.get('slip',.003)*math.sin(2*math.pi*s/case.get('slip_period',18))
        if case.get('step_slip',False):lateral+=.003 if 12<s<28 else (-.003 if 28<=s<44 else 0)
        if case.get('bump_cm') and case.get('bump_from',10.)<=s<case.get('bump_from',10.)+case.get('bump_length',1.):
            lateral+=case['bump_cm']/100*speed/case.get('bump_length',1.)
        y+=speed*math.sin(yaw)*dt+lateral*dt;yaw+=physical_w*dt
        if s>=0:
            ss.append(s);ys.append(nozzle);commands.append(cmd);est.append(g.e);reference.append(target);target_rates.append(tr)
            scales.append(core.response_scale)
        tick+=1;s=-3.5+tick*step;t+=dt
    a=np.array(ys);c=np.array(commands);h=np.unwrap(reference)
    return (dict(p95_cm=float(np.percentile(abs(a),95)*100),rms_cm=float(np.sqrt(np.mean(a*a))*100),
        max_cm=float(np.max(abs(a))*100),swing_cm=float((np.percentile(a,95)-np.percentile(a,5))*100),
        command_rms=float(np.sqrt(np.mean(c*c))),target_rate_max_deg_s=float(np.max(np.abs(target_rates))*180/math.pi),
        response=core.response.status() if core.response is not None else None),
        dict(s=ss,e=ys,estimate=est,cmd=commands,target=reference,response_scale=scales))
