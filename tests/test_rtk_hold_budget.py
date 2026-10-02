"""推算维持的喷头协方差、误差余量、停车预测及控制器限速；不连接车辆。"""
import dataclasses
import math
import os
import sys
from types import SimpleNamespace

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), os.path.join(ROOT, 'tests'), ROOT]
import gnss_fusion as gf
from rtk_hold_budget import evaluate_hold
from line_mission import LineMission
from line_tracker import LineConfig, LineTracker
from path_tracker import Path, PathConfig, PathTracker, Section, SectionTracker
from line_cascade import CascadeConfig, ImuLineTracker
from robot_core.kinematics import ChassisGeometry
from test_line_mission import FakeFusion
from test_rtk_hold import GEN5, simulate


def config(**kwargs):
    return gf.FusionConfig(**dict(dict(hold_budget_enabled=True, rtk_hold_s=5.0), **kwargs))


def pose(**kwargs):
    fields = dict(t=2.0, x=1.0, y=0.0, yaw=0.0, speed=0.0, yaw_rate=0.0,
                  sigma_xy=.005, sigma_yaw=math.radians(.1), gyro_bias=0.0,
                  mount_offset=0.0, quality='DEAD_RECKONING', usable_for_control=True,
                  reasons=('rtk_hold',), covariance_xy_yaw=(.000025, 0., 0., .000025, 0., math.radians(.1)**2),
                  hold_age_s=1.0, hold_distance_m=.4, hold_turn_rad=0., bias_sigma_rad_s=0.0)
    fields.update(kwargs)
    return gf.FusedPose(**fields)


def evaluate(p=None, cfg=None, **kwargs):
    options = dict(nozzle_x=.3, route_yaw=0., cruise_mps=.8, speed_peak=0., turn_peak=0.,
                   delay_s=0., decel_mps2=.5, angular_decel=1.0)
    options.update(kwargs)
    return evaluate_hold(p or pose(), cfg or config(), **options)


def purely_random(**kwargs):
    return config(**dict(dict(hold_wheel_scale_error=0., hold_slip_per_m=0., hold_slip_mps=0.,
                             hold_turn_slip_m_per_rad=0., hold_turn_scale_error=0.,
                             hold_sigma_multiplier=1., hold_lateral_limit_m=1., hold_along_limit_m=1.), **kwargs))


def rotate_covariance(p, angle):
    xx, xy, xh, yy, yh, hh = p
    c, s = math.cos(angle), math.sin(angle)
    return (c*c*xx-2*c*s*xy+s*s*yy, c*s*(xx-yy)+(c*c-s*s)*xy, c*xh-s*yh,
            s*s*xx+2*c*s*xy+c*c*yy, s*xh+c*yh, hh)


def test_nozzle_covariance_keeps_cross_terms_and_is_rotation_invariant():
    original = (.0001, .00002, .00001, .0004, -.00005, .0001)
    expected = math.sqrt(.0004 + 2*.3*(-.00005) + .3*.3*.0001)
    for heading in (0., .7, math.pi/2, math.pi, 4.2):
        p = pose(yaw=heading, covariance_xy_yaw=rotate_covariance(original, heading))
        result = evaluate(p, purely_random(), route_yaw=heading)
        assert result.lateral_m == pytest.approx(expected)
        assert result.along_m == pytest.approx(.01)
        reverse = evaluate(p, purely_random(), route_yaw=heading+math.pi)
        assert reverse.lateral_m == pytest.approx(result.lateral_m)
        assert reverse.along_m == pytest.approx(result.along_m)


def test_side_nozzle_yaw_uncertainty_affects_along_position():
    p = pose(covariance_xy_yaw=(0.,0.,0.,0.,0.,.01), hold_distance_m=0.)
    result = evaluate(p, purely_random(), nozzle_x=.3, nozzle_y=-.4)
    assert result.lateral_m == pytest.approx(.03)
    assert result.along_m == pytest.approx(.04)


def test_wheel_scale_error_uses_distance_and_small_covariance_cannot_hide_it():
    p = pose(covariance_xy_yaw=(0.,)*6, hold_distance_m=6.4)
    cfg = purely_random(hold_wheel_scale_error=.02)
    result = evaluate(p, cfg, nozzle_x=0.)
    assert result.along_m == pytest.approx(.128)
    assert result.lateral_m == pytest.approx(0.)
    assert evaluate(p, dataclasses.replace(cfg, hold_along_limit_m=.10)).stop


def test_accumulated_turn_and_slip_increase_budget_even_without_random_noise():
    straight = evaluate(pose(covariance_xy_yaw=(0.,)*6), config())
    turning = evaluate(pose(covariance_xy_yaw=(0.,)*6, hold_turn_rad=1.0), config())
    assert turning.lateral_m > straight.lateral_m
    assert turning.along_m > straight.along_m


def test_stop_forecast_pauses_before_current_estimate_crosses_limit():
    result = evaluate(speed_peak=.8, delay_s=.45)
    assert result.lateral_m < .03 and result.along_m < .10
    assert result.stop_along_m >= .10 or result.stop_lateral_m >= .03
    assert result.stop and result.reason == 'rtk_hold_budget_exceeded'
    assert result.stop_distance_m == pytest.approx(.8*.45 + .8*.8/(2*.5))


def test_speed_cap_starts_before_braking_budget_is_exhausted():
    cfg = config(hold_lateral_limit_m=1., hold_along_limit_m=1.)
    first = evaluate(cfg=cfg, speed_peak=.5, delay_s=.4)
    cfg = dataclasses.replace(cfg, hold_along_limit_m=first.stop_along_m/.8)
    limited = evaluate(cfg=cfg, speed_peak=.5, delay_s=.4)
    assert limited.state == 'slow'
    assert .05 < limited.speed_limit_mps < .8
    assert not limited.stop


def test_braking_time_is_reserved_before_five_second_deadline():
    cfg = config(hold_lateral_limit_m=10., hold_along_limit_m=10.)
    result = evaluate(pose(hold_age_s=4.5), cfg, speed_peak=.8, delay_s=.4)
    assert result.stop and result.reason == 'rtk_hold_braking_timeout'
    assert result.age_s < 5. and result.age_s + result.stop_time_s > 5.


def test_old_config_and_trusted_rtk_do_not_limit_speed():
    assert evaluate(cfg=gf.FusionConfig(rtk_hold_s=8.)) is None
    assert evaluate(pose(reasons=(), quality='RTK_FIXED')) is None


def test_missing_or_invalid_uncertainty_fails_closed():
    assert evaluate(pose(covariance_xy_yaw=())).reason == 'rtk_hold_budget_missing'
    assert evaluate(pose(covariance_xy_yaw=(math.nan,)*6)).stop
    assert evaluate(pose(covariance_xy_yaw=(-1.,0.,0.,0.,0.,0.))).stop
    legacy = SimpleNamespace(reasons=('rtk_hold',))
    assert evaluate(legacy).reason == 'rtk_hold_budget_missing'


def test_fusion_hold_statistics_survive_query_and_delayed_replay():
    cfg = dict(GEN5, hold_budget_enabled=True)
    f, rows = simulate(cfg, T=14., fixes_on=lambda t: not 10. <= t < 13.)
    held = next(p for t, _, _, p in rows if t >= 12.)
    assert held.hold_age_s > 2.
    assert held.hold_distance_m > 1.5
    assert len(held.covariance_xy_yaw) == 6
    latest = f.pose(rows[-1][0]); assert latest.hold_age_s < .1
    assert f.pose(rows[-1][0]) == latest
    # 迟到的数据按测量时刻回放，统计不会把已推过的路程累计两次。
    ordered, normal = simulate(cfg, T=14.)
    delayed, burst = simulate(cfg, T=14., stall=(10.,10.7))
    a, b = normal[-1][3], burst[-1][3]
    assert a.hold_distance_m == pytest.approx(b.hold_distance_m, abs=1e-8)
    assert a.hold_turn_rad == pytest.approx(b.hold_turn_rad, abs=1e-8)
    # 互检依到达次序存入观测；延迟的副天线可能产生很小的协方差差异，最终预算应等价到 0.02 mm。
    # 最后已恢复固定解；强制按维持状态比较同一份协方差的投影。
    ca = evaluate(dataclasses.replace(a, reasons=('rtk_hold',)), gf.FusionConfig(**cfg))
    cb = evaluate(dataclasses.replace(b, reasons=('rtk_hold',)), gf.FusionConfig(**cfg))
    assert ca.lateral_m == pytest.approx(cb.lateral_m, abs=.00002)
    assert ca.along_m == pytest.approx(cb.along_m, abs=.00002)


def test_reverse_travel_does_not_cancel_hold_distance():
    f = gf.GnssFusion(gf.FusionConfig(yaw_input='angle', drive_differential=True, rtk_hold_s=8.))
    f.set_origin(30.,120.)
    f.add_wheel(gf.WheelSample(0.,0.,0.)); f.add_imu(gf.ImuSample(0.,0.,0.))
    f.add_heading(gf.DualHeading(0.,180.,0.,.41,'L1_INT',.1,30))
    lat,lon=f.to_geodetic(.345,.165)
    f.add_fix(gf.GnssFix(0.,lat,lon,0.,'NARROW_INT',.01,.01,.02,1.,30))
    for i in range(1,101):
        t=i*.02
        f.add_wheel(gf.WheelSample(t,.5 if i<=50 else -.5,0.))
        f.add_imu(gf.ImuSample(t,0.,0.))
    p=f.pose(2.)
    assert p.hold_distance_m > .95
    assert abs(p.x) < .02


@pytest.mark.parametrize('controller', ['line', 'path', 'reverse', 'imu'])
def test_each_controller_obeys_cap_below_its_minimum_and_restores_speed(controller):
    geo = ChassisGeometry(drive='differential', diff_min_radius_m=.5)
    p = pose(reasons=(), quality='RTK_FIXED')
    if controller == 'line':
        tr = LineTracker(LineConfig(cruise_mps=.8), geo); tr.set_line((0.,0.),(20.,0.))
    elif controller == 'path':
        tr = PathTracker(Path((0.,0.,0.), [(20.,0.)]), PathConfig(cruise_mps=.8, delay_s=0.), geo)
    elif controller == 'reverse':
        tr = SectionTracker([Section(-1, Path((0.,0.,math.pi), [(20.,0.)]))],
                            PathConfig(cruise_mps=.8, delay_s=0.), geo)
    else:
        tr = ImuLineTracker(CascadeConfig(cruise_mps=.8), geo); tr.set_line((0.,0.),(20.,0.)); tr.sync(2.,(2.,0.,0.))
    if controller == 'reverse': p = dataclasses.replace(p, x=-1.)
    assert tr.start(p)
    tr.speed_limit_mps = .02
    for i in range(30):
        if controller == 'imu': tr.sync(2.+i*.05,(2.+i*.05,0.,0.))
        vx, _ = tr.update(p,.05)
        assert abs(vx) <= .02 + 1e-12
    tr.speed_limit_mps = None
    for i in range(30):
        if controller == 'imu': tr.sync(4.+i*.05,(4.+i*.05,0.,0.))
        vx, _ = tr.update(p,.05)
    assert abs(vx) > .02


def test_mission_brakes_for_pending_commands_and_does_not_resume_during_hold():
    fusion = FakeFusion()
    fusion.config = config(hold_lateral_limit_m=1., hold_along_limit_m=1.)
    fusion.current = pose(hold_age_s=7., speed=.05)
    geo = ChassisGeometry(drive='differential', diff_min_radius_m=.5)
    tr = LineTracker(LineConfig(cruise_mps=.8), geo); tr.set_line((0.,0.),(20.,0.))
    m = LineMission(fusion, gf, tracker=tr, nozzle_x=.3)
    assert tr.start(fusion.current)
    m.phase = 'RUNNING'; m.cruise = .8
    m._sent_motion.append((1.9, .8, 0.))
    vx, wz, _ = m.tick(2., 'NAVIGATION')
    assert (vx,wz) == (0.,0.) and tr.st.state == 'PAUSED'
    assert 'rtk_hold_braking_timeout' in m.pose.reasons
    assert m.status()['hold_budget']['stop_distance_m'] > .6
    fusion.current = pose(hold_age_s=7.1, speed=0.)
    m.tick(3., 'NAVIGATION'); assert tr.st.state == 'PAUSED'
    # 新鲜、已确认的 RTK 恢复后，正常的连续可用检查允许继续，限速解除。
    fusion.current = pose(reasons=(), quality='RTK_FIXED', speed=0., hold_age_s=0.)
    m.tick(4., 'NAVIGATION'); m.tick(5.1, 'NAVIGATION')
    assert tr.st.state == 'RUNNING' and tr.speed_limit_mps is None
    assert m.status()['hold_budget'] is None


def test_config_accepts_zero_model_margins_but_rejects_invalid_limits():
    assert purely_random().hold_budget_enabled
    for values in ({'hold_slow_ratio':1.}, {'hold_lateral_limit_m':0.},
                   {'hold_wheel_scale_error':-0.1}, {'hold_turn_scale_error':1.1},
                   {'hold_budget_enabled':1}):
        with pytest.raises(ValueError): config(**values)


def run_outage(cruise=.8, wheel_error=.02, outage_end=15.):
    """真实融合+任务控制，20 Hz GNSS、50 Hz IMU/轮速；底盘延迟 0.35 s、减速 0.5 m/s²。

    真值独立积分，轮速带比例误差。6 s 后断 RTK，不用融合器自身输出构造真值。
    """
    cfg=config(drive_differential=True, yaw_input='angle', antenna_xy=(.24,.2075),
               yaw_walk_deg_sqrt_s=.03, yaw_scale_sigma=.03)
    f=gf.GnssFusion(cfg); f.set_origin(30.,120.)
    geo=ChassisGeometry(drive='differential', diff_min_radius_m=.5)
    tr=LineTracker(LineConfig(cruise_mps=cruise, lead_s=0.),geo)
    tr.set_line((0.,0.),(30.,0.))
    m=LineMission(f,gf,tracker=tr,nozzle_x=.3); m.cruise=cruise
    x=y=yaw=v=w=0.; commands=[(-1.,0.,0.)]; applied=0; records=[]
    for i in range(1601):
        t=i*.01
        while applied+1<len(commands) and commands[applied+1][0] <= t-.35+1e-9:
            applied+=1
        target_v,target_w=commands[applied][1:]
        v+=max(-.5*.01,min(.5*.01,target_v-v))
        w+=max(-1.*.01,min(1.*.01,target_w-w))
        x+=v*.01*math.cos(yaw+w*.005); y+=v*.01*math.sin(yaw+w*.005); yaw+=w*.01
        if i%2==0:
            m.on_wheel(t,v*(1+wheel_error),w); m.on_imu(t,w,yaw)
        if i%5==0 and not 6.<=t<outage_end:
            ax,ay=cfg.antenna_xy; c,s=math.cos(yaw),math.sin(yaw)
            lat,lon=f.to_geodetic(x+c*ax-s*ay,y+s*ax+c*ay)
            m.on_gnss(dict(kind='heading',t=t,heading=(180.-math.degrees(yaw))%360.,pitch=0.,length=.41,
                           sol='L1_INT',sigma_heading=.1,sats=30,valid=True))
            m.on_gnss(dict(kind='fix',t=t,lat=lat,lon=lon,height=0.,sol='NARROW_INT',sigma_n=.01,
                           sigma_e=.01,sigma_u=.02,diff_age=1.,sats=30))
        if i%5==0 and t>=3.:
            if t==3.:
                assert tr.start(f.pose(t)); m.phase='RUNNING'
            vx,wz,_=m.tick(t,'NAVIGATION'); commands.append((t,vx,wz))
            p=m.pose
            if p is not None:
                px,py=p.x+.3*math.cos(p.yaw),p.y+.3*math.sin(p.yaw)
                records.append(dict(t=t,actual_speed=v,command=vx,along_error=px-(x+.3*math.cos(yaw)),
                                    lateral_error=py-(y+.3*math.sin(yaw)),
                                    tracker=tr.st.state,budget=m.status()['hold_budget']))
    return records


@pytest.mark.parametrize('speed,bias',[(.4,.02),(.8,.02),(.8,-.02)])
def test_real_fusion_outage_slowdown_and_physical_stop_fit_budget(speed,bias):
    rows=run_outage(speed,bias)
    held=[r for r in rows if r['budget'] is not None and r['t']<15.]
    slow=[r for r in held if r['budget']['state']=='slow']
    stopped=[r for r in held if r['budget']['state']=='stop']
    assert stopped
    if speed==.4:
        assert slow and slow[0]['t']<stopped[0]['t']
    assert stopped[0]['t']<11.
    assert all(r['command']==0. for r in rows if stopped[0]['t']<=r['t']<15.)
    physical=next(r for r in rows if r['t']>=stopped[0]['t'] and r['actual_speed']<.001)
    assert physical['t']<11. # 从最后可信 RTK 起算，停稳发生在 5 s 上限之前。
    assert abs(physical['lateral_error'])<.03
    assert abs(physical['along_error'])<.10
    print('HOLD_SIM speed=%g wheel_bias=%g slow_at=%s stop_command=%.2fs stopped=%.2fs actual_error=(%.1f,%.1f)mm' %
          (speed,bias,'%.2fs'%(slow[0]['t']-6.) if slow else 'immediate-braking',stopped[0]['t']-6.,physical['t']-6.,
           physical['lateral_error']*1000,physical['along_error']*1000))


def test_short_outage_recovers_and_releases_speed_cap():
    rows=run_outage(.8,.02,outage_end=6.1)
    during=[r for r in rows if 6.<=r['t']<6.1]
    assert all(r['tracker']=='RUNNING' for r in during)
    recovered=[r for r in rows if 7.<=r['t']<10.]
    assert all(r['budget'] is None for r in recovered)
    assert recovered[-1]['command'] == pytest.approx(.8)
