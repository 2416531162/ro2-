"""独立阿克曼真值、异常输入、延迟回放和一小时流式验收。"""
import ast
import gc
import math
import statistics
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'radar_system'))
from gnss_fusion import (DualHeading, FusedPose, FusionConfig, GnssFix, GnssFusion,
                         GnssVelocity, ImuSample, WheelSample)  # noqa: E402
from tools.fusion_sim import FusionSimulation, Segment, feed, metrics, run_simulation  # noqa: E402

RESULTS = {}


def report(name, values):
    RESULTS[name] = values
    print(name, values)


def angular_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def known_fusion(config=None):
    f = GnssFusion(config or FusionConfig(mount_offset_deg=1.5))
    f.set_origin(31.2304, 121.4737, 12.)
    return f


def fixed_at(f, t, x=0., y=0., yaw=0., sol='NARROW_INT', sigma=.01):
    ax, ay = f.config.antenna_xy
    cs, sn = math.cos(yaw), math.sin(yaw)
    lat, lon = f.to_geodetic(x+cs*ax-sn*ay, y+sn*ax+cs*ay)
    return GnssFix(t, lat, lon, 12., sol, sigma, sigma, 2*sigma, .2, 20)


def heading_at(f, t, yaw=0., **kw):
    heading = 90.-math.degrees(yaw)-f.config.baseline_yaw_body_deg+f.config.mount_offset_deg
    return replace(DualHeading(t, heading % 360, 0., .33, 'L1_INT', .3, 20), **kw)


@pytest.fixture(scope='module')
def normal():
    return run_simulation(FusionSimulation())


def test_straight_twenty_metres(normal):
    fusion, rows = normal
    result = metrics(rows)
    assert result['samples'] == 800
    assert result['yaw_rms_deg'] <= .1
    assert result['yaw_max_deg'] <= .3
    assert result['position_rms_m'] <= .015
    assert all(p.usable_for_control for t, p in rows if t.t >= 3.)
    assert abs(fusion.calibration()['wheel_scale'] - 1/1.02) < .003
    report('straight', result)


def test_heading_outage_ten_seconds():
    _, rows = run_simulation(FusionSimulation(heading_outage=(15., 25.)))
    result = metrics(rows, 15., 25.)
    assert result['yaw_max_deg'] <= .5
    assert all('heading_unavailable' in p.reasons for t, p in rows if 15.4 <= t.t < 25.)
    report('heading_outage', result)


def test_position_outage_and_recovery():
    fusion, rows = run_simulation(FusionSimulation(fix_outage=(15., 17.)))
    result = metrics(rows, 15., 17.)
    assert result['position_max_m'] <= .05
    for truth, p in rows:
        if 15.5 <= truth.t < 17.:
            assert not p.usable_for_control
            assert p.quality == 'DEAD_RECKONING'
            assert 'rtk_fixed_timeout' in p.reasons
    recovery = next(p.t-17. for truth, p in rows if truth.t >= 17. and p.usable_for_control)
    assert recovery <= 1.
    assert fusion.calibration()['position_resets'] == 0
    result['recovery_s'] = recovery
    report('position_outage', result)


def test_bias_converges_at_rest_without_heading_updates():
    f = known_fusion()
    f.add_heading(heading_at(f, 0.))
    f.add_fix(fixed_at(f, 0.))
    rng = np.random.default_rng(20260925)
    for i in range(61):
        t = i*.05
        f.add_wheel(WheelSample(t, 0., 0.))
        f.add_imu(ImuSample(t, math.radians(.1+float(rng.normal(0., .034)))))
    error = abs(math.degrees(f.pose(3.).gyro_bias)-.1)
    assert error <= .01
    report('stationary_bias_error_deg_s', error)


def test_bias_is_observable_while_moving():
    f, rows = run_simulation(FusionSimulation(segments=[Segment(40., .5)], spikes=False))
    assert abs(f.calibration()['gyro_bias_deg_s']-.1) < .01


def test_stationary_spike_cannot_rotate_pose():
    f, reference = known_fusion(), known_fusion()
    for obj in (f, reference):
        obj.add_heading(heading_at(obj, 0.))
        obj.add_fix(fixed_at(obj, 0.))
    largest = 0.
    for i in range(81):
        t = i*.05
        for obj in (f, reference):
            obj.add_wheel(WheelSample(t, 0., 0.))
            raw = math.radians(-3.8 if obj is f and i == 40 else .1)
            obj.add_imu(ImuSample(t, raw))
        largest = max(largest, abs(math.degrees(angular_error(f.pose(t).yaw, reference.pose(t).yaw))))
    assert largest < .02
    assert f.calibration()['rejected']['gyro_spikes'] == 1
    report('spike_yaw_change_deg', largest)


@pytest.mark.parametrize('speed', [.3, .5, 1.3, -.5, -1.3])
def test_physical_maximum_turn_is_not_a_spike(speed):
    f = known_fusion()
    w = speed*math.tan(.35)/.52
    for i in range(20):
        t = i*.05
        f.add_wheel(WheelSample(t, speed, w))
        f.add_imu(ImuSample(t, w+math.radians(.1)))
    assert f.calibration()['rejected']['gyro_spikes'] == 0


@pytest.mark.parametrize('reverse', [False, True])
def test_mount_calibration_reports_without_applying(reverse):
    segments = [Segment(3., 0.), Segment(40., .5)]
    if reverse:
        segments = [Segment(3., 0.), Segment(40., .5), Segment(20., -.5)]
    f, rows = run_simulation(FusionSimulation(segments=segments), FusionConfig(mount_offset_deg=0.))
    result = f.calibration()
    assert abs(result['mount_offset_deg']-1.5) <= .2
    assert result['mount_offset_samples'] > 100
    assert result['mount_offset_confident']
    assert result['mount_offset_ci95_deg'] <= .2
    assert abs(result['wheel_scale']-1/1.02) <= .003
    assert all(p.mount_offset == 0. for _, p in rows)
    assert f.config.mount_offset_deg == 0.
    # 标定未生效时输出仍保留约 1.5° 误差；不能偷偷把在线估计喂回滤波。
    assert metrics(rows)['yaw_rms_deg'] > 1.2
    report('mount_reverse' if reverse else 'mount_forward', result)


def test_bestvel_calibration_and_fallback():
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(40, .5)], spikes=False)
    f = known_fusion(FusionConfig(mount_offset_deg=0.))
    for event in sim.events():
        if event.sensor != 'fix':
            feed(f, event)
    cal = f.calibration()
    assert cal['mount_offset_source'] == 'bestvel'
    assert abs(cal['mount_offset_deg']-1.5) <= max(.2, cal['mount_offset_ci95_deg'])
    fallback = known_fusion()
    for event in sim.events():
        if event.sensor == 'heading':
            continue
        feed(fallback, event)
    pose = fallback.pose(sim.duration)
    assert pose is not None and 'course_heading' in pose.reasons
    assert abs(math.degrees(angular_error(pose.yaw, sim.truth(sim.duration).yaw))) < .5


def test_turning_antenna_velocity_does_not_calibrate_heading():
    sim = FusionSimulation(segments=[Segment(15., .5, .35)], spikes=False)
    f, _ = run_simulation(sim)
    assert f.calibration()['mount_offset_samples'] == 0
    assert f.calibration()['wheel_scale'] is None


def test_position_sequence_alone_supplies_course_heading():
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(30, .5)], spikes=False)
    f = known_fusion()
    for event in sim.events():
        if event.sensor not in ('heading', 'velocity'):
            feed(f, event)
    pose = f.pose(sim.duration)
    assert pose.quality == 'RTK_FIXED'
    assert abs(math.degrees(angular_error(pose.yaw, sim.truth(sim.duration).yaw))) < .5
    assert 'heading_unavailable' in pose.reasons


def test_delayed_and_cross_sensor_reordered_replay_matches_ideal():
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(8, .5), Segment(4, .5, .35), Segment(4, -.5)], spikes=False)
    events = list(sim.events())
    priority = {'wheel': 0, 'imu': 1, 'heading': 2, 'fix': 3, 'velocity': 4}
    # 重放同一批测量，只改变收到顺序，不重新随机生成观测。
    ideal = sorted(events, key=lambda e: (e.sample.t, priority[e.sensor]))
    delayed, ordered = known_fusion(), known_fusion()
    for e in events:
        feed(delayed, e)
    for e in ideal:
        feed(ordered, e)
    errors = []
    for t in np.linspace(sim.duration-.7, sim.duration, 15):
        a, b = delayed.pose(float(t)), ordered.pose(float(t))
        assert a is not None and b is not None
        errors.append(math.hypot(a.x-b.x, a.y-b.y))
        assert abs(angular_error(a.yaw, b.yaw)) < 1e-9
    assert max(errors) <= .005
    ca, cb = delayed.calibration(), ordered.calibration()
    assert ca['mount_offset_samples'] == cb['mount_offset_samples']
    assert ca['segments'] == cb['segments']
    assert ca['mount_offset_deg'] == pytest.approx(cb['mount_offset_deg'], abs=1e-9)
    assert any(a.sample.t > b.sample.t for a, b in zip(events, events[1:]))
    report('delay_position_difference_m', max(errors))


def test_duplicates_reversed_samples_and_history_limit(normal):
    sim = FusionSimulation(segments=[Segment(5., 0.)], spikes=False)
    f = known_fusion()
    events = list(sim.events())
    for event in events:
        feed(f, event)
    before = f.pose(5.)
    cal = f.calibration()
    for event in reversed(events):
        feed(f, event)
    assert f.pose(5.) == before
    after = f.calibration()
    for sensor in ('imu', 'wheel', 'heading', 'fix', 'velocity'):
        assert after['rejected'][sensor] > cal['rejected'][sensor]
    assert after['mount_offset_samples'] == cal['mount_offset_samples']
    assert f.pose(0.) is None
    assert after['history_samples'] <= f.config.history_max_samples


def test_wheel_can_arrive_after_imu_without_changing_result():
    sim = FusionSimulation(segments=[Segment(2, 0), Segment(3, .5, .35), Segment(2, 0)], spikes=False)
    events = list(sim.events())
    changed = sorted([replace(e, arrival=e.arrival+.12) if e.sensor == 'wheel' else e for e in events], key=lambda e: e.arrival)
    f, reference = known_fusion(), known_fusion()
    for e in events:
        feed(reference, e)
    for e in changed:
        feed(f, e)
    a, b = f.pose(7.), reference.pose(7.)
    assert math.hypot(a.x-b.x, a.y-b.y) < 1e-9
    assert abs(angular_error(a.yaw, b.yaw)) < 1e-9


@pytest.mark.parametrize('seed', [11, 23, 47])
def test_all_sensors_can_have_different_bounded_delays(seed):
    sim = FusionSimulation(segments=[Segment(2, 0), Segment(4, .5, .35), Segment(3, -.5)], spikes=False)
    events = list(sim.events())
    rng = np.random.default_rng(seed)
    previous = {k: -1. for k in ('wheel', 'imu', 'heading', 'fix', 'velocity')}
    changed = []
    for event in sorted(events, key=lambda e: e.sample.t):
        arrival = max(previous[event.sensor]+1e-6, event.sample.t+float(rng.uniform(.01, .4)))
        previous[event.sensor] = arrival
        changed.append(replace(event, arrival=arrival))
    f, reference = known_fusion(), known_fusion()
    for e in events:
        feed(reference, e)
    for e in sorted(changed, key=lambda e: e.arrival):
        feed(f, e)
    a, b = f.pose(9.), reference.pose(9.)
    assert math.hypot(a.x-b.x, a.y-b.y) < 1e-9
    assert abs(angular_error(a.yaw, b.yaw)) < 1e-9


def test_ninety_degree_minimum_radius_turn():
    radius = .52/math.tan(.35)+.3
    duration = radius*math.pi/(2*.5)
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(4, .5), Segment(duration, .5, .35), Segment(3, .5)], spikes=False)
    f, rows = run_simulation(sim)
    result = metrics(rows)
    assert result['yaw_max_deg'] <= .5
    assert f.calibration()['rejected']['gyro_spikes'] == 0
    assert math.degrees(sim.truth(7+duration).yaw-sim.initial_yaw) == pytest.approx(90.)
    report('turn', result)


@pytest.mark.parametrize('lat,lon,height', [(0., 0., 0.), (31.2304, 121.4737, 12.), (60., 179.999, 500.), (-80., -179.999, -20.), (89.99, 40., 100.)])
def test_wgs84_roundtrip_and_independent_ecef_reference(lat, lon, height):
    f = GnssFusion()
    f.set_origin(lat, lon, height)
    maximum = 0.
    for angle in np.linspace(-math.pi, math.pi, 17):
        x, y = 1000*math.cos(angle), 1000*math.sin(angle)
        a, b = f.to_geodetic(x, y)
        xx, yy = f.to_local(a, b)
        maximum = max(maximum, math.hypot(x-xx, y-yy))
        # 独立向量化 WGS84 ECEF→ENU，不用模块内的坐标函数算参考值。
        ll = np.radians(np.array([[lat, lon], [a, b]], dtype=np.longdouble))
        sine, cosine = np.sin(ll[:, 0]), np.cos(ll[:, 0])
        flattening = np.longdouble(1)/np.longdouble('298.257223563')
        eccentricity = flattening*(2-flattening)
        n = np.longdouble(6378137)/np.sqrt(1-eccentricity*sine*sine)
        ecef = np.column_stack(((n+height)*cosine*np.cos(ll[:, 1]),
                                (n+height)*cosine*np.sin(ll[:, 1]),
                                (n*(1-eccentricity)+height)*sine))
        p, l = ll[0]
        rot = np.array([[-np.sin(l), np.cos(l), 0.],
                        [-np.sin(p)*np.cos(l), -np.sin(p)*np.sin(l), np.cos(p)]], dtype=np.longdouble)
        oracle = rot @ (ecef[1]-ecef[0])
        assert math.hypot(float(oracle[0])-xx, float(oracle[1])-yy) < .001
    assert maximum < .001
    report('coordinates_'+str(lat), maximum)


@pytest.mark.parametrize('heading,expected', [(90., 90.), (180., 0.), (0., 180.), (270., -90.)])
def test_heading_convention_and_lever_arm(heading, expected):
    f = known_fusion(FusionConfig())
    yaw = math.radians(expected)
    f.add_heading(DualHeading(0., heading, 0., .33, 'L1_INT', .3, 20))
    f.add_fix(fixed_at(f, 0., 12., -5., yaw))
    p = f.pose(0.)
    assert math.degrees(p.yaw) == pytest.approx(expected)
    assert p.x == pytest.approx(12., abs=1e-6)
    assert p.y == pytest.approx(-5., abs=1e-6)


def test_arbitrary_baseline_and_mount_are_not_hardcoded():
    f = known_fusion(FusionConfig(baseline_yaw_body_deg=23., mount_offset_deg=2.))
    f.add_heading(heading_at(f, 0., math.radians(-177.)))
    f.add_fix(fixed_at(f, 0., yaw=math.radians(-177.)))
    assert math.degrees(f.pose(0.).yaw) == pytest.approx(-177.)


@pytest.mark.parametrize('changes', [dict(sol='NARROW_FLOAT'), dict(length_m=float('nan')), dict(sigma_heading_deg=float('nan'))])
def test_invalid_heading_is_never_used(changes):
    f = known_fusion()
    f.add_heading(heading_at(f, 0., **changes))
    f.add_fix(fixed_at(f, 0.))
    p = f.pose(0.)
    assert p.quality == 'INVALID'
    assert not p.usable_for_control
    assert 'heading_uninitialized' in p.reasons


def test_fusion_does_not_judge_heading_length():
    # 天线间距不写死、不实测：测向是否可用由 rtk_node 按两根天线固定解的平面实时距离核对（valid），
    # 融合不再按基线长度卡——整周固定的测向不论测得多长都能初始化航向
    for length in (.28, .33, .9):
        f = known_fusion()
        f.add_heading(heading_at(f, 0., length_m=length))
        f.add_fix(fixed_at(f, 0.))
        assert 'heading_uninitialized' not in f.pose(0.).reasons
    # 浮点测向照样不用
    g = known_fusion()
    g.add_heading(heading_at(g, 0., length_m=.28, sol='NARROW_FLOAT'))
    g.add_fix(fixed_at(g, 0.))
    assert 'heading_uninitialized' in g.pose(0.).reasons
    # 固定基线参数已取消
    for key in ('baseline_m', 'baseline_tol_m', 'baseline_check'):
        with pytest.raises(TypeError):
            FusionConfig(**{key: .33})


def test_heading_floor_and_outlier_gate():
    f = known_fusion()
    for i in range(100):
        t = i*.05
        f.add_wheel(WheelSample(t, 0., 0.))
        f.add_imu(ImuSample(t, math.radians(.1)))
        f.add_heading(heading_at(f, t, sigma_heading_deg=0.))
        f.add_fix(fixed_at(f, t))
    before = f.pose(4.95)
    f.add_heading(heading_at(f, 5., yaw=math.radians(90.), sigma_heading_deg=.1))
    assert f.calibration()['rejected']['heading_outliers'] == 1
    assert abs(math.degrees(angular_error(f.pose(5.).yaw, before.yaw))) < .02


def test_float_noise_and_single_no_position_update():
    f = known_fusion()
    sim = FusionSimulation(segments=[Segment(6, 0)], spikes=False)
    for event in sim.events():
        if event.sensor == 'fix' and event.sample.t >= 5.:
            event = replace(event, sample=replace(event.sample, sol='NARROW_FLOAT', sigma_n=.001, sigma_e=.001))
        feed(f, event)
    p = f.pose(6.)
    assert p.quality == 'RTK_FLOAT' and not p.usable_for_control
    assert 'rtk_fixed_timeout' in p.reasons
    assert p.sigma_xy > .01
    before = (p.x, p.y)
    f.add_fix(fixed_at(f, 6.05, x=100., sol='SINGLE'))
    p = f.pose(6.05)
    assert p.quality == 'DEAD_RECKONING'
    assert math.hypot(p.x-before[0], p.y-before[1]) < .001


def test_recovery_reset_is_visible_and_not_double_counted():
    f = known_fusion()
    sim = FusionSimulation(segments=[Segment(7, 0)], fix_outage=(3, 5), spikes=False)
    reset_poses = []
    for event in sim.events():
        if event.sensor == 'fix' and event.sample.t >= 5:
            event = replace(event, sample=fixed_at(f, event.sample.t, x=.2, yaw=sim.initial_yaw))
        feed(f, event)
        if event.sensor == 'imu':
            p = f.pose(event.sample.t)
            if p and 'position_reset' in p.reasons:
                reset_poses.append(p)
    assert reset_poses and all(not p.usable_for_control for p in reset_poses)
    assert f.calibration()['position_resets'] == 1
    assert f.pose(7.).usable_for_control


def test_normal_position_outlier_cannot_reset_pose():
    f = known_fusion()
    f.add_wheel(WheelSample(0., 0., 0.))
    f.add_imu(ImuSample(0., math.radians(.1)))
    f.add_heading(heading_at(f, 0.))
    f.add_fix(fixed_at(f, 0.))
    f.add_fix(fixed_at(f, .05, x=10.))
    assert f.calibration()['rejected']['position_outliers'] == 1
    assert f.calibration()['position_resets'] == 0
    assert abs(f.pose(.05).x) < .001


def _drive_with_rtk_offset(offset, until=16.):
    """直行 0.5 m/s，固定解按 offset(t) 向车头左侧偏；返回 (融合, [(t, 侧向误差, 位姿)])。"""
    sim = FusionSimulation(segments=[Segment(3., 0.), Segment(until-3., .5)])
    f = GnssFusion(sim.config)
    f.set_origin(31.2304, 121.4737, 12.)
    out = []
    for event in sim.events():
        if event.sensor == 'fix' and offset(event.sample.t):
            d, tr = offset(event.sample.t), sim.truth(event.sample.t)
            event = replace(event, sample=fixed_at(f, event.sample.t, x=tr.x-d*math.sin(tr.yaw),
                                                   y=tr.y+d*math.cos(tr.yaw), yaw=tr.yaw))
        feed(f, event)
        if event.sensor == 'imu':
            p = f.pose(event.sample.t)
            if p:
                tr = sim.truth(p.t)
                out.append((p.t, -(p.x-tr.x)*math.sin(tr.yaw)+(p.y-tr.y)*math.cos(tr.yaw), p))
    return f, out


def test_brief_sideways_rtk_drift_is_ridden_through():
    # 实车 2026-09-26：固定解 1 s 内侧移 8 cm 又跳回、接收机精度一直报 1 cm；旧融合跟着偏了 8 cm
    f, rows = _drive_with_rtk_offset(lambda t: .08*(t-6.) if 6. <= t <= 7. else 0.)
    assert max(abs(e) for t, e, p in rows if 6. <= t <= 9.) < .015
    assert all(p.usable_for_control for t, e, p in rows if t >= 4.)
    assert f.calibration()['position_resets'] == 0


def test_small_persistent_rtk_offset_is_absorbed_without_jumps():
    f, rows = _drive_with_rtk_offset(lambda t: .04 if t >= 6. else 0.)
    late = [e for t, e, p in rows if t >= 14.]
    assert min(late) > .03                                           # 最终跟上 RTK
    assert max(abs(b[1]-a[1]) for a, b in zip(rows, rows[1:]) if b[0] >= 6.) < .005      # 每拍不跳
    assert f.calibration()['position_resets'] == 0


def test_large_persistent_rtk_offset_resets_after_outlier_window():
    # 持续偏 30 cm：多半是推算错了（车被推动），outlier_reset_s 后照旧对齐 RTK 并标记重置，控制暂停
    f, rows = _drive_with_rtk_offset(lambda t: .30 if t >= 6. else 0.)
    unusable = [t for t, e, p in rows if t >= 4. and not p.usable_for_control]
    assert unusable and unusable[0] >= 6. + f.config.outlier_reset_s - .1
    assert f.calibration()['position_resets'] == 1
    assert abs(rows[-1][1] - .30) < .01


def test_very_late_reset_is_visible_to_current_controller():
    f = known_fusion()
    f.add_heading(heading_at(f, 0.))
    f.add_fix(fixed_at(f, 0.))
    for i in range(61):
        t = i*.05
        f.add_wheel(WheelSample(t, 0., 0.))
        f.add_imu(ImuSample(t, math.radians(.1)))
    f.add_fix(fixed_at(f, 2.25, x=.2))
    p = f.pose(3.)
    assert 'position_reset' in p.reasons
    assert not p.usable_for_control


def test_covariance_stays_positive_semidefinite():
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(5, .5, .35), Segment(5, -.5, -.35)])
    f = known_fusion()
    for event in sim.events():
        feed(f, event)
        if event.sensor == 'fix':
            xx, xy, xh, xb, yy, yh, yb, hh, hb, bb = f._s.p
            p = np.array([[xx, xy, xh, xb], [xy, yy, yh, yb], [xh, yh, hh, hb], [xb, yb, hb, bb]])
            assert np.linalg.eigvalsh(p)[0] > -1e-12


def test_no_gnss_grows_uncertainty_and_queries_do_not_mutate():
    f = known_fusion()
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(12, .5)], spikes=False)
    before = None
    for event in sim.events():
        if event.sample.t > 5. and event.sensor not in ('imu', 'wheel'):
            continue
        feed(f, event)
        if event.sensor == 'imu' and 4.9 < event.sample.t < 5:
            before = f.pose(event.sample.t)
    p = f.pose(sim.duration)
    assert p.sigma_yaw > before.sigma_yaw
    assert p.sigma_xy > before.sigma_xy
    assert not p.usable_for_control and 'gyro_only' in p.reasons
    cal = f.calibration()
    assert f.pose(sim.duration+.2) is not None
    assert f.pose(sim.duration+.2001) is None
    assert f.pose(sim.duration) == p
    assert f.calibration() == cal


def test_position_cannot_secretly_correct_heading_when_course_is_unavailable():
    sim = FusionSimulation(segments=[Segment(3, 0), Segment(2, .5), Segment(5, .5, .35)], heading_outage=(5, 11), spikes=False)
    f, without_position = known_fusion(), known_fusion()
    for event in sim.events():
        if event.sensor == 'velocity' and event.sample.t >= 5:
            continue
        feed(f, event)
        if event.sensor != 'fix' or event.sample.t < 5:
            feed(without_position, event)
    p, q = f.pose(10.), without_position.pose(10.)
    assert 'gyro_only' in p.reasons
    assert abs(angular_error(p.yaw, q.yaw)) < 1e-12
    assert p.sigma_yaw == pytest.approx(q.sigma_yaw, abs=1e-12)


def test_control_limits_and_stale_inputs():
    f = known_fusion(FusionConfig(mount_offset_deg=1.5, control_sigma_xy_m=.001, control_sigma_yaw_deg=.01))
    f.add_heading(heading_at(f, 0.))
    f.add_fix(fixed_at(f, 0.))
    p = f.pose(0.)
    assert not p.usable_for_control
    assert {'position_uncertain', 'heading_uncertain', 'imu_stale', 'wheel_stale'} <= set(p.reasons)


def test_origin_is_first_good_fix_and_no_ros_import():
    f = GnssFusion()
    bad = GnssFix(0., 31., 121., 5., 'SINGLE', .01, .01, .01, 0., 20)
    f.add_fix(bad)
    assert f.pose(0.) is None
    with pytest.raises(ValueError):
        f.to_local(31., 121.)
    f.add_fix(replace(bad, t=1., sol='NARROW_INT'))
    assert f.to_local(31., 121.) == (0., 0.)
    code = "import sys; sys.path.insert(0, 'radar_system'); import gnss_fusion; assert 'rclpy' not in sys.modules"
    subprocess.run([sys.executable, '-I', '-c', code], cwd=ROOT, check=True, timeout=10)
    for name in ('gnss_fusion.py', 'tools/fusion_sim.py'):
        ast.parse((ROOT/'radar_system'/name).read_text(), feature_version=(3, 10))


def test_sample_processing_median_below_100_microseconds():
    events = list(FusionSimulation(segments=[Segment(3, 0), Segment(12, .5)]).events())
    f = known_fusion()
    timings = {k: [] for k in ('imu', 'wheel', 'heading', 'fix', 'velocity')}
    for event in events:
        start = time.perf_counter_ns()
        feed(f, event)
        elapsed = (time.perf_counter_ns()-start)/1000.
        if event.sample.t > 3.:
            timings[event.sensor].append(elapsed)
    medians = {k: statistics.median(v) for k, v in timings.items()}
    assert max(medians.values()) < 100., medians
    report('median_us_by_sensor', medians)


def _retained_size(root):
    seen = set()
    def size(obj):
        if id(obj) in seen:
            return 0
        seen.add(id(obj))
        total = sys.getsizeof(obj)
        if isinstance(obj, dict):
            total += sum(size(k)+size(v) for k, v in obj.items())
        elif isinstance(obj, (list, tuple)):
            total += sum(size(v) for v in obj)
        elif hasattr(obj, '__dict__'):
            total += size(obj.__dict__)
        return total
    return size(root)


def test_one_hour_bounded_history_and_memory():
    # 不先生成一小时列表，计量滤波器真正保留的整个对象图，避免 RSS 缓存干扰。
    sim = FusionSimulation(segments=[Segment(3., 0.), Segment(3597., .5)], spikes=False)
    f = known_fusion()
    sizes, checkpoints, last_checkpoint = [], [], 0
    count = 0
    for event in sim.events():
        feed(f, event)
        count += 1
        checkpoint = int(event.sample.t // 300)
        if checkpoint > last_checkpoint:
            last_checkpoint = checkpoint
            gc.collect()
            sizes.append(_retained_size(f))
            checkpoints.append(f.calibration()['history_samples'])
    assert count >= 359000
    assert max(checkpoints) <= f.config.history_max_samples
    assert max(sizes)-min(sizes) < 64*1024
    assert sizes[-1] <= sizes[0]+32*1024
    report('one_hour', {'samples': count, 'retained_bytes_min': min(sizes),
                        'retained_bytes_max': max(sizes), 'retained_bytes_last': sizes[-1],
                        'history_max': max(checkpoints)})
