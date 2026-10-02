"""手动 IMU 维持测试：真实融合的观测隔离、5 秒边界、任务身份与中止恢复。"""
import json
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'radar_system'), ROOT]
import app_link
import gnss_fusion as gf
from line_mission import RTK_HOLD_S, LineMission
from line_tracker import LineConfig, LineTracker
from robot_core.kinematics import ChassisGeometry


def mission(**config):
    f = gf.GnssFusion(gf.FusionConfig(**dict(dict(drive_differential=True, yaw_input='angle', rtk_hold_s=RTK_HOLD_S), **config)))
    f.set_origin(30., 120.)
    m = LineMission(f, gf, tracker=LineTracker(LineConfig(), ChassisGeometry(drive='differential')), nozzle_x=.3)
    for i in range(61):
        sensors(m, i*.05)
    ok, message = m.command(dict(cmd='route_start', route=dict(kind='path', points=[[0, 0], [30, 0]], cruise=.1)), 3.)
    assert ok, message
    m.tick(3., 'NAVIGATION')
    assert m.phase == 'RUNNING' and m.tracker.st.state == 'RUNNING'
    return m


def sensors(m, t, speed=0., yaw=0., imu=True, gnss=True, offset=0.):
    m.on_wheel(t, speed, 0.)
    if imu:
        m.on_imu(t, 0., yaw=yaw)
    if gnss:
        ax, ay = m.fusion.config.antenna_xy
        lat, lon = m.fusion.to_geodetic(ax, ay+offset)
        m.on_gnss(dict(kind='heading', t=t, heading=180., pitch=0., length=.41, sol='NARROW_INT', sigma_heading=.1, sats=30, valid=True))
        m.on_gnss(dict(kind='fix', t=t, lat=lat, lon=lon, height=0., sol='NARROW_INT', sigma_n=.01,
                       sigma_e=.01, sigma_u=.02, diff_age=1., sats=30))


def command(m, **changes):
    return dict(dict(cmd='route_imu_hold', id='hold1', route_rev=m.route_rev, job_id=m.operations.record['id']), **changes)


def test_wire_requires_task_identity_and_id_and_preserves_status():
    raw = dict(type='route', action='imu_hold', id='hold1', route_rev=1, job_id='job1')
    assert app_link.parse_command(json.dumps(raw)) == raw
    for key in ('route_rev', 'job_id', 'id'):
        bad = dict(raw)
        bad.pop(key)
        with pytest.raises(app_link.CommandError):
            app_link.parse_command(json.dumps(bad))
    m = mission()
    assert m.command(command(m), 3.)[0]
    status = dict(m.status(), _rx_monotonic=3.)
    assert app_link.line_telemetry(status, 3.)['imu_hold_test']['state'] == 'running'


def test_five_second_hold_ignores_all_gnss_and_returns_to_rtk():
    m = mission()
    cmd = command(m)
    original_route = m.route
    assert m.command(cmd, 3.)[0]
    end = m._imu_hold_test['until']
    assert m.command(cmd, 4.)[0]  # 相同指令不会续时。
    assert not m.command(command(m, id='hold2'), 4.)[0]
    assert m._imu_hold_test['until'] == end == 8.
    assert m.status()['imu_hold_test']['duration_s'] == 5.
    for i in range(1, 99):
        t = 3. + i*.05
        sensors(m, t, offset=5.)
        m.on_gnss(dict(kind='vel', t=t, speed=100., track=0., sol='DOPPLER_VELOCITY'))
        m.on_gnss(dict(kind='fix2', t=t, sol='NARROW_INT', lat=30., lon=120.))
        m.tick(t, 'NAVIGATION')
    assert m.fusion._last[2:5] == [3., 3., -math.inf]
    assert abs(m.pose.y) < .01
    assert m.pose.quality == 'DEAD_RECKONING' and 'imu_hold_test' in m.pose.reasons
    assert m.status()['imu_hold_test']['remaining_s'] == pytest.approx(.1)
    sensors(m, 7.99, gnss=False)
    m.tick(7.99, 'NAVIGATION')
    assert m.status()['imu_hold_test']['state'] == 'running'  # 5 秒前仍维持，不能提前结束。
    sensors(m, 8.)
    m.tick(8., 'NAVIGATION')
    assert m.status()['imu_hold_test']['state'] == 'done'
    assert m.pose.quality == 'RTK_FIXED'
    assert 'imu_hold_test' not in m.pose.reasons
    assert m.route == original_route
    before = m.fusion._last[:]
    sensors(m, 7.9, gnss=True, imu=False, offset=5.)  # 测试期的迟到数据不能插回历史。
    assert m.fusion._last[2:] == before[2:]


@pytest.mark.parametrize('change', [dict(route_rev=99), dict(job_id='old-job')])
def test_stale_task_cannot_begin_test(change):
    m = mission()
    assert not m.command(command(m, **change), 3.)[0]
    assert m._imu_hold_test is None


@pytest.mark.parametrize('mode', ['IDLE', 'ESTOP', 'MANUAL'])
def test_loss_of_control_cancels_test_and_restores_observations(mode):
    m = mission()
    assert m.command(command(m), 3.)[0]
    sensors(m, 3.05)
    m.tick(3.05, mode)
    assert m.status()['imu_hold_test']['state'] == 'cancelled'
    sensors(m, 3.1)
    assert m.fusion._last[3] == 3.1


@pytest.mark.parametrize('kind', ['stop', 'route_pause'])
def test_operator_stop_or_pause_cancels_test(kind):
    m = mission()
    assert m.command(command(m), 3.)[0]
    assert m.command(command(m, cmd=kind, id='stop1'), 3.05)[0]
    assert m.status()['imu_hold_test']['state'] == 'cancelled'
    sensors(m, 3.1)
    assert m.fusion._last[3] == 3.1


def test_imu_outage_still_pauses_and_cancels_test():
    m = mission()
    assert m.command(command(m), 3.)[0]
    for i in range(1, 15):
        t = 3. + i*.05
        sensors(m, t, imu=False)
        m.tick(t, 'NAVIGATION')
    assert m.status()['imu_hold_test']['state'] == 'cancelled'


def test_existing_error_budget_can_end_test_before_five_seconds():
    m = mission(hold_budget_enabled=True, hold_lateral_limit_m=.01, hold_along_limit_m=.01)
    assert m.command(command(m), 3.)[0]
    sensors(m, 3.05)
    m.tick(3.05, 'NAVIGATION')
    assert m._hold_decision.stop
    assert m.status()['imu_hold_test']['state'] == 'cancelled'


def test_idle_missing_imu_or_unsupported_configuration_cannot_start():
    m = mission()
    m.phase = 'IDLE'
    assert not m.command(command(m), 3.)[0]
    m.phase = 'RUNNING'
    m._imu = None
    assert not m.command(command(m), 3.)[0]
    m = mission(rtk_hold_s=4.)
    assert not m.command(command(m), 3.)[0]
    assert m.status()['imu_hold_test'] is None
