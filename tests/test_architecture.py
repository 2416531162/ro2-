"""配置契约、发布与部署、底盘驱动的检查（与 ROS、硬件无关）。"""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'radar_system'))
sys.path.insert(0, str(ROOT/'wheeltec_protocol'))
sys.path.insert(0, str(ROOT))
from robot_core.config import PROFILE, load_profile, profile_hash
from motion_safety import ChassisGeometry
from deployment.manage import stage, verify, activate, units


@pytest.mark.parametrize('section,key,value', [
    ('geometry','front_m',-1), ('safety','scan_timeout_s',float('nan')),
    ('geometry','wheelbase_m',0), ('sensors','camera_x_m',3),
    ('localization','max_extrapolation_s',5), ('manual','high_mps',4)])
def test_invalid_profile_fails_before_device_start(tmp_path, section, key, value):
    data = copy.deepcopy(PROFILE)
    data[section][key] = value
    path = tmp_path/'bad.json'; path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_profile(path)


def test_profile_hash_changes():
    changed = copy.deepcopy(PROFILE); changed['geometry']['wheelbase_m'] += .01
    assert profile_hash(changed) != profile_hash(PROFILE)


def test_release_stage_and_tamper_detection(tmp_path):
    source = tmp_path/'src'
    for tree in ('robot_core','radar_system','wheeltec_protocol','deployment'):
        (source/tree).mkdir(parents=True)
    import shutil
    shutil.copytree(ROOT/'robot_core', source/'robot_core', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
    (source/'radar_system/test.py').write_text('a = 1\n')
    release = tmp_path/'release'
    release_id = stage(source, release)
    assert verify(release)['release_id'] == release_id
    (release/'radar_system/test.py').write_text('a = 2\n')
    with pytest.raises(ValueError):
        verify(release)


def test_release_from_git_commit_ignores_uncommitted_edits(tmp_path):
    """stage --git：只打那个提交里的文件（工作区没提交的改动不带），清单记下提交号，release_id 只看文件。"""
    import shutil
    from deployment.manage import stage_commit
    if shutil.which('git') is None:
        pytest.skip('没有 git')
    repo = tmp_path/'repo'
    for tree in ('robot_core', 'radar_system', 'deployment'):
        (repo/tree).mkdir(parents=True)
    shutil.copytree(ROOT/'robot_core', repo/'robot_core', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
    (repo/'radar_system/test.py').write_text('a = 1\n')
    git = lambda *a: subprocess.run(['git', '-C', str(repo), *a], check=True, capture_output=True, text=True)  # noqa: E731
    git('init', '-q')
    git('-c', 'user.name=t', '-c', 'user.email=t@t', 'add', '.')
    git('-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-q', '-m', 'x')
    head = git('rev-parse', 'HEAD').stdout.strip()
    (repo/'radar_system/test.py').write_text('a = 2\n')          # 工作区改了没提交
    (repo/'radar_system/extra.py').write_text('b = 1\n')         # 没加进仓库的新文件
    release_id, commit = stage_commit(repo, 'HEAD', tmp_path/'release')
    assert commit == head
    manifest = verify(tmp_path/'release')
    assert (manifest['release_id'], manifest['source_commit']) == (release_id, head)
    assert (tmp_path/'release/radar_system/test.py').read_text() == 'a = 1\n'
    assert not (tmp_path/'release/radar_system/extra.py').exists()
    # 同一个提交从工作区（已恢复成提交内容）打包，release_id 一样
    (repo/'radar_system/test.py').write_text('a = 1\n')
    (repo/'radar_system/extra.py').unlink()
    assert stage(repo, tmp_path/'release2') == release_id


def test_service_runner_is_release_rooted_and_has_only_current_components():
    rendered = units(Path('/opt/rk3588'))
    assert '/current/deployment/run_component.sh' in rendered['rk3588-wheeltec.service']
    runner = (ROOT/'deployment/run_component.sh').read_text()
    assert 'radar_system/run_web.sh' in runner
    for gone in ('person_follower', 'camera', 'lidar)', 'start_component'):
        assert gone not in runner, gone


@pytest.mark.parametrize('protocol', ['twist', 'steering_angle'])
@pytest.mark.parametrize('speed,steer', [(.5,.2), (.5,-.2), (-.5,.2), (-.5,-.2)])
def test_driver_and_behavior_use_same_steering_model(protocol, speed, steer):
    from wheeltec_driver import Config, ControlPolicy
    from robot_core.kinematics import yaw_from_steer
    geom = ChassisGeometry()
    cfg = Config(protocol=protocol, wheelbase_m=geom.wheelbase_m,
                 track_m=geom.track_m, max_speed_m_s=1.)
    p = ControlPolicy(cfg, 0.)
    p.armed = True
    yaw = yaw_from_steer(speed, steer, geom)
    p.command('twist', speed, yaw, 1.)
    assert p.latest[2] == pytest.approx(yaw if protocol == 'twist' else steer)
    p.command('ackermann', speed, steer, 1.)
    assert p.latest[2] == pytest.approx(yaw if protocol == 'twist' else steer)


def test_driver_collision_slowdown_preserves_curvature_and_physical_bound():
    from wheeltec_driver import Config, ControlPolicy
    from robot_core.kinematics import yaw_from_steer, max_yaw_at_speed
    g = ChassisGeometry()
    c = Config(protocol='twist', wheelbase_m=g.wheelbase_m, max_speed_m_s=1.,
               steering_rate_rad_s=6., acceleration_m_s2=6.)
    p = ControlPolicy(c, 0.)
    p.armed = p.connected = True
    p.last_rx = 4.
    yaw = yaw_from_steer(.5, .2, g)
    p.output = (.5, yaw)
    p.command('twist', .5, yaw, 4.)
    p.speed_filter = lambda speed, turn, now: (.1, 'obstacle')
    p.tick(4.)
    assert p.output == pytest.approx((.1, yaw*.2))
    p.last_rx = 4.05
    p.speed_filter = lambda speed, turn, now: (0., 'stop')
    p.tick(4.05)
    assert p.output == (0., 0.)
    assert abs(p.output[1]) <= max_yaw_at_speed(p.output[0], g)


def test_failed_activation_restores_previous_release_and_units(tmp_path, monkeypatch):
    import deployment.manage as deploy
    base, unit_dir = tmp_path/'opt', tmp_path/'units'
    old, new = tmp_path/'old', tmp_path/'new'
    for path in (base, unit_dir, old, new):
        path.mkdir()
    (base/'current').symlink_to(old)
    unit = unit_dir/'rk3588-wheeltec.service'
    unit.write_text('old verified unit\n')
    calls = []
    monkeypatch.setattr(deploy, 'verify', lambda _: {})
    monkeypatch.setattr(deploy, 'stop_stack', lambda: calls.append(('stop',)))
    def ctl(*args):
        calls.append(args)
        if args[0] == 'start':
            raise RuntimeError('injected start failure')
    monkeypatch.setattr(deploy, 'systemctl', ctl)
    with pytest.raises(RuntimeError, match='injected'):
        activate(new, base, unit_dir, 'rtk')
    assert (base/'current').resolve() == old
    assert unit.read_text() == 'old verified unit\n'
    assert not (unit_dir/'rk3588-perception@.service').exists()
    assert calls.count(('stop',)) == 2
    assert len([c for c in calls if c[0]=='start']) == 1  # rollback stays stopped


def test_activation_starts_and_enables_the_rtk_services(tmp_path, monkeypatch):
    # 断电重启后要自动起来的就是刚启用的这一套（rtk、line 曾经只 start 没 enable，重启后不起）；
    # 车上没有雷达，底盘驱动不能等雷达数据
    import deployment.manage as deploy
    base, unit_dir, new = tmp_path/'opt', tmp_path/'units', tmp_path/'new'
    for path in (base, unit_dir, new):
        path.mkdir()
    calls = []
    monkeypatch.setattr(deploy, 'verify', lambda _: {})
    monkeypatch.setattr(deploy, 'stop_stack', lambda: None)
    monkeypatch.setattr(deploy, 'systemctl', lambda *args: calls.append(args))
    activate(new, base, unit_dir, deploy.DEFAULT_PROFILE)
    want = {'rk3588-can.service', 'rk3588-wheeltec.service', 'rk3588-perception.target',
            *('rk3588-perception@' + c + '.service' for c in ('web', 'rtk', 'line', 'imu', 'jobrec'))}
    for action in ('start', 'enable'):
        assert set([c for c in calls if c[0] == action][0][1:]) == want, action
    assert not [c for c in calls if c[0] == 'disable']
    assert 'Environment=RK3588_LIDAR_INSTALLED=0\n' in (unit_dir/'rk3588-wheeltec.service').read_text()


def test_release_cannot_stage_itself_recursively(tmp_path):
    with pytest.raises(ValueError, match='outside source'):
        stage(tmp_path, tmp_path/'radar_system/release')
