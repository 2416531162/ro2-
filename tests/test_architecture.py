"""Production pose/configuration contracts, independent engine and deployment checks."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'radar_system'))
sys.path.insert(0, str(ROOT))
from robot_core.config import PROFILE, load_profile, profile_hash
from robot_core.odometry import PoseHistory
from robot_core.contracts import LocalPose, ScanFrame
from follower_engine import FollowerEngine
from follower_config import FollowerConfig, build_config
from follower_recovery import LocalRecovery
from motion_safety import ChassisGeometry, BrakeProfile
from footprint import VehicleFootprint
from deployment.manage import stage, verify, activate, rollback, units


def engine():
    clock = NS(t=100.)
    core = FollowerEngine(FollowerConfig(), now=lambda: clock.t, ros_time=lambda: clock.t, dry_run=True)
    core.print_dashboard = lambda _: None
    return core, clock


def test_engine_imports_without_ros():
    env = dict(os.environ, PYTHONPATH=str(ROOT/'radar_system'))
    subprocess.run([sys.executable, '-c',
        "import follower_engine,sys; assert 'rclpy' not in sys.modules; assert 'sensor_msgs' not in sys.modules"],
        check=True, cwd=ROOT, env=env)


def test_shared_geometry_and_sensor_defaults():
    sys.path.insert(0, str(ROOT/'wheeltec_protocol'))
    from scan_guard import GuardConfig
    f, g, geom = FollowerConfig(), GuardConfig(), ChassisGeometry()
    assert f.footprint_front_m == g.front_m == PROFILE['geometry']['front_m']
    assert f.lidar_offset_x_m == g.lidar_x_m == PROFILE['sensors']['lidar_x_m']
    assert f.geometry.wheelbase_m == geom.wheelbase_m == PROFILE['geometry']['wheelbase_m']
    assert f.max_steer_rad == geom.max_steer_rad


def test_lidar_defaults_to_standard_protocol_directions():
    """默认不得再把整圈点云额外旋转 140.7°。"""
    from n10p_pipeline import scan_payload

    assert PROFILE['sensors']['raw_lidar_yaw_deg'] == 0.0
    ranges = [math.inf] * 720
    ranges[0] = 1.0       # 前 0°
    ranges[180] = 2.0     # 左 90°
    ranges[360] = 3.0     # 后 180°
    ranges[540] = 4.0     # 右 270°
    scan = scan_payload(ranges, .15, 12.0,
                        angle_min=0.0, angle_increment=math.pi / 360)
    assert (scan['front'], scan['left'], scan['back'], scan['right']) == (1.0, 2.0, 3.0, 4.0)


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


def test_profile_hash_changes_and_cli_cannot_silently_diverge():
    changed = copy.deepcopy(PROFILE); changed['geometry']['wheelbase_m'] += .01
    assert profile_hash(changed) != profile_hash(PROFILE)
    with pytest.raises(ValueError, match='RK3588_ROBOT_CONFIG'):
        build_config(NS(max_steer_deg=30))


def test_external_pose_is_authoritative_never_double_integrated():
    core, clock = engine()
    assert core.observe_pose(LocalPose(100., 5., -2., .3))
    core.observe_driver(dict(armed=True, ready='ready', connected=True, holding=False,
                             age_ms=0, telemetry=dict(velocity=[.4, 0., .2])))
    core.step()
    assert core.people.odom.current() == (5., -2., .3)
    clock.t += .05
    core.step()
    assert core.people.odom.current() == (5., -2., .3)
    clock.t += .2
    core.step()
    assert core.cmd_vx == 0 and not core.diag['healthy']


def test_pose_frame_reset_discards_target_and_recovery_memory():
    core, clock = engine()
    core.observe_pose(LocalPose(100., 1., 0., 0.))
    core.recovery.history.append((100.,1.,0.,0.,0.))
    core.turnaround_phase = 'REVERSE'
    clock.t += .05
    core.observe_pose(LocalPose(100.05, 20., 0., 0.))
    assert not core.recovery.history
    assert core.turnaround_phase == 'IDLE'
    assert core.people.odom.current() == (20.,0.,0.)
    assert not core.observe_pose(LocalPose(100.05, 0., 0., 0., 'map'))
    assert core.people.odom.t is None


def test_pose_history_interpolates_wrap_and_rejects_unknown_history():
    b = PoseHistory(strict=True)
    assert b.pose_at(1) is None
    b.add(1, 0, 0, math.radians(179))
    b.add(1.1, .1, 0, math.radians(-179))
    x, y, yaw = b.pose_at(1.05)
    assert x == pytest.approx(.05)
    assert abs(abs(yaw)-math.pi) < .001
    assert b.pose_at(.99) is None and b.pose_at(1.3) is None
    assert not b.add(1.05, .1,0,0)


def test_recovery_uses_same_external_pose_and_measured_distance():
    r = LocalRecovery(VehicleFootprint(), ChassisGeometry(), BrakeProfile())
    r._observe(1., 1., 0., True, local_pose=(10.,5.,.1))
    assert r.pose == (10.,5.,.1)
    r.active = True
    r._observe(1.1, 1., 0., True, local_pose=(10.02,5.,.1))
    assert r.pose == (10.02,5.,.1)
    assert r.leg_distance == pytest.approx(.02)  # not speed*dt == .1


def test_odom_epoch_restarts_tracking_without_coordinate_mix():
    core, _ = engine()
    core.observe_pose(LocalPose(100.,1.,0.,0.))
    core.observe_driver(dict(odometry_epoch='new', telemetry={}, armed=False))
    assert core.people.odom.t is None


def test_camera_old_timestamp_is_not_freshened():
    core, _ = engine()
    assert core._meas_time(90., 100.) is None
    core.observe_targets([dict(label='person', conf=.9, x=0., y=0., z=2., stamp=90.)])
    assert core.people.target_id is None


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


def test_service_runner_is_passive_and_release_rooted():
    rendered = units(Path('/opt/rk3588'))
    assert '/current/deployment/run_component.sh' in rendered['rk3588-wheeltec.service']
    runner = (ROOT/'deployment/run_component.sh').read_text()
    assert 'person_follower.py" --passive' in runner


def test_production_follow_stops_on_pose_loss_then_reacquires():
    from test_follower_node import n10p_scan, camera_person, person_legs
    core, clock = engine()
    def cycle(with_pose=True):
        clock.t += .05
        if with_pose:
            assert core.observe_pose(LocalPose(clock.t, 0., 0., 0.))
        core.observe_driver(dict(armed=True, ready='ready', connected=True, holding=False,
                                 age_ms=0, telemetry=dict(velocity=[0., 0., 0.])))
        scan = n10p_scan(extra=person_legs(2., 0.))
        core.observe_scan(ScanFrame(scan.ranges, scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max, clock.t))
        targets = camera_person(2., 0.)
        targets[0]['stamp'] = clock.t
        core.observe_targets(targets)
        core.step()
    for _ in range(8):
        cycle()
    assert core.cmd_vx > 0 and core.diag['healthy']
    for _ in range(8):
        cycle(with_pose=False)
    assert core.cmd_vx == core.cmd_wz == 0 and not core.diag['healthy']
    # A long localization gap invalidates the previous target and manoeuvre.
    clock.t += .2
    core.turnaround_phase = 'REVERSE'
    for _ in range(8):
        cycle()
    assert core.turnaround_phase == 'IDLE'
    assert core.cmd_vx > 0 and core.diag['healthy']
    assert core.people.odom.current() == (0., 0., 0.)


def test_production_measurement_deadlines_use_capture_time():
    from test_follower_node import n10p_scan
    core, clock = engine()
    assert core._meas_time(None, clock.t) is None
    assert core._meas_time(float('nan'), clock.t) is None
    scan = n10p_scan()
    core.observe_scan(ScanFrame(scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_min, scan.range_max, clock.t-.4))
    assert core.scan_stamp == pytest.approx(clock.t-.4)
    core.observe_scan(ScanFrame(scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_min, scan.range_max, clock.t-.6))
    assert core.scan_stamp == 0


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
        activate(new, base, unit_dir, 'headless')
    assert (base/'current').resolve() == old
    assert unit.read_text() == 'old verified unit\n'
    assert not (unit_dir/'rk3588-perception@.service').exists()
    assert calls.count(('stop',)) == 2
    assert len([c for c in calls if c[0]=='start']) == 1  # rollback stays stopped


def test_release_cannot_stage_itself_recursively(tmp_path):
    with pytest.raises(ValueError, match='outside source'):
        stage(tmp_path, tmp_path/'radar_system/release')


def test_lidar_calibration_adds_residual_to_shared_raw_zero(tmp_path, monkeypatch):
    import calibrate_lidar as calibration
    path = tmp_path/'robot.json'
    path.write_text(json.dumps(PROFILE))
    monkeypatch.setattr(calibration, 'CALIB_FILE', str(path))
    calibration.update_config_file(2.)
    changed = load_profile(path)
    assert changed['sensors']['raw_lidar_yaw_deg'] == pytest.approx(2.0)
    assert changed['sensors']['lidar_yaw_rad'] == PROFILE['sensors']['lidar_yaw_rad']
    before = path.read_text()
    with pytest.raises(ValueError):
        calibration.update_config_file(float('nan'))
    assert path.read_text() == before


@pytest.mark.parametrize('pose', [(0., 0., 0.), (10., -3., math.pi),
                                  (-10., 4., math.pi / 2)])
@pytest.mark.parametrize('front_return', [True, False])
def test_rear_object_cannot_teleport_confirmed_front_target(pose, front_return):
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1, reacquire_after_s=.1, reacquire_radius_m=4.)
    tracker.odom.add(0., *pose)
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0.)
    original_id = tracker.target_id
    tracker.odom.add(.2, *pose)
    clusters = [(1.8, 0.), (-1., .1)] if front_return else [(-1., .1)]
    tracker.add_lidar(clusters, .2, .2)
    view = tracker.target_view(.2)
    assert view['id'] == original_id
    assert view['x'] == pytest.approx(1.8)
    assert tracker.reacquires == 0
    assert view['lidar_hits'] == int(front_return)


def rear_tracker(pose=(0., 0., 0.)):
    """Observe a continuous 0.9 m/s semicircle, not a 2.8m/0.2s teleport."""
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1)
    tracker.odom.add(0., *pose)
    in_view = lambda x, y: x > 0. and abs(y) < .3
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0., in_view=in_view)
    original_id = tracker.target_id
    for i in range(1, 127):
        now, angle = i * .05, math.pi * i / 126
        point = (1.8 * math.cos(angle), 1.8 * math.sin(angle))
        tracker.odom.add(now, *pose)
        tracker.add_camera([], now, now, in_view=in_view)
        tracker.add_lidar([point], now, now)
        view = tracker.target_view(now)
        assert view is not None and view['id'] == original_id
        assert math.hypot(view['x'] - point[0], view['y'] - point[1]) < .12
    return tracker, now


@pytest.mark.parametrize('pose', [(0., 0., 0.), (10., -3., math.pi),
                                  (-10., 4., math.pi / 2)])
def test_continuous_rear_tracking_then_stationary_with_clutter(pose):
    tracker, start = rear_tracker(pose)
    original_id = tracker.target_id
    for i in range(1, 301):
        now = start + i * .1
        tracker.odom.add(now, *pose)
        clusters = [(-1.8, 0.)]
        if i >= 20:
            clusters.append((-1.8, .65))
        tracker.add_lidar(clusters, now, now)
        view = tracker.target_view(now)
        assert view is not None and view['id'] == original_id
        assert abs(view['x'] + 1.8) < .12 and abs(view['y']) < .12
    assert math.hypot(view['v_fwd'], view['v_lat']) < .01
    assert tracker.dropped_unseen == 0
    tracker.add_lidar([], now + 2., now + 2.)
    assert tracker.target_view(now + 2.) is None


def test_ambiguous_rear_returns_do_not_change_position_or_refresh_age():
    tracker, now = rear_tracker()
    for i in range(1, 21):
        tracker.add_lidar([(-1.8, 0.)], now + i * .1, now + i * .1)
    now += 2.
    tr = tracker._get(tracker.target_id)
    before = tr.last_update
    tracker.add_lidar([(-1.8, -.08), (-1.8, .08)], now + .1, now + .1)
    assert tr.last_update == before
    assert tracker.lidar_ambiguous_frames == 1
    assert abs(tr.pos[1]) < .01
    for i in range(2, 31):
        tracker.add_lidar([(-1.8, -.08), (-1.8, .08)], now + i * .1, now + i * .1)
    assert tracker.target_view(now + 3.) is None


def test_visible_bystander_does_not_replace_lidar_target():
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1, reacquire_after_s=.1, reacquire_radius_m=4.)
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0.)
    original_id = tracker.target_id
    tracker.add_lidar([(1.8, 0.)], .2, .2)
    tracker.add_camera([{'x': 1.8, 'y': 2., 'conf': .95}], .2, .2)
    assert tracker.target_id == original_id
    assert tracker.switches == 0
    for i in range(3, 31):
        now = i * .1
        tracker.add_lidar([(1.8, 0.), (1.8, 2.)], now, now)
    assert len(tracker.tracks) == 1
    assert tracker.target_id == original_id


def test_lost_target_does_not_switch_to_distant_visible_person():
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1, reacquire_radius_m=4.)
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0.)
    original_id = tracker.target_id
    for i in range(1, 61):
        now = i * .1
        tracker.add_camera([{'x': 1.8, 'y': 2., 'conf': .95}], now, now)
    assert tracker.target_id == original_id
    assert tracker.target_view(6.) is None


def test_front_radar_clutter_still_expires_with_camera_negative_evidence():
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1)
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0.,
                       in_view=lambda x, y: x > 0.)
    for i in range(1, 21):
        now = i / 10.
        tracker.add_camera([], now, now, in_view=lambda x, y: x > 0.)
        tracker.add_lidar([(1.8, 0.)], now, now)
    assert tracker.target_view(2.) is None
    assert tracker.dropped_unseen == 1


def test_late_radar_return_does_not_revive_expired_identity():
    from person_tracker import PersonTracker
    tracker = PersonTracker(confirm_hits=1)
    tracker.add_camera([{'x': 1.8, 'y': 0., 'conf': .95}], 0., 0.)
    tracker.add_lidar([(1.8, 0.)], 3., 3.)
    assert tracker.target_view(3.) is None
