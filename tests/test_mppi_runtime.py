"""Timing, exact geometry and deployed MPPI backend regressions."""
import math
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'radar_system'))
from mppi_backend import get_backend, torch_available, cuda_available
from mppi_controller import MPPIController, MPPIConfig, FieldConfig, MPPISolution, rectangle_clearance
from test_person_follower import FollowerHarness, person
from person_follower import parse_args
from follower_config import build_config


def controller(device='numpy', **kwargs):
    kwargs.setdefault('control_dt_s', .05)
    return MPPIController(MPPIConfig(samples=32, horizon=12, **kwargs), prefer=device)


def test_real_entrypoint_requires_cuda_by_default_and_keeps_passive():
    with patch.dict('os.environ', {}, clear=True):
        args = parse_args(['--passive'])
        cfg = build_config(args)
    assert args.passive
    assert (cfg.controller, cfg.mppi_device, cfg.mppi_samples) == ('mppi', 'cuda', 1024)


def test_runtime_environment_and_explicit_cli_override():
    with patch.dict('os.environ', {'ROBOT_FOLLOW_CONTROLLER': 'pure-pursuit',
                                   'ROBOT_MPPI_DEVICE': 'numpy', 'ROBOT_MPPI_SAMPLES': '128'}):
        cfg = build_config(parse_args([]))
        assert (cfg.controller, cfg.mppi_device, cfg.mppi_samples) == ('pure-pursuit', 'numpy', 128)
        cfg = build_config(parse_args(['--controller', 'mppi', '--mppi-device', 'cuda',
                                      '--mppi-samples', '256']))
        assert (cfg.controller, cfg.mppi_device, cfg.mppi_samples) == ('mppi', 'cuda', 256)


@pytest.mark.parametrize('name,value', [('ROBOT_FOLLOW_CONTROLLER', 'unknown'),
                                     ('ROBOT_MPPI_DEVICE', 'typo'), ('ROBOT_MPPI_SAMPLES', '0')])
def test_bad_environment_cannot_silently_change_controller(name, value):
    with patch.dict('os.environ', {name: value}), pytest.raises(ValueError):
        build_config(parse_args([]))


def test_explicit_cuda_fails_without_torch():
    with patch('mppi_backend._HAS_TORCH', False), pytest.raises(RuntimeError, match='CUDA'):
        get_backend('cuda')


@pytest.mark.skipif(not torch_available(), reason='PyTorch not installed')
def test_batched_torch_noise_preserves_ar1_sampling():
    c = controller('cpu')
    c.b.seed(7)
    raw = c.b.to_numpy(c.b.randn((32, c.cfg.horizon, 2)))
    expected = raw.copy()
    beta = c.cfg.noise_correlation
    for t in range(1, c.cfg.horizon):
        expected[:, t] = beta*expected[:, t-1] + math.sqrt(1-beta*beta)*raw[:, t]
    c.b.seed(7)
    actual = c.b.to_numpy(c._correlated_noise(32, c.cfg.horizon))
    np.testing.assert_allclose(actual, expected, atol=5e-7)


def test_exact_geometry_matches_scalar_reference():
    rng = np.random.default_rng(45)
    poses = rng.uniform(-2., 2., (30, 3))
    points = rng.uniform(-4., 4., (100, 2))
    distances = []
    for x, y, theta in poses:
        c, s = math.cos(theta), math.sin(theta)
        for ox, oy in points:
            ex = abs((ox-x)*c + (oy-y)*s - (.67-.18)/2) - (.67+.18)/2
            ey = abs(-(ox-x)*s + (oy-y)*c) - .335
            distances.append(math.hypot(max(ex, 0), max(ey, 0)) + min(max(ex, ey), 0))
    assert rectangle_clearance(poses, points, .67, .18, .335) == pytest.approx(min(distances))


def test_thin_obstacle_discarded_by_field_cap_still_blocks_exact_verification():
    c = controller(field=FieldConfig(max_points=1))
    sol = c.solve([(3., 2.), (.67, 0.)], (3., 0.), (0., 0.), 0., 0.)
    assert c.field.obstacle_count == 1
    assert not sol.feasible
    assert sol.speed == 0.


@pytest.mark.parametrize('device', ['numpy', 'cpu'])
def test_cpu_verification_matches_batched_kinematics_including_latency(device):
    if device == 'cpu' and not torch_available():
        pytest.skip('PyTorch not installed')
    c = controller(device)
    rng = np.random.default_rng(7)
    sequence = rng.uniform([0., -.35], [.5, .35], (c.cfg.horizon, 2)).astype(np.float32)
    delay = math.ceil(c.cfg.latency_s/c.cfg.dt_s)
    held = np.tile([.3, -.15], (delay, 1))
    controls = c.b.array(np.concatenate([held, sequence])[None])
    goals = (c.b.zeros((c.cfg.horizon,)), c.b.zeros((c.cfg.horizon,)))
    trace = []
    state = {'speed': .3, 'steer': -.15}
    if device == 'cpu':  # Capture also uses device scalar tensors for initial state.
        tensor = c.b.array([.3, -.15])
        state = {'speed': tensor[0], 'steer': tensor[1]}
    c._rollout(controls, state, goals, goals, trace=trace)
    rollout = c.b.to_numpy(c.b.stack(trace))[:, 0]
    verified = c._verification_poses(sequence, .3, -.15, delay)
    np.testing.assert_allclose(verified[1:], rollout[:len(verified)-1], atol=2e-7)
    assert verified[0] == (0., 0., 0.)


def test_warm_start_advances_50ms_in_a_150ms_plan():
    c = controller()
    plan = np.arange(c.cfg.horizon*2, dtype=np.float32).reshape(-1, 2)
    shifted = c._advance_nominal(plan)
    np.testing.assert_allclose(shifted[:-1], plan[:-1]*2/3 + plan[1:]/3, atol=2e-6)
    np.testing.assert_array_equal(shifted[-1], plan[-1])


def test_invalid_state_stops_and_clears_previous_plan():
    c = controller()
    c._nominal[:] = .25
    sol = c.solve([], (float('nan'), 0.), (0., 0.), .2, 0.)
    assert not sol.feasible and sol.speed == 0.
    assert np.count_nonzero(c._nominal) == 0


@pytest.fixture
def follower():
    h = FollowerHarness(controller='mppi', mppi_device='numpy', mppi_samples=32,
                        mppi_horizon=12, mppi_solve_budget_ms=10000.)
    yield h
    h.close()


def test_first_timeout_stops_same_cycle_before_fallback(follower):
    h = follower
    h.settle(8, people=[person(3.)])
    h.cfg.mppi_solve_budget_ms = 25.
    h.node.mppi.solve = lambda *a, **kw: MPPISolution(.4, .2, True, 1., 2., 'ok', solve_ms=30.)
    status = h.tick(people=[person(3.)])
    assert status['cmd_vx'] == 0.
    assert status['limit_reason'] == 'mppi_timeout'
    assert status['mppi']['failure_streak'] == 1
    assert not status['mppi']['fell_back']


def test_exception_stops_this_cycle_and_uses_fallback_next_cycle(follower):
    h = follower
    h.settle(8, people=[person(3.)])
    def fail(*args, **kwargs):
        raise RuntimeError('device failure')
    h.node.mppi.solve = fail
    status = h.tick(people=[person(3.)])
    assert status['cmd_vx'] == 0.
    assert status['limit_reason'] == 'mppi_error'
    assert status['controller'] == 'pure-pursuit'
    assert h.settle(12, people=[person(3.)])['cmd_vx'] > 0.


def test_nonfinite_solution_stops_and_keeps_status_valid_json(follower):
    h = follower
    h.settle(8, people=[person(3.)])
    h.node.mppi.solve = lambda *a, **kw: MPPISolution(
        float('nan'), .2, True, float('inf'), float('nan'), 'ok', solve_ms=float('nan'))
    status = h.tick(people=[person(3.)])
    assert status['cmd_vx'] == 0.
    assert status['limit_reason'] == 'mppi_invalid'
    assert status['mppi']['cost'] is None
    json.dumps(status, allow_nan=False)


def test_observations_expiring_during_solve_cannot_issue_a_fresh_command(follower):
    h = follower
    h.settle(8, people=[person(3.)])
    def slow(*args, **kwargs):
        h.t += .65
        return MPPISolution(.4, .1, True, 1., 2., 'ok', solve_ms=1.)
    h.node.mppi.solve = slow
    status = h.tick(people=[person(3.)])
    assert status['cmd_vx'] == 0.
    assert status['limit_reason'] == 'observations_expired_during_control'


def test_new_control_epoch_clears_old_plan_and_fallback(follower):
    h = follower
    h.node.mppi._nominal[:] = .3
    h.node.mppi_fallback = True
    h.node.mppi_infeasible_streak = 12
    h.node.engine.reset_tracking()
    assert np.count_nonzero(h.node.mppi._nominal) == 0
    assert not h.node.mppi_fallback
    assert h.node.mppi_infeasible_streak == 0


def test_target_change_does_not_reuse_previous_person_plan(follower):
    h = follower
    h.settle(8, people=[person(3.)])
    h.node.mppi._nominal[:] = .3
    h.node.engine._mppi_target_id = -1
    plans = []
    def solve(*args, **kwargs):
        plans.append(h.node.mppi._nominal.copy())
        return MPPISolution(.1, 0., True, 1., 2., 'ok', solve_ms=1.)
    h.node.mppi.solve = solve
    h.tick(people=[person(3.)])
    assert plans and np.count_nonzero(plans[0]) == 0


@pytest.mark.skipif(not cuda_available(), reason='Requires a CUDA device')
def test_cuda_graph_matches_eager_rollout_after_sensor_and_state_updates():
    graph = controller('cuda')
    eager = controller('cuda', cuda_graph=False)
    graph.warmup()
    assert graph._graph is not None
    for points, target, velocity, speed, steer in [
        ([], (3., .2), (.1, 0.), .2, .1),
        ([(1.5, .5), (2., -.7)], (2.5, -.3), (.2, .1), .35, -.2),
        ([], (3., 0.), (0., 0.), 0., 0.)]:
        a = graph.solve(points, target, velocity, speed, steer)
        b = eager.solve(points, target, velocity, speed, steer)
        assert a.feasible == b.feasible
        assert a.speed == pytest.approx(b.speed, abs=1e-5)
        assert a.steer == pytest.approx(b.steer, abs=1e-5)
        assert a.cost == pytest.approx(b.cost, rel=1e-5)
