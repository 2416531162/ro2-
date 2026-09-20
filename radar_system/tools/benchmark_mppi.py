#!/usr/bin/env python3
"""Measure MPPI with synthetic scans; does not import ROS or send motion commands."""
import argparse
import json
import math
import platform
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from follower_config import FollowerConfig
from mppi_controller import MPPIConfig, MPPIController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=('cuda', 'cpu', 'numpy'), default='cuda')
    parser.add_argument('--samples', type=int, default=1024)
    parser.add_argument('--iterations', type=int, default=100)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--budget-ms', type=float, default=25.)
    parser.add_argument('--no-cuda-graph', action='store_true')
    parser.add_argument('--require-budget', action='store_true',
                        help='Exit nonzero if measured P95 exceeds the solve budget')
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0 or args.samples < 2 or args.budget_ms <= 0:
        parser.error('iterations/samples/budget must be positive; warmup must be nonnegative')
    cfg = MPPIConfig.from_follower(FollowerConfig())
    cfg.samples = args.samples
    cfg.cuda_graph = not args.no_cuda_graph
    controller = MPPIController(cfg, prefer=args.device)
    start = time.perf_counter()
    controller.warmup()
    startup_ms = (time.perf_counter()-start)*1000
    cases = {
        'open_400_returns': [(2.5*math.cos(a), 2.5*math.sin(a)) for a in np.linspace(-math.pi, math.pi, 400)],
        'door_80cm': [(2., sign*y) for sign in (-1., 1.) for y in np.linspace(.4, 2., 100)],
        'front_wall': [(1.1, y) for y in np.linspace(-1.5, 1.5, 160)],
    }
    report = dict(platform=platform.platform(), python=platform.python_version(),
                  numpy=np.__version__, backend=controller.b.describe(),
                  cuda_graph=controller._graph is not None, startup_ms=round(startup_ms, 2),
                  samples=cfg.samples, horizon=cfg.horizon, control_hz=1/cfg.control_dt_s,
                  budget_ms=args.budget_ms, iterations=args.iterations, cases={})
    if controller.b.kind == 'torch':
        import torch
        report.update(torch=torch.__version__, cuda_runtime=torch.version.cuda)
        if controller.b.is_gpu:
            report['gpu'] = torch.cuda.get_device_name(controller.b.device)
    for name, points in cases.items():
        controller.reset()
        values = []
        for index in range(args.warmup+args.iterations):
            sol = controller.solve(points, (3., .2), (.2, 0.), .2, 0.)
            if index >= args.warmup:
                values.append(sol.solve_ms)
        report['cases'][name] = dict(
            p50_ms=round(float(np.median(values)), 3), p95_ms=round(float(np.percentile(values, 95)), 3),
            max_ms=round(max(values), 3), over_budget=sum(v > args.budget_ms for v in values))
    report['within_budget'] = all(case['p95_ms'] <= args.budget_ms for case in report['cases'].values())
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.require_budget and not report['within_budget']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
