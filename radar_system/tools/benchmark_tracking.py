#!/usr/bin/env python3
"""Measure scan preparation and five candidate sweeps on the host running this script."""
import argparse
import json
import math
import platform
import sys
import time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from footprint import VehicleFootprint,SensorMount
from follower_recovery import ScanEvidence,LocalRecovery
from motion_safety import ChassisGeometry,BrakeProfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=200)
    args=parser.parse_args()
    if args.iterations<1:parser.error('iterations must be positive')
    fp=VehicleFootprint(margin_m=.035)
    mount=SensorMount(.53)
    recovery=LocalRecovery(fp,ChassisGeometry(),BrakeProfile())
    ranges=np.full(720,5.);ranges[::3]=np.inf
    prep=[];paths=[]
    for i in range(args.iterations+10):
        start=time.perf_counter()
        scan=ScanEvidence(ranges.tolist(),0,math.pi/360,.15,12,mount,fp,((155.,-130.),),.05)
        middle=time.perf_counter()
        for steer in (-.35,-.175,0,.175,.35):recovery.clearance(scan,steer)
        end=time.perf_counter()
        if i>=10:
            prep.append((middle-start)*1000);paths.append((end-middle)*1000)
    report=dict(platform=platform.platform(),python=platform.python_version(),numpy=np.__version__,
                iterations=args.iterations,scan_bins=720,candidates=5,
                scan_prepare_ms=dict(median=float(np.median(prep)),p95=float(np.percentile(prep,95))),
                five_paths_ms=dict(median=float(np.median(paths)),p95=float(np.percentile(paths,95))))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
