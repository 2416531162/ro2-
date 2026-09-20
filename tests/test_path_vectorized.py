"""Vector batches must preserve scalar unknown-space and blind-sector rules."""
from pathlib import Path
import sys
import math
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_system'))
from follower_recovery import ScanEvidence,LocalRecovery
from footprint import SensorMount,VehicleFootprint
from motion_safety import ChassisGeometry,BrakeProfile


@pytest.mark.parametrize('n,angle_min,increment',[(720,0,math.pi/360),(720,math.pi,-math.pi/360),
                                                 (180,-math.pi/2,math.pi/180)])
def test_batch_matches_scalar_for_sparse_scans(n,angle_min,increment):
    rng=np.random.default_rng(3588)
    ranges=rng.uniform(.1,8.,n);ranges[::3]=np.inf;ranges[5::11]=-np.inf;ranges[7::17]=np.nan
    ev=ScanEvidence(ranges.tolist(),angle_min,increment,.15,12,SensorMount(.53,.1,.12),
                    VehicleFootprint(),((155.,-130.),(10.5,17.5)),.05)
    x,y=rng.uniform(-5,5,(2,3000))
    covered,free=ev.query_many(x,y)
    np.testing.assert_array_equal(covered,[ev.covered(a,b) for a,b in zip(x,y)])
    np.testing.assert_array_equal(free,[ev.free(a,b) for a,b in zip(x,y)])
    np.testing.assert_array_equal(ev.masked_many(x,y),[ev.masked(a,b) for a,b in zip(x,y)])


def test_empty_scan_batch_is_unknown():
    ev=ScanEvidence([],0,0,.15,12,SensorMount(),VehicleFootprint())
    covered,free=ev.query_many(np.ones((2,3)),0)
    assert not covered.any() and not free.any()


def test_newer_observed_obstacle_cannot_be_overridden_by_older_free_space():
    mount=SensorMount(.53);fp=VehicleFootprint();rec=LocalRecovery(fp,ChassisGeometry(),BrakeProfile())
    old=ScanEvidence([5.]*720,0,math.pi/360,.15,12,mount,fp)
    new=ScanEvidence([1.]*720,0,math.pi/360,.15,12,mount,fp)
    rec.scan_history=[(1,0,0,0,old),(2,0,0,0,new)]
    assert not rec._memory_many(np.array([2.]),np.array([0.]),True,False)[0]


def test_no_scan_stops_both_gears():
    rec=LocalRecovery(VehicleFootprint(),ChassisGeometry(),BrakeProfile())
    for gear in (-1,1):
        assert rec.clearance(None,.2,gear)==0
        assert rec.last_block[0]=='no_scan'
