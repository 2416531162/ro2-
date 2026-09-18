"""Distinguish dropped angular bins from actual samples without usable returns."""
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'radar_system'))
from n10p_pipeline import scan_coverage, SweepAssembler


def test_unsampled_and_noecho_have_identical_range_but_different_provenance():
    ranges = [2.] * 720
    sampled = [True] * 720
    for i in range(660, 680):
        ranges[i] = math.inf
        sampled[i] = i % 2 == 0
    result = scan_coverage(ranges, sampled)
    assert result['gaps'] == [dict(start_deg=-30., end_deg=-20.5,
                                  width_deg=10., unsampled=10, too_near=0, no_valid_echo=10)]


def test_gap_crossing_zero_is_one_sector():
    ranges = [2.] * 720
    for i in list(range(716,720)) + list(range(4)):
        ranges[i] = -math.inf
    result = scan_coverage(ranges, [True]*720)
    assert len(result['gaps']) == 1
    assert result['gaps'][0]['start_deg'] == -2.
    assert result['gaps'][0]['end_deg'] == 1.5
    assert result['gaps'][0]['too_near'] == 8


def test_yaw_rotation_keeps_sampling_mask_aligned():
    ranges = [2.] * 720
    sampled = [True] * 720
    for i in range(10):
        ranges[i], sampled[i] = math.inf, False
    result = scan_coverage(ranges[-20:]+ranges[:-20], sampled[-20:]+sampled[:-20])
    assert result['gaps'][0]['start_deg'] == 10.
    assert result['gaps'][0]['unsampled'] == 10


def test_assembler_marks_received_noecho_as_sampled():
    a = SweepAssembler()
    a.add([(350, 2., 1)], 0.)
    a.add([(0., math.inf, 0), (90., 2., 1), (350., 2., 1)], .1)
    scan = a.add([(0., 2., 1)], .2)[0]
    assert scan['sampled'][0] is True
    assert scan['sampled'][1] is False
    assert scan['ranges'][0] == scan['ranges'][1] == math.inf
    assert scan_coverage(scan['ranges'], scan['sampled'])['counts']['no_valid_echo'] == 1


def test_empty_and_all_noecho_scans():
    assert scan_coverage([],[]) == dict(counts=dict(valid=0,unsampled=0,too_near=0,no_valid_echo=0),gaps=[])
    result = scan_coverage([math.inf]*720,[True]*720)
    assert result['gaps'][0]['width_deg'] == 360.
    assert result['gaps'][0]['no_valid_echo'] == 720
