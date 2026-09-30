"""实测延迟底盘上的原地转向回归；全程只使用数学仿真。"""
import math
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from gen5_turn_eval import pivot


@pytest.mark.parametrize('angle',[-180.,-90.,90.,180.])
def test_pivot_delay_settles_once_without_reversal(angle):
    result=pivot(angle_deg=angle)
    assert result['settled'],result
    assert result['turns']==1 and result['reversals']==0,result
    assert abs(result['error_deg'])<=1.,result
    assert result['seconds']<=(7.2 if abs(angle)==180 else 5.2),result


def test_delay_gain_grid_has_no_repeated_oscillation_or_timeout():
    # 失配极端（延迟 0.6 s）转完可以差到 3° 左右：用户规则（2026-09-30）差 5°（pivot_min）以内不补转，
    # 直接慢速往前开，由跟踪纠正（名义参数仍 ≤ 1°，见上一条）；不允许来回摆动、反复重启转向。
    for angle in (-180.,-90.,90.,180.):
        for delay in (.3,.39,.45,.5,.6):
            for gain in (.9,1.,1.03,1.2):
                result=pivot(angle_deg=angle,delay=delay,gain=gain)
                assert result['settled'] and result['seconds']<12.,result
                assert result['reversals']<=1 and result['turns']<=2,result
                assert abs(result['error_deg'])<=5.,result


def test_pause_flushes_pending_turn_commands_before_resume():
    result=pivot(angle_deg=180.,pause_at=1.1)
    assert result['settled'] and abs(result['error_deg'])<=1.,result


def test_turn_command_history_stays_bounded():
    import path_tracker as pt
    from gen5_line_sim import GEN5,make_pose
    section=pt.Section(1,pt.Path((0,0,math.pi/2),[(3,0)]))
    tracker=pt.SectionTracker([section],pt.differential_config(),GEN5,pivot_rate=.8)
    pose=make_pose(0,0,0,0,0)
    tracker.start(pose)
    tracker.pause('operator')
    for _ in range(10000):
        assert tracker.update(pose,.05)==(0.,0.)
    assert len(tracker._turn_sent)<32
