"""Authority safety invariants and integration with the real driver policy.

No hardware or DDS; methods execute against injected clocks/telemetry.
"""
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'wheeltec_protocol'))
from motion_authority import MotionAuthority
from wheeltec_driver import Config, ControlPolicy, WheeltecDriver, ScanGuard


def healthy(a, t=1.):
    a.health(True, True, t)
    a.health(True, True, t + .31)


def submit(a, source='follow', vx=.3, t=2., epoch=None, stamp=None):
    return a.submit(source, dict(vx=vx, wz=.1, stamp=t if stamp is None else stamp,
                                epoch=a.epoch if epoch is None else epoch), t, t)


def following():
    a = MotionAuthority()
    healthy(a)
    assert a.select('follow')
    healthy(a, 1.5)
    assert submit(a)
    return a


def test_idle_requires_explicit_autonomous_selection():
    a = MotionAuthority()
    healthy(a)
    assert not submit(a)
    assert a.output(2).vx == 0


def test_manual_takeover_clears_follow_and_waits_for_measured_stop():
    a = following()
    old = a.epoch
    assert a.output(2).vx == .3
    assert submit(a, 'manual', .2, t=2.01)
    assert a.epoch != old and a.mode == 'MANUAL'
    assert a.output(2.02).vx == 0
    a.health(True, False, 2.03)
    assert a.output(2.1).vx == 0
    healthy(a, 2.1)
    assert submit(a, 'manual', .2, t=2.42)
    assert a.output(2.42).vx == .2
    assert not submit(a, t=2.43, epoch=old)
    assert not submit(a, t=2.43)


def test_manual_release_never_resumes_follow():
    a = following()
    assert submit(a, 'manual', 0., t=2.02)
    assert a.mode == 'IDLE'
    assert not submit(a, t=2.03)
    assert a.output(2.03).vx == 0


def test_timeout_invalidates_epoch_and_requires_new_follow_selection():
    a = following()
    old = a.epoch
    assert a.output(2.351).vx == 0
    assert a.mode == 'IDLE' and old != a.epoch
    assert not submit(a, t=2.36, epoch=old)
    assert not submit(a, t=2.36)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), '1', None, True])
def test_bad_command_does_not_enter_control(bad):
    a = following()
    assert not submit(a, vx=bad, t=2.1)


def test_source_stamp_age_future_and_reordering():
    a = following()
    assert not submit(a, t=2.1, stamp=2.)
    assert not submit(a, t=2.1, stamp=2.2)
    assert not submit(a, t=2.5, stamp=2.1)
    assert submit(a, t=2.4, stamp=2.15)
    assert a.output(2.51).vx == 0  # expiry measured from source, not receipt


@pytest.mark.parametrize('fault', ['health', 'estop'])
def test_fault_requires_stationary_explicit_reset_and_new_task(fault):
    a = following()
    if fault == 'health':
        a.health(False, False, 2.01)
    else:
        a.stop(emergency=True)
    assert a.output(2.02).vx == 0
    assert not submit(a, 'manual', t=2.02)
    assert not a.reset()
    healthy(a, 2.1)
    assert not a.select('follow')
    assert a.reset()
    assert a.mode == 'IDLE'
    assert not submit(a, t=2.5)
    assert a.select('follow')


def test_transient_health_drop_stops_without_latching_or_replaying_command():
    a = MotionAuthority(fault_grace_s=1.0)
    healthy(a)
    assert a.select('follow')
    healthy(a, 1.5)
    assert submit(a)

    a.health(False, False, 2.01)
    assert a.mode == 'FOLLOW'
    assert not a.healthy and a.status(2.02)['fault_pending']
    assert a.output(2.02).vx == 0
    assert not submit(a, t=2.03)

    healthy(a, 2.10)
    assert a.mode == 'FOLLOW'
    assert a.output(2.42).vx == 0
    assert submit(a, t=2.43)
    assert a.output(2.43).vx == .3


def test_persistent_health_drop_latches_after_grace():
    a = MotionAuthority(fault_grace_s=1.0)
    healthy(a)
    assert a.select('follow')
    healthy(a, 1.5)
    assert submit(a)

    a.health(False, False, 2.01)
    a.health(False, False, 3.02)
    assert a.mode == 'FAULT'
    assert not a.status(3.02)['fault_pending']


def test_releasing_old_follow_does_not_stop_manual():
    a = following()
    submit(a, 'manual', .2, t=2.1)
    epoch = a.epoch
    a.release('follow')
    assert a.mode == 'MANUAL' and a.epoch == epoch


def test_restart_cannot_accept_previous_process_lease():
    a = following()
    b = MotionAuthority()
    healthy(b)
    b.select('follow')
    assert not submit(b, epoch=a.epoch)


def driver():
    # Node base can be object if ROS is absent. Never opens a serial port.
    d = WheeltecDriver.__new__(WheeltecDriver)
    d.config = Config(protocol='twist', protocol_confirmed=True, receive_only=False,
                      max_speed_m_s=1., acceleration_m_s2=3.5)
    d.policy = ControlPolicy(d.config, 0.)
    d.policy.link(True, 0.)
    d.authority = MotionAuthority(fault_grace_s=d.config.feedback_grace_s)
    d.lock = threading.RLock()
    d.guard = ScanGuard()
    d.scan_health_at = 4.
    d.legacy_commands = False
    d.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(4.4e9)))
    d.policy.speed_filter = lambda speed, turn, now: d.guard.limit(speed, abs(turn) > .05, now)
    return d


def feedback(d, now, speed=0., voltage=24.):
    for _ in range(6):
        d.policy.feedback(dict(velocity=[speed, 0., 0.], voltage=voltage), now)
    d.scan_health_at = now


def activate(d):
    feedback(d, 4.)
    d.refresh_motion_health(4.)
    assert d.authority.select('follow')
    d.apply_motion(4.)
    feedback(d, 4.31)
    d.apply_motion(4.31)
    assert submit(d.authority, t=4.32)
    feedback(d, 4.32)
    d.apply_motion(4.32)
    d.policy.tick(4.32)
    assert d.policy.output[0] > 0


def test_driver_health_fault_clears_serial_output_and_does_not_auto_resume():
    d = driver()
    activate(d)
    d.apply_motion(4.8)
    d.policy.tick(4.8)
    assert d.policy.output == (0., 0.)
    assert d.authority.mode == 'FOLLOW'
    assert d.authority.status(4.8)['fault_pending']
    d.apply_motion(5.81)
    d.policy.tick(5.81)
    assert d.authority.mode == 'FAULT'
    feedback(d, 6.)
    d.apply_motion(6.)
    assert d.authority.mode == 'FAULT'
    assert d.policy.latest is None


def test_driver_transient_feedback_gap_recovers_but_needs_fresh_command():
    d = driver()
    activate(d)
    d.apply_motion(4.8)
    d.policy.tick(4.8)
    assert d.authority.mode == 'FOLLOW'
    assert d.policy.output == (0., 0.)

    feedback(d, 4.9)
    d.apply_motion(4.9)
    assert d.authority.mode == 'FOLLOW'
    assert d.authority.output(4.9).vx == 0.


def test_driver_final_collision_filter_still_applies():
    d = driver()
    activate(d)
    d.guard.update_scan([.17], 0., .01, .15, 12., 4.33)  # bumper + 3 cm
    feedback(d, 4.33)
    d.apply_motion(4.33)
    d.policy.tick(4.33)
    assert d.policy.output[0] == 0.
    assert d.policy.guard_reason == 'guard_stop'


def test_driver_estop_cannot_be_bypassed_by_arm_or_source():
    d = driver()
    activate(d)
    d.on_stop(None, NS())
    with patch('wheeltec_driver.time.monotonic', return_value=4.33):
        assert not d.on_arm(NS(data=True), NS()).success
        d.apply_motion(4.33)
    d.policy.tick(4.33)
    assert d.policy.output == (0., 0.)
    assert d.authority.mode == 'ESTOP'


def test_legacy_entry_points_are_closed():
    d = driver()
    activate(d)
    old = d.policy.latest
    d.on_twist(NS())
    d.on_ackermann(NS())
    assert d.policy.latest == old


def test_stale_or_empty_scan_stops_immediately_and_latches_after_grace():
    d = driver()
    activate(d)
    msg = NS(header=NS(stamp=NS(sec=1, nanosec=0)), angle_min=0.,
             angle_increment=.01, range_min=.15, range_max=12., ranges=[3.] * 100)
    with patch('wheeltec_driver.time.monotonic', return_value=4.4):
        d.on_scan(msg)
    assert d.scan_health_at is None
    d.apply_motion(4.4)
    d.policy.tick(4.4)
    assert d.policy.output == (0., 0.)
    assert d.authority.mode == 'FOLLOW'
    d.apply_motion(5.41)
    assert d.authority.mode == 'FAULT'


def test_navigation_and_follow_are_mutually_exclusive_with_fresh_epoch():
    a = following()
    previous = a.epoch
    assert a.select('navigation')
    assert a.epoch != previous and a.output(2.01).vx == 0
    assert not submit(a, 'follow', t=2.02)
    healthy(a, 2.1)
    assert submit(a, 'navigation', t=2.42)
    assert a.output(2.42).vx > 0
    assert submit(a, 'manual', t=2.43)
    assert a.mode == 'MANUAL' and a.output(2.43).vx == 0
