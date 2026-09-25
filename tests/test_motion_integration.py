"""Behavior/HTTP adapters wired to actual authority logic with ROS message stubs."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'wheeltec_protocol'))
import test_follower_node as helpers
import test_web_control as webtests
from motion_authority import MotionAuthority
from motion_client import MotionClient
from runtime_config import PROFILE, profile_hash


def status(mode='FOLLOW', epoch='lease'):
    return helpers.String(data=json.dumps(dict(mode=mode, epoch=epoch, healthy=True, profile_hash=profile_hash(PROFILE))))


def test_follower_no_longer_writes_or_arms_chassis():
    h = helpers.FollowerHarness()
    assert '/cmd_vel' not in h.node.publishers_
    assert '/follow/command' in h.node.publishers_
    assert not hasattr(h.node, 'cli_arm')


def test_follower_pauses_during_manual_takeover_and_cancels_turnaround():
    n = helpers.pf.PersonFollowerNode(helpers.pf.FollowerConfig(), passive=True)
    n.print_dashboard = lambda _: None
    n.turnaround_phase = 'REVERSE'
    n.cmd_vx = .2
    n.motion.on_status(status('MANUAL'))
    n.control_loop()
    assert n.state == 'PAUSED' and n.cmd_vx == 0.
    assert not n.motion.publisher.sent
    n.motion.on_status(status('FOLLOW', 'new-follow-lease'))
    n.control_loop()
    assert n.turnaround_phase == 'IDLE'
    assert n.motion.publisher.sent
    sent = json.loads(n.motion.publisher.sent[-1].data)
    assert sent['epoch'] == 'new-follow-lease'
    assert sent['vx'] == 0.  # no healthy sensor input in this harness


def test_dry_run_never_selects_or_sends_motion():
    n = helpers.FollowerHarness().node
    with patch.object(n.motion, 'select') as select:
        n.control_loop()
        n.stop_robot()
        select.assert_not_called()
    assert not n.motion.publisher.sent


def test_client_discards_stale_status_and_carries_source_stamp():
    node = helpers.ros_stubs.Node()
    client = MotionClient(node, 'manual')
    with patch('motion_client.time.monotonic', return_value=10.):
        client.on_status(status('IDLE', 'epoch'))
        assert client.publish(.2, .1)
    data = json.loads(client.publisher.sent[-1].data)
    assert data['stamp'] == 1000. and data['epoch'] == 'epoch'
    with patch('motion_client.time.monotonic', return_value=10.51):
        assert not client.publish(.2, .1)


def test_web_manual_stream_selects_authority_without_killing_follower():
    bridge = webtests.web.TrackingBridgeNode()
    a = MotionAuthority()
    a.health(True, True, 10.)
    a.select('follow')
    bridge.motion.on_status(status('FOLLOW', a.epoch))
    with patch('radar_web_server.stop_follower') as killed:
        bridge.send_manual_twist(.2, .1)
        bridge.manual_loop()
        killed.assert_not_called()
    data = json.loads(bridge.motion.publisher.sent[-1].data)
    assert a.submit('manual', data, 10., 1000.)
    assert a.mode == 'MANUAL' and a.output(10.).vx == 0.
    bridge.motion.on_status(status('MANUAL', a.epoch))
    a.health(True, True, 10.01)
    a.health(True, True, 10.32)
    bridge._clock.seconds += .01
    bridge.manual_loop()
    data = json.loads(bridge.motion.publisher.sent[-1].data)
    assert a.submit('manual', data, 10.32, 1000.01)
    assert a.output(10.32).vx == .2
    assert '/cmd_vel' not in bridge.publishers_


def test_stop_follower_uses_service_keeps_process_available():
    bridge = webtests.web.TrackingBridgeNode()
    with patch.object(webtests.web, 'bridge_node', bridge), \
         patch.object(bridge, 'select_follow', return_value=(True, 'IDLE')) as select, \
         patch.object(webtests.web.subprocess, 'run') as run:
        assert webtests.web.stop_follower() == (True, 'IDLE')
        select.assert_called_once_with(False)
        run.assert_not_called()


def test_client_configuration_mismatch_refuses_selection_and_commands():
    node = helpers.ros_stubs.Node()
    client = MotionClient(node, 'follow')
    wrong = json.loads(status().data)
    wrong['profile_hash'] = 'different-profile'
    client.on_status(helpers.String(data=json.dumps(wrong)))
    assert not client.fresh()
    assert not client.publish(.2, 0.)
    assert client.select(True) is None


def test_managed_web_start_uses_service_instead_of_spawning_duplicate(monkeypatch):
    bridge = webtests.web.TrackingBridgeNode()
    monkeypatch.setenv('RK3588_MANAGED_FOLLOWER', '1')
    detecting = {'fps': 19.0, '_rx_monotonic': webtests.web.time.monotonic()}
    with patch.object(webtests.web, 'bridge_node', bridge), \
         patch.dict(webtests.web.state, {'ai_status': detecting}), \
         patch.object(bridge, 'select_follow', return_value=(True, 'FOLLOW')), \
         patch.object(webtests.web.subprocess, 'run', return_value=NS(returncode=3)) as run, \
         patch.object(webtests.web.os, 'geteuid', return_value=1000), \
         patch.object(webtests.web.subprocess, 'Popen') as spawn:
        assert webtests.web.start_follower() == (True, 'FOLLOW')
        # 先问 systemd 服务在不在跑，不在才 sudo -n 启动；不自己另起一个跟随进程
        assert run.call_args_list[0].args[0][:2] == ['systemctl', 'is-active']
        assert run.call_args.args[0] == ['sudo', '-n', 'systemctl', 'start', 'rk3588-perception@follower.service']
        spawn.assert_not_called()


def test_follow_is_refused_while_person_detection_is_off(monkeypatch):
    bridge = webtests.web.TrackingBridgeNode()
    monkeypatch.setenv('RK3588_MANAGED_FOLLOWER', '1')
    stopped = {'fps': 19.0, '_rx_monotonic': webtests.web.time.monotonic() - 10.0}
    for ai_status in ({}, stopped):
        with patch.object(webtests.web, 'bridge_node', bridge), \
             patch.dict(webtests.web.state, {'ai_status': ai_status}), \
             patch.object(bridge, 'select_follow') as select, \
             patch.object(webtests.web.subprocess, 'run') as run:
            ok, message = webtests.web.start_follower()
            assert not ok and '人体识别' in message
            select.assert_not_called()
            run.assert_not_called()


def test_ros_odometry_adapter_feeds_timestamped_local_pose():
    import math
    node = helpers.pf.PersonFollowerNode(helpers.pf.FollowerConfig(), passive=True)
    node.engine.now = lambda: 1000.
    node.engine.ros_time = lambda: 1000.
    q = NS(x=0., y=0., z=math.sin(.2), w=math.cos(.2))
    msg = NS(header=NS(stamp=NS(sec=1000, nanosec=0), frame_id='odom'),
             child_frame_id='base_link', pose=NS(pose=NS(
                 position=NS(x=2., y=-1.), orientation=q)))
    node.on_odom(msg)
    x, y, yaw = node.people.odom.current()
    assert (x, y) == (2., -1.) and abs(yaw - .4) < 1e-9
    q.w = 0.  # invalid quaternion must not update localization
    msg.pose.pose.position.x = 10.
    node.on_odom(msg)
    assert node.people.odom.current()[0] == 2.
