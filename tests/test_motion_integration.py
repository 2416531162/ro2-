"""控制端适配（motion_client）：状态过期、配置不一致时不发指令。ROS 消息用 tests/ros_stubs 的桩。"""
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
sys.path.insert(0, str(ROOT / 'radar_system'))
sys.path.insert(0, str(ROOT / 'wheeltec_protocol'))
import ros_stubs  # noqa: E402

ros_stubs.install()
from motion_client import MotionClient  # noqa: E402
from runtime_config import PROFILE, profile_hash  # noqa: E402


def status(mode='NAVIGATION', epoch='lease'):
    return ros_stubs.String(data=json.dumps(dict(mode=mode, epoch=epoch, healthy=True,
                                                 profile_hash=profile_hash(PROFILE))))


def test_client_discards_stale_status_and_carries_source_stamp():
    client = MotionClient(ros_stubs.Node(), 'manual')
    with patch('motion_client.time.monotonic', return_value=10.):
        client.on_status(status('IDLE', 'epoch'))
        assert client.publish(.2, .1)
    data = json.loads(client.publisher.sent[-1].data)
    assert data['stamp'] == 1000. and data['epoch'] == 'epoch'
    with patch('motion_client.time.monotonic', return_value=10.51):
        assert not client.publish(.2, .1)


def test_client_configuration_mismatch_refuses_selection_and_commands():
    client = MotionClient(ros_stubs.Node(), 'navigation')
    wrong = json.loads(status().data)
    wrong['profile_hash'] = 'different-profile'
    client.on_status(ros_stubs.String(data=json.dumps(wrong)))
    assert not client.fresh()
    assert not client.publish(.2, 0.)
    assert client.select(True) is None
