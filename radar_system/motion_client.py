"""Behavior-side ROS adapter. Behaviors never arm or write the chassis directly."""
import json
import threading
import time
from runtime_config import PROFILE, profile_hash
from std_msgs.msg import String
from std_srvs.srv import SetBool


class MotionClient:
    def __init__(self, node, source, timeout_s=0.5):
        self.node, self.source = node, source
        self.timeout_s = float(timeout_s)
        self.state, self.received = {}, None
        self.lock = threading.RLock()
        self.publisher = node.create_publisher(String, '/' + source + '/command', 1)
        node.create_subscription(String, '/motion/status', self.on_status, 1)
        self.selector = node.create_client(SetBool, '/motion/' + source) if source != 'manual' else None

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and isinstance(data.get('epoch'), str):
                with self.lock:
                    self.state, self.received = data, time.monotonic()
        except (ValueError, TypeError):
            pass

    def fresh(self):
        with self.lock:
            return (self.received is not None and 0 <= time.monotonic() - self.received < self.timeout_s
                    and self.state.get('profile_hash') == profile_hash(PROFILE)
                    and not self.state.get('legacy_commands', False))

    def active(self):
        with self.lock:
            return self.fresh() and self.state.get('mode') == self.source.upper()

    def select(self, enabled=True):
        if (enabled and not self.fresh()) or self.selector is None or not self.selector.service_is_ready():
            return None
        return self.selector.call_async(SetBool.Request(data=enabled))

    def publish(self, vx, wz):
        with self.lock:
            if not self.fresh():
                return False
            payload = dict(vx=float(vx), wz=float(wz), epoch=self.state['epoch'],
                           profile_hash=profile_hash(PROFILE),
                           stamp=self.node.get_clock().now().nanoseconds / 1e9)
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        return True
