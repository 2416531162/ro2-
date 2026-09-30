"""Exercise the real main HTTP loop and its SIGTERM handler in a subprocess."""
import os
from pathlib import Path
import signal
import subprocess
import sys


def test_sigterm_exits_without_systemd_kill():
    root = Path(__file__).resolve().parents[1]
    code = '''
import sys, time
sys.path.insert(0, 'tests')
import test_web_control as fixture
w = fixture.web
w.PORT = 0
w.rclpy.ok = lambda: False
w.NETWORK.start = lambda: None
w.discovery_beacon = lambda: None
def fake_ros():
    while True:
        time.sleep(.05)
w.ros_worker = fake_ros
w.main()
'''
    env = {k: v for k, v in os.environ.items() if k not in ('NOTIFY_SOCKET', 'WATCHDOG_USEC', 'WATCHDOG_PID')}
    p = subprocess.Popen([sys.executable, '-u', '-c', code], cwd=root, env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        import select
        ready, _, _ = select.select([p.stdout], [], [], 15)
        assert ready, 'HTTP server did not start'
        assert 'http://0.0.0.0:0' in p.stdout.readline()
        p.send_signal(signal.SIGTERM)
        _, err = p.communicate(timeout=4)
        assert p.returncode == 0, err
    finally:
        if p.poll() is None:
            p.kill()
            p.communicate()
