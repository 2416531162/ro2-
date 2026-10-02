import os
import socket
import threading
from pathlib import Path

from robot_core.service_watchdog import ServiceWatchdog


def test_no_background_thread_outside_systemd():
    w = ServiceWatchdog()
    w.start('ros', 'io', environ={})
    assert w._thread is None
    w.tick('io')
    assert not w.notify()


def test_one_live_thread_cannot_hide_a_stalled_io_loop(tmp_path):
    now = [10.0]
    w = ServiceWatchdog(lambda: now[0])
    # macOS has a short sockaddr_un limit; use a short /tmp socket path.
    path = '/tmp/rk-watchdog-%s.sock' % os.getpid()
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver:
        receiver.bind(path)
        receiver.settimeout(0.1)
        try:
            w.start('ros', 'io', environ={'NOTIFY_SOCKET': path, 'RK3588_WATCHDOG_SEC': '20'})
            assert receiver.recv(100) == b'WATCHDOG_USEC=20000000'
            assert not w.healthy()
            w.tick('ros')
            assert not w.healthy()
            w.tick('io')
            assert w.notify()
            assert receiver.recv(100) == b'WATCHDOG=1'
            now[0] += 5.1
            w.tick('ros')
            assert not w.notify()  # notifier and ROS are alive, I/O is stuck
            w.tick('io')
            assert w.healthy()
        finally:
            w.close()
            Path(path).unlink()


def test_child_cannot_feed_parent_watchdog():
    w = ServiceWatchdog()
    w.start('ros', environ={'NOTIFY_SOCKET': '/not-used', 'WATCHDOG_USEC': '1000000',
                            'WATCHDOG_PID': str(os.getpid() + 1)})
    assert w._thread is None


def test_managed_units_allow_only_main_process_notifications():
    from deployment.manage import units
    rendered = units(Path('/opt/rk3588'))
    for name in ('rk3588-wheeltec.service', 'rk3588-perception@.service'):
        assert 'NotifyAccess=main\n' in rendered[name]
        assert 'Environment=RK3588_WATCHDOG_SEC=20\n' in rendered[name]
