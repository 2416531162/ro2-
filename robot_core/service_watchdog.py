"""Feed systemd only while every required execution loop makes progress.

Sensor availability is deliberately separate: a live loop reporting a missing
sensor is healthy here. Missing data must stop motion, not cause a restart loop.
Without systemd's watchdog environment this module starts no thread.
"""
import os
import socket
import threading
import time


class ServiceWatchdog:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._progress = {}
        self._stop = threading.Event()
        self._thread = None
        self.address = None
        self.max_age_s = 5.0

    def start(self, *loops, environ=None):
        env = os.environ if environ is None else environ
        usec = int(env.get('WATCHDOG_USEC', '0')) or int(float(env.get('RK3588_WATCHDOG_SEC', '0')) * 1_000_000)
        pid = int(env.get('WATCHDOG_PID', str(os.getpid())))
        if usec <= 0 or not env.get('NOTIFY_SOCKET') or pid != os.getpid():
            return
        if not loops or self._thread is not None:
            raise ValueError('watchdog needs loops and may only start once')
        self.address = env['NOTIFY_SOCKET']
        self.max_age_s = min(5.0, usec / 2_000_000)
        # Arm only instrumented entry points, including when a shared systemd
        # template also runs legacy programs. Supported since systemd 233:
        # https://www.freedesktop.org/software/systemd/man/sd_notify.html
        if not self._send(('WATCHDOG_USEC=%d' % usec).encode()):
            raise RuntimeError('could not arm systemd watchdog')
        # Every loop must actually run once before the first notification.
        self._progress = dict.fromkeys(loops)
        period = min(1.0, usec / 4_000_000)
        self._thread = threading.Thread(target=self._run, args=(period,), name='service-watchdog', daemon=True)
        self._thread.start()

    def tick(self, loop):
        with self._lock:
            if loop in self._progress:
                self._progress[loop] = self.clock()

    def healthy(self):
        now = self.clock()
        with self._lock:
            return bool(self._progress) and all(
                stamp is not None and 0 <= now - stamp <= self.max_age_s
                for stamp in self._progress.values())

    def notify(self):
        if not self.address or not self.healthy():
            return False
        return self._send(b'WATCHDOG=1')

    def _send(self, message):
        address = '\0' + self.address[1:] if self.address.startswith('@') else self.address
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
                sock.settimeout(0.2)
                sock.sendto(message, address)
            return True
        except OSError:
            # Never let a blocked notification socket hold a control loop.
            return False

    def _run(self, period):
        while not self._stop.wait(period):
            self.notify()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)


watchdog = ServiceWatchdog()
