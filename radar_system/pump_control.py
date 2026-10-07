"""板子保持已明确开启的水泵；输出进程仍由板子续租，断开平板不关泵。"""
import json
import math
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path


def validate_command(msg):
    action = msg.get('action')
    if action not in ('set', 'keepalive', 'mode'):
        raise ValueError('pump.action 必须是 set/keepalive/mode')
    run = msg.get('run')
    if not isinstance(run, str) or not 1 <= len(run) <= 32 or not run.isalnum():
        raise ValueError('pump.run 必须是 1–32 位字母或数字')
    out = {'type': 'pump', 'action': action, 'run': run}
    if action in ('set', 'mode'):
        if type(msg.get('enabled')) is not bool:
            raise ValueError('pump.enabled 必须是布尔值')
        pressure = msg.get('pressure')
        if type(pressure) is not int or not 1 <= pressure <= 100:
            raise ValueError('pump.pressure 必须是 1–100 的整数')
        out.update(enabled=msg['enabled'], pressure=pressure)
    return out


class PumpControl:
    def __init__(self, enabled=False, frequency=50.0, launcher=None, backend='gpio'):
        if backend not in ('gpio', 'hc08'):
            raise ValueError('水泵输出方式必须为 gpio/hc08')
        self.backend = backend
        self.configured = bool(enabled)
        self.frequency = float(frequency)
        if not math.isfinite(self.frequency) or not 1 <= self.frequency <= 100:
            raise ValueError('水泵 PWM 频率必须为 1–100 Hz')
        self.lock = threading.RLock()
        self.process = None
        self.ready = False
        self.active = False
        self.wanted = False
        self.pressure = 50
        self.target_pressure = 50
        self.owner = self.run = self.token = None
        self.deadline = 0.0
        self.error = None
        self.reason = 'idle'
        self.latched = False
        self.closed = False
        # 旧 App 手动开关继续兼容；新 App 明确选择 simulation 或 real。
        # real 允许手动开关与作业自动喷线，不在切换模式时直接打开输出。
        self.output_mode = 'manual'
        self.manual_requested = False
        self.launcher = launcher or self._launch
        self.started = threading.Event()
        self.supervisor_stop = threading.Event()
        self.supervisor = None
        self.blocker = lambda: None

    @classmethod
    def from_env(cls):
        return cls(os.environ.get('RK3588_PUMP_ENABLED') == '1',
                   float(os.environ.get('RK3588_PUMP_PWM_HZ', '50')),
                   backend=os.environ.get('RK3588_PUMP_BACKEND', 'hc08'))

    def _launch(self):
        worker = 'hc08_pump_worker.py' if self.backend == 'hc08' else 'pump_worker.py'
        return subprocess.Popen([sys.executable, '-u', str(Path(__file__).with_name(worker))],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                text=True, bufsize=1)

    def start(self, blocker=None):
        with self.lock:
            if not self.configured or self.closed or self.process is not None:
                return
            self.started.clear()
            try:
                self.process = self.launcher()
                process = self.process
                self.blocker = blocker or (lambda: None)
                threading.Thread(target=self._read, args=(process,), daemon=True).start()
                self.supervisor = threading.Thread(target=self._supervise, daemon=True)
                self.supervisor.start()
            except Exception as exc:
                self.error = str(exc)
                self.started.set()
        # 不在网络指令接收循环里初始化 GPIO 或蓝牙。
        self.started.wait(2.0)

    def _supervise(self):
        while not self.supervisor_stop.wait(0.5):
            try:
                blocker = self.blocker()
            except Exception:
                blocker = 'chassis_stale'
            self.tick(blocker)

    def tick(self, blocker=None):
        """板子检查安全状态并给输出进程续租，不依赖平板在线。"""
        with self.lock:
            if not self.wanted or self.closed:
                return
            if blocker or self.latched:
                self.stop(blocker or 'estop')
                return
            self._send({'action': 'keepalive'})

    def release_session(self, session):
        with self.lock:
            if self.owner == session:
                self.owner = self.run = None

    def _read(self, process):
        try:
            for raw in process.stdout:
                msg = json.loads(raw)
                with self.lock:
                    if process is not self.process:
                        return
                    if 'ready' in msg:
                        self.ready = bool(msg['ready'])
                        if self.ready:
                            self.error = None
                        self.started.set()
                    if 'frequency_hz' in msg:
                        self.frequency = msg['frequency_hz']
                    if msg.get('error'):
                        self.error = msg['error']
                        self.wanted = self.active = False
                    elif msg.get('token') == self.token and 'enabled' in msg:
                        self.active = bool(msg['enabled'])
                        self.pressure = msg.get('pressure', self.pressure)
                    if msg.get('reason') == 'command_timeout':
                        self.active = self.wanted = False
                        self.reason = 'command_timeout'
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            with self.lock:
                self.ready = self.active = self.wanted = False
                if not self.closed and not self.error:
                    self.error = '水泵输出进程已退出，检查后重启车端服务'
                self.started.set()

    def _send(self, command):
        if self.process is None or self.process.poll() is not None:
            self.ready = self.active = self.wanted = False
            self.error = '水泵输出进程不可用'
            return False
        try:
            self.process.stdin.write(json.dumps(command) + '\n')
            self.process.stdin.flush()
            return True
        except (OSError, ValueError) as exc:
            self.ready = self.active = self.wanted = False
            self.error = str(exc)
            return False

    def command(self, session, command, blocker=None, now=None):
        cmd = validate_command(command)
        now = time.monotonic() if now is None else now
        with self.lock:
            if cmd['action'] == 'mode':
                if cmd['enabled'] and (not self.configured or self.closed or not self.ready or self.error):
                    return False, self.error or '车端水泵输出未就绪'
                if cmd['enabled'] and (blocker or self.latched):
                    return False, blocker or '急停已锁存，请先复位'
                self.stop('output_mode')
                self.target_pressure = cmd['pressure']
                self.output_mode = 'real' if cmd['enabled'] else 'simulation'
                return True, '真实驱动已开启，合格喷线时自动开泵' if cmd['enabled'] else '真实驱动已关闭，水泵仅模拟'
            if cmd['action'] == 'keepalive':
                if not self.wanted or self.owner != session or self.run != cmd['run']:
                    return False, '水泵已关闭，请重新手动打开'
                if blocker or self.latched:
                    self.stop(blocker or 'estop')
                    return False, '水泵已停止，请重新手动打开'
                self.deadline = now + 2.0
                return self._send({'action': 'keepalive'}), ''
            if not cmd['enabled']:
                # 任何连接都可关泵；关闭不受急停/占用限制。
                if self.output_mode == 'real' and not self.manual_requested:
                    self.output_mode = 'simulation'
                self.target_pressure = cmd['pressure']
                if not self.wanted and not self.active:
                    self.pressure = cmd['pressure']
                self.stop('user')
                return True, '水泵已关闭'
            if self.output_mode == 'simulation':
                return False, '真实驱动已关闭，水泵仅模拟'
            if not self.configured or self.closed or not self.ready or self.error:
                return False, self.error or '车端水泵输出未启用'
            if blocker or self.latched:
                return False, blocker or '急停已锁存，请先复位'
            if self.wanted and self.owner is not None and self.owner != session:
                return False, '另一台平板正在控制水泵，请先关闭'
            # 启动命令不是续租。每次显式开启/调压都用新的 token，丢弃迟到的旧确认。
            self.token = uuid.uuid4().hex
            self.owner, self.run = session, cmd['run']
            self.target_pressure = cmd['pressure']
            self.wanted = True
            self.manual_requested = session != 'automatic'
            self.deadline = now + 2.0
            self.reason = 'user'
            ok = self._send(dict(cmd, token=self.token))
            return ok, '' if ok else self.error

    def automatic(self, painting, blocker=None, *, job_active=True):
        """由 50 Hz 真实喷线资格触发；不根据 App 图形或虚拟喷量打开硬件。"""
        with self.lock:
            if self.output_mode != 'real':
                return
            # 没有自动任务时，手动开泵由操作员保持；空闲的 trace 不能每拍把它关掉。
            if not job_active:
                if blocker or self.latched or not self.manual_requested:
                    self.stop(blocker or ('estop' if self.latched else 'paint_protected'))
                return
            # 开始 / 暂停作业后交回喷线资格控制，先结束此前的手动输出。
            if self.manual_requested:
                self.stop('automatic_control')
            if painting is not True or blocker or self.latched:
                self.stop(blocker or 'paint_protected')
                return
            if not self.wanted:
                self.command('automatic', dict(type='pump', action='set', enabled=True,
                    pressure=self.target_pressure, run='automatic'), blocker)

    def stop(self, reason='user', session=None, latch=False):
        with self.lock:
            if session is not None and session != self.owner:
                return
            self.latched = self.latched or latch
            was_wanted = self.wanted
            self.wanted = False
            self.manual_requested = False
            if was_wanted:
                self.token = uuid.uuid4().hex
            self.reason = reason
            self.owner = self.run = None
            if self.ready and was_wanted:
                self._send({'action': 'set', 'enabled': False, 'pressure': self.target_pressure, 'token': self.token})

    def reset(self):
        with self.lock:
            self.latched = False

    def status(self, blocker=None, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.wanted and (blocker or self.latched):
                self.stop(blocker or 'estop')
            return {'supported': self.configured, 'available': self.ready and not self.error and not self.closed,
                    'output_mode_supported': True, 'output_mode': self.output_mode,
                    'enabled': self.active, 'pending': self.wanted != self.active,
                    'pressure': self.pressure, 'frequency_hz': self.frequency,
                    'pin': 'MCU_RIGHT_DOWN' if self.backend == 'hc08' else 'GPIO0_C6',
                    'active_level': 'controller' if self.backend == 'hc08' else 'high',
                    'off_level': 'controller' if self.backend == 'hc08' else 'low',
                    'connection_independent': True,
                    'session': self.owner, 'run': self.run, 'reason': self.reason,
                    'error': self.error, 'blocked': blocker or ('estop' if self.latched else None)}

    def close(self):
        with self.lock:
            self.stop('shutdown')
            self.closed = True
            self.supervisor_stop.set()
            process = self.process
            if process is not None:
                self._send({'action': 'close'})
                process.stdin.close()
        if process is not None:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=2.0)
