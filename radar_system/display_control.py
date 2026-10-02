#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车载窗口开关的本机通信。只读状态与固定动作，不执行 App 传来的命令。"""
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

ACTIONS = ('desktop', 'show')


class ScreenControl:
    def __init__(self, directory=None):
        runtime = os.environ.get('XDG_RUNTIME_DIR', '/run/user/%d' % os.getuid())
        self.directory = Path(directory or os.environ.get('RK3588_DISPLAY_STATE_DIR',
                                                         os.path.join(runtime, 'rk3588-display')))

    def _read(self, name):
        try:
            text = (self.directory / name).read_text(encoding='utf-8')
            if len(text) > 4096:
                return {}
            value = json.loads(text)
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write(self, name, value):
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        fd, path = tempfile.mkstemp(prefix='screen-', dir=str(self.directory))
        try:
            os.fchmod(fd, 0o600)
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(value, stream)
            os.replace(path, self.directory / name)
        finally:
            if os.path.exists(path):
                os.unlink(path)

    def desired(self):
        value = self._read('request.json')
        if value.get('action') not in ACTIONS or not isinstance(value.get('request_id'), str):
            return {'action': 'show', 'request_id': None}
        return value

    def report(self, visible, request_id, running=True, desktop=None):
        self._write('status.json', {'visible': bool(visible), 'running': bool(running),
                                   'desktop': desktop, 'applied_request_id': request_id, 'heartbeat': time.monotonic()})

    def status(self):
        actual = self._read('status.json')
        stamp = actual.get('heartbeat')
        fresh = isinstance(stamp, (int, float)) and 0 <= time.monotonic() - stamp < 4
        running = fresh and actual.get('running') is True
        requested = self.desired()
        visible = actual.get('visible') if running and isinstance(actual.get('visible'), bool) else None
        return {'supported': True, 'actions': list(ACTIONS), 'window_running': bool(running),
                'visible': visible, 'requested': requested['action'], 'request_id': requested['request_id'],
                'desktop': actual.get('desktop') if running else None,
                'applied_request_id': actual.get('applied_request_id') if running else None}

    def request(self, action):
        if action not in ACTIONS:
            raise ValueError('invalid_action')
        if not self.status()['window_running']:
            raise RuntimeError('window_not_running')
        request = {'action': action, 'request_id': uuid.uuid4().hex}
        self._write('request.json', request)
        return request
