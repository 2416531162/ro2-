#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CORS 配置的校验、原子保存和热重载，不依赖 ROS，不在响应或日志中返回密码。"""
import math
import os
import re
import stat
import tempfile
import threading

from um982 import load_ntrip_env


FIELDS = {'host': 'NTRIP_HOST', 'port': 'NTRIP_PORT', 'mount': 'NTRIP_MOUNT',
          'user': 'NTRIP_USER', 'password': 'NTRIP_PASS', 'fallback_mount': 'NTRIP_MOUNT_FALLBACK'}


def validate_profile(value):
    if not isinstance(value, dict):
        raise ValueError('配置必须是 JSON 对象')
    out = {}
    for field in FIELDS:
        raw = value.get(field, '' if field != 'port' else 2101)
        if field == 'port':
            if isinstance(raw, bool) or not isinstance(raw, (int, str)):
                raise ValueError('端口应为 1–65535')
            try:
                port = int(raw)
            except ValueError:
                raise ValueError('端口应为 1–65535') from None
            if not 1 <= port <= 65535:
                raise ValueError('端口应为 1–65535')
            out[field] = port
            continue
        if not isinstance(raw, str) or len(raw) > 1024 or any(ord(c) < 32 or ord(c) == 127 for c in raw):
            raise ValueError('%s 格式无效' % field)
        if raw != raw.strip():
            raise ValueError('%s 首尾不能包含空白' % field)
        if field != 'fallback_mount' and not raw:
            raise ValueError('%s 不能为空' % field)
        out[field] = raw
    if not re.fullmatch(r'[A-Za-z0-9.:-]{1,253}', out['host']):
        raise ValueError('服务器应为域名或 IP，不包含网址路径')
    for field in ('mount', 'fallback_mount'):
        if out[field] and not re.fullmatch(r'[A-Za-z0-9._~+-]{1,256}', out[field]):
            raise ValueError('挂载点格式无效')
    if ':' in out['user'] or len(out['user']) > 256:
        raise ValueError('账号不能包含冒号且不能超过 256 字符')
    return out


def profile_env(profile):
    return {key: str(profile[field]) for field, key in FIELDS.items() if profile[field] != ''}


def ntrip_enabled(env):
    if env is None:
        return False
    value = env.get('NTRIP_ENABLED', '1')
    if value not in ('0', '1'):
        raise ValueError('NTRIP_ENABLED 应为 0 或 1')
    return value == '1'


def revision(path):
    try:
        info = os.stat(path)
    except FileNotFoundError:
        return None
    return '%d-%d-%d' % (info.st_mtime_ns, info.st_size, info.st_ino)


class CorsConfigStore:
    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.lock = threading.Lock()

    def public(self):
        with self.lock:
            return self._public()

    def _public(self):
        env = load_ntrip_env(self.path)
        if env is None:
            return dict(configured=False, enabled=False, revision=None, profile=None)
        profile = {field: env.get(key, '') for field, key in FIELDS.items() if field != 'password'}
        profile['port'] = int(profile['port'])
        profile['password_set'] = bool(env.get('NTRIP_PASS'))
        return dict(configured=True, enabled=ntrip_enabled(env), revision=revision(self.path), profile=profile)

    def save(self, value):
        profile = validate_profile(value)
        with self.lock:
            return self._write(dict(profile_env(profile), NTRIP_ENABLED='1'))

    def set_enabled(self, enabled):
        if not isinstance(enabled, bool):
            raise ValueError('连接开关必须是布尔值')
        with self.lock:
            env = load_ntrip_env(self.path)
            if env is None:
                raise ValueError('车端尚未配置账号')
            validate_profile({field: env.get(key, '') for field, key in FIELDS.items()})
            if ntrip_enabled(env) == enabled:
                return self._public()
            # 保留车端凭据，不用平板尚未应用的编辑内容覆盖账号。
            env['NTRIP_ENABLED'] = '1' if enabled else '0'
            return self._write(env)

    def _write(self, env):
        content = ''.join('%s=%s\n' % (key, val) for key, val in env.items())
        parent = os.path.dirname(self.path)
        os.makedirs(parent, mode=0o700, exist_ok=True)
        if os.path.islink(self.path):
            raise ValueError('账号文件不能是符号链接')
        fd, temporary = tempfile.mkstemp(prefix='.ntrip-', dir=parent)
        try:
            os.fchmod(fd, stat.S_IRUSR | stat.S_IWUSR)
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
            directory = os.open(parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return self._public()


def configuration_allowed(snapshot, now):
    """配置变更会中断差分，只允许底盘在线、静止且没有自动作业时应用。"""
    motion = snapshot.get('motion') or {}
    wheel = snapshot.get('wheeltec') or {}
    if not snapshot.get('motion_fresh') or motion.get('mode') not in ('IDLE', 'ESTOP'):
        return False
    velocity = (wheel.get('telemetry') or {}).get('velocity') or []
    speed = velocity[0] if velocity else None
    rx = wheel.get('_rx_monotonic')
    if not wheel.get('connected'):
        return False
    if not isinstance(speed, (int, float)) or not math.isfinite(speed) or abs(speed) > 0.02:
        return False
    if not isinstance(rx, (int, float)) or not 0 <= now - rx <= 1.0:
        return False
    return not snapshot.get('follower_running') and (snapshot.get('line') or {}).get('phase', 'IDLE') == 'IDLE'


class NtripReload:
    def __init__(self, path, client_factory):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.client_factory = client_factory
        self.client = None
        self.seen = object()
        self.applied_revision = None
        self.configured = False
        self.enabled = False
        self.error = 'not_configured'
        # 暂停原因（'tablet'：接收机数据来自平板串口，平板用同一账号自己连 CORS，车端再连会互相顶掉）；
        # 暂停时照常跟踪账号文件（app 的保存 / 断开回执不受影响），只是不起客户端
        self.paused = None
        self.env = None
        self.poll()

    def pause(self, reason):
        if self.paused == reason:
            return
        self.paused = reason
        if self.client is not None:
            self.client.stop()
            self.client = None

    def resume(self):
        if self.paused is None:
            return
        self.paused = None
        self._start()

    def poll(self):
        try:
            current = revision(self.path)
            if current == self.seen:
                return
            env = load_ntrip_env(self.path)
            enabled = ntrip_enabled(env)
            if env is not None:
                validate_profile({field: env.get(key, '') for field, key in FIELDS.items()})
        except (OSError, ValueError, TypeError):
            self.error = 'config_invalid'
            return
        if self.client is not None:
            self.client.stop()
            self.client = None
        self.seen = current
        self.applied_revision = current
        self.configured = env is not None
        self.enabled = enabled
        self.env = env
        self.error = None if env else 'not_configured'
        self._start()

    def _start(self):
        if not self.enabled or self.env is None or self.paused is not None or self.client is not None:
            return
        try:
            self.client = self.client_factory(self.env).start()
        except (OSError, RuntimeError):
            self.applied_revision = None
            self.seen = object()
            self.error = 'client_start_failed'

    def stop(self):
        if self.client is not None:
            self.client.stop()
