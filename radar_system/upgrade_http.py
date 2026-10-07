#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板 app 给车升级（打包、验签、激活见 deployment/upgrade.py）：

    GET  /app/upgrade            当前版本、上一版本、车上已有的发布、升级是否在进行、上一次升级的结果
    POST /app/upgrade/package    请求体是 .rkup 原样（Content-Length 必须给），验签、解包放进 releases/；不激活
    POST /app/upgrade/apply      {"release_id": "…"}：车空闲（底盘在线、静止、没在作业）才起临时单元去激活
    POST /app/upgrade/rollback   {}：回到上一版本（rollback.json 记的那个），条件同 apply

和差分账号一样拒绝带 Origin 的网页跨源请求。激活时网页服务会被停掉重启，app 断开后轮询 GET 看结果。
"""
import json
import os
import sys
import threading
import time
import uuid
from pathlib import Path

_DEPLOY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'deployment')
if _DEPLOY not in sys.path:
    sys.path.insert(0, _DEPLOY)
import upgrade as release_upgrade    # noqa: E402

from ntrip_config import configuration_allowed    # noqa: E402

_lock = threading.Lock()          # 同一时刻只收一个包 / 发起一次激活
CHUNK = 65536


def _send(handler, data, status=200):
    body = json.dumps(data, ensure_ascii=False).encode('utf-8')
    handler.send_response(status)
    handler.send_header('Content-Type', 'application/json; charset=utf-8')
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Cache-Control', 'no-store')
    if handler.close_connection:
        handler.send_header('Connection', 'close')
    handler.end_headers()
    handler.wfile.write(body)


def _fail(handler, status, error, message, close=False):
    if close:
        handler.close_connection = True
    _send(handler, {'ok': False, 'error': error, 'message': message}, status)


def _json_body(handler, limit=4096):
    size = int(handler.headers.get('Content-Length', 0) or 0)
    if not 0 <= size <= limit:
        raise ValueError('请求体过大')
    data = json.loads(handler.rfile.read(size).decode('utf-8') or '{}') if size else {}
    if not isinstance(data, dict):
        raise ValueError('请求体要是 JSON 对象')
    return data


def _receive(handler, base):
    size = int(handler.headers.get('Content-Length', 0) or 0)
    if not 0 < size <= release_upgrade.MAX_PACKAGE + (1 << 20):
        return _fail(handler, 413, 'bad_size', '升级包大小不对（要给 Content-Length，不超过 64 MB）', close=True)
    incoming = Path(base) / 'upgrade' / 'incoming'
    incoming.mkdir(parents=True, exist_ok=True)
    path = incoming / (uuid.uuid4().hex + '.rkup')
    try:
        left = size
        with open(path, 'wb') as out:
            while left:
                chunk = handler.rfile.read(min(CHUNK, left))
                if not chunk:
                    return _fail(handler, 400, 'truncated', '升级包没传完（连接断了）', close=True)
                out.write(chunk)
                left -= len(chunk)
        cur = release_upgrade.current_release(base)
        pubkey = (cur or Path('/nonexistent')) / release_upgrade.PUBKEY
        release, manifest = release_upgrade.unpack(path, Path(base) / 'releases', pubkey)
        rid = manifest['release_id']
        _send(handler, {'ok': True, 'release_id': rid, 'source_commit': manifest.get('source_commit'),
                        'current': cur is not None and cur.name == rid})
    except ValueError as exc:
        _fail(handler, 400, 'invalid_package', str(exc))
    finally:
        try:
            path.unlink()
        except OSError:
            pass


def _start(handler, base, snapshot, now, release):
    if not configuration_allowed(snapshot(), now()):
        return _fail(handler, 409, 'vehicle_not_idle', '车要停着、底盘在线、没在作业才能升级')
    if release_upgrade.unit_running():
        return _fail(handler, 409, 'busy', '上一次升级还在进行')
    cur = release_upgrade.current_release(base)
    if cur is not None and cur.name == release.name:
        return _fail(handler, 400, 'already_current', '车上已经是这个版本')
    try:
        release_upgrade.manage.verify(release)
    except Exception as exc:
        return _fail(handler, 400, 'invalid_release', '车上这个发布不完整：%s' % exc)
    try:
        release_upgrade.write_status(base, state='queued', release=release.name, message='准备升级',
                                     from_release=release_upgrade.describe(cur), error=None,
                                     started=time.time(), finished=None)
        release_upgrade.launch(release, base)
    except (RuntimeError, ValueError, OSError) as exc:
        release_upgrade.write_status(base, state='failed', message='没能开始升级', error=str(exc)[:300],
                                     finished=time.time())
        return _fail(handler, 500, 'launch_failed', '没能开始升级：%s' % exc)
    _send(handler, {'ok': True, 'status': 'started', 'release_id': release.name}, 202)


def handle(handler, snapshot, base=None, now=time.monotonic):
    """http.server 的 handler 上处理 /app/upgrade…；不是这些路径返回 False。"""
    base = Path(base or release_upgrade.BASE)
    path = handler.path.split('?')[0]
    if path != '/app/upgrade' and not path.startswith('/app/upgrade/'):
        return False
    if handler.headers.get('Origin'):
        _fail(handler, 403, 'origin_not_allowed', '不接受网页跨源请求', close=True)
        return True
    if handler.command == 'GET' and path == '/app/upgrade':
        _send(handler, dict(release_upgrade.status(base), ok=True))
        return True
    if handler.command != 'POST' or path not in ('/app/upgrade/package', '/app/upgrade/apply', '/app/upgrade/rollback'):
        _fail(handler, 404, 'not_found', '没有这个接口', close=True)
        return True
    if not _lock.acquire(blocking=False):
        _fail(handler, 409, 'busy', '正在收另一个升级包或在发起升级', close=True)
        return True
    try:
        if path == '/app/upgrade/package':
            _receive(handler, base)
            return True
        try:
            req = _json_body(handler)
        except (ValueError, UnicodeError):
            _fail(handler, 400, 'bad_request', '请求格式不对', close=True)
            return True
        if path == '/app/upgrade/apply':
            rid = req.get('release_id')
            if not isinstance(rid, str) or not release_upgrade.RELEASE_ID.match(rid):
                _fail(handler, 400, 'bad_request', 'release_id 不对')
                return True
            release = base / 'releases' / rid
            if not release.is_dir():
                _fail(handler, 404, 'unknown_release', '车上没有这个发布：先上传升级包')
                return True
        else:
            release = release_upgrade.previous_release(base)
            if release is None:
                _fail(handler, 404, 'no_previous', '车上没有记录上一版本')
                return True
        _start(handler, base, snapshot, now, release)
        return True
    finally:
        _lock.release()
