"""CORS 从平板 HTTP 保存到 RTK 热重载；测试账号只连本机假 caster。"""
import http.client
import json
import os
import sys
import threading
import time
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'radar_system'))
from ntrip_config import CorsConfigStore, NtripReload, configuration_allowed, validate_profile
from um982 import NtripClient
from test_um982 import FakeCaster, GGA

PROFILE = dict(host='127.0.0.1', port=2101, mount='AUTO', user='u', password='p', fallback_mount='')


def idle(now):
    return dict(motion_fresh=True, motion={'mode': 'IDLE'}, follower_running=False, line={'phase': 'IDLE'},
                wheeltec={'connected': True, '_rx_monotonic': now, 'telemetry': {'velocity': [0.0, 0.0, 0.0]}})


def test_atomic_private_save_and_redaction(tmp_path):
    store = CorsConfigStore(str(tmp_path / 'rtk/ntrip.env'))
    assert not store.public()['configured']
    saved = store.save(dict(PROFILE, password='private_test_only'))
    assert saved['configured'] and saved['profile']['password_set']
    assert 'private_test_only' not in json.dumps(saved)
    assert 'password' not in saved['profile']
    assert os.stat(store.path).st_mode & 0o777 == 0o600
    old = saved['revision']
    assert store.save(PROFILE)['revision'] != old
    before = Path(store.path).read_bytes()
    with pytest.raises(ValueError):
        store.save(dict(PROFILE, password='p\nNTRIP_HOST=bad'))
    assert Path(store.path).read_bytes() == before


@pytest.mark.parametrize('change', [dict(port=0), dict(port=True), dict(port=1.5), dict(user='u:p'),
                                  dict(host='http://server'), dict(mount='../AUTO'), dict(password=' p'),
                                  dict(password=''), dict(user='u\rtest')])
def test_invalid_profiles(change):
    with pytest.raises(ValueError):
        validate_profile(dict(PROFILE, **change))


def test_only_idle_fresh_stationary_vehicle():
    s = idle(10)
    assert configuration_allowed(s, 10.1)
    assert not configuration_allowed(s, 12)
    s['wheeltec']['telemetry']['velocity'][0] = .1
    assert not configuration_allowed(s, 10.1)
    s['wheeltec']['telemetry']['velocity'][0] = 0
    s['motion']['mode'] = 'MANUAL'
    assert not configuration_allowed(s, 10.1)
    s['motion']['mode'] = 'IDLE'
    s['line']['phase'] = 'SELECTING'
    assert not configuration_allowed(s, 10.1)


@pytest.mark.parametrize('chunks', [
    [b'HTTP/1.1 200 OK\r\nServer: test\r\n', b'\r\n' + b'\xd3\x00\x03\x43\xe0\x00\x00\x00\x00'],
    [b'ICY 200 OK\r\n\xd3\x00\x03\x43\xe0\x00\x00\x00\x00']])
def test_split_headers_and_first_correction_are_not_lost(chunks):
    class Socket:
        def __init__(self):
            self.chunks = iter(chunks)
        def recv(self, _n):
            return next(self.chunks, b'')
        def sendall(self, _data):
            pass
        def settimeout(self, _value):
            pass
        def close(self):
            pass
    received = []
    env = dict(NTRIP_HOST='127.0.0.1', NTRIP_PORT='2101', NTRIP_MOUNT='AUTO', NTRIP_USER='u', NTRIP_PASS='p')
    client = NtripClient(env, lambda: None, received.append)
    with patch('um982.socket.create_connection', return_value=Socket()):
        client._session('AUTO')
    assert b''.join(received) == b'\xd3\x00\x03\x43\xe0\x00\x00\x00\x00'
    assert client.status()['bytes'] == 9 and client.status()['rtcm_types'] == [1086]


def test_reload_stops_old_client_and_keeps_good_config_on_invalid_file(tmp_path):
    clients = []
    class Client:
        def __init__(self, env):
            self.env, self.stopped = env, False
            clients.append(self)
        def start(self):
            return self
        def stop(self):
            self.stopped = True
    store = CorsConfigStore(str(tmp_path / 'ntrip.env'))
    reload = NtripReload(store.path, Client)
    assert reload.error == 'not_configured'
    saved = store.save(PROFILE)
    reload.poll()
    assert reload.applied_revision == saved['revision'] and len(clients) == 1
    reload.poll()
    assert len(clients) == 1
    store.save(dict(PROFILE, mount='NEW'))
    reload.poll()
    assert clients[0].stopped and clients[1].env['NTRIP_MOUNT'] == 'NEW'
    Path(store.path).write_text('bad')
    reload.poll()
    assert reload.error == 'config_invalid' and not clients[1].stopped
    os.unlink(store.path)
    reload.poll()
    assert clients[1].stopped and reload.client is None and reload.applied_revision is None


def test_http_save_reload_and_receive_corrections(tmp_path, monkeypatch):
    # 复用现有 ROS 替身，走真实 HTTP 和 NTRIP socket，验证整条配置链路。
    from test_web_control import web
    caster = FakeCaster()
    received = []
    store = CorsConfigStore(str(tmp_path / 'ntrip.env'))
    monkeypatch.setattr(web, 'CORS_CONFIG', store)
    monkeypatch.setattr(web, 'snapshot_state', lambda: idle(time.monotonic()))
    server = web.ThreadedHTTPServer(('127.0.0.1', 0), web.RadarHTTPHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    reload = NtripReload(store.path, lambda env: NtripClient(env, lambda: GGA + '\r\n', received.append))
    conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=2)
    def request(method, value=None, headers=None):
        conn.request(method, '/api/rtk/cors', None if value is None else json.dumps(value),
                     headers or {'Content-Type': 'application/json'})
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    try:
        assert request('GET')[1]['supported']
        status, saved = request('POST', dict(PROFILE, port=caster.port))
        assert status == 200 and saved['status'] == 'pending_reload'
        assert 'password' not in saved['profile']
        reload.poll()
        assert reload.applied_revision == saved['revision']
        deadline = time.monotonic() + 3
        while not received and time.monotonic() < deadline:
            time.sleep(.01)
        assert received and any(b'\xd3' in b for b in received)
        assert caster.ggas
        status, data = request('POST', PROFILE, {'Origin': 'http://untrusted.example'})
        assert status == 403
        bad = idle(time.monotonic())
        bad['motion']['mode'] = 'FOLLOW'
        monkeypatch.setattr(web, 'snapshot_state', lambda: bad)
        assert request('POST', PROFILE)[0] == 409
        assert store.public()['revision'] == saved['revision']
        reload.stop()
        assert not reload.client.thread.is_alive()
    finally:
        reload.stop()
        conn.close()
        server.shutdown()
        server.server_close()
        caster.srv.close()
