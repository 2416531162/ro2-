"""屏幕操作的请求/实际状态分离，HTTP 不触碰运动控制。"""
import http.client
import json
from pathlib import Path
import sys
import threading
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'radar_system'))
from display_control import ScreenControl


def test_window_ack_and_desktop_state_are_explicit(tmp_path):
    control = ScreenControl(tmp_path)
    assert control.desired() == {'action': 'show', 'request_id': None}
    assert control.status()['visible'] is None
    with pytest.raises(RuntimeError):
        control.request('show')
    control.report(True, None, desktop=False)
    request = control.request('desktop')
    assert control.desired() == request
    assert control.status()['visible'] is True
    assert control.status()['applied_request_id'] is None
    control.report(False, request['request_id'], desktop=True)
    assert control.status()['desktop'] is True
    assert control.status()['applied_request_id'] == request['request_id']
    reopened = ScreenControl(tmp_path)
    show = reopened.request('show')
    control.report(True, show['request_id'], desktop=False)
    assert control.status()['visible'] is True
    assert control.status()['desktop'] is False
    with pytest.raises(ValueError):
        control.request('show; shutdown')
    assert (tmp_path / 'request.json').stat().st_mode & 0o777 == 0o600


def test_stale_window_never_looks_like_a_confirmed_desktop(tmp_path):
    control = ScreenControl(tmp_path)
    with patch('display_control.time.monotonic', return_value=10):
        control.report(False, 'old', desktop=True)
    with patch('display_control.time.monotonic', return_value=20):
        state = control.status()
    assert state['window_running'] is False
    assert state['visible'] is None and state['desktop'] is None and state['applied_request_id'] is None


def test_display_http_only_accepts_fixed_actions_and_never_reads_motion(tmp_path, monkeypatch):
    from test_web_control import web
    control = ScreenControl(tmp_path)
    monkeypatch.setattr(web, 'SCREEN_CONTROL', control)
    monkeypatch.setattr(web, 'snapshot_state', lambda: (_ for _ in ()).throw(AssertionError('不应读取或修改运动控制')))
    server = web.ThreadedHTTPServer(('127.0.0.1', 0), web.RadarHTTPHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=3)
    def request(method, body=None, origin=False):
        headers = {'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = 'https://untrusted.invalid'
        conn.request(method, '/api/display', json.dumps(body) if body is not None else None, headers)
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    try:
        assert request('GET')[1]['actions'] == ['desktop', 'show']
        assert request('POST', {'action': 'show'})[0] == 503
        control.report(True, None, desktop=False)
        for bad in ({'action': 'reboot'}, {'action': 'show', 'cmd': 'x'}, [], None):
            assert request('POST', bad)[0] == 400
        assert request('GET', origin=True)[0] == 403
        assert request('POST', {'action': 'desktop'}, origin=True)[0] == 403
        code, response = request('POST', {'action': 'desktop'})
        assert code == 200 and response['status'] == 'pending'
        assert control.status()['visible'] is True
        control.report(False, response['request_id'], desktop=True)
        assert request('GET')[1]['desktop'] is True
        assert request('POST', {'action': 'show'})[0] == 200
    finally:
        conn.close(); server.shutdown(); server.server_close()
