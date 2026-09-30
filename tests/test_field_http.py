"""Actual HTTP handler and ROS forwarding, without vehicle hardware."""
import http.client
import json
import threading
import time
import unittest
from unittest.mock import patch
from test_web_control import web
from test_field_planner import BOUNDARY, LINE
import app_link


class TestFieldHttp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=web.ThreadedHTTPServer(('127.0.0.1',0),web.RadarHTTPHandler)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown();cls.server.server_close()
    def request(self,headers=None,job=None,**line_changes):
        line=dict(origin=[30,120],phase='IDLE',motion_mode='IDLE',_rx_monotonic=time.monotonic(),
                  pose=dict(x=3,y=5,yaw_deg=0,speed=0,usable=True),route=dict(rev=7),
                  route_limits=dict(drive='differential',body=[.911,.604,.39],min_radius=.556))
        line.update(line_changes)
        data=dict(origin=[30,120],nozzle=[.3,.24],job=job or dict(boundary=BOUNDARY,elements=[LINE]))
        conn=http.client.HTTPConnection('127.0.0.1',self.server.server_address[1],timeout=5)
        try:
            with patch.object(web,'snapshot_state',return_value={'line':line}):
                conn.request('POST','/api/field/plan',json.dumps(data),headers or {'Content-Type':'application/json'})
                response=conn.getresponse();self.assertIsNotNone(response.getheader('Content-Length'))
                return response.status,json.loads(response.read())
        finally:conn.close()
    def test_real_post_prepares_without_motion_and_forwarding_uses_only_cached_motions(self):
        bridge=web.WebBridgeNode();before=len(bridge.pub_line.sent)
        status,data=self.request()
        self.assertEqual(200,status,data);self.assertTrue(data['ok'])
        self.assertEqual(before,len(bridge.pub_line.sent));self.assertNotIn('motions',data['plan'])
        token=data['plan']['token']
        cmd=app_link.parse_command(json.dumps(dict(type='route',action='field_start',id='start',preview_token=token,
                                           allow_outside=True,field_plan={'motions':'forged'})))
        bridge.route_command(cmd)
        forwarded=json.loads(bridge.pub_line.sent[-1].data)
        self.assertEqual('route_field_start',forwarded['cmd'])
        self.assertEqual(web.FIELD_PLANS.get(token)['motions'],forwarded['field_plan']['motions'])
        self.assertTrue(forwarded['allow_outside'])
        self.assertEqual(7,forwarded['field_plan']['route_rev'])
    def test_stale_busy_and_cross_origin_requests_rejected(self):
        for changes in ({'_rx_monotonic':time.monotonic()-10},{'phase':'RUNNING'}):
            status,data=self.request(**changes)
            self.assertEqual(400,status);self.assertFalse(data['ok'])
        status,data=self.request({'Origin':'https://example.invalid','Content-Type':'application/json'})
        self.assertEqual(403,status);self.assertFalse(data['ok'])
    def test_missing_token_never_publishes_motion(self):
        bridge=web.WebBridgeNode();before=len(bridge.pub_line.sent)
        with self.assertRaises(ValueError):bridge.route_command(dict(action='field_start',preview_token='missing'))
        self.assertEqual(before,len(bridge.pub_line.sent))
    def test_http_selection_is_preserved_in_cached_start(self):
        chosen=[dict(element_id='frame',edge_index=3)]
        job=dict(boundary=BOUNDARY,elements=[dict(id='frame',kind='polyline',points=BOUNDARY,closed=True)],selected_strokes=chosen)
        status,data=self.request(job=job)
        self.assertEqual(200,status,data)
        self.assertEqual('selected',data['plan']['selection_mode'])
        self.assertEqual(chosen,[v['selection'] for v in data['plan']['order']])
        cached=web.FIELD_PLANS.get(data['plan']['token'])
        self.assertEqual(chosen,cached['source_job']['selected_strokes'])
        self.assertEqual(1,sum(m['role']=='paint' for m in cached['motions']))
        status,data=self.request(job=dict(job,selected_strokes=[]))
        self.assertEqual(400,status);self.assertFalse(data['ok'])


if __name__=='__main__':unittest.main()
