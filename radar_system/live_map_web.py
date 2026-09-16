#!/usr/bin/env python3
"""8088: live mapping + original controls, shared by the device screen."""
import json
import os
import signal
import threading
from pathlib import Path
from urllib.parse import urlsplit,parse_qs

# Legacy subscriptions combine /map and /projected_map. Keep them off; the new
# bridge is authoritative and never replaces the navigation map with projection.
os.environ['ENABLE_MAP']='0'
import radar_web_server as legacy
import rclpy
from rclpy.executors import MultiThreadedExecutor
from live_map_node import LiveMapNode
from mapping_session import MappingSession

ROOT=Path(__file__).resolve().parent


class DisplayControlBridge(legacy.SLAMBridgeNode):
    def wheeltec_cb(self,msg):
        # Merely opening a map/telemetry page must not automatically arm a robot.
        try:
            data=json.loads(msg.data)
            self.is_armed=bool(data.get('armed',False))
            with legacy.data_lock:
                legacy.state['wheeltec']=data
        except (ValueError,TypeError):
            pass


class LiveMapHandler(legacy.RadarHTTPHandler):
    def send_bytes(self,body,kind,status=200):
        self.send_response(status)
        self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path=urlsplit(self.path).path
        if path in ('/','/map','/index.html'):
            return self.send_bytes((ROOT/'templates/live_map.html').read_bytes(),'text/html; charset=utf-8')
        if path=='/control':
            # Keep all original controls and endpoints, with a return link.
            html=Path(legacy.TEMPLATE_PATH).read_text(encoding='utf-8')
            link='<a href="/map" style="position:fixed;right:20px;bottom:20px;z-index:99999;background:#fff;color:#076d76;padding:12px;border:1px solid #aaa;border-radius:8px">返回实时地图</a>'
            return self.send_bytes(html.replace('</body>',link+'</body>').encode(),'text/html; charset=utf-8')
        if path=='/api/live_map':
            data=self.server.map_node.snapshot()
            data['session']=self.server.mapping_session.status()
            return self._send_json(data)
        if path=='/api/live_map/cloud':
            return self._send_json(self.server.map_node.cloud_snapshot())
        if path=='/api/live_map/image':
            try:
                rev=int(parse_qs(urlsplit(self.path).query).get('v',['0'])[0])
            except ValueError:
                return self.send_error(400)
            png=self.server.map_node.image(rev)
            if png is None:
                return self.send_error(409,'Map revision expired; refresh snapshot')
            return self.send_bytes(png,'image/png')
        return super().do_GET()

    def do_POST(self):
        path=urlsplit(self.path).path
        if not path.startswith('/api/live_map/'):
            return super().do_POST()
        # Reject cross-origin browser writes to mapping/session controls.
        origin=self.headers.get('Origin')
        if origin and urlsplit(origin).netloc!=self.headers.get('Host'):
            return self._send_json(dict(ok=False,error='cross-origin write rejected'),403)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<=size<=2048:
                raise ValueError('请求过大')
            data=json.loads(self.rfile.read(size) or b'{}')
            if not isinstance(data,dict):
                raise ValueError('请求必须为 JSON 对象')
            session=self.server.mapping_session
            # Switching map coordinates while following is unsafe.
            if path in ('/api/live_map/start','/api/live_map/stop','/api/live_map/initial_pose') and legacy.is_follower_running():
                raise ValueError('请先停止电子跟随并停稳底盘，再切换地图/定位')
            if path.endswith('/start'):
                out=session.start(data.get('mode','mapping'),data.get('name',''))
                self.server.map_node.reset_display()
            elif path.endswith('/stop'):
                out=session.stop()
            elif path.endswith('/save'):
                if self.server.map_node.map_info is None:
                    raise ValueError('尚未收到地图，不能保存')
                out=session.save(data.get('name',''))
            elif path.endswith('/initial_pose'):
                self.server.map_node.initial_pose(float(data['x']),float(data['y']),float(data['yaw']))
                out=dict(ok=True,message='已发送初始位置；请观察 AMCL 是否收敛，非定位成功确认')
            else:
                return self.send_error(404)
            return self._send_json(out)
        except Exception as exc:
            return self._send_json(dict(ok=False,error=str(exc)),400)


def main():
    rclpy.init()
    control=DisplayControlBridge()
    legacy.bridge_node=control
    node=LiveMapNode()
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(control)
    executor.add_node(node)
    session=MappingSession()
    server=legacy.ThreadedHTTPServer(('0.0.0.0',legacy.PORT),LiveMapHandler)
    server.map_node=node
    server.mapping_session=session
    worker=threading.Thread(target=executor.spin,daemon=True)
    worker.start()
    # HTTP shutdown must be called outside serve_forever's thread.
    signal.signal(signal.SIGTERM,lambda *_: threading.Thread(target=server.shutdown,daemon=True).start())
    print('实时地图 :8088/map | 原有相机/遥控 :8088/control | 不自动启动底盘',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        session.stop()
        executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        node.destroy_node()
        control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
