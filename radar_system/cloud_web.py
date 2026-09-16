#!/usr/bin/env python3
"""3D-first WebGL UI + native display binary transport. Reuses existing controls."""
import signal
import threading
from urllib.parse import urlsplit,parse_qs
import rclpy
from rclpy.executors import MultiThreadedExecutor
from live_map_web import LiveMapHandler,DisplayControlBridge,MappingSession,ROOT,legacy
from live_cloud_node import LiveCloudNode


class CloudHandler(LiveMapHandler):
    ASSETS={'/static/cloud_viewer.js':('cloud_viewer.js','text/javascript; charset=utf-8'),
            '/static/cloud_app.js':('cloud_app.js','text/javascript; charset=utf-8'),
            '/static/cloud.css':('cloud.css','text/css; charset=utf-8')}

    def do_GET(self):
        path=urlsplit(self.path).path
        if path in ('/','/map','/index.html'):
            return self.send_bytes((ROOT/'templates/cloud_map.html').read_bytes(),'text/html; charset=utf-8')
        if path=='/map2d':
            return self.send_bytes((ROOT/'templates/live_map.html').read_bytes(),'text/html; charset=utf-8')
        if path in self.ASSETS:
            name,kind=self.ASSETS[path]
            return self.send_bytes((ROOT/'static'/name).read_bytes(),kind)
        if path=='/api/live_map/scene.bin':
            query=parse_qs(urlsplit(self.path).query)
            try:
                epoch=query.get('epoch',[''])[0]
                revision=int(query.get('v',['-1'])[0])
            except (ValueError,TypeError):
                return self.send_error(400,'Invalid revision')
            compressed='gzip' in self.headers.get('Accept-Encoding','')
            data=self.server.map_node.scene_bytes(epoch,revision,compressed)
            if data is None:
                return self.send_error(409,'Scene changed; fetch metadata again')
            self.send_response(200)
            self.send_header('Content-Type','application/octet-stream')
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('Vary','Accept-Encoding')
            self.send_header('X-Scene-Epoch',epoch)
            self.send_header('X-Scene-Revision',str(revision))
            if compressed:
                self.send_header('Content-Encoding','gzip')
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()


def main():
    rclpy.init()
    control=DisplayControlBridge()
    legacy.bridge_node=control
    node=LiveCloudNode()
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(control); executor.add_node(node)
    session=MappingSession()
    server=legacy.ThreadedHTTPServer(('0.0.0.0',legacy.PORT),CloudHandler)
    server.map_node=node; server.mapping_session=session
    worker=threading.Thread(target=executor.spin,daemon=True); worker.start()
    signal.signal(signal.SIGTERM,lambda *_:threading.Thread(target=server.shutdown,daemon=True).start())
    print('3D 点云主视图 :8088/map | 2D 导航层 :8088/map2d | 原控制 :8088/control',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close(); session.stop(); executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.); node.destroy_node(); control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
