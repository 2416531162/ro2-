#!/usr/bin/env python3
"""独立模拟喷水 HTTP 服务；不加载 ROS、不接管正在运行的车辆服务。"""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from spray_sim_service import SpraySimulations, handle
from spray_live_trace import LiveSprayTrace, handle as handle_trace


class Handler(BaseHTTPRequestHandler):
    timeout=10
    def _send_json(self,value,status=200):
        body=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
        self.end_headers();self.wfile.write(body)

    def do_GET(self):
        if handle_trace(self,self.server.trace):return
        if self.server.observe_only or not handle(self,self.server.simulations):self._send_json(dict(error='没有这个模拟接口'),404)

    def do_POST(self):
        if handle_trace(self,self.server.trace):return
        if self.server.observe_only or not handle(self,self.server.simulations):self._send_json(dict(error='没有这个模拟接口'),404)

    def log_message(self,*_):pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=19088);parser.add_argument('--root')
    parser.add_argument('--observe-only',action='store_true',help='仅处理实车喷线观测，不运行离线虚拟车辆')
    args=parser.parse_args()
    server=ThreadingHTTPServer(('0.0.0.0',args.port),Handler)
    server.simulations=SpraySimulations(args.root)
    server.trace=LiveSprayTrace()
    server.observe_only=args.observe_only
    try:server.serve_forever()
    finally:server.simulations.close();server.server_close()


if __name__=='__main__':main()
