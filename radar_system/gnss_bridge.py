#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板串口转发：UM982 接在安卓平板（cpdevice 工业平板，Android 6）的串口上时，平板 app 用 TCP 把串口
原始字节原样转给车（端口 TABLET_PORT），本模块把这条连接当成一个串口交给 rtk_node：

    read()   接收机的输出（BESTPOSA / HEADINGA / … 与 GGA，平板不解析、不改）
    write()  车发给接收机的东西：NTRIP 差分改正数、日志命令（平板原样写进串口）

解析、测量时刻换算（GnssClock 取窗口内最小延迟，Wi-Fi 抖动不影响）、融合、CORS 都和 USB 串口一样。
一次只接一个平板，新连接顶替旧连接（平板断线重连时旧连接可能还没超时）。不依赖 ROS。
和网页、app 的控制接口一样不做鉴权：同一局域网里的设备都能连（见车端 docs/APP_PROTOCOL.md）。
"""
import select
import socket
import threading

TABLET_PORT = 48622


class TabletSerialServer:
    """监听平板的串口转发连接；read / write / close 与 pyserial 同名，rtk_node 按串口用。"""

    def __init__(self, port=TABLET_PORT, host='0.0.0.0', timeout=0.1, on_connect=None):
        self.timeout = timeout
        self.on_connect = on_connect
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(2)
        self.port = self.server.getsockname()[1]
        self.lock = threading.Lock()        # 保护连接的替换和写
        self.client = None
        self.peer = None
        self.connections = 0
        self.bytes_in = 0
        self.bytes_out = 0

    def read(self, n=4096):
        """最多等 timeout 秒；没有平板连着或这段时间没数据时返回 b''。"""
        client = self.client
        watch = [self.server] if client is None else [self.server, client]
        try:
            ready, _, _ = select.select(watch, [], [], self.timeout)
        except (OSError, ValueError):       # 连接刚被别的线程关掉
            return b''
        if self.server in ready:
            self._accept()
        if client is None or client not in ready:
            return b''
        try:
            data = client.recv(n)
        except OSError:
            data = b''
        if not data:                        # 平板关了连接（app 退出、换车）
            self._drop(client)
            return b''
        self.bytes_in += len(data)
        return data

    def write(self, data):
        """写给接收机；没有平板连着返回 False。Wi-Fi 卡住时最多阻塞 1 s，然后断开这条连接等平板重连。"""
        with self.lock:
            client = self.client
            if client is None:
                return False
            try:
                client.sendall(data)
                self.bytes_out += len(data)
                return True
            except OSError:
                pass
        self._drop(client)
        return False

    def status(self):
        return dict(port=self.port, connected=self.client is not None, peer=self.peer,
                    connections=self.connections, bytes_in=self.bytes_in, bytes_out=self.bytes_out)

    def close(self):
        with self.lock:
            client, self.client, self.peer = self.client, None, None
        for s in (client, self.server):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass

    def _accept(self):
        try:
            conn, addr = self.server.accept()
        except OSError:
            return
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        conn.settimeout(1.0)
        with self.lock:
            old, self.client, self.peer = self.client, conn, '%s:%d' % (addr[0], addr[1])
            self.connections += 1
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        if self.on_connect is not None:
            self.on_connect()

    def _drop(self, client):
        with self.lock:
            if self.client is client:
                self.client, self.peer = None, None
        try:
            client.close()
        except OSError:
            pass
