#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板串口转发：UM982 接在安卓平板（cpdevice 工业平板，Android 6）的串口上时，平板 app 用 TCP 把串口
原始字节原样转给车（端口 TABLET_PORT），本模块把这条连接当成一个只读串口交给 rtk_node：

    read()   接收机的输出（BESTPOSA / HEADINGA / … 与 GGA，平板不解析、不改）

单向：车不往平板写。差分由平板 app 用本机保存的 CORS 账号自己连、直接写进平板串口（照 chaoshengbo），
这时 rtk_node 暂停车端 CORS；副天线日志 BESTNAVHA 也由平板按串口带宽自己补（平板串口固定 115200）。
解析、测量时刻换算（GnssClock 取窗口内最小延迟，Wi-Fi 抖动不影响）、融合都和 USB 串口一样。
一次只接一个平板，新连接顶替旧连接（平板断线重连时旧连接可能还没超时）。不依赖 ROS。
和网页、app 的控制接口一样不做鉴权：同一局域网里的设备都能连（见车端 docs/APP_PROTOCOL.md）。

也可以走蓝牙（BluetoothSerialServer）：经典蓝牙 RFCOMM，平板不用和车连同一个 WiFi，差分照旧走平板的 4G。
BlueZ 收连接、把连好的 socket 交过来，之后和 TCP 那一路一样是一根只读串口线；rtk_node 把它当另一路（'bluetooth'）。
"""
import select
import socket
import threading
import time

TABLET_PORT = 48622
# 蓝牙那一路：RFCOMM 通道和服务 UUID（自定义的，平板按它找服务，不会连到别的蓝牙串口设备上）。
# 平板端 um982-relay 的 BtForwarder.SERVICE_UUID 必须一样
BT_CHANNEL = 22
BT_UUID = '5254b2c1-4d1e-4a7b-9f3e-6c2a8d0e4b71'
BT_PROFILE_PATH = '/com/rk3588/rtk_relay'


class TabletSerialServer:
    """监听平板的串口转发连接；read / close 与 pyserial 同名，rtk_node 按串口用。"""

    def __init__(self, port=TABLET_PORT, host='0.0.0.0', timeout=0.1, on_connect=None):
        self.timeout = timeout
        self.on_connect = on_connect
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(2)
        self.port = self.server.getsockname()[1]
        self.lock = threading.Lock()        # 保护连接的替换
        self.client = None
        self.peer = None
        self.connections = 0
        self.bytes_in = 0

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

    def status(self):
        return dict(port=self.port, connected=self.client is not None, peer=self.peer,
                    connections=self.connections, bytes_in=self.bytes_in)

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


class BluetoothSerialServer:
    """平板经蓝牙（RFCOMM）转发串口：read / status / close 与 TabletSerialServer 一样，rtk_node 当另一路串口用。

    向 BlueZ 注册一个 server 角色的 Profile（ProfileManager1.RegisterProfile，UUID BT_UUID、通道 BT_CHANNEL），
    平板连上来时 BlueZ 调 NewConnection 把连好的 socket 交给 adopt()。不鉴权、不授权、不要配对（和 WiFi 那一路一样），
    一次一台，新连接顶替旧的；bluetoothd 重启后自己重新注册。机器上没有 python3-dbus / gi 或没有蓝牙时
    只把原因写进 status()['error']，别的路照常。register=False 不碰 BlueZ（测试直接调 adopt）。
    """

    def __init__(self, channel=BT_CHANNEL, uuid=BT_UUID, timeout=0.1, on_connect=None, register=True):
        self.channel, self.uuid, self.timeout, self.on_connect = channel, uuid, timeout, on_connect
        self.lock = threading.Lock()
        self.client = None
        self.peer = None
        self.connections = 0
        self.bytes_in = 0
        self.registered = False
        self.error = None
        self._loop = None
        self._unregister = None
        if register:
            try:
                self._start_bluez()
            except Exception as exc:        # 没有系统总线、BlueZ 没装……：蓝牙这一路不可用，状态里写明
                self.error = '%s: %s' % (type(exc).__name__, exc)

    def adopt(self, sock, peer):
        """收下一条新连接（BlueZ 的 NewConnection 里调），顶替旧连接。"""
        sock.setblocking(True)
        with self.lock:
            old, self.client, self.peer = self.client, sock, peer
            self.connections += 1
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        if self.on_connect is not None:
            self.on_connect()

    def read(self, n=4096):
        """最多等 timeout 秒；没有平板连着或这段时间没数据时返回 b''。"""
        client = self.client
        if client is None:
            time.sleep(self.timeout)
            return b''
        try:
            ready, _, _ = select.select([client], [], [], self.timeout)
        except (OSError, ValueError):       # 连接刚被顶替或关掉
            return b''
        if not ready:
            return b''
        try:
            data = client.recv(n)
        except OSError:
            data = b''
        if not data:                        # 平板断开（关了 app、走远了、关了蓝牙）
            self._drop(client)
            return b''
        self.bytes_in += len(data)
        return data

    def drop_peer(self):
        client = self.client
        if client is not None:
            self._drop(client)

    def status(self):
        return dict(channel=self.channel, registered=self.registered, connected=self.client is not None,
                    peer=self.peer, connections=self.connections, bytes_in=self.bytes_in, error=self.error)

    def close(self):
        if self._unregister is not None:
            try:
                self._unregister()
            except Exception:
                pass
        if self._loop is not None:
            self._loop.quit()
        self.drop_peer()

    def _drop(self, client):
        with self.lock:
            if self.client is client:
                self.client, self.peer = None, None
        try:
            client.close()
        except OSError:
            pass

    def _start_bluez(self):
        import dbus
        import dbus.service
        import dbus.mainloop.glib
        from gi.repository import GLib

        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        server = self

        class Profile(dbus.service.Object):
            @dbus.service.method('org.bluez.Profile1', in_signature='', out_signature='')
            def Release(self):
                server.registered = False

            @dbus.service.method('org.bluez.Profile1', in_signature='oha{sv}', out_signature='')
            def NewConnection(self, device, fd, properties):
                server.adopt(socket.socket(fileno=fd.take()), bt_address(device))

            @dbus.service.method('org.bluez.Profile1', in_signature='o', out_signature='')
            def RequestDisconnection(self, device):
                server.drop_peer()

        self._profile = Profile(bus, BT_PROFILE_PATH)

        def manager():
            return dbus.Interface(bus.get_object('org.bluez', '/org/bluez'), 'org.bluez.ProfileManager1')

        def register(owner):
            if not owner:                   # bluetoothd 没在跑：等它起来再注册
                self.registered, self.error = False, 'bluetoothd 没在运行'
                return
            try:
                manager().RegisterProfile(BT_PROFILE_PATH, self.uuid, {
                    'Name': 'RTK relay', 'Role': 'server', 'Channel': dbus.UInt16(self.channel),
                    'RequireAuthentication': False, 'RequireAuthorization': False, 'AutoConnect': False})
                self.registered, self.error = True, None
            except dbus.exceptions.DBusException as exc:
                self.registered = False
                if exc.get_dbus_name() == 'org.bluez.Error.AlreadyExists':
                    self.registered, self.error = True, None
                else:
                    self.error = 'BlueZ 注册失败：%s' % (exc.get_dbus_message() or exc.get_dbus_name())

        self._unregister = lambda: manager().UnregisterProfile(BT_PROFILE_PATH)
        bus.watch_name_owner('org.bluez', register)
        self._loop = GLib.MainLoop()
        threading.Thread(target=self._loop.run, name='bt-bridge', daemon=True).start()


def bt_address(device_path):
    """BlueZ 设备对象路径 → 蓝牙地址。

    >>> bt_address('/org/bluez/hci0/dev_AA_BB_CC_DD_EE_0F')
    'AA:BB:CC:DD:EE:0F'
    >>> bt_address('/org/bluez/hci0')
    '/org/bluez/hci0'
    """
    tail = str(device_path).rsplit('/', 1)[-1]
    return tail[4:].replace('_', ':') if tail.startswith('dev_') else str(device_path)
