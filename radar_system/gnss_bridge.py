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

还可以走 CAN（CanSerialBridge）：平板（cpdevice SPRING2_MIX）MCU 的 CAN1/CAN2 接到车的 can0（500 kbit/s，
和 OID 驱动器、IMU 同一条总线）。rtk_node 当第三路（'can'）。帧格式见 CAN_DATA_ID。

还可以走 USB 数据线（TabletUsbSerial）：平板 USB-A 经 A 转 C 数据线插板子 J17，板子当 USB 设备（CDC-ACM 虚拟串口，
usb_gadget）。不丢帧、带宽够转原始观测；rtk_node 当第四路（'tablet_usb'），车停着时优先于 WiFi / 蓝牙 / CAN。
"""
import os
import select
import socket
import struct
import threading
import time
from gnss_can_reassembly import CanGnssReassembler

TABLET_PORT = 48622
# 蓝牙那一路：RFCOMM 通道和服务 UUID（自定义的，平板按它找服务，不会连到别的蓝牙串口设备上）。
# 平板端 um982-relay 的 BtForwarder.SERVICE_UUID 必须一样
BT_CHANNEL = 22
BT_UUID = '5254b2c1-4d1e-4a7b-9f3e-6c2a8d0e4b71'
BT_PROFILE_PATH = '/com/rk3588/rtk_relay'

# CAN 那一路（平板端 um982-relay 的 CanForwarder 必须一样）。标准帧，占 0x7F0–0x7F7 这一段：标准帧里优先级最低
# （驱动器 0x74、IMU 0x185–0x585 总是先发），CANopen 预定义连接集也不用这段。OID 驱动、IMU 的套接字按
# CAN_RELAY_BLOCK 装了反向过滤器，这些帧到不了它们那里。
#   CAN_DATA_ID  平板 → 车：第 0 字节帧序号（0–255 循环，车按它数丢帧），1–7 字节是平板串口原始字节
#   CAN_ACK_ID   车 → 平板：收到数据时每 CAN_ACK_S 一帧：这段时间收到的帧数 u16、累计丢帧 u16、车在不在用这一路 u8（小端）
CAN_CHANNEL = 'can0'
CAN_DATA_ID = 0x7F3
CAN_ACK_ID = 0x7F4
CAN_RELAY_BLOCK = (0x7F0, 0x7F8)        # (id, mask)：0x7F0–0x7F7
CAN_ACK_S = 0.5
CAN_QUIET_S = 2.0                       # 这么久没收到数据帧算断开（平板没在发）
# Linux SocketCAN 常量（macOS 上的 python 没有，测试用假套接字）
AF_CAN = getattr(socket, 'AF_CAN', 29)
CAN_RAW = getattr(socket, 'CAN_RAW', 1)
SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
CAN_RAW_FILTER = getattr(socket, 'CAN_RAW_FILTER', 1)
CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG, CAN_INV_FILTER = 0x80000000, 0x40000000, 0x20000000, 0x20000000
CAN_FRAME = '=IB3x8s'


def can_exclude_relay(sock):
    """给 OID 驱动、IMU 的 CAN 套接字装反向过滤器：收总线上除 CAN_RELAY_BLOCK 以外的所有帧（标准帧、扩展帧都收），
    平板转来的定位数据帧（约 1500 帧/s）不进它们的接收缓冲，驱动器、IMU 的帧不会被挤掉。"""
    block, mask = CAN_RELAY_BLOCK
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER,
                    struct.pack('=II', block | CAN_INV_FILTER, mask | CAN_EFF_FLAG | CAN_RTR_FLAG))


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


class CanSerialBridge:
    """平板经 CAN 转发串口：read / status / close 与 TabletSerialServer 一样，rtk_node 当另一路串口用。

    只收 CAN_DATA_ID（内核过滤器），重复分片不拼入；缺帧/乱序时丢弃半包，只交付通过校验的完整消息。
    CAN 是广播、没有连接：CAN_QUIET_S 内收到过数据帧算「连着」。收到数据时每 CAN_ACK_S 回一帧 CAN_ACK_ID，
    平板据此显示车端确认；平板没在发时车一帧也不发。can0 没起来、被拔掉时 2 s 后重开，原因写进 status()['error']。
    in_use：rtk_node 现在用不用这一路（回给平板显示）。sock_factory、clock 测试用。
    """

    def __init__(self, channel=CAN_CHANNEL, timeout=0.1, on_connect=None, in_use=None, sock_factory=None,
                 clock=time.monotonic):
        self.channel, self.timeout, self.on_connect = channel, timeout, on_connect
        self.in_use = in_use or (lambda: False)
        self.sock_factory, self.clock = sock_factory, clock
        self.sock = None
        self.error = None
        self.frames = self.gaps = self.bytes_in = self.acks = 0
        self.malformed_frames = 0
        self.reassembly = CanGnssReassembler(CAN_QUIET_S)
        self.last_rx = None
        self._window = 0                # 上次回确认以来收到的帧
        self._ack_at = None
        self._reopen_at = 0.0
        self._open()

    def _open(self):
        try:
            if self.sock_factory is not None:
                sock = self.sock_factory()
            else:
                sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
                sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER,
                                struct.pack('=II', CAN_DATA_ID, 0x7FF | CAN_EFF_FLAG | CAN_RTR_FLAG))
                sock.bind((self.channel,))
            sock.settimeout(self.timeout)
            self.sock, self.error = sock, None
        except (OSError, AttributeError) as exc:        # 没有 can0、没权限、不是 Linux
            self.sock, self.error = None, '%s: %s' % (type(exc).__name__, exc)
            self._reopen_at = self.clock() + 2.0

    def connected(self, now=None):
        now = self.clock() if now is None else now
        return self.last_rx is not None and now - self.last_rx <= CAN_QUIET_S

    def read(self, n=4096):
        """最多等 timeout 秒；只返回完整 GNSS 消息。坏帧/重复帧也计读取上限，避免持续坏流饿死读循环。"""
        if self.sock is None:
            if self.clock() >= self._reopen_at:
                self._open()
            if self.sock is None:
                time.sleep(self.timeout)
                return b''
        out = bytearray()
        sock = self.sock
        received = 0
        try:
            raw = sock.recv(16)
            sock.settimeout(0.0)
            try:
                while True:
                    self._take(raw, out)
                    received += 1
                    if len(out) >= n or received >= max(1,(n+6)//7):
                        break
                    raw = sock.recv(16)
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                sock.settimeout(self.timeout)
        except socket.timeout:
            pass
        except OSError as exc:          # 接口掉了（ip link down、总线关断）：关掉，过会儿重开
            self.error = '%s: %s' % (type(exc).__name__, exc)
            self._close_sock()
            self._reopen_at = self.clock() + 2.0
            return bytes(out)          # 已完成且验过的消息照常交付，只放弃未完成的半包。
        now = self.clock()
        self._ack(now)
        return bytes(out)

    def _take(self, raw, out):
        if len(raw)!=struct.calcsize(CAN_FRAME):
            self.malformed_frames+=1
            return
        can_id, dlc, data = struct.unpack(CAN_FRAME, raw)
        if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG) or can_id & 0x7FF != CAN_DATA_ID or not 2 <= dlc <= 8:
            return
        now=self.clock()
        was_connected=self.connected(now)
        if not was_connected:self._window,self._ack_at=0,None
        out += self.reassembly.feed(data[0],data[1:dlc],now)
        self.frames,self.gaps,self.bytes_in=self.reassembly.frames,self.reassembly.gaps,self.reassembly.bytes_in
        self._window += 1
        self.last_rx=now
        if not was_connected and self.on_connect is not None:self.on_connect()

    def _ack(self, now):
        if not self.connected(now) or self.sock is None:
            self._ack_at, self._window = None, 0
            return
        if self._ack_at is None:
            self._ack_at = now
        if now - self._ack_at < CAN_ACK_S:
            return
        payload = struct.pack('<HHB', min(self._window, 0xFFFF), self.gaps & 0xFFFF, 1 if self.in_use() else 0)
        self._ack_at, self._window = now, 0
        try:
            self.sock.send(struct.pack(CAN_FRAME, CAN_ACK_ID, len(payload), payload.ljust(8, b'\0')))
            self.acks += 1
        except OSError:                 # 发送缓冲满（总线上没别的节点应答）：下次再说，不影响收
            pass

    def status(self):
        return dict(channel=self.channel, open=self.sock is not None, connected=self.connected(), peer=None,
                    **self.reassembly.status(),acks=self.acks,error=self.error,malformed_frames=self.malformed_frames)

    def _close_sock(self):
        sock, self.sock = self.sock, None
        self.reassembly.reset()
        self.last_rx=None
        self._window,self._ack_at=0,None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def close(self):
        self._close_sock()


USB_QUIET_S = 2.0                       # 这么久没收到数据算没连着（平板拔线、停了 USB 输出）
USB_RETRY_S = 1.0                       # 虚拟串口打不开 / 断了以后隔这么久再开


class TabletUsbSerial:
    """平板经 USB 数据线转发串口：read / status / close 与 TabletSerialServer 一样，rtk_node 当另一路串口用。

    板子 J17 配成 CDC-ACM USB 设备（usb_gadget.ensure，setup 回调给出 /dev/ttyGS<N>），平板 USB-A 当主机插上来，
    「UM982 转发」1.5.0 起把接收机串口原始字节原样写进来。只读：车不往这根线写（echo 也关掉，回显会把定位数据
    原样送回平板）。tty 设成原始模式（不转换回车换行、不按行缓冲），二进制日志原样到 Um982Stream。
    平板拔线或停了 USB 输出时内核挂断这个 tty（读到 EOF / EIO）：关掉、USB_RETRY_S 后重开，原因写进 status()['error']。
    device：固定节点（不建 gadget）；None 时每次打开前调 setup() 建好 gadget、取它的节点。opener、clock 测试用。
    """

    def __init__(self, device=None, setup=None, timeout=0.1, on_connect=None, opener=None, clock=time.monotonic):
        self.device, self.setup, self.timeout, self.on_connect = device, setup, timeout, on_connect
        self.opener, self.clock = opener, clock
        self.fd = None
        self.path = device
        self.gadget = None              # setup() 的结果（udc、serial_device、created）
        self.error = None
        self.opens = self.bytes_in = self.hangups = 0
        self.last_rx = None
        self._reopen_at = 0.0

    def _resolve(self):
        if self.device is not None:
            return self.device
        info = self.setup() if self.setup is not None else None
        self.gadget = info
        path = (info or {}).get('serial_device')
        if not path:
            raise OSError('USB 虚拟串口没有节点（gadget 的 port_num 读不到）')
        return path

    def _open(self):
        try:
            path = self._resolve()
            fd = self.opener(path) if self.opener is not None else os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                raw_tty(fd)
            except Exception:
                os.close(fd)
                raise
        except Exception as exc:        # 没有 configfs、控制器被占、节点不存在、没权限……
            self.fd, self.error = None, '%s: %s' % (type(exc).__name__, exc)
            self._reopen_at = self.clock() + USB_RETRY_S
            return False
        self.fd, self.path, self.error = fd, path, None
        self.opens += 1
        return True

    def connected(self, now=None):
        now = self.clock() if now is None else now
        return self.fd is not None and self.last_rx is not None and now - self.last_rx <= USB_QUIET_S

    def read(self, n=4096):
        """最多等 timeout 秒；没连着或这段时间没数据时返回 b''。"""
        if self.fd is None:
            if self.clock() < self._reopen_at or not self._open():
                time.sleep(self.timeout)
                return b''
        fd = self.fd
        try:
            ready, _, _ = select.select([fd], [], [], self.timeout)
            if not ready:
                return b''
            data = os.read(fd, n)
        except (BlockingIOError, InterruptedError):
            return b''
        except (OSError, ValueError) as exc:    # EIO：平板拔线，tty 被挂断
            self._hangup('%s: %s' % (type(exc).__name__, exc))
            return b''
        if not data:                    # EOF：同上
            self._hangup('平板拔线或停了 USB 输出')
            return b''
        now = self.clock()
        was = self.connected(now)
        self.last_rx = now
        self.bytes_in += len(data)
        if not was and self.on_connect is not None:
            self.on_connect()
        return data

    def _hangup(self, why):
        self.hangups += 1
        self._close_fd()
        self.error = why
        self._reopen_at = self.clock() + USB_RETRY_S

    def status(self):
        g = self.gadget or {}
        return dict(device=self.path, open=self.fd is not None, connected=self.connected(), peer=None,
                    udc=g.get('udc'), opens=self.opens, hangups=self.hangups, bytes_in=self.bytes_in, error=self.error)

    def _close_fd(self):
        fd, self.fd = self.fd, None
        self.last_rx = None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def close(self):
        self._close_fd()


def raw_tty(fd):
    """tty 原始模式（同 cfmakeraw）+ 不等载波；清掉打开前积压的旧数据（重连不补发旧定位）。"""
    import termios
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR | termios.IGNCR
                  | termios.ICRNL | termios.IXON | termios.IXOFF | termios.INPCK)
    attrs[1] &= ~termios.OPOST
    attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB)) | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 1, 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def bt_address(device_path):
    """BlueZ 设备对象路径 → 蓝牙地址。

    >>> bt_address('/org/bluez/hci0/dev_AA_BB_CC_DD_EE_0F')
    'AA:BB:CC:DD:EE:0F'
    >>> bt_address('/org/bluez/hci0')
    '/org/bluez/hci0'
    """
    tail = str(device_path).rsplit('/', 1)[-1]
    return tail[4:].replace('_', ':') if tail.startswith('dev_') else str(device_path)
