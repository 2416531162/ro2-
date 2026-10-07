"""水泵独立蓝牙进程：父会话 2 秒续租，MCU 右下指令 200 ms 续租。"""
import json
import os
import re
import selectors
import signal
import sys
import time

from hc08_pump import HC08_ADDRESS, SERIAL_UUID, Hc08Pump


class BluezSerial:
    """BlueZ GATT 串口；仅访问指定 HC-08 的 FFE1，不操作车与平板的 RFCOMM。"""
    def __init__(self, address=HC08_ADDRESS, adapter='hci0'):
        if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', address):
            raise ValueError('HC-08 蓝牙地址无效')
        if not re.fullmatch(r'hci[0-9]+', adapter):
            raise ValueError('HC-08 蓝牙适配器无效')
        import dbus
        from dbus.mainloop.glib import DBusGMainLoop
        from gi.repository import GLib
        DBusGMainLoop(set_as_default=True)
        self.dbus, self.context = dbus, GLib.MainContext.default()
        self.bus = dbus.SystemBus()
        self.device = '/org/bluez/' + adapter + '/dev_' + address.upper().replace(':', '_')
        self.char = self.char_path = None
        self.match = self.owner_match = None
        self.alive = False
        self.write_type = 'command'
        self.connect_pending = False
        self.next_connect = 0.0
        self.connect_error = None

    def open(self, receive):
        # 先分派异步连接结果；未连接阶段也要让 BlueZ 回调得到处理。
        self.poll()
        self.close()
        dbus = self.dbus
        device = self.bus.get_object('org.bluez', self.device)
        properties = dbus.Interface(device, 'org.freedesktop.DBus.Properties')
        if not properties.Get('org.bluez.Device1', 'Connected', timeout=0.3):
            if not self.connect_pending and time.monotonic() >= self.next_connect:
                self.connect_pending = True
                def connected():
                    self.connect_pending = False
                    self.connect_error = None
                def failed(error):
                    self.connect_pending = False
                    self.next_connect = time.monotonic() + 5.0
                    self.connect_error = str(error)
                try:
                    # LE 建连可能等待模块广播十几秒；2 秒同步超时不会取消 BlueZ
                    # 已发起的连接，之后反复重试只会撞上 Operation already in progress。
                    dbus.Interface(device, 'org.bluez.Device1').Connect(
                        reply_handler=connected, error_handler=failed, timeout=40.0)
                except Exception as exc:
                    failed(exc)
            raise RuntimeError('HC-08 尚未连接，正在等待模块；请检查供电及是否被其他设备占用')
        if not properties.Get('org.bluez.Device1', 'ServicesResolved', timeout=0.3):
            raise RuntimeError('正在等待 HC-08 蓝牙服务，请稍后重试')
        objects = dbus.Interface(self.bus.get_object('org.bluez', '/'),
                                 'org.freedesktop.DBus.ObjectManager').GetManagedObjects(timeout=0.5)
        for path, interfaces in objects.items():
            props = interfaces.get('org.bluez.GattCharacteristic1', {})
            flags = set(props.get('Flags', []))
            if str(path).startswith(self.device + '/') and str(props.get('UUID', '')).lower() == SERIAL_UUID:
                if not flags.intersection(('write', 'write-without-response')) or 'notify' not in flags:
                    continue
                self.char_path = str(path)
                self.write_type = 'command' if 'write-without-response' in flags else 'request'
                self.char = dbus.Interface(self.bus.get_object('org.bluez', path),
                                           'org.bluez.GattCharacteristic1')
                break
        if self.char is None:
            raise RuntimeError('HC-08 未提供可收发的 FFE1 串口服务')
        self.alive = True

        def changed(interface, values, _invalidated, path=None):
            if path == self.device and interface == 'org.bluez.Device1' and 'Connected' in values:
                self.alive = bool(values['Connected'])
            elif path == self.char_path and interface == 'org.bluez.GattCharacteristic1' and 'Value' in values:
                receive(bytes(values['Value']))

        def owner_changed(name, _old, new):
            if name == 'org.bluez' and not new:
                self.alive = False

        self.match = self.bus.add_signal_receiver(changed, signal_name='PropertiesChanged',
            dbus_interface='org.freedesktop.DBus.Properties', bus_name='org.bluez', path_keyword='path')
        self.owner_match = self.bus.add_signal_receiver(owner_changed, signal_name='NameOwnerChanged',
            dbus_interface='org.freedesktop.DBus', arg0='org.bluez')
        try:
            self.char.StartNotify(timeout=3.0)
        except Exception as exc:
            raise RuntimeError('HC-08 串口通知订阅失败：' + str(exc)) from exc

    def write(self, frame):
        if not self.alive or self.char is None:
            raise RuntimeError('HC-08 蓝牙未连接')
        self.char.WriteValue(self.dbus.Array(frame, signature='y'),
            self.dbus.Dictionary({'type': self.write_type}, signature='sv'), timeout=0.2)

    def poll(self):
        # 有界处理，密集通知不能拖住停止命令或 200 ms 续租。
        for _ in range(64):
            if not self.context.pending():
                break
            self.context.iteration(False)

    def connected(self):
        return self.alive

    def close(self):
        if self.char is not None:
            try:
                self.char.StopNotify(timeout=0.2)
            except Exception:
                pass
        for match in (self.match, self.owner_match):
            if match is not None:
                match.remove()
        self.match = self.owner_match = self.char = self.char_path = None
        self.alive = False


def emit(**message):
    print(json.dumps(message, ensure_ascii=False), flush=True)


def main(transport_factory=BluezSerial):
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    pump = None
    selector = selectors.DefaultSelector()
    try:
        transport = transport_factory(os.environ.get('RK3588_HC08_ADDRESS', HC08_ADDRESS),
                                      os.environ.get('RK3588_HC08_ADAPTER', 'hci0'))
        pump = Hc08Pump(transport, lambda **msg: emit(**msg))
        selector.register(sys.stdin, selectors.EVENT_READ)
        os.set_blocking(sys.stdin.fileno(), False)
        buffer = b''
        while not stopped:
            if selector.select(0.02):
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    break
                buffer += data
                if len(buffer) > 65536:
                    raise ValueError('水泵蓝牙指令积压')
                while b'\n' in buffer:
                    raw, buffer = buffer.split(b'\n', 1)
                    command = json.loads(raw)
                    if command.get('action') == 'close':
                        stopped = True
                        break
                    try:
                        pump.command(command)
                    except Exception as exc:
                        pump.fail(str(exc))
            if not stopped:
                pump.tick()
    except Exception as exc:
        emit(ready=False, enabled=False, error=str(exc))
    finally:
        selector.close()
        if pump is not None:
            try:
                pump.close()
            except Exception:
                pass


if __name__ == '__main__':
    main()
