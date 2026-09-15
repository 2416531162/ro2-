#!/usr/bin/env python3
"""
斑图智控 (LaserLevelControl) BLE GATT 服务器
为 RK3588 提供与手机 App 完全兼容的 FFE0/FFE1 蓝牙透传服务
"""

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib
import array
import time
import os

BLUEZ_SERVICE_NAME = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE = 'org.bluez.GattDescriptor1'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'

class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'

class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()
        return response

class Service(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'characteristics': dbus.Array(
                    self.get_characteristic_paths(),
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'descriptors': dbus.Array(
                    self.get_descriptor_paths(),
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

class Descriptor(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_DESC_IFACE: {
                'Characteristic': self.chrc.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_DESC_IFACE]

# ================= 斑图通信特征值 (0xFFE1) =================
class BantuCharacteristic(Characteristic):
    def __init__(self, bus, index, service, on_rx_callback=None):
        Characteristic.__init__(
            self, bus, index,
            '0000ffe1-0000-1000-8000-00805f9b34fb',
            ['read', 'write', 'write-without-response', 'notify'],
            service)
        self.value = [0]
        self.notifying = False
        self.on_rx = on_rx_callback

    def notify_data(self, data_bytes):
        if not self.notifying:
            return
        val = [dbus.Byte(b) for b in data_bytes]
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': val}, [])

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        data = bytes(value)
        if self.on_rx:
            self.on_rx(data, self)

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        print("[*] 斑图 App 已开启数据通知通道 (StartNotify)！")

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        print("[*] 斑图 App 已关闭数据通知通道 (StopNotify)")

class CccdDescriptor(Descriptor):
    def __init__(self, bus, index, characteristic):
        Descriptor.__init__(
            self, bus, index,
            '00002902-0000-1000-8000-00805f9b34fb',
            ['read', 'write'],
            characteristic)
        self.value = [0, 0]

    @dbus.service.method(GATT_DESC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        self.value = value

class BantuService(Service):
    def __init__(self, bus, index, on_rx_callback=None):
        Service.__init__(self, bus, index, '0000ffe0-0000-1000-8000-00805f9b34fb', True)
        self.chrc = BantuCharacteristic(bus, 0, self, on_rx_callback)
        self.chrc.add_descriptor(CccdDescriptor(bus, 0, self.chrc))
        self.add_characteristic(self.chrc)

# ================= 蓝牙 BLE 广播 (Advertisement) =================
class BantuAdvertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index, local_name):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.local_name = local_name
        self.service_uuids = ['0000ffe0-0000-1000-8000-00805f9b34fb']
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            LE_ADVERTISEMENT_IFACE: {
                'Type': 'peripheral',
                'LocalName': dbus.String(self.local_name),
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
                'Includes': dbus.Array(['tx-power'], signature='s')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        pass

def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()
    for o, props in objects.items():
        if GATT_MANAGER_IFACE in props.keys():
            return o
    return None

def start_bantu_server(on_rx_handler=None):
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    adapter = find_adapter(bus)
    if not adapter:
        raise Exception("未找到支持 GATT 的蓝牙适配器")

    adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), DBUS_PROP_IFACE)
    # 设置蓝牙别名，包含“斑图”，满足 App 的推荐设备识别
    device_name = "斑图-RK3588控制器"
    adapter_props.Set('org.bluez.Adapter1', 'Alias', dbus.String(device_name))
    adapter_props.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(True))
    adapter_props.Set('org.bluez.Adapter1', 'Discoverable', dbus.Boolean(True))

    service_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), GATT_MANAGER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), LE_ADVERTISING_MANAGER_IFACE)

    app = Application(bus)
    bantu_service = BantuService(bus, 0, on_rx_handler)
    app.add_service(bantu_service)

    adv = BantuAdvertisement(bus, 0, device_name)

    print("[*] 正在注册斑图 FFE0 GATT 服务应用...")
    service_manager.RegisterApplication(app.get_path(), {},
                                         reply_handler=lambda: print("[*] 斑图 GATT 应用注册成功！"),
                                         error_handler=lambda e: print(f"[!] GATT 注册失败: {e}"))

    print(f"[*] 正在广播 BLE 设备名: {device_name} (服务: 0xFFE0)...")
    ad_manager.RegisterAdvertisement(adv.get_path(), {},
                                     reply_handler=lambda: print("[*] BLE 广播注册成功！手机已可被推荐搜索！"),
                                     error_handler=lambda e: print(f"[!] 广播注册失败: {e}"))

    return app, bantu_service, adv

if __name__ == "__main__":
    def test_rx(data, chrc):
        print(f"[收到 App 数据]: {data.hex().upper()}")
        # 握手帧 BB FF CC FF 55
        if data.startswith(b'\xBB\xFF\xCC\xFF\x55'):
            print(">>> 识别到斑图 App 查询参数帧，正在回复参数...")
            # 回复标准参数响应帧序列
            frames = [
                b'\xAA\x06\x1E\x00\x55',               # 左死区偏移 0
                b'\xAA\x07\x1E\x00\x55',               # 右死区偏移 0
                b'\xAA\x05\x05\x05\x14\x1E\x1E\x00\x55', # 基础参数
                b'\xAA\x08\x03\xE8\x00\x55',           # 频率 1000
                b'\xAA\x09\x50\x50\x00\x55',           # 限制 80%
                b'\xAA\x0F\x01\x00\x55',               # 阀门类型
                b'\xAA\x10\x1E\x1E\x1E\x1E\x55',       # PWM 门限
                b'\xAA\x12\x07\xD0\x00\x55'            # 超时 2000ms
            ]
            for f in frames:
                chrc.notify_data(f)
                time.sleep(0.02)

    app, svc, adv = start_bantu_server(test_rx)
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
