#!/usr/bin/env python3
"""
================================================================================
  RK3588 全维硬件深度探针 + 《斑图智控》BLE GATT 仿真下位机平台
  - 硬件平台: 正点原子 ATK-DLRK3588 (Rockchip RK3588 / Linux 6.x)
  - 斑图适配:
      1. 广播名称: 斑图-RK3588控制器 (自动命中 App 推荐设备白名单)
      2. GATT 串口透传: 0xFFE0 服务 + 0xFFE1 读写/通知特征值 (Score 100)
      3. 完整协议握手: 响应 BB FF CC FF 55 参数请求，返回合法 AA 序列帧
      4. 实时遥测心跳: 周期性上报 AA 01 / AA 04 遥测帧，维持 App 低延迟绿标
      5. 网页控制台同步: 手机 App 发送的所有指令在网页上毫秒级高亮展示
================================================================================
"""

import http.server
import socketserver
import json
import os
import re
import time
import glob
import threading
import multiprocessing
from urllib.parse import parse_qs, urlparse

# D-Bus & GLib for BlueZ GATT
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

PORT = 8888

# 全局系统状态
cpu_last_stats = {}

stress_processes = []
stress_status = {"running": False, "start_time": 0, "duration": 30}

# Display-safe fan arbitration. The boot DT used with this dashboard removes
# the vendor rockchip,temp-trips notifier, so the original display kernel is
# retained and this process becomes the single fan policy writer.
FAN_CONTROL_FILE = "/root/fan_control_state.json"
FAN_TEMP_PWM_CURVE = (
    (32000, 50), (34000, 100), (36000, 120), (38000, 150),
    (40000, 170), (42000, 180), (44000, 200), (46000, 205),
    (48000, 210), (50000, 215), (52000, 220), (54000, 225),
    (56000, 230), (58000, 235), (60000, 240), (62000, 245),
    (64000, 250), (66000, 251), (68000, 252), (70000, 253),
)
fan_control_lock = threading.RLock()
fan_control_state = {
    "mode": "auto",
    "requested_pwm": 153,
    "overheat": False,
}

# 斑图蓝牙数据与下位机参数状态
# Parameter handlers call log_bantu_event while holding this lock.
bantu_lock = threading.RLock()
bantu_rx_history = []
bantu_chrc_instance = None
bantu_connected = False
bantu_telemetry_active = False

bantu_settings = {
    # 找平零偏 (-30..30)
    "left_zero_offset": 0,
    "right_zero_offset": 0,
    # 死区与基础比例增益
    "laser_deadzone": 5,
    "level_deadzone": 5,
    "proportional_gain": 20,
    # 四通道细分比例增益 Kp x100 (2000 即 20.00)
    "kp_x100": [2000, 2000, 2000, 2000],
    # 全局微分增益 Kd (0..100)
    "derivative_gain": 0,
    # 四向起动/最小 PWM 门限 (0..200)
    "left_min_pwm": 30,
    "left_down_min_pwm": 30,
    "right_min_pwm": 30,
    "right_down_min_pwm": 30,
    # PWM 输出频率 (Hz) 与输出限幅 (%)
    "pwm_frequency": 1000,
    "left_output_limit": 80,
    "right_output_limit": 80,
    "channel_max_percent": [80, 80, 80, 80],
    # 曲线掩码 (bit0..3 代表 CH1..CH4: 1=S曲线, 0=线性) 与斜坡时间 (ms)
    "channel_curve_mask": 0x0F,
    "ramp_time_ms": 10,
    "s_curve_strength": [100, 100, 100, 100],
    # 阀门响应特性 (1=比例阀, 0=开关阀, 2=丹佛斯) 与引脚映射编码
    "valve_type": 1,
    "pin_map_code": 0xE4,
    "pin_idle_code": 0x0F,
    # 蓝牙失联保护超时 (ms)
    "app_timeout_ms": 2000,
    # 工作模式 (0=手动, 1=自动平地) 与手动动作码
    "mode": 0,
    "manual_action": 0
}

BANTU_SETTINGS_FILE = "/root/bantu_settings.json"
FALLBACK_SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bantu_settings.json")

def get_settings_file_path():
    try:
        test_file = "/root/.test_write"
        with open(test_file, "w") as f:
            f.write("1")
        os.remove(test_file)
        return BANTU_SETTINGS_FILE
    except Exception:
        return FALLBACK_SETTINGS_FILE

def load_bantu_settings():
    global bantu_settings
    path = get_settings_file_path()
    try:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                saved = json.load(f)
                with bantu_lock:
                    for k, v in saved.items():
                        if k in bantu_settings:
                            bantu_settings[k] = v
            print(f"[*] 成功从 {path} 加载已持久化的斑图参数")
    except Exception as e:
        print(f"[!] 加载斑图参数失败: {e}")




def read_str(path, default=""):
    try:
        with open(path, "r", errors="ignore") as f:
            return f.read().strip()
    except Exception:
        return default

def write_str(path, val):
    try:
        with open(path, "w") as f:
            f.write(str(val).strip())
        return True
    except Exception:
        return False

# ================= 斑图 BLE GATT 服务器实现 =================
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
            for chrc in service.get_characteristics():
                response[chrc.get_path()] = chrc.get_properties()
                for desc in chrc.get_descriptors():
                    response[desc.get_path()] = desc.get_properties()
        return response

class Service(dbus.service.Object):
    PATH_BASE = '/org/bluez/bantu/service'

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
                    [chrc.get_path() for chrc in self.characteristics],
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

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
                    [desc.get_path() for desc in self.descriptors],
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE: raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated): pass

class Descriptor(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {GATT_DESC_IFACE: {'Characteristic': self.chrc.get_path(), 'UUID': self.uuid, 'Flags': self.flags}}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE: raise InvalidArgsException()
        return self.get_properties()[GATT_DESC_IFACE]

class BantuCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            '0000ffe1-0000-1000-8000-00805f9b34fb',
            ['read', 'write', 'write-without-response', 'notify'],
            service)
        self.value = [0]
        self.notifying = False

    def notify_data(self, data_bytes):
        if not self.notifying:
            return
        val = [dbus.Byte(b) for b in data_bytes]
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': val}, [])
        record_ble_packet('tx', len(data_bytes))

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='a{sv}', out_signature='ay')
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        data = bytes(value)
        record_ble_packet('rx', len(data))
        handle_bantu_app_write(data, self)

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        global bantu_chrc_instance, bantu_connected, bantu_telemetry_active
        self.notifying = True
        bantu_chrc_instance = self
        bantu_connected = True
        bantu_telemetry_active = True
        log_bantu_event("App 通知使能", "已建立数据通道，握手完成", "READY")
        print("[*] 斑图 App 已成功使能 0xFFE1 数据通知通道！")

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        global bantu_connected, bantu_telemetry_active
        self.notifying = False
        bantu_connected = False
        bantu_telemetry_active = False
        log_bantu_event("App 断开通知", "数据通道已关闭", "CLOSED")
        print("[*] 斑图 App 关闭了数据通道")

class BantuAdvertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/bantu/advertisement'

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
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s')
            }
        }

    def get_path(self): return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE: raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        mark_ble("advertisement_registered", False)


def get_all_parameter_frames():
    """生成满足 App 握手及 SettingsSync 回读核验的全套参数帧（与 STM32 固件及 App 协议字节数严格一致）"""
    with bantu_lock:
        l_off = min(60, max(0, bantu_settings["left_zero_offset"] + 30))
        r_off = min(60, max(0, bantu_settings["right_zero_offset"] + 30))
        dz_laser = min(255, max(0, bantu_settings["laser_deadzone"]))
        dz_level = min(255, max(0, bantu_settings["level_deadzone"]))
        kp = min(255, max(0, bantu_settings["proportional_gain"]))
        l_min = min(200, max(0, bantu_settings["left_min_pwm"]))
        r_min = min(200, max(0, bantu_settings["right_min_pwm"]))
        l_down_min = min(200, max(0, bantu_settings["left_down_min_pwm"]))
        r_down_min = min(200, max(0, bantu_settings["right_down_min_pwm"]))
        freq = min(20000, max(1, bantu_settings["pwm_frequency"]))
        l_lim = min(100, max(1, bantu_settings["left_output_limit"]))
        r_lim = min(100, max(1, bantu_settings["right_output_limit"]))
        v_type = min(2, max(0, bantu_settings["valve_type"]))
        to_ms = min(10000, max(0, bantu_settings["app_timeout_ms"]))
        kp_ch = [min(6000, max(25, k)) for k in bantu_settings["kp_x100"]]
        kd = min(100, max(0, bantu_settings["derivative_gain"]))
        ch_max = [min(100, max(1, c)) for c in bantu_settings["channel_max_percent"]]
        c_mask = bantu_settings["channel_curve_mask"] & 0x0F
        ramp10 = min(30, max(0, bantu_settings["ramp_time_ms"] // 10))
        s_curve = [min(100, max(0, s)) for s in bantu_settings["s_curve_strength"]]
        pin_map = bantu_settings["pin_map_code"] & 0xFF
        pin_idle = bantu_settings["pin_idle_code"] & 0xFF

    return [
        # 1. 0x06: 左找平零偏 (5字节)
        bytes([0xAA, 0x06, l_off, 0x00, 0x55]),
        # 2. 0x07: 右找平零偏 (5字节)
        bytes([0xAA, 0x07, r_off, 0x00, 0x55]),
        # 3. 0x05: 死区/Kp/起动PWM (8字节)
        bytes([0xAA, 0x05, dz_laser, dz_level, kp, l_min, r_min, 0x55]),
        # 4. 0x08: PWM频率 (5字节)
        bytes([0xAA, 0x08, (freq >> 8) & 0xFF, freq & 0xFF, 0x55]),
        # 5. 0x09: 阀门输出限制 (5字节)
        bytes([0xAA, 0x09, l_lim, r_lim, 0x55]),
        # 6. 0x0F: 阀门响应特性 (5字节)
        bytes([0xAA, 0x0F, v_type, 0x00, 0x55]),
        # 7. 0x10: 四向门限 (7字节)
        bytes([0xAA, 0x10, l_min, l_down_min, r_min, r_down_min, 0x55]),
        # 8. 0x12: 蓝牙联锁超时 (5字节)
        bytes([0xAA, 0x12, (to_ms >> 8) & 0xFF, to_ms & 0xFF, 0x55]),
        # 9. 0x13: 引脚映射及空闲电平 (5字节)
        bytes([0xAA, 0x13, pin_map, pin_idle, 0x55]),
        # 10. 0x14: 四通道最大输出限制 (7字节)
        bytes([0xAA, 0x14, ch_max[0], ch_max[1], ch_max[2], ch_max[3], 0x55]),
        # 11. 0x16: S曲线使能掩码及Ramp步长 (5字节)
        bytes([0xAA, 0x16, c_mask, ramp10, 0x55]),
        # 12. 0x17: 四通道S曲线强度 (7字节)
        bytes([0xAA, 0x17, s_curve[0], s_curve[1], s_curve[2], s_curve[3], 0x55]),
        # 13-16. 0x20..0x23: 四通道细分 Kp (各5字节)
        bytes([0xAA, 0x20, (kp_ch[0] >> 8) & 0xFF, kp_ch[0] & 0xFF, 0x55]),
        bytes([0xAA, 0x21, (kp_ch[1] >> 8) & 0xFF, kp_ch[1] & 0xFF, 0x55]),
        bytes([0xAA, 0x22, (kp_ch[2] >> 8) & 0xFF, kp_ch[2] & 0xFF, 0x55]),
        bytes([0xAA, 0x23, (kp_ch[3] >> 8) & 0xFF, kp_ch[3] & 0xFF, 0x55]),
        # 17. 0x30: 全局微分增益 Kd (5字节)
        bytes([0xAA, 0x30, kd, 0x00, 0x55]),
        # 18. 0x0B: 双接收器能力 (5字节, receivers=2, mode=2)
        bytes([0xAA, 0x0B, 0x02, 0x02, 0x55]),
        # 19. 0x0C: 接收器在线与信号状态 (5字节, 双在线且有信号)
        bytes([0xAA, 0x0C, 0x03, 0x03, 0x55]),
        # 20. 0x0E: 固件版本 v2.4.0 (6字节)
        bytes([0xAA, 0x0E, 0x02, 0x04, 0x00, 0x55]),
        # 21. 0x18: 引导校准状态报告 (7字节)
        bytes([0xAA, 0x18, 0x00, 0x00, 0x00, 0x00, 0x55]),
        # 22. 0x19: 自整定状态报告 (8字节)
        bytes([0xAA, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]),
        # 23. 0x1A: 自整定最小PWM报告 (8字节)
        bytes([0xAA, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]),
        # 24. 0xD0: 诊断流状态 (8字节)
        bytes([0xAA, 0xD0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x55])
    ]

def handle_bantu_app_write(data, chrc):
    """处理来自《斑图智控》手机 App 发过来的控制帧与参数读写帧"""
    global bantu_settings
    if not data or len(data) < 2:
        return

    # 按 5 字节帧边界解包处理（支持多帧粘包与快速连发）
    idx = 0
    while idx + 5 <= len(data):
        if data[idx] == 0xBB and data[idx + 4] == 0x55:
            frame = data[idx:idx + 5]
            _process_single_frame(frame, chrc)
            idx += 5
        else:
            idx += 1

def _process_single_frame(frame, chrc):
    global bantu_settings
    hex_str = " ".join([f"{b:02X}" for b in frame])
    cmd = frame[1]
    d2 = frame[2]
    d3 = frame[3]

    # 1. 握手 / 读参数 / 链路心跳帧: BB FF CC FF 55
    if cmd == 0xFF and d2 == 0xCC and d3 == 0xFF:
        mark_ble("last_parameter_query_at", time.time())
        log_bantu_event("收到 App 握手", "读取系统参数 / 链路心跳 (readSettings)", hex_str)
        # App uses this query as a 100ms heartbeat. Never sleep in the
        # D-Bus callback: 24 * 12ms blocks reception faster than it can drain.
        # Coalesce repeated queries while one paced reply is in progress.
        if not getattr(chrc, "_settings_reply_active", False):
            chrc._settings_reply_active = True
            chrc._settings_reply_index = 0
            def send_next():
                if not chrc.notifying:
                    chrc._settings_reply_active = False
                    return False
                # Read current state at send time, not an obsolete snapshot.
                frames = get_all_parameter_frames()
                index = chrc._settings_reply_index
                if index >= len(frames):
                    chrc._settings_reply_active = False
                    return False
                chrc.notify_data(frames[index])
                chrc._settings_reply_index += 1
                if chrc._settings_reply_index == len(frames):
                    mark_ble("last_parameter_reply_at", time.time())
                    chrc._settings_reply_active = False
                    return False
                return True
            GLib.timeout_add(12, send_next)
        return

    # 2. 诊断数据流控制 BB D0 <0/1> 00 55
    if cmd == 0xD0:
        en = (d2 != 0)
        log_bantu_event("诊断流控制", f"{'开启' if en else '关闭'} 实时遥测诊断流", hex_str)
        chrc.notify_data(b'\xAA\xD0\x03\x00\x00\x00\x00\x55')
        return

    # 3. 手动控制动作 BB C1 <action> 00 55
    if cmd == 0xC1:
        action = d2
        act_map = {
            0: "停止操作",
            1: "左铲刀 抬起",
            2: "左铲刀 下降",
            3: "右铲刀 抬起",
            4: "右铲刀 下降",
            5: "双刀同步 抬起",
            6: "双刀同步 下降"
        }
        act_desc = act_map.get(action, f"动作码 {action}")
        with bantu_lock:
            bantu_settings["manual_action"] = action
            cur_mode = bantu_settings["mode"]
        log_bantu_event("手动控制指令", act_desc, hex_str)
        if action > 0:
            write_str("/sys/class/leds/work/trigger", "none")
            write_str("/sys/class/leds/work/brightness", "1")
        else:
            write_str("/sys/class/leds/work/trigger", "none")
            write_str("/sys/class/leds/work/brightness", "0")
        chrc.notify_data(bytes([0xAA, 0x03, 0x00, 0x00, cur_mode, 0x55]))
        return

    # 4. 自动找平模式切换 BB C2 <auto> 00 55
    if cmd == 0xC2:
        is_auto = (d2 != 0)
        with bantu_lock:
            bantu_settings["mode"] = 1 if is_auto else 0
            cur_mode = bantu_settings["mode"]
        log_bantu_event("工作模式切换", f"{'进入' if is_auto else '退出'} 自动激光平地模式", hex_str)
        chrc.notify_data(bytes([0xAA, 0x03, 0x00, 0x00, cur_mode, 0x55]))
        return

    # 5. 心跳保活 BB C3 5A A5 55
    if cmd == 0xC3:
        return

    # 6. 单项参数设置写入指令 (BB 01..30)
    changed = False
    with bantu_lock:
        if cmd == 0x01:
            bantu_settings["laser_deadzone"] = d2
            log_bantu_event("设置激光死区", f"数值: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x02:
            bantu_settings["level_deadzone"] = d2
            log_bantu_event("设置水平死区", f"数值: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x03:
            bantu_settings["proportional_gain"] = d2
            bantu_settings["kp_x100"] = [d2 * 100, d2 * 100, d2 * 100, d2 * 100]
            log_bantu_event("设置比例增益 Kp", f"整数 Kp: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            for channel in range(4):
                kp_raw = bantu_settings["kp_x100"][channel]
                chrc.notify_data(bytes([0xAA, 0x20 + channel, kp_raw >> 8, kp_raw & 0xFF, 0x55]))
            changed = True

        elif cmd == 0x04:
            bantu_settings["left_min_pwm"] = d2
            bantu_settings["left_down_min_pwm"] = d2
            log_bantu_event("设置左起动 PWM", f"数值: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            chrc.notify_data(bytes([0xAA, 0x10, bantu_settings["left_min_pwm"], bantu_settings["left_down_min_pwm"], bantu_settings["right_min_pwm"], bantu_settings["right_down_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x05:
            bantu_settings["right_min_pwm"] = d2
            bantu_settings["right_down_min_pwm"] = d2
            log_bantu_event("设置右起动 PWM", f"数值: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            chrc.notify_data(bytes([0xAA, 0x10, bantu_settings["left_min_pwm"], bantu_settings["left_down_min_pwm"], bantu_settings["right_min_pwm"], bantu_settings["right_down_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x06:
            if d2 <= 60:
                bantu_settings["left_zero_offset"] = d2 - 30
                log_bantu_event("设置左找平零偏", f"数值: {d2 - 30} (原始: {d2})", hex_str)
                changed = True
            chrc.notify_data(bytes([0xAA, 0x06, d2, 0x00, 0x55]))

        elif cmd == 0x07:
            if d2 <= 60:
                bantu_settings["right_zero_offset"] = d2 - 30
                log_bantu_event("设置右找平零偏", f"数值: {d2 - 30} (原始: {d2})", hex_str)
                changed = True
            chrc.notify_data(bytes([0xAA, 0x07, d2, 0x00, 0x55]))

        elif cmd == 0x08:
            freq = (d2 << 8) | d3
            bantu_settings["pwm_frequency"] = freq
            log_bantu_event("设置 PWM 频率", f"数值: {freq} Hz", hex_str)
            chrc.notify_data(bytes([0xAA, 0x08, d2, d3, 0x55]))
            changed = True

        elif cmd == 0x09:
            bantu_settings["left_output_limit"] = d2
            bantu_settings["channel_max_percent"][0] = d2
            bantu_settings["channel_max_percent"][1] = d2
            log_bantu_event("设置左输出限幅", f"数值: {d2}%", hex_str)
            chrc.notify_data(bytes([0xAA, 0x09, bantu_settings["left_output_limit"], bantu_settings["right_output_limit"], 0x55]))
            ch_max = bantu_settings["channel_max_percent"]
            chrc.notify_data(bytes([0xAA, 0x14, ch_max[0], ch_max[1], ch_max[2], ch_max[3], 0x55]))
            changed = True

        elif cmd == 0x0A:
            bantu_settings["right_output_limit"] = d2
            bantu_settings["channel_max_percent"][2] = d2
            bantu_settings["channel_max_percent"][3] = d2
            log_bantu_event("设置右输出限幅", f"数值: {d2}%", hex_str)
            chrc.notify_data(bytes([0xAA, 0x09, bantu_settings["left_output_limit"], bantu_settings["right_output_limit"], 0x55]))
            ch_max = bantu_settings["channel_max_percent"]
            chrc.notify_data(bytes([0xAA, 0x14, ch_max[0], ch_max[1], ch_max[2], ch_max[3], 0x55]))
            changed = True

        elif cmd == 0x0F:
            bantu_settings["valve_type"] = d2
            log_bantu_event("设置阀门响应特性", f"档位: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x0F, d2, 0x00, 0x55]))
            changed = True

        elif cmd == 0x10:
            bantu_settings["left_min_pwm"] = d2
            bantu_settings["left_down_min_pwm"] = d3
            log_bantu_event("设置左向 PWM 门限", f"升: {d2} 降: {d3}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x10, d2, d3, bantu_settings["right_min_pwm"], bantu_settings["right_down_min_pwm"], 0x55]))
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x11:
            bantu_settings["right_min_pwm"] = d2
            bantu_settings["right_down_min_pwm"] = d3
            log_bantu_event("设置右向 PWM 门限", f"升: {d2} 降: {d3}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x10, bantu_settings["left_min_pwm"], bantu_settings["left_down_min_pwm"], d2, d3, 0x55]))
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x12:
            to = (d2 << 8) | d3
            bantu_settings["app_timeout_ms"] = to
            log_bantu_event("设置失联保护超时", f"时长: {to} ms", hex_str)
            chrc.notify_data(bytes([0xAA, 0x12, d2, d3, 0x55]))
            changed = True

        elif cmd == 0x13:
            bantu_settings["pin_map_code"] = d2
            bantu_settings["pin_idle_code"] = d3
            log_bantu_event("设置引脚映射与空闲", f"映射: 0x{d2:02X}, 空闲: 0x{d3:02X}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x13, d2, d3, 0x55]))
            changed = True

        elif cmd == 0x14:
            bantu_settings["channel_max_percent"][0] = d2
            bantu_settings["channel_max_percent"][1] = d3
            bantu_settings["left_output_limit"] = max(d2, d3)
            log_bantu_event("设置CH1/CH2限幅", f"CH1: {d2}% CH2: {d3}%", hex_str)
            ch_max = bantu_settings["channel_max_percent"]
            chrc.notify_data(bytes([0xAA, 0x14, ch_max[0], ch_max[1], ch_max[2], ch_max[3], 0x55]))
            chrc.notify_data(bytes([0xAA, 0x09, bantu_settings["left_output_limit"], bantu_settings["right_output_limit"], 0x55]))
            changed = True

        elif cmd == 0x15:
            bantu_settings["channel_max_percent"][2] = d2
            bantu_settings["channel_max_percent"][3] = d3
            bantu_settings["right_output_limit"] = max(d2, d3)
            log_bantu_event("设置CH3/CH4限幅", f"CH3: {d2}% CH4: {d3}%", hex_str)
            ch_max = bantu_settings["channel_max_percent"]
            chrc.notify_data(bytes([0xAA, 0x14, ch_max[0], ch_max[1], ch_max[2], ch_max[3], 0x55]))
            chrc.notify_data(bytes([0xAA, 0x09, bantu_settings["left_output_limit"], bantu_settings["right_output_limit"], 0x55]))
            changed = True

        elif cmd == 0x16:
            bantu_settings["channel_curve_mask"] = d2 & 0x0F
            bantu_settings["ramp_time_ms"] = d3 * 10
            log_bantu_event("设置S曲线及Ramp", f"Mask: 0x{d2:02X}, Ramp: {d3 * 10} ms", hex_str)
            chrc.notify_data(bytes([0xAA, 0x16, d2 & 0x0F, d3, 0x55]))
            changed = True

        elif cmd == 0x17:
            bantu_settings["s_curve_strength"][0] = d2
            bantu_settings["s_curve_strength"][1] = d3
            log_bantu_event("设置CH1/CH2曲线强度", f"CH1: {d2}% CH2: {d3}%", hex_str)
            s_cur = bantu_settings["s_curve_strength"]
            chrc.notify_data(bytes([0xAA, 0x17, s_cur[0], s_cur[1], s_cur[2], s_cur[3], 0x55]))
            changed = True

        elif cmd == 0x1C:
            bantu_settings["s_curve_strength"][2] = d2
            bantu_settings["s_curve_strength"][3] = d3
            log_bantu_event("设置CH3/CH4曲线强度", f"CH3: {d2}% CH4: {d3}%", hex_str)
            s_cur = bantu_settings["s_curve_strength"]
            chrc.notify_data(bytes([0xAA, 0x17, s_cur[0], s_cur[1], s_cur[2], s_cur[3], 0x55]))
            changed = True

        elif 0x20 <= cmd <= 0x23:
            ch = cmd - 0x20
            kp_val = (d2 << 8) | d3
            bantu_settings["kp_x100"][ch] = kp_val
            if ch == 0:
                bantu_settings["proportional_gain"] = max(1, kp_val // 100)
            log_bantu_event(f"设置CH{ch+1} 细分Kp", f"数值: {kp_val / 100.0:.2f} (x100={kp_val})", hex_str)
            chrc.notify_data(bytes([0xAA, cmd, d2, d3, 0x55]))
            chrc.notify_data(bytes([0xAA, 0x05, bantu_settings["laser_deadzone"], bantu_settings["level_deadzone"], bantu_settings["proportional_gain"], bantu_settings["left_min_pwm"], bantu_settings["right_min_pwm"], 0x55]))
            changed = True

        elif cmd == 0x30:
            bantu_settings["derivative_gain"] = d2
            log_bantu_event("设置微分增益 Kd", f"数值: {d2}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x30, d2, 0x00, 0x55]))
            changed = True

        elif cmd == 0x18:
            log_bantu_event("引导校准指令", f"Op: {d2}, Arg: {d3}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x18, 0x00, 0x00, 0x00, 0x00, 0x55]))

        elif cmd == 0x19:
            log_bantu_event("自整定指令", f"Op: {d2}, Arg: {d3}", hex_str)
            chrc.notify_data(bytes([0xAA, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]))

        elif cmd == 0x1E:
            log_bantu_event("整定档位应用", f"档位: {d2}", hex_str)

        else:
            log_bantu_event("自定义指令写入", "已确认接收", hex_str)

    if changed:
        save_bantu_settings()

def bantu_telemetry_loop():
    """后台维持 App 遥测心跳与低延迟绿标"""
    global bantu_chrc_instance, bantu_telemetry_active, bantu_settings
    step = 0
    while True:
        try:
            if bantu_telemetry_active and bantu_chrc_instance and bantu_chrc_instance.notifying:
                # 动态 PWM 反馈
                with bantu_lock:
                    act = bantu_settings["manual_action"]
                    mode = bantu_settings["mode"]
                left_pwm = 50 if act in [1, 2, 5, 6] else 0
                right_pwm = 50 if act in [3, 4, 5, 6] else 0

                # 1. AA 01: 激光传感器标高信号帧 (左右光靶都在中位 28=0x1C, 倾角正常)
                bantu_chrc_instance.notify_data(b'\xAA\x01\x1C\x1C\x00\x00\x55')
                # 2. AA 04: 当前输出 PWM 帧 (7字节)
                bantu_chrc_instance.notify_data(bytes([0xAA, 0x04, 0x00, left_pwm, 0x00, right_pwm, 0x55]))
                # 3. AA 0D: 激光差值与控制延迟测算帧 (左右误差0, 驱动 App 毫秒级延时计算与绿标, 7字节)
                bantu_chrc_instance.notify_data(b'\xAA\x0D\x00\x00\x00\x00\x55')

                # 4. 间歇上报模式、双接收器与固件版本、扩展诊断 (每 1 秒 1 次)
                if step % 5 == 0:
                    bantu_chrc_instance.notify_data(bytes([0xAA, 0x03, 0x00, 0x00, mode, 0x55]))
                    bantu_chrc_instance.notify_data(b'\xAA\x0B\x02\x02\x55')  # 5 字节: 双接收器
                    bantu_chrc_instance.notify_data(b'\xAA\x0C\x03\x03\x55')  # 5 字节: 信号良好
                    bantu_chrc_instance.notify_data(b'\xAA\x0E\x02\x04\x00\x55')  # 6 字节: 固件版本
                    bantu_chrc_instance.notify_data(b'\xAA\xD0\x03\x00\x00\x00\x00\x55')  # 8 字节: 扩展诊断

                step += 1
        except Exception:
            pass
        time.sleep(0.2)

def start_bantu_gatt_thread():
    def _run():
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
        adapter = None
        for o, props in remote_om.GetManagedObjects().items():
            if GATT_MANAGER_IFACE in props.keys():
                adapter = o
                break
        if not adapter:
            mark_ble("registration_error", "未发现 GATT 适配器")
            print("[!] 未找到支持 GATT 的蓝牙适配器")
            return

        adapter_props = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), DBUS_PROP_IFACE)
        device_name = ble_health["advertised_name"]
        adapter_props.Set('org.bluez.Adapter1', 'Alias', dbus.String(device_name))
        adapter_props.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(True))
        adapter_props.Set('org.bluez.Adapter1', 'Discoverable', dbus.Boolean(True))

        service_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), GATT_MANAGER_IFACE)
        ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), LE_ADVERTISING_MANAGER_IFACE)

        app = Application(bus)
        bantu_svc = Service(bus, 0, '0000ffe0-0000-1000-8000-00805f9b34fb', True)
        bantu_chrc = BantuCharacteristic(bus, 0, bantu_svc)
        bantu_svc.add_characteristic(bantu_chrc)
        app.add_service(bantu_svc)

        adv = BantuAdvertisement(bus, 0, device_name)

        service_manager.RegisterApplication(app.get_path(), {},
                                             reply_handler=lambda: mark_ble("gatt_registered", True),
                                             error_handler=lambda e: mark_ble("registration_error", "GATT: " + str(e)))

        def on_adv_ready():
            mark_ble("advertisement_registered", True)
            print("[*] 斑图 BLE 广播注册就绪！")
            # 立即向硬件下发强制使能指令，确保 Realtek 芯片开始物理发射
            os.system("hcitool -i hci0 cmd 0x08 0x0039 01 01 01 00 00 00 >/dev/null 2>&1")
            os.system("hciconfig hci0 leadv 0 >/dev/null 2>&1")

        ad_manager.RegisterAdvertisement(adv.get_path(), {},
                                         reply_handler=on_adv_ready,
                                         error_handler=lambda e: mark_ble("registration_error", "广播: " + str(e)))

        # 启动遥测线程
        t = threading.Thread(target=bantu_telemetry_loop, daemon=True)
        t.start()


        loop = GLib.MainLoop()
        loop.run()

    gt = threading.Thread(target=_run, daemon=True)
    gt.start()

# ================= 硬件状态采集 (CPU/温区/风扇/烤机) =================





def get_fan_path():
    paths = glob.glob("/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1")
    return paths[0] if paths else ""

def get_fan_control_mode_path(fp=None):
    fp = fp or get_fan_path()
    if not fp: return ""
    path = os.path.join(os.path.dirname(fp), "control_mode")
    return path if os.path.exists(path) else ""

def load_fan_control_state():
    try:
        with open(FAN_CONTROL_FILE, "r", encoding="utf-8") as f:
            saved = json.load(f)
        mode = saved.get("mode", "auto")
        requested = int(saved.get("requested_pwm", 153))
        if mode not in ("auto", "manual"):
            mode = "auto"
        with fan_control_lock:
            fan_control_state["mode"] = mode
            fan_control_state["requested_pwm"] = max(0, min(255, requested))
            fan_control_state["overheat"] = False
    except Exception:
        pass

def save_fan_control_state():
    tmp = FAN_CONTROL_FILE + ".tmp"
    try:
        with fan_control_lock:
            saved = {
                "mode": fan_control_state["mode"],
                "requested_pwm": fan_control_state["requested_pwm"],
            }
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(saved, f, separators=(",", ":"))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, FAN_CONTROL_FILE)
        return True
    except Exception:
        try:
            if os.path.exists(tmp): os.remove(tmp)
        except Exception:
            pass
        return False

def get_fan_temperature():
    try:
        value = int(read_str("/sys/class/thermal/thermal_zone0/temp", ""))
        return value if -40000 <= value <= 200000 else None
    except Exception:
        return None

def fan_policy_target(mode, requested_pwm, temp_millideg, overheat):
    if temp_millideg is None:
        return 255, True
    if temp_millideg >= 60000:
        overheat = True
    elif temp_millideg <= 55000:
        overheat = False
    if overheat:
        return 255, True
    if mode == "manual":
        return max(0, min(255, int(requested_pwm))), False
    target = 0
    for threshold, pwm in FAN_TEMP_PWM_CURVE:
        if temp_millideg < threshold:
            break
        target = pwm
    return target, False











# Dashboard v2: real collectors, bounded history and truthful control feedback.
import collections
import copy
import math
import signal
import socket
import subprocess
import secrets

SCHEMA_VERSION = 2
CONTROL_TOKEN = secrets.token_urlsafe(24)
SESSION_ID = secrets.token_hex(8)
SAMPLE_INTERVAL = 1.0
snapshot_lock = threading.RLock()
control_lock = threading.RLock()
stress_lock = threading.RLock()
snapshot = {}
trend_history = collections.deque(maxlen=1801)
event_history = collections.deque(maxlen=100)
last_alerts = set()
last_counters = {}
slow_cache = {}
cpu_sample_at = None
cpu_usage_cache = [None] * (os.cpu_count() or 8)
total_usage_cache = None
bantu_log_seq = 0
ble_health = {"gatt_registered": False, "advertisement_registered": False,
              "registration_error": None, "rx_packets": 0, "tx_packets": 0,
              "rx_bytes": 0, "tx_bytes": 0, "last_rx_at": None,
              "last_tx_at": None, "rx_interval_ms": None, "tx_interval_ms": None,
              "settings_saved_at": None, "settings_save_error": None,
              "last_parameter_query_at": None, "last_parameter_reply_at": None,
              "advertised_name": "斑图-RK3588", "telemetry_source": "simulated"}
stress_restore = None


def number(path, scale=1.0):
    raw = read_str(path, None)
    try:
        value = float(raw) / scale
        return value if math.isfinite(value) else None
    except (ValueError, TypeError, OverflowError):
        return None


def rate(key, values, now=None):
    now = time.monotonic() if now is None else now
    previous = last_counters.get(key)
    last_counters[key] = (now, values)
    if previous is None or now <= previous[0]:
        return [None for _ in values]
    dt = now - previous[0]
    return [round((v - p) / dt, 2) if v is not None and p is not None and v >= p else None
            for v, p in zip(values, previous[1])]


def cached(key, ttl, callback):
    now = time.monotonic()
    old = slow_cache.get(key)
    if old is not None and now - old[0] < ttl:
        return old[1]
    value = callback()
    slow_cache[key] = (now, value)
    return value


def record_event(level, message):
    with snapshot_lock:
        event_history.append({"time": time.time(), "level": level, "message": str(message)})


def mark_ble(key, value=True):
    with bantu_lock:
        ble_health[key] = value


def record_ble_packet(direction, size):
    now = time.time()
    with bantu_lock:
        old = ble_health.get("last_" + direction + "_at")
        ble_health[direction + "_interval_ms"] = round((now - old) * 1000, 1) if old else None
        ble_health["last_" + direction + "_at"] = now
        ble_health[direction + "_packets"] += 1
        ble_health[direction + "_bytes"] += size


def log_bantu_event(tag, text, hex_str=""):
    global bantu_log_seq
    with bantu_lock:
        bantu_log_seq += 1
        bantu_rx_history.append({"id": bantu_log_seq, "timestamp": time.time(),
                                 "time": time.strftime("%H:%M:%S"),
                                 "tag": tag, "text": text, "hex": hex_str})
        del bantu_rx_history[:-300]


def save_bantu_settings():
    path = get_settings_file_path()
    # Preserve the existing parameter protocol; only persistence/observability changes.
    with bantu_lock:
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(dict(bantu_settings), f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(path + ".tmp", path)
            ble_health["settings_saved_at"] = time.time()
            ble_health["settings_save_error"] = None
            return True
        except Exception as exc:
            ble_health["settings_save_error"] = str(exc)
            return False


def update_cpu_metrics():
    global cpu_last_stats, cpu_usage_cache, total_usage_cache, cpu_sample_at
    text = read_str("/proc/stat", "")
    new = {}
    for line in text.splitlines():
        parts = line.split()
        if parts and re.fullmatch(r"cpu\d*", parts[0]) and len(parts) >= 5:
            values = [int(v) for v in parts[1:9]]  # guest already included in user/nice
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            new[parts[0]] = (idle, sum(values))
    def usage(key):
        if key not in new or key not in cpu_last_stats:
            return None
        idle, total = new[key]
        old_idle, old_total = cpu_last_stats[key]
        if total <= old_total or idle < old_idle:
            return None
        return round(max(0, min(100, 100 * (1 - (idle - old_idle) / (total - old_total)))), 1)
    n = max([int(k[3:]) + 1 for k in new if k != "cpu"] or [os.cpu_count() or 8])
    cpu_usage_cache = [usage("cpu" + str(i)) for i in range(n)]
    total_usage_cache = usage("cpu")
    cpu_last_stats = new
    cpu_sample_at = time.time() if new else None


def get_soc_info():
    uptime = number("/proc/uptime")  # uptime file has two fields
    raw = read_str("/proc/uptime", "").split()
    try: uptime = int(float(raw[0]))
    except (ValueError, IndexError): uptime = None
    with stress_lock:
        stress = dict(stress_status)
        stress["elapsed"] = max(0, int(time.monotonic() - stress.get("monotonic_start", time.monotonic()))) if stress["running"] else 0
        stress.pop("monotonic_start", None)
    u = os.uname()
    return {"model": read_str("/proc/device-tree/model", "未知型号").strip("\x00"),
            "kernel": u.sysname + " " + u.release, "arch": u.machine,
            "uptime_seconds": uptime, "uptime": (f"{uptime//3600}小时 {(uptime%3600)//60}分 {uptime%60}秒" if uptime is not None else "未采集"),
            "loadavg": list(os.getloadavg()), "total_cpu_usage": total_usage_cache,
            "cpu_sample_at": cpu_sample_at, "stress": stress}


def get_cpu_deep_info():
    clusters, cores = [], []
    for base in sorted(glob.glob("/sys/devices/system/cpu/cpufreq/policy*")):
        ids = [int(x) for x in read_str(base + "/related_cpus", "").split() if x.isdigit()]
        cid = os.path.basename(base)
        cluster = {"id": cid, "core_ids": ids, "name": "效率核 A55" if ids and max(ids) < 4 else "性能核 A76 · " + cid.replace("policy", ""),
                   "freq_mhz": number(base + "/scaling_cur_freq", 1000),
                   "min_mhz": number(base + "/scaling_min_freq", 1000),
                   "max_mhz": number(base + "/scaling_max_freq", 1000),
                   "hardware_max_mhz": number(base + "/cpuinfo_max_freq", 1000),
                   "governor": read_str(base + "/scaling_governor", None),
                   "available_governors": read_str(base + "/scaling_available_governors", "").split(),
                   "source": base, "frequency_kind": "driver_reported"}
        cluster["policy_limited"] = (cluster["max_mhz"] < cluster["hardware_max_mhz"]) if cluster["max_mhz"] is not None and cluster["hardware_max_mhz"] is not None else None
        clusters.append(cluster)
        for i in ids:
            online = read_str(f"/sys/devices/system/cpu/cpu{i}/online", "1") == "1"
            cores.append({"id": i, "name": "Core " + str(i), "cluster": cid,
                          "type": "Cortex-A55" if i < 4 else "Cortex-A76", "online": online,
                          "usage": cpu_usage_cache[i] if online and i < len(cpu_usage_cache) else None,
                          **{k: cluster[k] for k in ("freq_mhz", "min_mhz", "max_mhz", "governor")}})
    return {"clusters": clusters, "cores": cores, "status": "ok" if cores else "unavailable"}


def get_all_thermals():
    names = {"soc-thermal": "SoC", "bigcore0-thermal": "大核集群 0", "bigcore1-thermal": "大核集群 1",
             "littlecore-thermal": "小核集群", "gpu-thermal": "GPU", "npu-thermal": "NPU", "center-thermal": "芯片中心"}
    zones = []
    for base in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        kind = read_str(base + "/type", os.path.basename(base))
        temp = number(base + "/temp", 1000)
        if temp is not None and not -40 <= temp <= 200: temp = None
        trips = []
        for f in glob.glob(base + "/trip_point_*_temp"):
            trips.append({"type": read_str(f.replace("_temp", "_type"), "unknown"), "temp": number(f, 1000), "hysteresis": number(f.replace("_temp", "_hyst"), 1000)})
        zones.append({"zone": os.path.basename(base), "type": kind, "name": names.get(kind, kind),
                      "temp": round(temp, 1) if temp is not None else None, "trips": trips,
                      "source": base + "/temp", "status": "ok" if temp is not None else "unavailable"})
    return zones


def get_hetero_engines():
    paths = glob.glob("/sys/class/devfreq/*")
    result = {}
    for key, needle, label in (("gpu", ".gpu", "Mali-G610"), ("npu", ".npu", "RK3588 NPU"), ("dmc", "dmc", "DMC 内存总线")):
        path = next((p for p in paths if os.path.basename(p).endswith(needle)), None)
        freq = number(path + "/cur_freq", 1e6) if path else None
        load_raw = read_str(path + "/load", "") if path else ""
        match = re.match(r"^(\d+(?:\.\d+)?)@", load_raw)
        load_pct = float(match.group(1)) if match else None
        if load_pct is not None and not 0 <= load_pct <= 100: load_pct = None
        item = {"name": label, "freq_mhz": freq, "load_percent": load_pct,
                "load": f"{load_pct:g}%" if load_pct is not None else None,
                "governor": read_str(path + "/governor", None) if path else None,
                "source": path, "load_source": path + "/load" if path else None,
                "status": "ok" if freq is not None else "unavailable"}
        if key == "npu":
            raw = read_str("/sys/kernel/debug/rknpu/load", "")
            loads = {int(i): float(v) for i, v in re.findall(r"Core(\d+):\s*(\d+(?:\.\d+)?)%", raw)}
            item["core_loads"] = [loads.get(i) for i in range(3)]
            item["load_percent"] = round(sum(loads.values()) / len(loads), 1) if loads else None
            item["load"] = raw or None
            item["load_source"] = "/sys/kernel/debug/rknpu/load"
            item["driver_status"] = "负载节点可读" if loads else "负载节点未采集"
            item["inference"] = {"status": "not_connected", "model": None, "latency_ms": None, "fps": None}
        result[key] = item
    return result


def get_memory_info():
    values = {}
    for line in read_str("/proc/meminfo", "").splitlines():
        m = re.match(r"(\w+):\s+(\d+)", line)
        if m: values[m[1]] = int(m[2]) * 1024
    total, available = values.get("MemTotal"), values.get("MemAvailable")
    used = max(0, total - available) if total is not None and available is not None else None
    swap_total, swap_free = values.get("SwapTotal"), values.get("SwapFree")
    return {"total_bytes": total, "available_bytes": available, "used_bytes": used,
            "used_percent": round(100 * used / total, 1) if used is not None and total else None,
            "swap_total_bytes": swap_total, "swap_used_bytes": swap_total - swap_free if swap_total is not None and swap_free is not None else None,
            "source": "/proc/meminfo", "status": "ok" if used is not None else "unavailable"}


def get_storage_info():
    volumes = []
    for line in read_str("/proc/mounts", "").splitlines():
        parts = line.split()
        if len(parts) < 3 or not parts[0].startswith("/dev/"): continue
        device, mount, filesystem = parts[:3]
        mount = re.sub(r"\\([0-7]{3})", lambda m: chr(int(m[1], 8)), mount)
        try:
            st = os.statvfs(mount)
            total, free, available = st.f_blocks * st.f_frsize, st.f_bfree * st.f_frsize, st.f_bavail * st.f_frsize
            volumes.append({"device": device, "mount": mount, "filesystem": filesystem,
                            "total_bytes": total, "used_bytes": total - free, "available_bytes": available,
                            "used_percent": round((total - free) * 100 / total, 1) if total else None})
        except OSError: continue
    devices = []
    for line in read_str("/proc/diskstats", "").splitlines():
        p = line.split()
        if len(p) < 14: continue
        name = p[2]
        if not os.path.exists("/sys/block/" + name) or name.startswith(("loop", "ram", "zram")) or "boot" in name: continue
        read_bps, write_bps = rate("disk:" + name, [int(p[5]) * 512, int(p[9]) * 512])
        devices.append({"name": name, "read_bps": read_bps, "write_bps": write_bps})
    return {"volumes": volumes, "devices": devices, "source": "/proc/mounts + statvfs + /proc/diskstats", "status": "ok" if volumes else "unavailable"}


def network_addresses():
    try:
        data = json.loads(subprocess.check_output(["ip", "-j", "address", "show"], timeout=1, stderr=subprocess.DEVNULL))
        return {i["ifname"]: [a["local"] for a in i.get("addr_info", []) if a.get("scope") == "global"] for i in data}
    except Exception: return {}


def get_network_info():
    addresses = cached("addresses", 5, network_addresses)
    items = []
    for base in sorted(glob.glob("/sys/class/net/*")):
        name = os.path.basename(base)
        if name == "lo": continue
        rx = number(base + "/statistics/rx_bytes")
        tx = number(base + "/statistics/tx_bytes")
        rb, tb = rate("net:" + name, [rx, tx])
        speed = number(base + "/speed")
        items.append({"name": name, "state": read_str(base + "/operstate", "unknown"),
                      "carrier": number(base + "/carrier"), "addresses": addresses.get(name, []),
                      "rx_bps": rb, "tx_bps": tb, "speed_mbps": speed if speed is not None and speed > 0 else None,
                      "rx_errors": number(base + "/statistics/rx_errors"), "tx_errors": number(base + "/statistics/tx_errors")})
    return {"interfaces": items, "source": "/sys/class/net + ip address", "status": "ok" if items else "unavailable"}


def get_peripherals():
    displays, usb = [], []
    for base in sorted(glob.glob("/sys/class/drm/card*-*")):
        if not os.path.exists(base + "/status"): continue
        displays.append({"name": os.path.basename(base), "status": read_str(base + "/status", "unknown"),
                         "enabled": read_str(base + "/enabled", "unknown"),
                         "modes": read_str(base + "/modes", "").splitlines()})
    for base in sorted(glob.glob("/sys/bus/usb/devices/*")):
        if not os.path.exists(base + "/idVendor"): continue
        usb.append({"name": read_str(base + "/product", os.path.basename(base)),
                    "vendor": read_str(base + "/idVendor", ""), "product": read_str(base + "/idProduct", ""),
                    "speed_mbps": number(base + "/speed"), "path": os.path.basename(base)})
    return {"displays": displays, "usb": usb,
            "serial": sorted(glob.glob("/dev/ttyS*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")),
            "i2c": sorted(glob.glob("/dev/i2c-*")), "sampled_at": time.time(), "source": "sysfs / devfs（仅枚举）"}


def bluez_state():
    try:
        bus = dbus.SystemBus()
        obj = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
        objects = obj.GetManagedObjects(timeout=1)
        adapters, peers = [], []
        for path, interfaces in objects.items():
            if 'org.bluez.Adapter1' in interfaces:
                p = interfaces['org.bluez.Adapter1']
                adapters.append({"path": str(path), "powered": bool(p.get('Powered', False)), "name": str(p.get('Alias', ''))})
            if 'org.bluez.Device1' in interfaces:
                p = interfaces['org.bluez.Device1']
                if p.get('Connected', False): peers.append({"name": str(p.get('Alias', '设备')), "connected": True})
        return {"adapters": adapters, "peers": peers, "error": None, "sampled_at": time.time()}
    except Exception as exc:
        return {"adapters": [], "peers": [], "error": str(exc), "sampled_at": time.time()}


def get_ble_info():
    state = cached("bluez", 3, bluez_state)
    with bantu_lock:
        health = dict(ble_health)
        health.update({"notify_subscribed": bool(bantu_connected), "telemetry_active": bool(bantu_telemetry_active),
                       "connected": bool(state["peers"]), "settings": copy.deepcopy(bantu_settings),
                       "last_log_id": bantu_log_seq, "session_id": SESSION_ID, **state})
    health["rx_age_seconds"] = max(0, time.time() - health["last_rx_at"]) if health["last_rx_at"] else None
    health["tx_age_seconds"] = max(0, time.time() - health["last_tx_at"]) if health["last_tx_at"] else None
    return health


def get_fan_info():
    fp = get_fan_path()
    val = number(fp) if fp else None
    if val is not None and not 0 <= val <= 255: val = None
    mode_path = get_fan_control_mode_path(fp) if fp else None
    with fan_control_lock:
        state = dict(fan_control_state)
    return {"exists": bool(fp), "speed_raw": val, "percent": round(val / 255 * 100) if val is not None else None,
            "requested_percent": round(state["requested_pwm"] / 255 * 100),
            "mode": read_str(mode_path, state["mode"]) if mode_path else state["mode"],
            "mode_supported": bool(fp), "controller": "kernel" if mode_path else "userspace",
            "overheat": state["overheat"], "rpm": number(os.path.join(os.path.dirname(fp), "fan1_input")) if fp else None,
            "source": fp or None, "status": "ok" if val is not None else "unavailable",
            "control_temperature": get_fan_temperature(), "override_at_c": 60, "release_at_c": 55,
            "curve": [{"temp_c": t/1000, "pwm_percent": round(p/255*100)} for t,p in FAN_TEMP_PWM_CURVE]}


def apply_fan_policy():
    # Serialize policy calculation and sysfs write with manual/automatic changes.
    with fan_control_lock:
        fp = get_fan_path()
        if not fp: return False
        mode = fan_control_state["mode"]
        target, overheat = fan_policy_target(mode, fan_control_state["requested_pwm"],
                                             get_fan_temperature(), fan_control_state["overheat"])
        fan_control_state["overheat"] = overheat
        if not write_str(fp + "_enable", "1"): return False
        mode_path = get_fan_control_mode_path(fp)
        if mode_path:
            if mode == "auto":
                return write_str(mode_path, "auto") and read_str(mode_path, None) == "auto"
            if not write_str(mode_path, "manual"): return False
        if read_str(fp, None) != str(target) and not write_str(fp, str(target)): return False
        return number(fp) == target


def set_fan_mode_checked(mode, requested=None):
    with fan_control_lock:
        old = dict(fan_control_state)
        fan_control_state["mode"] = mode
        if requested is not None: fan_control_state["requested_pwm"] = requested
        if save_fan_control_state() and apply_fan_policy():
            info = get_fan_info()
            if info["mode"] == mode and info["speed_raw"] is not None:
                return True
        fan_control_state.update(old)
        save_fan_control_state()
        apply_fan_policy()
        return False


def set_fan_auto():
    return set_fan_mode_checked("auto")


def set_fan_speed(raw_or_pct, is_percent=True):
    if not get_fan_path(): return False
    raw = round(max(0, min(100, int(raw_or_pct))) / 100 * 255) if is_percent else max(0, min(255, int(raw_or_pct)))
    return set_fan_mode_checked("manual", raw)


def get_io_and_peripherals():
    leds = []
    for base in glob.glob("/sys/class/leds/*"):
        raw = read_str(base + "/trigger", "")
        match = re.search(r"\[(.*?)\]", raw)
        brightness = number(base + "/brightness")
        leds.append({"name": os.path.basename(base), "trigger": match[1] if match else None,
                     "brightness": brightness, "is_on": brightness > 0 if brightness is not None else None,
                     "source": base, "status": "ok" if match and brightness is not None else "unavailable"})
    return {"leds": leds}


def get_cooling():
    return [{"name": read_str(p + "/type", os.path.basename(p)), "state": number(p + "/cur_state"),
             "max_state": number(p + "/max_state")} for p in glob.glob("/sys/class/thermal/cooling_device*")]


def collect_snapshot():
    started = time.monotonic()
    result = {}
    jobs = {"soc": get_soc_info, "cpus": get_cpu_deep_info, "thermals": get_all_thermals,
            "hetero": get_hetero_engines, "memory": get_memory_info, "storage": get_storage_info,
            "network": get_network_info, "fan": get_fan_info, "io": get_io_and_peripherals,
            "bantu": get_ble_info, "peripherals": lambda: cached("peripherals", 10, get_peripherals),
            "cooling": get_cooling}
    errors = {}
    for key, fn in jobs.items():
        try: result[key] = fn()
        except Exception as exc:
            errors[key] = str(exc)
            result[key] = [] if key in ("thermals", "cooling") else {"status": "error", "error": str(exc)}
    result["meta"] = {"schema_version": SCHEMA_VERSION, "session_id": SESSION_ID,
                      "sampled_at": time.time(), "interval_seconds": SAMPLE_INTERVAL,
                      "collection_ms": round((time.monotonic() - started) * 1000, 1), "errors": errors}
    return result


def evaluate_alerts(data):
    alerts = []
    for z in data.get("thermals", []):
        if z.get("temp") is None:
            alerts.append({"id": "missing-" + z["zone"], "level": "warning", "message": z["name"] + "温度未采集"})
        for trip in z.get("trips", []):
            if z.get("temp") is not None and trip.get("temp") is not None and z["temp"] >= trip["temp"]:
                alerts.append({"id": z["zone"] + trip["type"], "level": "danger", "message": z["name"] + "达到 " + trip["type"] + " 阈值"})
    for name, engine in data.get("hetero", {}).items():
        if not isinstance(engine, dict): continue
        if engine.get("freq_mhz") is None or engine.get("load_percent") is None:
            alerts.append({"id": "engine-" + name, "level": "info", "message": name.upper() + " 部分指标未采集"})
    for name in ("memory", "storage", "network", "fan"):
        if data.get(name, {}).get("status") in ("error", "unavailable"):
            alerts.append({"id": "unavailable-" + name, "level": "warning", "message": name + " 指标未采集"})
    if data.get("fan", {}).get("overheat"):
        alerts.append({"id": "fan-override", "level": "warning", "message": "风扇满速接管：温度达到 60°C 或温度读取异常；55°C 以下解除"})
    if (data.get("memory", {}).get("used_percent") or 0) > 90:
        alerts.append({"id": "memory", "level": "warning", "message": "内存使用超过 90%"})
    for v in data.get("storage", {}).get("volumes", []):
        if (v.get("used_percent") or 0) > 90: alerts.append({"id": "disk" + v["mount"], "level": "warning", "message": v["mount"] + " 存储使用超过 90%"})
    for key in data.get("meta", {}).get("errors", {}):
        alerts.append({"id": "collector-" + key, "level": "warning", "message": key + " 采集失败"})
    b = data.get("bantu", {})
    if b.get("registration_error") or b.get("error"):
        alerts.append({"id": "ble-service", "level": "warning", "message": "BLE 服务状态异常，请查看通信页"})
    if b.get("settings_save_error"):
        alerts.append({"id": "ble-save", "level": "warning", "message": "BLE 参数持久化失败"})
    if b.get("notify_subscribed") and (b.get("tx_age_seconds") is None or b["tx_age_seconds"] > 3):
        alerts.append({"id": "ble-heartbeat", "level": "warning", "message": "BLE 通知已订阅，但遥测发送超过 3 秒未更新"})
    if data.get("soc", {}).get("total_cpu_usage") is None:
        alerts.append({"id": "cpu-sampling", "level": "info", "message": "CPU 占用等待有效采样"})
    return alerts


def publish_snapshot(data):
    global snapshot, last_alerts
    data["alerts"] = evaluate_alerts(data)
    ids = {a["id"] for a in data["alerts"]}
    for a in data["alerts"]:
        if a["id"] not in last_alerts: record_event(a["level"], a["message"])
    if last_alerts - ids: record_event("info", "部分告警已恢复")
    last_alerts = ids
    temps = [t["temp"] for t in data.get("thermals", []) if t.get("temp") is not None]
    point = {"time": data["meta"]["sampled_at"], "cpu": data.get("soc", {}).get("total_cpu_usage"),
             "temperature": max(temps) if temps else None, "memory": data.get("memory", {}).get("used_percent"),
             "fan": data.get("fan", {}).get("percent")}
    with snapshot_lock:
        snapshot = data
        trend_history.append(point)


def telemetry_loop():
    while True:
        started = time.monotonic()
        try:
            update_cpu_metrics()
            publish_snapshot(collect_snapshot())
        except Exception as exc:
            record_event("warning", "采样失败: " + str(exc))
        time.sleep(max(0.05, SAMPLE_INTERVAL - (time.monotonic() - started)))


def monitor_loop():
    last_error = False
    while True:
        try:
            ok = apply_fan_policy()
            if not ok and not last_error: record_event("warning", "风扇策略写入失败")
            last_error = not ok
            with stress_lock:
                stop = stress_status["running"] and time.monotonic() - stress_status.get("monotonic_start", 0) >= stress_status["duration"]
            if stop: stop_stress_internal()
        except Exception as exc:
            if not last_error: record_event("warning", "控制线程: " + str(exc))
            last_error = True
        time.sleep(0.5)


def _burn_worker():
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    while True: _ = sum(i*i for i in range(10000))


def start_stress_internal(duration=30):
    global stress_processes, stress_status, stress_restore
    if not 1 <= duration <= 120: raise ValueError("压测时长应为 1–120 秒")
    with control_lock, stress_lock:
        if stress_status["running"]: raise ValueError("压测已在运行")
        temp = get_fan_temperature()
        if temp is None or temp >= 60000: raise ValueError("压测启动条件：温度有效且低于 60°C")
        policies = glob.glob("/sys/devices/system/cpu/cpufreq/policy*/scaling_governor")
        with fan_control_lock: saved_fan = dict(fan_control_state)
        stress_restore = {"governors": {p: read_str(p, None) for p in policies}, "fan": saved_fan}
        stress_processes = []
        try:
            if not policies or any(v is None for v in stress_restore["governors"].values()): raise ValueError("CPU 策略读取失败")
            for p in policies:
                if not write_str(p, "performance"): raise ValueError("性能策略写入失败")
            if not set_fan_speed(100): raise ValueError("风扇满速设置失败")
            stress_status = {"running": True, "start_time": time.time(), "monotonic_start": time.monotonic(), "duration": duration}
            for _ in range(len(cpu_usage_cache)):
                p = multiprocessing.Process(target=_burn_worker, daemon=True)
                p.start()
                stress_processes.append(p)
            record_event("info", f"压测启动 · {duration} 秒；结束后恢复原策略")
        except Exception:
            stop_stress_internal()
            raise
    return True


def stop_stress_internal():
    global stress_processes, stress_restore
    with control_lock, stress_lock:
        for p in stress_processes:
            if p.is_alive(): p.terminate()
            p.join(timeout=0.5)
            if p.is_alive(): p.kill(); p.join(timeout=0.5)
        stress_processes = []
        stress_status["running"] = False
        errors = []
        if stress_restore:
            for path, gov in stress_restore["governors"].items():
                if gov is not None and (not write_str(path, gov) or read_str(path, None) != gov): errors.append(path)
            with fan_control_lock:
                fan_control_state.update({k: stress_restore["fan"][k] for k in ("mode", "requested_pwm")})
            if not save_fan_control_state() or not apply_fan_policy(): errors.append("fan")
            stress_restore = None
            record_event("warning" if errors else "info", "压测结束 · " + ("恢复失败: " + ", ".join(errors) if errors else "已恢复原调频与风扇模式"))
        if errors: raise ValueError("恢复失败: " + ", ".join(errors))
    return True


class FullProbeHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def respond(self, status, payload, content_type="application/json; charset=utf-8"):
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode() if content_type.startswith("application/json") else payload.encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        try: self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError): pass

    def do_GET(self):
        parsed = urlparse(self.path)
        params = parse_qs(parsed.query)
        if parsed.path in ("/", "/index.html"):
            return self.respond(200, HTML_PAGE.replace("__CONTROL_TOKEN__", CONTROL_TOKEN), "text/html; charset=utf-8")
        if parsed.path == "/favicon.ico": return self.respond(204, "", "image/svg+xml")
        if parsed.path == "/api/all":
            try: since = max(0, int(params.get("since", [0])[0]))
            except ValueError: return self.respond(400, {"error": "日志游标格式错误"})
            with snapshot_lock:
                data = copy.deepcopy(snapshot)
                data["events"] = list(event_history)[-30:]
            if "meta" not in data: return self.respond(503, {"error": "采集器正在启动"})
            data["meta"]["age_seconds"] = round(max(0, time.time() - data["meta"]["sampled_at"]), 2)
            data["meta"]["stale"] = data["meta"]["age_seconds"] > 3
            # Control status is read through, not served from the 1 s sensor cache.
            # This prevents an earlier idle snapshot from undoing a successful start.
            with control_lock:
                data["soc"]["stress"] = get_soc_info()["stress"]
                data["fan"] = get_fan_info()
                data["io"] = get_io_and_peripherals()
                governors = {}
                for cluster in data.get("cpus", {}).get("clusters", []):
                    cluster["governor"] = read_str(cluster["source"] + "/scaling_governor", None)
                    governors[cluster["id"]] = cluster["governor"]
                for core in data.get("cpus", {}).get("cores", []):
                    core["governor"] = governors.get(core["cluster"])
                data["meta"]["controls_sampled_at"] = time.time()
            with bantu_lock:
                if params.get("session", [SESSION_ID])[0] != SESSION_ID: since = 0
                data["bantu"]["history"] = [dict(x) for x in bantu_rx_history if x["id"] > since]
                data["bantu"]["last_log_id"] = bantu_log_seq
                data["bantu"]["first_log_id"] = bantu_rx_history[0]["id"] if bantu_rx_history else None
            return self.respond(200, data)
        if parsed.path == "/api/history":
            try: seconds = min(1800, max(60, int(params.get("seconds", [300])[0])))
            except ValueError: return self.respond(400, {"error": "时间范围格式错误"})
            with snapshot_lock:
                points = [dict(x) for x in trend_history if x["time"] >= time.time() - seconds]
            return self.respond(200, {"points": points, "session_id": SESSION_ID, "retention_seconds": 1800, "persistence": "process_memory"})
        return self.respond(404, {"error": "接口不存在"})

    def do_POST(self):
        if self.headers.get("X-Dashboard-Token") != CONTROL_TOKEN:
            return self.respond(403, {"status": "error", "error": "控制凭据已过期，请刷新页面"})
        parsed = urlparse(self.path)
        params = parse_qs(parsed.query)
        def param(k, default=None): return params.get(k, [default])[0]
        try:
            with control_lock:
                if stress_status["running"] and parsed.path in ("/api/fan", "/api/governor"):
                    raise ValueError("压测运行中，先停止压测再修改策略")
                if parsed.path == "/api/fan":
                    mode = param("mode", "manual")
                    if mode == "auto": ok = set_fan_auto()
                    elif mode == "manual":
                        pct = int(param("pct", "50"))
                        if not 0 <= pct <= 100: raise ValueError("PWM 应为 0–100")
                        ok = set_fan_speed(pct)
                    else: raise ValueError("风扇模式格式错误")
                    if not ok: raise ValueError("风扇写入、持久化或回读失败")
                    result = {"fan": get_fan_info()}
                elif parsed.path == "/api/governor":
                    gov = param("gov")
                    bases = glob.glob("/sys/devices/system/cpu/cpufreq/policy*")
                    if not bases or any(gov not in read_str(b + "/scaling_available_governors", "").split() for b in bases):
                        raise ValueError("该策略未在全部集群中提供")
                    old = {b: read_str(b + "/scaling_governor", None) for b in bases}
                    try:
                        for b in bases:
                            p = b + "/scaling_governor"
                            if not write_str(p, gov) or read_str(p, None) != gov: raise ValueError("策略写入/回读失败")
                    except Exception:
                        for b, v in old.items():
                            if v: write_str(b + "/scaling_governor", v)
                        raise
                    result = {"cpus": get_cpu_deep_info()}
                elif parsed.path == "/api/led":
                    mode = param("mode")
                    if mode not in ("on", "off", "heartbeat"): raise ValueError("LED 模式格式错误")
                    base = "/sys/class/leds/work/"
                    ok = write_str(base + "trigger", "heartbeat" if mode == "heartbeat" else "none")
                    if mode != "heartbeat": ok = write_str(base + "brightness", "1" if mode == "on" else "0") and ok
                    expected = "[heartbeat]" if mode == "heartbeat" else "[none]"
                    if not ok or expected not in read_str(base + "trigger", ""): raise ValueError("LED 写入/回读失败")
                    if mode != "heartbeat" and read_str(base + "brightness", None) != ("1" if mode == "on" else "0"): raise ValueError("LED 亮度回读失败")
                    result = {"io": get_io_and_peripherals()}
                elif parsed.path == "/api/stress":
                    if param("action") == "start": start_stress_internal(int(param("sec", "30")))
                    elif param("action") == "stop": stop_stress_internal()
                    else: raise ValueError("压测操作格式错误")
                    result = {"soc": get_soc_info()}
                else: return self.respond(404, {"error": "接口不存在"})
            record_event("info", parsed.path.replace("/api/", "") + " 控制已回读")
            return self.respond(200, {"status": "ok", **result})
        except (ValueError, TypeError) as exc:
            return self.respond(409, {"status": "error", "error": str(exc)})
        except Exception as exc:
            record_event("warning", "控制失败: " + str(exc))
            return self.respond(500, {"status": "error", "error": str(exc)})


def main():
    multiprocessing.set_start_method("fork", force=True)
    load_bantu_settings()
    load_fan_control_state()
    def shutdown(signum, frame):
        stop_stress_internal()
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    threading.Thread(target=monitor_loop, daemon=True).start()
    threading.Thread(target=telemetry_loop, daemon=True).start()
    start_bantu_gatt_thread()
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    socketserver.ThreadingTCPServer.daemon_threads = True
    with socketserver.ThreadingTCPServer(("0.0.0.0", PORT), FullProbeHandler) as httpd:
        print("[*] RK3588 Monitor v2 listening :" + str(PORT))
        httpd.serve_forever()

HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="color-scheme" content="dark">
<title>RK3588 · 硬件监控台</title>
<style>
:root{--bg:#0b111b;--panel:#121c2b;--panel2:#172336;--line:#324154;--text:#edf3fb;--muted:#a6b5cb;--accent:#80b5ff;--green:#7be0ba;--orange:#ffc184;--red:#ff9a9f;--radius:14px;font-family:Inter,-apple-system,BlinkMacSystemFont,"PingFang SC","Microsoft YaHei",sans-serif;color:var(--text);background:var(--bg);font-synthesis:none}
*{box-sizing:border-box}body{margin:0;font-size:14px;line-height:1.55}button,input,select{font:inherit}button,select{min-height:40px;border:1px solid #586c86;border-radius:8px;background:#1a2940;color:var(--text);padding:8px 13px;cursor:pointer;transition:background .15s,border-color .15s}button:hover,select:hover{background:#253955;border-color:var(--accent)}button:focus-visible,input:focus-visible,select:focus-visible,a:focus-visible{outline:3px solid var(--accent);outline-offset:3px}button:disabled{opacity:.5;cursor:not-allowed}button[aria-pressed=true],button.selected{background:#294367;border-color:var(--accent);color:#fff}button.primary{background:#9bc5ff;color:#0b1728;border-color:#9bc5ff;font-weight:650}button.danger{color:var(--red);border-color:#b3747e}button.small{min-height:34px;padding:5px 10px;font-size:12px}.muted,small{color:var(--muted)}small{font-size:12px}.mono,.value,.metric strong,table{font-variant-numeric:tabular-nums}.mono{font-family:ui-monospace,SFMono-Regular,Consolas,monospace}h1,h2,h3,p{margin:0}h1{font-size:22px;letter-spacing:-.5px}h2{font-size:16px}h3{font-size:14px}a{color:var(--accent)}[hidden]{display:none!important}.shell{max-width:1500px;margin:auto;padding:24px 30px 40px}.top{display:flex;justify-content:space-between;align-items:center;gap:20px;margin-bottom:22px}.brand{display:flex;gap:14px;align-items:center}.chip-logo{width:44px;height:44px;border:1px solid #4c729d;border-radius:12px;display:grid;place-items:center;color:var(--accent);background:#1c304a;flex-shrink:0}.eyebrow{font-size:10px;letter-spacing:2px;text-transform:uppercase;color:var(--muted);font-weight:650;margin-bottom:3px}.top-actions{display:flex;gap:9px;align-items:center;flex-wrap:wrap;justify-content:flex-end}.status{display:inline-flex;align-items:center;gap:7px;font-size:12px;padding:5px 9px;border-radius:20px;background:#142c2b;color:var(--green);white-space:nowrap}.status:before{content:"";width:6px;height:6px;border-radius:50%;background:currentColor}.status.warn{color:var(--orange);background:#352a1e}.status.error{color:var(--red);background:#36232d}.tabs{display:flex;gap:22px;border-bottom:1px solid var(--line);margin-bottom:24px}.tabs button{border:0;border-radius:0;background:none;padding:11px 0 14px;color:var(--muted);min-height:46px;border-bottom:2px solid transparent}.tabs button[aria-selected=true]{color:var(--accent);border-bottom-color:var(--accent)}.tab-count{padding:1px 5px;font-size:11px;background:#263950;border-radius:4px;margin-left:5px}.section-head>.tag{white-space:nowrap;flex-shrink:0}.section-head{display:flex;align-items:center;justify-content:space-between;gap:16px;margin:0 0 16px}.subline{margin-top:4px;color:var(--muted);font-size:12px}.metrics{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:12px;margin-bottom:20px}.metric{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:17px 16px;min-width:0}.metric .label{color:var(--muted);font-size:12px;display:flex;justify-content:space-between;align-items:center}.metric strong{font-size:30px;font-weight:600;letter-spacing:-.8px;display:block;line-height:1.3;margin:10px 0 6px;white-space:nowrap}.metric strong.compact{font-size:22px;padding:5px 0}.metric .unit{font-size:14px;color:var(--muted);font-weight:400;margin-left:3px}.metric .detail{font-size:11px;color:var(--muted);min-height:18px;overflow-wrap:anywhere}.metric .mark{color:var(--accent)}.dashboard-grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(290px,1fr);gap:18px;align-items:start}.stack{display:grid;gap:18px;min-width:0}.card{border:1px solid var(--line);border-radius:var(--radius);background:var(--panel);padding:20px;min-width:0}.card-head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:15px}.card-head small{font-size:11px}.row{display:flex;justify-content:space-between;gap:12px;align-items:center}.legend{font-size:11px;color:var(--muted);display:flex;gap:12px;align-items:center}.legend i{display:inline-block;width:7px;height:7px;border-radius:2px;background:var(--accent);margin-right:5px}.legend .orange{background:var(--orange)}.charts{display:grid;grid-template-columns:1fr 1fr;gap:18px}.chart{min-width:0}.chart-title{display:flex;justify-content:space-between;align-items:baseline;font-size:12px;margin-bottom:4px}.chart-title b{font-size:17px;font-weight:550}.chart svg{display:block;width:100%;height:148px;overflow:visible}.chart .axis{fill:var(--muted);font-size:10px}.chart .gridline{stroke:#2b3a50;stroke-dasharray:3 4}.chart .trend{fill:none;stroke:var(--accent);stroke-width:2;stroke-linejoin:round;stroke-linecap:round;vector-effect:non-scaling-stroke}.chart .area{fill:#80b5ff12}.chart .orange-line{stroke:var(--orange)}.chart-foot{font-size:11px;color:var(--muted);border-top:1px solid var(--line);padding-top:10px;margin-top:10px}.cluster{border-bottom:1px solid var(--line);padding:13px 0}.cluster:first-child{padding-top:0}.cluster:last-child{border-bottom:0;padding-bottom:0}.cluster-head{display:flex;justify-content:space-between;gap:10px;margin-bottom:11px}.cluster-head .freq{color:var(--accent);font-weight:650}.core-list{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px}.core{background:#0d1726;border:1px solid #293b52;border-radius:7px;padding:8px 9px}.core .row{font-size:11px}.bar{height:4px;border-radius:4px;background:#29394f;overflow:hidden;margin-top:8px}.bar span{display:block;background:var(--accent);height:100%;max-width:100%;transition:width .3s}.bar.green span{background:var(--green)}.resource{padding:13px 0;border-bottom:1px solid var(--line)}.resource:first-child{padding-top:0}.resource:last-child{border-bottom:0;padding-bottom:0}.resource p{font-size:12px;color:var(--muted);margin-top:5px}.resource .bar{margin:9px 0}.thermal-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.thermal{padding:10px 12px;background:#0d1726;border:1px solid #29394e;border-radius:8px}.thermal small{display:block;font-size:11px}.thermal b{display:block;font-size:19px;margin-top:4px;font-weight:550}.thermal.over{border-color:#ba7981;color:var(--red)}.engine-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}.engine{padding:16px;background:#0d1726;border:1px solid #293b52;border-radius:10px;min-width:0}.engine .freq{font-size:22px;margin:10px 0;color:var(--accent)}.kv{display:grid;grid-template-columns:1fr auto;gap:8px;font-size:12px}.kv dt{color:var(--muted)}.kv dd{margin:0;text-align:right;overflow-wrap:anywhere}.note{font-size:12px;color:var(--muted);padding:11px 13px;background:#0d1726;border-left:2px solid #526f92;border-radius:4px;line-height:1.7;margin-top:14px}.alert-banner{border:1px solid #946d45;background:#30261b;color:var(--orange);padding:11px 15px;margin-bottom:16px;border-radius:9px;font-size:13px}.alerts{display:grid;gap:8px}.alert-row{font-size:12px;padding:8px 12px;border-radius:6px;background:#263045;color:var(--muted)}.alert-row.warning{color:var(--orange);background:#32281e}.alert-row.danger{color:var(--red);background:#35232d}.empty{color:var(--muted);font-size:13px;padding:22px 8px;text-align:center}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:18px}.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.info-tile{padding:14px;background:#0d1726;border-radius:9px;border:1px solid #293b52}.info-tile strong{display:block;margin-top:8px}.info-tile small{font-size:12px}.control-buttons{display:flex;flex-wrap:wrap;gap:8px;margin:15px 0}.control-value{font-size:34px;font-weight:550;margin:12px 0 0}.control-value small{font-size:15px}.slider-row{display:flex;gap:12px;align-items:center;margin:18px 0 12px}.slider-row input[type=range]{width:100%;accent-color:var(--accent);height:24px;min-width:0}.numeric{width:78px;border:1px solid #64758b;border-radius:6px;padding:9px;background:var(--bg);color:var(--text)}.table-wrap{overflow:auto}table{width:100%;border-collapse:collapse;text-align:left;font-size:12px}th{color:var(--muted);font-weight:450;padding:11px 9px;border-bottom:1px solid var(--line);white-space:nowrap}td{padding:12px 9px;border-bottom:1px solid #29394e;vertical-align:top}tbody tr:last-child td{border-bottom:0}.control-help{font-size:12px;color:var(--muted);line-height:1.8}.source{font-family:ui-monospace,monospace;font-size:10px;color:var(--muted);overflow-wrap:anywhere;margin-top:10px}.log-tools{display:flex;gap:8px;flex-wrap:wrap}.log-console{height:310px;overflow:auto;background:#09111f;border:1px solid #293b52;border-radius:8px;padding:12px;margin-top:14px;font-size:12px;font-family:ui-monospace,SFMono-Regular,monospace}.log-line{padding:8px 0;border-bottom:1px solid #243246;overflow-wrap:anywhere}.log-line time{color:var(--muted);margin-right:10px}.log-line b{color:var(--accent);font-weight:500}.log-line code{color:var(--muted);display:block;font-size:11px;margin-top:4px}.tag{display:inline-block;padding:2px 7px;background:#23374e;border-radius:5px;color:var(--accent);font-size:11px}.details-list{display:grid;gap:10px}.device-row{padding:12px;background:#0d1726;border:1px solid #293b52;border-radius:8px;font-size:12px}.device-row .row{align-items:flex-start}.device-row p{margin-top:5px;color:var(--muted);overflow-wrap:anywhere}.footer{display:flex;justify-content:space-between;gap:16px;color:var(--muted);font-size:11px;margin-top:25px;padding-top:14px;border-top:1px solid var(--line)}.toast{position:fixed;right:24px;bottom:24px;max-width:min(460px,calc(100vw - 32px));background:#24374f;border:1px solid #86b6ff;color:var(--text);padding:14px 18px;border-radius:10px;box-shadow:0 12px 45px #0007;z-index:100}.toast.error{border-color:var(--red)}details{margin-top:14px}summary{cursor:pointer;color:var(--muted);font-size:12px;min-height:32px}#action-message{min-height:23px;margin-top:12px;color:var(--muted)}#action-message.error{color:var(--red)}.health-line{display:flex;align-items:center;gap:12px;flex-wrap:wrap}.running{color:var(--orange)}.skip-link{position:absolute;top:-70px;left:20px;background:var(--panel);padding:10px;z-index:200}.skip-link:focus{top:5px}
@media(min-width:1400px){.shell{padding:28px 38px 44px}.metric strong{font-size:34px}}
@media(max-width:1050px){.metrics{grid-template-columns:repeat(3,minmax(0,1fr))}.dashboard-grid{grid-template-columns:minmax(0,1.55fr) minmax(250px,1fr)}.engine-grid{grid-template-columns:1fr}.engine .freq{font-size:19px;margin:6px 0}.shell{padding:20px}.charts{gap:12px}}
@media(max-width:760px){.top{align-items:flex-start;flex-direction:column;gap:14px}.top-actions{justify-content:flex-start;width:100%}.top-actions .timestamp{margin-right:auto}.dashboard-grid,.grid2{grid-template-columns:1fr}.shell{padding:18px 15px 30px}.tabs{gap:19px;margin-bottom:20px}.metrics{gap:9px}.metric{padding:13px 11px}.metric strong{font-size:25px}.metric strong.compact{font-size:18px}.card{padding:16px}.engine-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.engine{padding:11px}.engine .kv{grid-template-columns:1fr;gap:4px}.engine .kv dd{text-align:left}.grid3{grid-template-columns:1fr}.footer{flex-direction:column;gap:5px}.thermal-grid{grid-template-columns:repeat(4,minmax(0,1fr))}button{min-height:42px}}
@media(max-width:440px){h1{font-size:20px}.metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.charts{grid-template-columns:1fr}.engine-grid{grid-template-columns:1fr}.engine .kv{grid-template-columns:1fr auto}.engine .kv dd{text-align:right}.thermal-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.core-list{grid-template-columns:repeat(2,minmax(0,1fr))}.tabs{gap:16px}.tabs button{font-size:13px}.card-head{align-items:flex-start;flex-wrap:wrap}.section-head{align-items:flex-start}.brand{gap:10px}.chip-logo{width:40px;height:40px}.timestamp{width:calc(100% - 95px)}.cluster-head{flex-wrap:wrap}}
@media(prefers-reduced-motion:reduce){*,*:before,*:after{transition:none!important;animation:none!important;scroll-behavior:auto!important}}
</style>
</head>
<body>
<a class="skip-link" href="#main-content">跳到监控内容</a>
<div class="shell">
<header class="top"><div class="brand"><div class="chip-logo" aria-hidden="true"><svg width="27" height="27" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><rect x="6" y="6" width="12" height="12" rx="2"/><rect x="9" y="9" width="6" height="6" rx="1"/><path d="M9 2v4m6-4v4M9 18v4m6-4v4M2 9h4m-4 6h4M18 9h4m-4 6h4"/></svg></div><div><div class="eyebrow">EDGE DEVICE / HARDWARE OBSERVABILITY</div><h1>RK3588 <span style="font-weight:400;color:var(--muted)">硬件监控台</span></h1></div></div><div class="top-actions"><span id="online-status" class="status warn" role="status">正在连接</span><span id="update-time" class="muted timestamp" style="font-size:11px">等待首次采样</span><button id="pause-btn" class="small" aria-pressed="false">暂停刷新</button><button id="refresh-btn" class="small">刷新</button><button id="export-btn" class="small">导出快照</button></div></header>
<nav class="tabs" role="tablist" aria-label="面板分区"><button role="tab" aria-selected="true" aria-controls="overview" id="tab-overview" data-tab="overview">设备总览</button><button role="tab" aria-selected="false" aria-controls="communication" id="tab-communication" data-tab="communication" tabindex="-1">BLE 通信<span class="tab-count" id="ble-count">0</span></button><button role="tab" aria-selected="false" aria-controls="controls" id="tab-controls" data-tab="controls" tabindex="-1">调试控制</button><button role="tab" aria-selected="false" aria-controls="peripherals" id="tab-peripherals" data-tab="peripherals" tabindex="-1">外设详情</button></nav>
<div id="connection-banner" class="alert-banner" role="status" hidden></div>
<main id="main-content">
<section id="overview" role="tabpanel" aria-labelledby="tab-overview">
<div class="section-head"><div><h2>设备总览</h2><p class="subline" id="device-subtitle">读取设备型号与内核版本…</p></div><span class="tag">真实采集 · 1 秒周期</span></div>
<div class="metrics">
<article class="metric"><div class="label">CPU 总占用 <span class="mark">CPU</span></div><strong id="metric-cpu">—</strong><div class="detail" id="metric-cpu-detail">等待差分采样</div></article>
<article class="metric"><div class="label">内存使用 <span class="mark">RAM</span></div><strong id="metric-memory">—</strong><div class="detail" id="metric-memory-detail">基于 MemAvailable</div></article>
<article class="metric"><div class="label">最高温区 <span class="mark">TEMP</span></div><strong id="metric-temp">—</strong><div class="detail" id="metric-temp-detail">等待温度节点</div></article>
<article class="metric"><div class="label">风扇 PWM <span class="mark">FAN</span></div><strong id="metric-fan">—</strong><div class="detail" id="metric-fan-detail">控制输出 ≠ 实测转速</div></article>
<article class="metric"><div class="label">根分区使用 <span class="mark">DISK</span></div><strong id="metric-disk">—</strong><div class="detail" id="metric-disk-detail">读取磁盘空间</div></article>
<article class="metric"><div class="label">BLE 数据通道 <span class="mark">BLE</span></div><strong class="compact" id="metric-ble">—</strong><div class="detail" id="metric-ble-detail">检查 BlueZ 状态</div></article>
</div>
<div class="dashboard-grid"><div class="stack">
<article class="card"><div class="card-head"><div><h2>运行趋势</h2><p class="subline">CPU · 温度 · 内存 · 风扇</p></div><select id="history-range" aria-label="历史时间范围"><option value="300">最近 5 分钟</option><option value="1800">最近 30 分钟</option></select></div><div class="charts">
<div class="chart"><div class="chart-title"><span>CPU 总占用</span><b id="chart-cpu-value">—</b></div><svg id="chart-cpu" viewBox="0 0 360 148" role="img" aria-label="CPU 占用历史曲线"></svg></div>
<div class="chart"><div class="chart-title"><span>最高温度</span><b id="chart-temperature-value" style="color:var(--orange)">—</b></div><svg id="chart-temperature" viewBox="0 0 360 148" role="img" aria-label="最高温度历史曲线"></svg></div>
<div class="chart"><div class="chart-title"><span>内存使用率</span><b id="chart-memory-value">—</b></div><svg id="chart-memory" viewBox="0 0 360 148" role="img" aria-label="内存使用历史曲线"></svg></div>
<div class="chart"><div class="chart-title"><span>风扇 PWM 输出</span><b id="chart-fan-value">—</b></div><svg id="chart-fan" viewBox="0 0 360 148" role="img" aria-label="风扇 PWM 历史曲线"></svg></div>
</div><div id="history-note" class="chart-foot">等待有效样本；缺失数据保留断点，不以 0 填充。</div></article>
<article class="card"><div class="card-head"><h2>CPU 集群</h2><small>4 × A55 + 2 × A76 + 2 × A76</small></div><div id="cpu-clusters"></div><div class="note">频率为驱动报告值；集群共用调频策略，单核分别统计占用。策略上限降低与热降频证据分开展示。</div></article>
<article class="card"><div class="card-head"><h2>异构计算引擎</h2><span class="tag">devfreq / rknpu</span></div><div id="engines" class="engine-grid"></div><details><summary>NPU 推理业务指标</summary><div class="note" id="inference-note">未接入推理应用：模型、推理耗时和 FPS 显示“未接入”，硬件规格不作为实测性能。</div></details></article>
</div><aside class="stack">
<article class="card"><div class="card-head"><h2>系统资源</h2><small id="uptime-small">—</small></div><div id="resources"></div></article>
<article class="card"><div class="card-head"><h2>芯片温区</h2><small id="thermal-count">— 个节点</small></div><div id="thermals" class="thermal-grid"></div><details><summary>温控阈值与降频证据</summary><div id="thermal-details" class="details-list"></div></details></article>
<article class="card"><div class="card-head"><h2>网络接口</h2><small>速率为采样差分</small></div><div id="network-summary" class="details-list"></div></article>
<article class="card"><div class="card-head"><h2>告警与事件</h2><span class="tag" id="alert-count">0 项</span></div><div id="alerts" class="alerts"></div><details><summary>最近事件</summary><div id="events" class="details-list"></div></details></article>
</aside></div>
</section>
<section id="communication" role="tabpanel" aria-labelledby="tab-communication" hidden>
<div class="section-head"><div><h2>BLE 通信与参数</h2><p class="subline">连接、订阅和协议活动独立呈现；遥测样本来源明确标注</p></div><span class="tag">FFE0 / FFE1</span></div>
<div class="grid2"><article class="card"><div class="card-head"><h2>链路状态</h2><span class="tag">BlueZ</span></div><div id="ble-state"></div><div class="note">当前遥测属于协议仿真：激光中位、零误差和 PWM 反馈不是外接传感器实测。启用通知不等于参数同步完成。</div></article><article class="card"><div class="card-head"><h2>收发与参数持久化</h2><small>本次服务启动以来</small></div><div id="ble-traffic"></div></article></div>
<article class="card" style="margin-top:18px"><div class="card-head"><h2>控制器参数</h2><span class="tag">配置值 · 非测量值</span></div><div id="ble-parameters"></div></article>
<article class="card" style="margin-top:18px"><div class="card-head"><div><h2>协议日志</h2><p class="subline">按递增 ID 接收，环形缓冲满后继续更新</p></div><div class="log-tools"><button id="follow-log" class="small" aria-pressed="true">跟随最新：开</button><button id="clear-log" class="small">清空当前视图</button><button id="export-log" class="small">导出日志</button></div></div><div id="log-note" class="subline">服务端保留最近 300 条；清空只影响当前视图。</div><div id="bantu-console" class="log-console" tabindex="0" aria-label="BLE 协议日志"><div class="empty">暂无协议日志，等待设备通信。</div></div></article>
</section>
<section id="controls" role="tabpanel" aria-labelledby="tab-controls" hidden>
<div class="section-head"><div><h2>调试控制</h2><p class="subline">控制结果以回读为准；数据过期时暂停操作</p></div><span id="control-health" class="status warn">等待设备</span></div>
<div class="grid2"><article class="card"><div class="card-head"><h2>风扇与温控</h2><span id="fan-mode-tag" class="tag">—</span></div><div class="grid2"><div><small>当前 PWM 输出</small><div id="fan-current" class="control-value">—</div></div><div><small>实测转速</small><div id="fan-rpm" class="control-value">—</div></div></div><div class="control-buttons"><button id="fan-auto" data-hardware="fan">自动温控</button><button id="fan-manual" data-hardware="fan">手动调节</button></div><label for="fan-slider">手动目标 PWM <span class="muted">（0–100%）</span></label><div class="slider-row"><input id="fan-slider" type="range" min="0" max="100" value="0" data-hardware="fan" aria-describedby="fan-explanation"><input id="fan-input" class="numeric" type="number" min="0" max="100" value="0" data-hardware="fan" aria-label="手动目标 PWM 百分比"><button id="fan-apply" class="primary" data-hardware="fan">应用</button></div><div class="control-buttons"><button data-fan-pct="0" data-hardware="fan">0%</button><button data-fan-pct="30" data-hardware="fan">30%</button><button data-fan-pct="60" data-hardware="fan">60%</button><button data-fan-pct="100" data-hardware="fan">100%</button></div><div id="fan-explanation" class="control-help">读取当前控制来源与温控阈值…</div><details><summary>自动温控曲线</summary><div id="fan-curve"></div></details></article>
<article class="card"><div class="card-head"><h2>CPU 调频策略</h2><span class="tag">按集群回读</span></div><div id="governor-state"></div><div id="governor-buttons" class="control-buttons"></div><div class="note">仅列出所有集群共同支持的策略。performance 请求策略上限，不承诺固定频率。</div><hr style="border:0;border-top:1px solid var(--line);margin:22px 0"><h2>板载工作指示灯</h2><p class="subline" id="led-state">读取状态…</p><div class="control-buttons"><button data-led="on" data-hardware="led">常亮</button><button data-led="off" data-hardware="led">熄灭</button><button data-led="heartbeat" data-hardware="led">心跳</button></div></article></div>
<article class="card" style="margin-top:18px"><div class="card-head"><div><h2>限时 CPU 压测</h2><p class="subline">启动前保存策略；完成、停止和服务正常退出时恢复原调频与风扇模式</p></div><span id="stress-state" class="tag">未运行</span></div><div id="stress-max" class="control-help"></div><div class="control-buttons"><button data-stress="30" data-hardware="stress" class="danger">运行 30 秒</button><button data-stress="60" data-hardware="stress" class="danger">运行 60 秒</button><button id="stress-stop" data-hardware="stop" class="primary">停止并恢复</button></div><p class="control-help">启动条件：SoC 温度有效且低于 60°C；压测期间保留过热满速接管，并锁定风扇与调频编辑。</p></article>
<p id="action-message" role="status">等待操作。</p>
</section>
<section id="peripherals" role="tabpanel" aria-labelledby="tab-peripherals" hidden>
<div class="section-head"><div><h2>外设与接口</h2><p class="subline" id="peripheral-time">仅枚举现有节点，不扫描总线、不改变设备状态</p></div><span class="tag">10 秒更新</span></div>
<div class="grid2"><article class="card"><div class="card-head"><h2>显示输出</h2><small>DRM connectors</small></div><div id="displays" class="details-list"></div></article><article class="card"><div class="card-head"><h2>USB 设备</h2><small>已枚举设备</small></div><div id="usb-devices" class="details-list"></div></article><article class="card"><div class="card-head"><h2>串口与 I²C</h2><small>节点存在 ≠ 外设已通信</small></div><div id="serial-i2c"></div></article><article class="card"><div class="card-head"><h2>网络详情</h2><small>IPv4 / IPv6 / 链路</small></div><div id="network-details" class="details-list"></div></article></div>
</section>
</main>
<footer class="footer"><span>RK3588 Monitor <span class="tag">v2.0</span> · 正点原子 ATK-DLRK3588</span><span id="footer-source">procfs / sysfs / BlueZ · 原始数据保留来源</span></footer>
</div><div id="toast" class="toast" role="status" hidden></div>
<script>
'use strict';
const TOKEN='__CONTROL_TOKEN__';
const $=id=>document.getElementById(id);
const esc=v=>String(v??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const valid=n=>typeof n==='number'&&Number.isFinite(n);
const fmt=(n,d=1)=>valid(n)?n.toFixed(d).replace(/\.0$/,''):'—';
const unit=(n,u,d=1)=>valid(n)?`${fmt(n,d)}${u}`:'未采集';
const bytes=n=>{if(!valid(n))return '未采集';const units=['B','KiB','MiB','GiB','TiB'];let i=0;while(n>=1024&&i<4){n/=1024;i++}return `${fmt(n,1)} ${units[i]}`};
const rateText=n=>valid(n)?bytes(n)+'/s':'等待采样';
const timeText=n=>valid(n)?new Date(n*1000).toLocaleTimeString('zh-CN',{hour12:false}):'暂无';
const ageText=n=>valid(n)?`${fmt(n,1)} 秒前`:'暂无';
const kv=entries=>`<dl class="kv">${entries.map(([a,b])=>`<dt>${esc(a)}</dt><dd>${esc(b)}</dd>`).join('')}</dl>`;
const setText=(id,t)=>{if($(id).textContent!==String(t))$(id).textContent=t};
const setHTML=(id,h)=>{if($(id)._html!==h){$(id).innerHTML=h;$(id)._html=h}};
let latest=null, session='', logCursor=0, logs=[], points=[], range=300, paused=false, inflight=false, pollTimer=null, lastReceived=0, lastFetchLatency=0, actionBusy=false, fanDirty=false, activeTab='overview', followLog=true, historyReady=false, historyError='', historyBusy=false, toastTimer=null;
function metric(id,n,u,d=1){setHTML(id,valid(n)?`${fmt(n,d)}<span class="unit">${esc(u)}</span>`:'—')}
function notify(message,error=false){setText('action-message',message);$('action-message').classList.toggle('error',error);setText('toast',message);$('toast').classList.toggle('error',error);$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,6000)}
function activateTab(name){activeTab=name;document.querySelectorAll('[role=tab]').forEach(b=>{const selected=b.dataset.tab===name;b.setAttribute('aria-selected',String(selected));b.tabIndex=selected?0:-1;$(b.getAttribute('aria-controls')).hidden=!selected});if(latest)render(latest);if(name==='overview')drawCharts()}
document.querySelectorAll('[role=tab]').forEach((b,index)=>{b.addEventListener('click',()=>activateTab(b.dataset.tab));b.addEventListener('keydown',e=>{const bs=[...document.querySelectorAll('[role=tab]')];let next;if(e.key==='ArrowRight')next=(index+1)%bs.length;else if(e.key==='ArrowLeft')next=(index+bs.length-1)%bs.length;else if(e.key==='Home')next=0;else if(e.key==='End')next=bs.length-1;else return;e.preventDefault();bs[next].focus();activateTab(bs[next].dataset.tab)})});
function isFresh(){return !!latest&&!latest.meta.stale&&Date.now()/1000-latest.meta.sampled_at<4&&Date.now()-lastReceived<5000&&!paused}
function updateHealth(){const fresh=isFresh(),age=latest?Math.max(0,Date.now()/1000-latest.meta.sampled_at):null;const label=paused?'刷新已暂停':fresh?'设备在线':latest?'数据已过期':'连接中';setText('online-status',label);$('online-status').className='status '+(fresh?'':paused?'warn':'error');setText('control-health',fresh?'控制已就绪':'等待新鲜数据');$('control-health').className='status '+(fresh?'':'warn');setText('update-time',latest?`${timeText(latest.meta.sampled_at)} · ${fmt(age,0)} 秒前`:'等待首次采样');const banner=$('connection-banner');if(paused){banner.hidden=false;banner.textContent='自动刷新已暂停，当前是保留快照。点击“恢复刷新”继续。'}else if(!fresh&&latest){banner.hidden=false;banner.textContent='数据已过期：读数为最后有效快照，控制已暂停。正在重试，也可点击“刷新”。'}else if(fresh){banner.hidden=true}const running=!!latest?.soc?.stress?.running;document.querySelectorAll('[data-hardware]').forEach(b=>{const kind=b.dataset.hardware;b.disabled=!fresh||actionBusy||(running&&['fan','governor','stress'].includes(kind))||(kind==='stop'&&!running)||(kind==='fan'&&!latest?.fan?.exists)})}
function render(d){renderHeader(d);appendLogs(d.bantu||{});if(activeTab==='overview')renderOverview(d);if(activeTab==='communication')renderBLE(d);if(activeTab==='controls')renderControls(d);if(activeTab==='peripherals')renderPeripherals(d);updateHealth()}
function renderHeader(d){const s=d.soc||{};setText('device-subtitle',`${s.model||'未知型号'} · ${s.kernel||'内核未采集'} · ${s.arch||'—'}`);setText('ble-count',d.bantu?.rx_packets??0);setText('footer-source',`采集 ${fmt(d.meta.collection_ms)} ms · 请求 ${fmt(lastFetchLatency,0)} ms · ${Object.keys(d.meta.errors||{}).length} 项采集错误`)}
function renderOverview(d){const s=d.soc||{},m=d.memory||{},f=d.fan||{},b=d.bantu||{},cores=d.cpus?.cores||[],temps=(d.thermals||[]).filter(t=>valid(t.temp));const hottest=temps.reduce((a,t)=>!a||t.temp>a.temp?t:a,null),root=(d.storage?.volumes||[]).find(v=>v.mount==='/');metric('metric-cpu',s.total_cpu_usage,'%');metric('metric-memory',m.used_percent,'%');metric('metric-temp',hottest?.temp,'°C');metric('metric-fan',f.percent,'%');metric('metric-disk',root?.used_percent,'%');setText('metric-cpu-detail',`${cores.filter(c=>c.online).length} 核在线 · 1 分钟负载 ${fmt(s.loadavg?.[0],2)}`);setText('metric-memory-detail',`${bytes(m.used_bytes)} / ${bytes(m.total_bytes)}`);setText('metric-temp-detail',hottest?`${hottest.name} · ${temps.length} 个有效温区`:'温度未采集');setText('metric-fan-detail',`${f.mode==='auto'?'自动温控':'手动控制'} · ${valid(f.rpm)?unit(f.rpm,' RPM',0):'未接转速反馈'}`);setText('metric-disk-detail',root?`可用 ${bytes(root.available_bytes)}`:'根分区未采集');setText('metric-ble',b.notify_subscribed?'通知已订阅':b.connected?'已连接':b.advertisement_registered?'广播已注册':'未就绪');setText('metric-ble-detail',`${b.advertised_name||'名称未采集'} · RX ${b.rx_packets??0}`);renderClusters(d.cpus||{});renderThermals(d);renderEngines(d.hetero||{});renderResources(d);renderNetwork(d.network||{});renderAlerts(d);drawCharts()}
function renderClusters(data){const clusters=data.clusters||[],cores=data.cores||[];const signature=JSON.stringify(clusters.map(c=>[c.id,c.core_ids]));if($('cpu-clusters')._signature!==signature){$('cpu-clusters')._signature=signature;setHTML('cpu-clusters',clusters.length?clusters.map(c=>`<div class="cluster"><div class="cluster-head"><div><h3>${esc(c.name)}</h3><small id="cluster-${esc(c.id)}-meta"></small></div><div class="freq" id="cluster-${esc(c.id)}-freq"></div></div><div class="core-list">${c.core_ids.map(i=>`<div class="core"><div class="row"><span>CPU ${i}</span><b id="core-${i}-usage">—</b></div><div class="bar"><span id="core-${i}-bar" style="width:0"></span></div></div>`).join('')}</div></div>`).join(''):'<div class="empty">CPU 集群节点未采集</div>')}
clusters.forEach(c=>{setText(`cluster-${c.id}-freq`,unit(c.freq_mhz,' MHz',0));setText(`cluster-${c.id}-meta`,`${c.governor||'策略未采集'} · 上限 ${unit(c.max_mhz,' MHz',0)}${c.policy_limited?' · 策略受限':''}`)});cores.forEach(c=>{if($(`core-${c.id}-usage`)){setText(`core-${c.id}-usage`,c.online?unit(c.usage,'%',1):'离线');$(`core-${c.id}-bar`).style.width=(valid(c.usage)?c.usage:0)+'%'}})}
function renderThermals(d){const zones=d.thermals||[];setText('thermal-count',zones.length+' 个节点');const signature=zones.map(t=>t.zone).join(',');if($('thermals')._signature!==signature){$('thermals')._signature=signature;setHTML('thermals',zones.map(t=>`<div class="thermal" id="temp-${esc(t.zone)}"><small>${esc(t.name)}</small><b>—</b></div>`).join('')||'<div class="empty">温度未采集</div>')}
zones.forEach(t=>{const el=$('temp-'+t.zone);el.querySelector('b').textContent=unit(t.temp,' °C');const limits=(t.trips||[]).map(p=>p.temp).filter(valid);el.classList.toggle('over',valid(t.temp)&&limits.length>0&&t.temp>=Math.min(...limits))});setHTML('thermal-details',zones.map(t=>`<div class="device-row"><b>${esc(t.name)}</b><p>${(t.trips||[]).map(p=>`${esc(p.type)} ${esc(unit(p.temp,'°C'))}`).join(' · ')||'驱动未提供阈值'}</p></div>`).join('')+`<div class="note">${(d.cooling||[]).map(c=>`${esc(c.name)}：${esc(fmt(c.state,0))}/${esc(fmt(c.max_state,0))}`).join('<br>')}<br>CPU/GPU cooling state &gt; 0 才作为热限制证据；低频本身不等于过热降频。</div>`)}
function renderEngines(engines){setHTML('engines',['gpu','npu','dmc'].map(k=>{const e=engines[k]||{};let fields=[['实时负载',unit(e.load_percent,'%')],['调度策略',e.governor||'未采集']];if(k==='npu')fields=[['三核负载',(e.core_loads||[]).map(n=>unit(n,'%')).join(' / ')||'未采集'],['驱动',e.driver_status||'未采集']];return `<div class="engine"><h3>${esc(e.name||k.toUpperCase())}</h3><div class="freq">${esc(unit(e.freq_mhz,' MHz',0))}</div>${kv(fields)}<div class="source">${esc(e.source||'节点未发现')}</div></div>`}).join(''))}
function renderResources(d){const m=d.memory||{},v=d.storage?.volumes||[],devs=d.storage?.devices||[];setText('uptime-small',d.soc?.uptime||'运行时间未采集');setHTML('resources',`<div class="resource"><div class="row"><b>系统负载</b><small>1 / 5 / 15 分钟</small></div><p>${(d.soc?.loadavg||[]).map(n=>fmt(n,2)).join(' / ')}</p></div><div class="resource"><div class="row"><b>内存</b><span>${esc(bytes(m.available_bytes))} 可用</span></div><div class="bar green"><span style="width:${valid(m.used_percent)?m.used_percent:0}%"></span></div><p>Swap ${esc(bytes(m.swap_used_bytes))} / ${esc(bytes(m.swap_total_bytes))}</p></div>`+v.map(x=>`<div class="resource"><div class="row"><b>${esc(x.mount)}</b><span>${esc(unit(x.used_percent,'%'))}</span></div><div class="bar"><span style="width:${valid(x.used_percent)?x.used_percent:0}%"></span></div><p>${esc(bytes(x.used_bytes))} 已用 / ${esc(bytes(x.total_bytes))} · ${esc(x.filesystem)}</p></div>`).join('')+devs.map(x=>`<div class="resource"><b>${esc(x.name)} I/O</b><p>读取 ${esc(rateText(x.read_bps))}<br>写入 ${esc(rateText(x.write_bps))}</p></div>`).join(''))}
function renderNetwork(n){const items=n.interfaces||[];setHTML('network-summary',items.map(i=>`<div class="device-row"><div class="row"><b>${esc(i.name)}</b><span class="tag">${i.state==='up'?'链路在线':esc(i.state)}</span></div><p>↓ ${esc(rateText(i.rx_bps))} &nbsp; ↑ ${esc(rateText(i.tx_bps))}</p></div>`).join('')||'<div class="empty">网络接口未采集</div>')}
function renderAlerts(d){const a=d.alerts||[];setText('alert-count',a.length+' 项');setHTML('alerts',a.map(x=>`<div class="alert-row ${esc(x.level)}">${esc(x.message)}</div>`).join('')||'<div class="empty">当前采样未触发告警</div>');setHTML('events',(d.events||[]).slice().reverse().map(x=>`<div class="device-row"><small>${timeText(x.time)}</small><p>${esc(x.message)}</p></div>`).join('')||'<div class="empty">暂无事件</div>')}
function renderBLE(d){const b=d.bantu||{},s=b.settings||{};setHTML('ble-state',kv([['适配器',(b.adapters||[]).map(a=>`${a.name} · ${a.powered?'已开启':'已关闭'}`).join(', ')||'未采集'],['广播注册',b.advertisement_registered?'已注册（不代表射频实测）':'未注册'],['广播名称',b.advertised_name||'未采集'],['GATT 服务',b.gatt_registered?'已注册':'未注册'],['设备连接',b.connected?'已连接':'未连接'],['通知订阅',b.notify_subscribed?'已订阅':'未订阅'],['遥测来源','协议仿真'],['状态异常',b.registration_error||b.error||'无']]))
;setHTML('ble-traffic',kv([['RX 写入次数 / 字节',`${b.rx_packets??0} / ${b.rx_bytes??0}`],['TX 通知次数 / 字节',`${b.tx_packets??0} / ${b.tx_bytes??0}`],['最近接收',`${timeText(b.last_rx_at)} · ${ageText(b.rx_age_seconds)}`],['最近发送',`${timeText(b.last_tx_at)} · ${ageText(b.tx_age_seconds)}`],['接收 / 发送间隔',`${unit(b.rx_interval_ms,' ms')} / ${unit(b.tx_interval_ms,' ms')}`],['参数查询 / 回复完成',`${timeText(b.last_parameter_query_at)} / ${timeText(b.last_parameter_reply_at)}`],['参数持久化',b.settings_save_error?'失败：'+b.settings_save_error:b.settings_saved_at?'最近保存 '+timeText(b.settings_saved_at):'本次运行暂无保存'],['端到端 ACK','协议未提供，未推断为同步成功']]))
;const array=(k,i)=>Array.isArray(s[k])?s[k][i]:null;const mins=[s.left_min_pwm,s.left_down_min_pwm,s.right_min_pwm,s.right_down_min_pwm];setHTML('ble-parameters',`<div class="grid3"><div class="info-tile"><small>工作模式 / 零偏</small><strong>${s.mode===1?'自动平地':'手动操作'} · ${esc(s.left_zero_offset)} / ${esc(s.right_zero_offset)}</strong></div><div class="info-tile"><small>激光 / 水平死区</small><strong>${esc(s.laser_deadzone)} / ${esc(s.level_deadzone)}</strong></div><div class="info-tile"><small>PWM 频率 / 微分增益</small><strong>${esc(s.pwm_frequency)} Hz · Kd ${esc(s.derivative_gain)}</strong></div></div><div class="table-wrap"><table><thead><tr><th>通道</th><th>Kp</th><th>最小 PWM</th><th>输出上限</th><th>曲线</th><th>S 曲线强度</th></tr></thead><tbody>${[0,1,2,3].map(i=>`<tr><td>CH${i+1} · ${['左上','左下','右上','右下'][i]}</td><td>${esc(fmt(valid(array('kp_x100',i))?array('kp_x100',i)/100:null,2))}</td><td>${esc(mins[i])}</td><td>${esc(unit(array('channel_max_percent',i),'%'))}</td><td>${valid(s.channel_curve_mask)?(s.channel_curve_mask&(1<<i)?'S 曲线':'线性'):'未采集'}</td><td>${esc(array('s_curve_strength',i))}</td></tr>`).join('')}</tbody></table></div><div class="note">斜坡 ${esc(s.ramp_time_ms)} ms · 失联超时 ${esc(s.app_timeout_ms)} ms · 阀类型编码 ${esc(s.valve_type)} · 引脚映射 ${esc(s.pin_map_code)} · 空闲编码 ${esc(s.pin_idle_code)}</div>`)}
function appendLogs(b){if(b.session_id&&session&&b.session_id!==session){logs=[];logCursor=0;points=[];historyReady=false}if(b.session_id)session=b.session_id;const incoming=(b.history||[]).filter(x=>x.id>logCursor);if(incoming.length){if(logCursor&&incoming[0].id>logCursor+1)setText('log-note','采样期间部分旧日志已被环形缓冲覆盖；已从最新可用 ID 继续。');logs.push(...incoming);logs=logs.slice(-300);logCursor=Math.max(logCursor,...incoming.map(x=>x.id));renderLogs()}logCursor=Math.max(logCursor,b.last_log_id||0)}
function renderLogs(){const box=$('bantu-console'),oldTop=box.scrollTop;setHTML('bantu-console',logs.length?logs.map(x=>`<div class="log-line"><time>${esc(x.time)}</time><b>#${x.id} ${esc(x.tag)}</b><div>${esc(x.text)}</div>${x.hex?`<code>${esc(x.hex)}</code>`:''}</div>`).join(''):'<div class="empty">当前视图为空，等待新的协议日志。</div>');box.scrollTop=followLog?box.scrollHeight:oldTop}
$('clear-log').onclick=()=>{logCursor=Math.max(logCursor,latest?.bantu?.last_log_id||0);logs=[];renderLogs();setText('log-note',`当前视图已清空，从 ID ${logCursor+1} 继续；不删除服务端日志。`)};
$('follow-log').onclick=()=>{followLog=!followLog;$('follow-log').setAttribute('aria-pressed',String(followLog));setText('follow-log','跟随最新：'+(followLog?'开':'关'));if(followLog)$('bantu-console').scrollTop=$('bantu-console').scrollHeight};
function renderControls(d){const f=d.fan||{},s=d.soc?.stress||{},clusters=d.cpus?.clusters||[];setText('fan-mode-tag',`${f.mode==='auto'?'自动温控':'手动'} · ${f.controller||'来源未采集'}`);setHTML('fan-current',valid(f.percent)?`${fmt(f.percent,0)} <small>%</small>`:'未采集');setHTML('fan-rpm',valid(f.rpm)?`${fmt(f.rpm,0)} <small>RPM</small>`:'<small>未接转速反馈</small>');$('fan-auto').setAttribute('aria-pressed',String(f.mode==='auto'));$('fan-manual').setAttribute('aria-pressed',String(f.mode==='manual'));if(!fanDirty&&document.activeElement!==$('fan-slider')&&document.activeElement!==$('fan-input')){const v=f.mode==='manual'?f.requested_percent:f.percent;$('fan-slider').value=valid(v)?v:0;$('fan-input').value=valid(v)?v:0}setText('fan-explanation',`模式：${f.mode==='auto'?'自动温控':'手动'}；控制来源：${f.controller||'未采集'}；SoC ${unit(valid(f.control_temperature)?f.control_temperature/1000:null,'°C')}。${f.overheat?'当前由满速策略接管。':''}60°C 触发满速，55°C 以下解除；温度读取异常时满速。`);setHTML('fan-curve',`<div class="table-wrap"><table><thead><tr><th>温度 ≥</th><th>PWM 输出</th></tr></thead><tbody>${(f.curve||[]).map(p=>`<tr><td>${esc(unit(p.temp_c,'°C'))}</td><td>${esc(unit(p.pwm_percent,'%'))}</td></tr>`).join('')}</tbody></table></div><p class="control-help">温度低于首个阈值时目标为 0%；满速接管优先于曲线。</p>`);setHTML('governor-state',kv(clusters.map(c=>[c.name,`${c.governor||'未采集'} · 上限 ${unit(c.max_mhz,' MHz',0)}`])));const common=clusters.length?(clusters[0].available_governors||[]).filter(g=>clusters.every(c=>(c.available_governors||[]).includes(g))):[];const preferred=['schedutil','performance','powersave'];const options=preferred.filter(g=>common.includes(g));const optionsKey=options.join(',');if($('governor-buttons')._options!==optionsKey){$('governor-buttons')._options=optionsKey;setHTML('governor-buttons',options.map(g=>`<button data-governor="${esc(g)}" data-hardware="governor">${{schedutil:'动态调度',performance:'性能优先',powersave:'低频优先'}[g]}</button>`).join('')||'<small>暂无可用策略</small>');document.querySelectorAll('[data-governor]').forEach(b=>b.onclick=()=>command('/api/governor?gov='+encodeURIComponent(b.dataset.governor),'调频策略'))}document.querySelectorAll('[data-governor]').forEach(b=>b.setAttribute('aria-pressed',String(clusters.length>0&&clusters.every(c=>c.governor===b.dataset.governor))));const led=(d.io?.leds||[]).find(x=>x.name==='work');const lm=led?.trigger==='heartbeat'?'heartbeat':led?.is_on?'on':'off';setText('led-state',led?`work · ${lm==='heartbeat'?'心跳':lm==='on'?'常亮':'熄灭'}（读取 trigger 配置）`:'LED 节点未采集');document.querySelectorAll('[data-led]').forEach(b=>b.setAttribute('aria-pressed',String(!!led&&b.dataset.led===lm)));setText('stress-state',s.running?`运行中 ${s.elapsed} / ${s.duration} 秒`:'未运行');$('stress-state').classList.toggle('running',!!s.running);setText('stress-max',clusters.map(c=>`${c.name}：当前策略上限 ${unit(c.max_mhz,' MHz',0)}`).join(' · '));}
function renderPeripherals(d){const p=d.peripherals||{};setText('peripheral-time',`最近枚举 ${timeText(p.sampled_at)} · 只读枚举，不扫描总线、不改变设备状态`);setHTML('displays',(p.displays||[]).map(x=>`<div class="device-row"><div class="row"><b>${esc(x.name)}</b><span class="tag">${esc(x.status)}</span></div><p>${esc(x.enabled)} · 可用模式：${esc(x.modes?.join(' / ')||'未提供')}</p></div>`).join('')||'<div class="empty">未发现 DRM 连接器节点</div>');setHTML('usb-devices',(p.usb||[]).map(x=>`<div class="device-row"><b>${esc(x.name)}</b><p>${esc(x.path)} · ${esc(x.vendor)}:${esc(x.product)} · 链路 ${esc(unit(x.speed_mbps,' Mbps'))}</p></div>`).join('')||'<div class="empty">暂无 USB 设备</div>');setHTML('serial-i2c',`<h3>串口节点</h3><p class="note">${(p.serial||[]).map(esc).join('<br>')||'未发现'}</p><h3 style="margin-top:16px">I²C 总线节点</h3><p class="note">${(p.i2c||[]).map(esc).join(' · ')||'未发现'}</p>`);setHTML('network-details',(d.network?.interfaces||[]).map(x=>`<div class="device-row"><b>${esc(x.name)} · ${esc(x.state)}</b><p>${(x.addresses||[]).map(esc).join('<br>')||'暂无全局地址'}</p><p>协商速率 ${esc(unit(x.speed_mbps,' Mbps'))} · RX/TX 错误 ${esc(fmt(x.rx_errors,0))}/${esc(fmt(x.tx_errors,0))}</p></div>`).join('')||'<div class="empty">网络未采集</div>')}
function addPoint(d){const ts=d.meta.sampled_at;if(points.some(p=>p.time===ts))return;const temps=(d.thermals||[]).map(t=>t.temp).filter(valid);points.push({time:ts,cpu:d.soc?.total_cpu_usage??null,memory:d.memory?.used_percent??null,temperature:temps.length?Math.max(...temps):null,fan:d.fan?.percent??null});points=points.filter(p=>p.time>=Date.now()/1000-1800).sort((a,b)=>a.time-b.time)}
function drawCharts(){if(activeTab!=='overview')return;const end=Date.now()/1000,start=end-range,ps=points.filter(p=>p.time>=start);['cpu','temperature','memory','fan'].forEach(key=>{const yMax=key==='temperature'?Math.max(60,...ps.map(p=>valid(p[key])?Math.ceil(p[key]/20)*20:0)):100;const vals=ps.filter(p=>valid(p[key]));const current=vals.length?vals[vals.length-1][key]:null;setText(`chart-${key}-value`,unit(current,key==='temperature'?' °C':' %'));let paths=[],segment=[];let previous=null;const x=t=>35+(t-start)/range*313,y=v=>117-v/yMax*95;for(const p of ps){if(!valid(p[key])||(previous&&p.time-previous>3)){if(segment.length)paths.push(segment);segment=[]}if(valid(p[key]))segment.push([x(p.time),y(p[key])]);previous=p.time}if(segment.length)paths.push(segment);let html=[0,yMax/2,yMax].map(n=>`<line class="gridline" x1="35" y1="${y(n)}" x2="348" y2="${y(n)}"/><text class="axis" x="27" y="${y(n)+3}" text-anchor="end">${n}</text>`).join('');html+=`<text class="axis" x="35" y="140">${Math.round(range/60)} 分钟前</text><text class="axis" x="348" y="140" text-anchor="end">现在</text>`;html+=paths.map(seg=>seg.length===1?`<circle cx="${seg[0][0]}" cy="${seg[0][1]}" r="2.5" fill="${key==='temperature'?'#ffc184':'#80b5ff'}"/>`:`<path class="trend ${key==='temperature'?'orange-line':''}" d="${seg.map((p,i)=>(i?'L':'M')+p.map(v=>v.toFixed(2)).join(' ')).join(' ')}"/>`).join('');if(!vals.length)html+='<text class="axis" x="190" y="70" text-anchor="middle">等待有效采样</text>';setHTML('chart-'+key,html)});const span=ps.length>1?ps[ps.length-1].time-ps[0].time:0;setText('history-note',historyError||`已积累 ${fmt(span/60,1)} 分钟 / ${ps.length} 个样本 · 服务内存保留 30 分钟，服务重启后重新积累 · 缺失段不补 0`)}
async function request(url,opts={},timeout=4500){const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),timeout);try{const r=await fetch(url,{...opts,signal:ctrl.signal,cache:'no-store'});let data;try{data=await r.json()}catch{throw new Error('响应格式异常')}if(!r.ok)throw new Error(data.error||`HTTP ${r.status}`);return data}finally{clearTimeout(timer)}}
async function loadHistory(){if(historyBusy)return;historyBusy=true;try{const h=await request('/api/history?seconds='+range);if(session&&h.session_id!==session)return;const map=new Map([...h.points,...points].map(p=>[p.time,p]));points=[...map.values()].sort((a,b)=>a.time-b.time).filter(p=>p.time>=Date.now()/1000-1800);historyReady=true;historyError='';drawCharts()}catch(e){historyError='历史数据读取失败：'+e.message+'；继续显示当前实时样本。';drawCharts()}finally{historyBusy=false}}
async function poll(manual=false){clearTimeout(pollTimer);if(inflight)return;if((paused||document.hidden)&&!manual)return;inflight=true;const t=performance.now();try{const d=await request('/api/all?since='+logCursor+'&session='+encodeURIComponent(session));if(!d.meta||d.meta.schema_version!==2)throw new Error('面板与服务版本不一致，请刷新');if(session&&session!==d.meta.session_id){points=[];logs=[];logCursor=0;historyReady=false;renderLogs()}session=d.meta.session_id;latest=d;lastReceived=Date.now();lastFetchLatency=performance.now()-t;addPoint(d);render(d);if(!historyReady)loadHistory()}catch(e){$('connection-banner').hidden=false;setText('connection-banner','读取失败：'+(e.name==='AbortError'?'请求超时':e.message)+'。保留最后快照，自动重试。');lastReceived=0;updateHealth()}finally{inflight=false;if(!paused&&!document.hidden)pollTimer=setTimeout(poll,1000)}}
async function command(url,label){if(!isFresh()||actionBusy)return;actionBusy=true;updateHealth();setText('action-message',label+'执行中…');try{const result=await request(url,{method:'POST',headers:{'X-Dashboard-Token':TOKEN}},6500);if(result.status!=='ok')throw new Error(result.error||'设备未确认');fanDirty=false;for(const k of ['fan','cpus','soc','io'])if(result[k])latest[k]=result[k];render(latest);notify(label+'已执行并回读');clearTimeout(pollTimer);pollTimer=setTimeout(()=>poll(true),1000)}catch(e){notify(label+'失败：'+e.message,true)}finally{actionBusy=false;updateHealth()}}
$('fan-slider').oninput=()=>{fanDirty=true;$('fan-input').value=$('fan-slider').value};$('fan-input').oninput=()=>{fanDirty=true;if($('fan-input').validity.valid)$('fan-slider').value=$('fan-input').value};$('fan-apply').onclick=()=>{if(!$('fan-input').reportValidity()||$('fan-input').value===''){notify('请输入 0–100 的 PWM 百分比',true);return}command('/api/fan?mode=manual&pct='+encodeURIComponent($('fan-input').value),'风扇目标')};$('fan-auto').onclick=()=>command('/api/fan?mode=auto','自动温控');$('fan-manual').onclick=()=>command('/api/fan?mode=manual&pct='+encodeURIComponent(latest?.fan?.percent??0),'手动模式');document.querySelectorAll('[data-fan-pct]').forEach(b=>b.onclick=()=>command('/api/fan?mode=manual&pct='+b.dataset.fanPct,'风扇目标'));document.querySelectorAll('[data-led]').forEach(b=>b.onclick=()=>command('/api/led?mode='+b.dataset.led,'LED 状态'));document.querySelectorAll('[data-stress]').forEach(b=>b.onclick=()=>command('/api/stress?action=start&sec='+b.dataset.stress,'限时压测'));$('stress-stop').onclick=()=>command('/api/stress?action=stop','停止并恢复');
$('history-range').onchange=()=>{range=Number($('history-range').value);drawCharts();loadHistory()};$('pause-btn').onclick=()=>{paused=!paused;$('pause-btn').setAttribute('aria-pressed',String(paused));setText('pause-btn',paused?'恢复刷新':'暂停刷新');clearTimeout(pollTimer);updateHealth();if(!paused){historyReady=false;poll(true)}};$('refresh-btn').onclick=()=>poll(true);document.addEventListener('visibilitychange',()=>{clearTimeout(pollTimer);if(!document.hidden&&!paused){historyReady=false;poll(true)}});
function download(name,data){const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),5000)}$('export-btn').onclick=()=>{if(latest)download('rk3588-snapshot-'+Date.now()+'.json',latest);else notify('等待首次采样后再导出',true)};$('export-log').onclick=()=>download('rk3588-ble-log-'+Date.now()+'.json',{session_id:session,logs});
setInterval(updateHealth,1000);poll();
</script>
</body></html>
"""

if __name__ == "__main__":
    main()
