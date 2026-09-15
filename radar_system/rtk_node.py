#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 CUAV C-RTK 2HP (Unicore UM982) 高精度卫星定位定向 ROS2 节点
- 接口: /dev/ttyUSB0 (默认 921600 波特率)
- 自动唤醒与配置和芯星通 UM982 (MODE ROVER, NMEA + BESTPOSA + HEADINGA)
- 集成网络 CORS / NTRIP 客户端，直灌 RTCM3 差分数据包，实现厘米级 RTK 固定解
- 实时发布:
    1. /rtk/status (std_msgs/msg/String, JSON 格式，包含搜星数、双天线状态、CORS连接态、经纬度等)
    2. /gps/fix (sensor_msgs/msg/NavSatFix, ROS 2 标准卫星导航消息)
"""

import os
import sys
import time
import json
import math
import socket
import base64
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus

PORT = "/dev/ttyUSB0"
BAUD = 921600
CORS_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cors_config.json")

def safe_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default

def safe_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default

FIX_TYPE_MAP = {
    0: "未定位 (无天线/搜星中)",
    1: "单点定位 (3D SPS)",
    2: "差分定位 (DGPS)",
    4: "RTK 厘米级固定解 (Fixed)",
    5: "RTK 浮点解 (Float)"
}

def classify_ant_fix(pos_type, sol_stat="SOL_COMPUTED", fix_quality=None, sats_used=0):
    """
    统一归纳为用户指定的严格三态：单点 / 浮点 / 固定 (以及未定位时的 搜星中)
    """
    p = str(pos_type or '').upper()
    s = str(sol_stat or '').upper()
    q = fix_quality if fix_quality is not None else 0

    # 1. 固定解 (载波相位整周模糊度固定，含窄巷/宽巷/L1固定及GGA 4)
    if (('INT' in p or 'FIX' in p or 'FIXED' in p or q == 4) and 'FLOAT' not in p):
        return "固定"

    # 2. 浮点解 (RTK Float，含窄巷/宽巷浮点及GGA 5)
    if ('FLOAT' in p or q == 5):
        return "浮点"

    # 3. 单点定位 (SPS / PSRDIFF / SBAS / 单点粗定位及GGA 1, 2)
    if ('SINGLE' in p or 'PSRDIFF' in p or 'SBAS' in p or 'SPS' in p or q in [1, 2] or (sats_used >= 4 and s == "SOL_COMPUTED")):
        return "单点"

    if sats_used >= 4:
        return "单点"
    return "搜星中"

class RTKNode(Node):
    def __init__(self):
        super().__init__('rtk_node')
        self.pub_status = self.create_publisher(String, '/rtk/status', 10)
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.ser = None
        self.running = True

        self.data_lock = threading.Lock()
        self.gsv_satellites = {} # prn -> {'az': az, 'el': el, 'snr': snr, 'ts': timestamp, 'sys': sys}
        self.state = {
            'connected': False,
            'port': PORT,
            'baud': BAUD,
            'fix_quality': 0,
            'fix_type_str': "未连接",
            'sats_used': 0,
            'sats_tracked': 0,
            'sats_in_view': 0,
            # 双天线 (左天线 ANT1 / 右天线 ANT2) 详细搜星解算指标
            'ant1_used': 0,       # 主天线(左 ANT1) 参与解算卫星数
            'ant1_tracked': 0,    # 主天线(左 ANT1) 跟踪卫星数
            'ant2_used': 0,       # 辅天线(右 ANT2) 参与解算卫星数
            'ant2_tracked': 0,    # 辅天线(右 ANT2) 跟踪卫星数
            'heading_sats_common': 0, # 双天线共视定向卫星数
            # 空间机体左右侧天空卫星分布 (相对于车身/雷达前方)
            'sats_left': 0,       # 机体左侧天空可见卫星数 (方位 180°~360°)
            'sats_right': 0,      # 机体右侧天空可见卫星数 (方位 0°~180°)
            'satellites': [],     # 可见卫星天空极坐标列表
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'hdop': 99.0,
            'heading': 0.0,
            'pitch': 0.0,
            'baseline_m': 0.0,
            'has_heading': False,
            'heading_status': "未定向",
            'speed_kmh': 0.0,
            'sol_status': "INSUFFICIENT_OBS",
            'pos_type': "NONE",
            'sol_status2': "INSUFFICIENT_OBS",
            'pos_type2': "NONE",
            'ant1_fix': "搜星中",  # 用户指定三态：单点 / 浮点 / 固定
            'ant2_fix': "搜星中",  # 用户指定三态：单点 / 浮点 / 固定
            'raw_time': "",
            'last_update': 0.0,
            # CORS 网络差分客户端状态
            'cors': {
                'enabled': False,
                'connected': False,
                'status': "未启用",
                'server': "",
                'user': "",
                'bytes_received': 0,
                'rtcm_age': 99.0,
                'rate_kbs': 0.0
            }
        }

        self.last_raw_gga = ""
        self.cors_socket = None
        self.cors_reconnect_event = threading.Event()
        self.cors_config = self.load_cors_config()
        self.sub_cors_cmd = self.create_subscription(String, '/rtk/cors_cmd', self.cors_cmd_cb, 10)

        # 启动后台读取线程
        self.reader_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.reader_thread.start()

        # 启动后台 CORS / NTRIP 差分流注入线程
        self.cors_thread = threading.Thread(target=self.cors_loop, daemon=True)
        self.cors_thread.start()

        # 1Hz 发布定时器
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f">>> [RTK Node] 启动成功，正在连接 {PORT} @ {BAUD}...")

    def open_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if not os.path.exists(PORT):
            return False

        try:
            ser = serial.Serial(PORT, BAUD, timeout=1.0)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.05)
            ser.reset_input_buffer()

            # 发送配置指令开启 NMEA 和 Unicore 格式数据 (含双天线主辅数据与多星座星历)
            init_cmds = [
                b"MODE ROVER\r\n",
                b"LOG GNGGA ONTIME 1\r\n",
                b"LOG GNRMC ONTIME 1\r\n",
                b"LOG GNGSA ONTIME 1\r\n",
                b"LOG GPGSV ONTIME 1\r\n",
                b"LOG GBGSV ONTIME 1\r\n",
                b"LOG GLGSV ONTIME 1\r\n",
                b"LOG GAGSV ONTIME 1\r\n",
                b"LOG HEADINGA ONTIME 1\r\n",
                b"LOG BESTPOSA ONTIME 1\r\n",
                b"LOG BESTPOS2A ONTIME 1\r\n",
                b"SAVECONFIG\r\n"
            ]
            for cmd in init_cmds:
                ser.write(cmd)
                time.sleep(0.03)

            self.ser = ser
            with self.data_lock:
                self.state['connected'] = True
                self.state['fix_type_str'] = FIX_TYPE_MAP.get(self.state['fix_quality'], "搜星中")
            self.get_logger().info(f">>> [RTK Node] 成功连接并配置 {PORT} @ {BAUD}")
            return True
        except Exception as e:
            self.get_logger().warn(f"打开串口 {PORT} 失败: {e}", throttle_duration_sec=3.0)
            return False

    def load_cors_config(self):
        if os.path.exists(CORS_CONFIG_PATH):
            try:
                with open(CORS_CONFIG_PATH, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception:
                pass
        return {
            "enabled": False,
            "server": "rtk.ntrip.qxwz.com",
            "port": 8002,
            "mountpoint": "AUTO",
            "username": "",
            "password": "",
            "preset": "qxwz"
        }

    def cors_cmd_cb(self, msg):
        self.get_logger().info(f">>> [CORS] 收到重载/连接指令: {msg.data}")
        self.cors_config = self.load_cors_config()
        if self.cors_socket:
            try:
                self.cors_socket.close()
            except Exception:
                pass
            self.cors_socket = None
        self.cors_reconnect_event.set()

    def cors_loop(self):
        time.sleep(2.0)
        while self.running:
            cfg = self.cors_config
            if not cfg.get('enabled') or not cfg.get('username'):
                with self.data_lock:
                    self.state['cors'] = {
                        'enabled': False,
                        'connected': False,
                        'status': '未启用 (请配置CORS账号)',
                        'server': f"{cfg.get('server', '')}:{cfg.get('port', 8002)}/{cfg.get('mountpoint', '')}",
                        'user': cfg.get('username', ''),
                        'bytes_received': 0,
                        'rtcm_age': 99.0,
                        'rate_kbs': 0.0
                    }
                time.sleep(1.5)
                continue

            host = cfg.get('server', '').strip()
            port = int(cfg.get('port', 8002))
            mount = cfg.get('mountpoint', '').strip().lstrip('/')
            user = cfg.get('username', '').strip()
            pwd = cfg.get('password', '').strip()

            with self.data_lock:
                self.state['cors'] = {
                    'enabled': True,
                    'connected': False,
                    'status': f'正在连接 {host}:{port}...',
                    'server': f"{host}:{port}/{mount}",
                    'user': user,
                    'bytes_received': 0,
                    'rtcm_age': 99.0,
                    'rate_kbs': 0.0
                }

            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(6.0)
                s.connect((host, port))
                self.cors_socket = s

                # 发送 NTRIP HTTP GET 请求
                auth = base64.b64encode(f"{user}:{pwd}".encode()).decode()
                req = (
                    f"GET /{mount} HTTP/1.1\r\n"
                    f"Host: {host}:{port}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: NTRIP RK3588_RTK/2.0\r\n"
                    f"Authorization: Basic {auth}\r\n"
                    f"Accept: */*\r\n"
                    f"Connection: close\r\n\r\n"
                )
                s.sendall(req.encode())

                # 读取响应头 (处理 ICY 200 OK 或 HTTP/1.1 200 OK)
                resp = b""
                while b"\r\n\r\n" not in resp and b"\n\n" not in resp:
                    chunk = s.recv(512)
                    if not chunk:
                        break
                    resp += chunk

                header_str = resp.decode('latin1', errors='ignore')
                if "200 OK" not in header_str and "ICY 200" not in header_str:
                    err_msg = "鉴权失败 (账号密码错误)" if "401" in header_str else "挂载点不存在 (404)" if "404" in header_str else "CORS 拒绝"
                    self.get_logger().warn(f">>> [CORS] 连接失败: {err_msg} ({header_str[:50]})")
                    with self.data_lock:
                        self.state['cors']['status'] = f"失败: {err_msg}"
                        self.state['cors']['connected'] = False
                    s.close()
                    self.cors_socket = None
                    time.sleep(5.0)
                    continue

                self.get_logger().info(f">>> [CORS] 成功握手 {host}:{port}/{mount}，开始下发 RTCM3 差分流...")
                with self.data_lock:
                    self.state['cors']['status'] = "已连接 · 接收差分流"
                    self.state['cors']['connected'] = True

                s.settimeout(6.0)
                last_gga_time = 0
                total_bytes = 0
                calc_time = time.time()
                bytes_in_sec = 0
                last_rtcm_ts = time.time()

                while self.running and not self.cors_reconnect_event.is_set():
                    now = time.time()
                    # 1. 周期性上传本车坐标 (VRS 基站依赖本车位置下发临近差分数据，每 5 秒发送一次)
                    if now - last_gga_time >= 5.0:
                        raw_gga = self.last_raw_gga
                        if raw_gga:
                            try:
                                s.sendall((raw_gga + "\r\n").encode())
                                last_gga_time = now
                            except Exception:
                                break

                    # 2. 接收 RTCM 差分包
                    try:
                        rtcm_chunk = s.recv(2048)
                    except socket.timeout:
                        continue
                    except Exception:
                        break

                    if not rtcm_chunk:
                        break

                    # 3. 将 RTCM 二进制差分流直接灌入串口给 UM982 芯片解算
                    if self.ser:
                        try:
                            self.ser.write(rtcm_chunk)
                        except Exception:
                            pass

                    total_bytes += len(rtcm_chunk)
                    bytes_in_sec += len(rtcm_chunk)
                    last_rtcm_ts = now

                    # 4. 计算速率与龄期
                    if now - calc_time >= 1.0:
                        rate_kbs = round(bytes_in_sec / (now - calc_time) / 1024.0, 1)
                        bytes_in_sec = 0
                        calc_time = now
                        with self.data_lock:
                            self.state['cors']['bytes_received'] = total_bytes
                            self.state['cors']['rate_kbs'] = rate_kbs
                            self.state['cors']['rtcm_age'] = round(now - last_rtcm_ts, 1)
                            self.state['cors']['status'] = f"差分流注入中 ({rate_kbs} KB/s)"
                            self.state['cors']['connected'] = True

            except Exception as e:
                self.get_logger().warn(f">>> [CORS] 异常: {e}")
                with self.data_lock:
                    self.state['cors']['status'] = f"连接断开: {e}"
                    self.state['cors']['connected'] = False
            finally:
                if s:
                    try:
                        s.close()
                    except Exception:
                        pass
                self.cors_socket = None
                self.cors_reconnect_event.clear()

            time.sleep(3.0)

    def parse_lat_lon(self, val, direction):
        if not val or not direction:
            return 0.0
        try:
            deg_len = 2 if direction in ["N", "S"] else 3
            deg = float(val[:deg_len])
            minute = float(val[deg_len:])
            res = deg + minute / 60.0
            if direction in ["S", "W"]:
                res = -res
            return res
        except Exception:
            return 0.0

    def serial_loop(self):
        while self.running:
            if not self.ser:
                if not self.open_serial():
                    with self.data_lock:
                        self.state['connected'] = False
                        self.state['fix_type_str'] = "未连接设备"
                    time.sleep(1.0)
                    continue

            try:
                line_b = self.ser.readline()
                if not line_b:
                    continue
                line = line_b.decode(errors="replace").strip()
                if not line:
                    continue

                now = time.time()

                try:
                    # 1. 解析 $GNGGA
                    if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                        self.last_raw_gga = line
                        parts = line.split("*")[0].split(",")
                        if len(parts) >= 10:
                            raw_time = parts[1]
                            lat = self.parse_lat_lon(parts[2], parts[3])
                            lon = self.parse_lat_lon(parts[4], parts[5])
                            q = safe_int(parts[6], 0)
                            sats = safe_int(parts[7], 0)
                            hdop = safe_float(parts[8], 99.0)
                            alt = safe_float(parts[9], 0.0)

                            with self.data_lock:
                                self.state['raw_time'] = raw_time
                                self.state['fix_quality'] = q
                                self.state['fix_type_str'] = FIX_TYPE_MAP.get(q, f"解算模式({q})")
                                if sats > 0:
                                    self.state['sats_used'] = sats
                                self.state['hdop'] = hdop
                                if q > 0 and (lat != 0 or lon != 0):
                                    self.state['lat'] = lat
                                    self.state['lon'] = lon
                                    self.state['alt'] = alt
                                self.state['last_update'] = now

                    # 2. 解析 $GNRMC
                    elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                        parts = line.split("*")[0].split(",")
                        if len(parts) >= 9:
                            spd_knots = safe_float(parts[7], 0.0)
                            spd_kmh = round(spd_knots * 1.852, 1)
                            with self.data_lock:
                                self.state['speed_kmh'] = spd_kmh

                    # 3. 解析 GSV (包含北斗GBGSV、GPS GPGSV、GLONASS GLGSV、Galileo GAGSV等天空方位与仰角)
                    elif line.startswith(("$GPGSV", "$GBGSV", "$GLGSV", "$GAGSV", "$GQGSV", "$BDGSV")):
                        parts = line.split("*")[0].split(",")
                        if len(parts) >= 4:
                            sys_prefix = parts[0][1:3]
                            idx = 4
                            while idx + 3 < len(parts):
                                sat_id = parts[idx].strip()
                                if sat_id:
                                    prn = f"{sys_prefix}{sat_id}"
                                    el = safe_float(parts[idx+1], 0.0)
                                    az = safe_float(parts[idx+2], 0.0)
                                    snr = safe_float(parts[idx+3], 0.0)
                                    self.gsv_satellites[prn] = {
                                        'prn': prn,
                                        'sys': sys_prefix,
                                        'el': el,
                                        'az': az,
                                        'snr': snr,
                                        'ts': now
                                    }
                                idx += 4

                    # 4. 解析 #BESTPOSA (主天线 / 左天线 ANT1 实时定位与搜星)
                    elif line.startswith("#BESTPOSA"):
                        if ";" in line:
                            body = line.split(";")[1].split("*")[0].split(",")
                            if len(body) >= 15:
                                sol_stat = body[0]
                                pos_type = body[1]
                                b_lat = safe_float(body[2], 0.0)
                                b_lon = safe_float(body[3], 0.0)
                                b_hgt = safe_float(body[4], 0.0)
                                tracked = safe_int(body[13], 0)
                                used = safe_int(body[14], 0)
                                with self.data_lock:
                                    self.state['sol_status'] = sol_stat
                                    self.state['pos_type'] = pos_type
                                    if tracked > 0:
                                        self.state['sats_tracked'] = tracked
                                        self.state['ant1_tracked'] = tracked
                                    if used > 0:
                                        self.state['sats_used'] = used
                                        self.state['ant1_used'] = used
                                    if sol_stat == "SOL_COMPUTED" and (b_lat != 0 or b_lon != 0):
                                        self.state['lat'] = b_lat
                                        self.state['lon'] = b_lon
                                        self.state['alt'] = b_hgt
                                        if self.state['fix_quality'] == 0:
                                            self.state['fix_quality'] = 1
                                            self.state['fix_type_str'] = "单点定位 (3D SPS)"

                    # 5. 解析 #BESTPOS2A (辅天线 / 右天线 ANT2 实时搜星与解算)
                    elif line.startswith("#BESTPOS2A"):
                        if ";" in line:
                            body = line.split(";")[1].split("*")[0].split(",")
                            if len(body) >= 15:
                                sol_stat2 = body[0]
                                pos_type2 = body[1]
                                tracked2 = safe_int(body[13], 0)
                                used2 = safe_int(body[14], 0)
                                with self.data_lock:
                                    self.state['sol_status2'] = sol_stat2
                                    self.state['pos_type2'] = pos_type2
                                    if tracked2 > 0:
                                        self.state['ant2_tracked'] = tracked2
                                    if used2 > 0:
                                        self.state['ant2_used'] = used2

                    # 6. 解析 #HEADINGA (双天线基线长度、相对航向与共视卫星)
                    elif line.startswith("#HEADINGA"):
                        if ";" in line:
                            hbody = line.split(";")[1].split("*")[0].split(",")
                            if len(hbody) >= 12:
                                h_stat = hbody[0]
                                h_pos = hbody[1]
                                length = safe_float(hbody[2], 0.0)
                                heading = safe_float(hbody[3], 0.0)
                                pitch = safe_float(hbody[4], 0.0)
                                common_sats = safe_int(hbody[10], 0)
                                with self.data_lock:
                                    self.state['baseline_m'] = length
                                    self.state['heading'] = heading
                                    self.state['pitch'] = pitch
                                    self.state['heading_type'] = h_pos
                                    if common_sats > 0:
                                        self.state['heading_sats_common'] = common_sats
                                    if h_stat == "SOL_COMPUTED" and heading > 0:
                                        self.state['has_heading'] = True
                                        self.state['heading_status'] = f"已锁定 {heading:.1f}° ({h_pos})"
                                    else:
                                        self.state['has_heading'] = False
                                        self.state['heading_status'] = "未锁定 (搜星中)"
                except Exception as parse_err:
                    pass

            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f"串口物理断开或读取异常: {exc}, 正在重连...")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(1.0)

    def publish_status(self):
        now = time.time()
        # 统计分析当前可见卫星的天空左右分布情况 (相对于航向)
        with self.data_lock:
            hdg = self.state.get('heading', 0.0) or 0.0

        # 清理 6 秒未刷新的旧星历数据
        stale_keys = [k for k, v in self.gsv_satellites.items() if now - v['ts'] > 6.0]
        for k in stale_keys:
            del self.gsv_satellites[k]

        active_sats = list(self.gsv_satellites.values())
        left_count = 0
        right_count = 0
        sat_list = []

        for s in active_sats:
            # 相对航向角的方位角：0°为正前方(顺时针)
            rel_az = (s['az'] - hdg) % 360.0
            # 0°~180° 为机体右侧天空，180°~360° 为机体左侧天空
            is_right = (0.0 < rel_az < 180.0)
            side = 'right' if is_right else 'left'
            if is_right:
                right_count += 1
            else:
                left_count += 1

            sat_list.append({
                'prn': s['prn'],
                'sys': s['sys'],
                'az': round(s['az'], 1),
                'el': round(s['el'], 1),
                'snr': round(s['snr'], 1),
                'rel_az': round(rel_az, 1),
                'side': side
            })

        with self.data_lock:
            # 计算左右天线独立的固定情况：单点 / 浮点 / 固定
            ant1_pos = self.state.get('pos_type', '')
            ant1_sol = self.state.get('sol_status', '')
            ant1_used = self.state.get('ant1_used', 0)
            fix_q = self.state.get('fix_quality', 0)
            ant1_fix = classify_ant_fix(ant1_pos, ant1_sol, fix_q, ant1_used)

            ant2_pos = self.state.get('pos_type2', '')
            ant2_sol = self.state.get('sol_status2', '')
            ant2_used = self.state.get('ant2_used', 0)
            heading_type = self.state.get('heading_type', '')
            if ant2_pos and ant2_pos != "NONE":
                ant2_fix = classify_ant_fix(ant2_pos, ant2_sol, None, ant2_used)
            else:
                ant2_fix = classify_ant_fix(heading_type, "SOL_COMPUTED" if self.state.get('has_heading') else "", None, ant2_used)

            self.state['ant1_fix'] = ant1_fix
            self.state['ant2_fix'] = ant2_fix

            if len(active_sats) > 0:
                self.state['sats_in_view'] = len(active_sats)
            self.state['sats_left'] = left_count
            self.state['sats_right'] = right_count
            self.state['satellites'] = sat_list
            payload = dict(self.state)

        # 1. 发布 JSON 状态
        msg_str = String()
        msg_str.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg_str)

        # 2. 发布 ROS 2 NavSatFix 格式消息
        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "gps"
        if payload['fix_quality'] > 0:
            fix_msg.status.status = NavSatStatus.STATUS_FIX if payload['fix_quality'] == 1 else NavSatStatus.STATUS_GBAS_FIX
        else:
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        fix_msg.latitude = payload['lat']
        fix_msg.longitude = payload['lon']
        fix_msg.altitude = payload['alt']
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(fix_msg)

    def destroy_node(self):
        self.running = False
        if self.cors_socket:
            try:
                self.cors_socket.close()
            except Exception:
                pass
            self.cors_socket = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTKNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
