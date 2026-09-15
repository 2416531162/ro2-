#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 板载屏幕原生 PyQt5 激光雷达 + 奥比中光 3D 深度相机可视化控制台 (1920x1080 横屏满屏优化版)
- 运行在开发板屏幕上 (DISPLAY=:0)
- 完美融合：
    1. 激光雷达 (LiDAR) 360° 极坐标空间雷达盘
    2. 奥比中光 Astra S 3D 深度相机实时视窗 (RGB 实景 / 深度距离图一键切换)
    3. 四向安全避障数字仪表盘
    4. 触控操作栏 (量程切换、视角选择、全屏控制)
"""

import sys
import os
import math
import time
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QFrame, QGridLayout, QSizePolicy,
                             QDialog, QLineEdit, QComboBox)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QPointF, QSize, QTimer, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QImage, QPixmap, QFontMetrics

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from n10p_pipeline import scan_payload, project_point
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
import numpy as np
import cv2

CORS_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cors_config.json")

DEPTH_VALID_MIN_MM = 200
DEPTH_VALID_MAX_MM = 5500
HEAT_BG_RGB = (20, 25, 34)
HEAT_DEFAULT_NEAR_MM = 200.0
HEAT_DEFAULT_FAR_MM = 2000.0


def _heat_lut():
    # Near red -> yellow -> green -> cyan -> blue far; identical LUT in legend.
    stops=np.array([[255,55,45],[255,215,40],[60,220,95],[30,205,240],[45,75,225]],np.float32)
    t=np.linspace(0,4,256)
    return np.stack([np.interp(t,np.arange(5),stops[:,i]) for i in range(3)],axis=1).round().astype(np.uint8)


HEAT_LUT = _heat_lut()


def decode_depth_mm(msg):
    encoding=msg.encoding.lower()
    if encoding in ('16uc1','mono16'):
        dtype=np.dtype('>u2' if msg.is_bigendian else '<u2'); multiplier=1.0
    elif encoding=='32fc1':
        dtype=np.dtype('>f4' if msg.is_bigendian else '<f4'); multiplier=1000.0
    else:raise ValueError('unsupported depth encoding: '+msg.encoding)
    if msg.step<msg.width*dtype.itemsize or len(msg.data)<msg.step*msg.height:
        raise ValueError('invalid depth stride/buffer')
    depth=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                     strides=(msg.step,dtype.itemsize)).astype(np.float32)*multiplier
    depth[~np.isfinite(depth)]=0
    return depth


def depth_quality(depth, far_mm):
    finite=np.isfinite(depth)
    missing=(~finite)|(depth<=0)|(depth==65535)
    valid=finite&(depth>=DEPTH_VALID_MIN_MM)&(depth<=DEPTH_VALID_MAX_MM)
    outside=(~missing)&(~valid)
    return dict(valid=float(np.mean(valid)),missing=float(np.mean(missing)),
                outside=float(np.mean(outside)),clipped=float(np.mean(valid&(depth>far_mm))))


def depth_center_mm(depth):
    h,w=depth.shape;roi=depth[max(0,h//2-5):h//2+6,max(0,w//2-5):w//2+6]
    values=roi[(roi>=DEPTH_VALID_MIN_MM)&(roi<=DEPTH_VALID_MAX_MM)&np.isfinite(roi)]
    if values.size<max(12,roi.size*.35):return 0
    q25,median,q75=np.percentile(values,[25,50,75])
    if q75-q25>max(120,median*.1):return 0
    return int(round(median))


def _draw_heatmap_legend(rgb, near_mm, far_mm):
    h,w=rgb.shape[:2];canvas=np.empty((h+54,w,3),np.uint8)
    canvas[:h]=rgb;canvas[h:]=HEAT_BG_RGB
    x0=12;bar_w=max(8,w-24)
    bar=HEAT_LUT[np.rint(np.linspace(0,255,bar_w)).astype(int)]
    canvas[h+5:h+17,x0:x0+bar_w]=bar
    for fraction in [0,.25,.5,.75,1]:
        value=(near_mm+(far_mm-near_mm)*fraction)/1000
        text=f'{value:.2f} m';size=cv2.getTextSize(text,cv2.FONT_HERSHEY_SIMPLEX,.40,1)[0][0]
        tx=int(x0+fraction*bar_w-size*fraction)
        cv2.putText(canvas,text,(tx,h+35),cv2.FONT_HERSHEY_SIMPLEX,.40,(225,233,242),1,cv2.LINE_AA)
    cv2.putText(canvas,'NEAR / RED    DEPTH Z (m)    FAR / BLUE    DARK = INVALID / OUT OF RANGE',
                (x0,h+49),cv2.FONT_HERSHEY_SIMPLEX,.31,(169,185,204),1,cv2.LINE_AA)
    return canvas


def render_depth_heatmap(depth_mm, rgb=None, near_mm=None, far_mm=None, scale=1):
    """Pure metric depth colors. rgb is ignored for backwards API compatibility.

    Never fill missing measurements; never normalize colors per-frame. Scale is
    nearest-neighbor so no intermediate colors masquerade as measured depths.
    """
    depth=np.asarray(depth_mm)
    near_mm=HEAT_DEFAULT_NEAR_MM if near_mm is None else float(near_mm)
    far_mm=HEAT_DEFAULT_FAR_MM if far_mm is None else float(far_mm)
    if not 0<=near_mm<far_mm<=DEPTH_VALID_MAX_MM:raise ValueError('invalid metric color range')
    valid=np.isfinite(depth)&(depth>=DEPTH_VALID_MIN_MM)&(depth<=DEPTH_VALID_MAX_MM)
    normalized=np.clip((np.where(valid,depth,near_mm)-near_mm)/(far_mm-near_mm),0,1)
    out=HEAT_LUT[np.rint(normalized*255).astype(np.uint8)]
    out[~valid]=HEAT_BG_RGB
    if scale!=1:
        h,w=depth.shape
        out=cv2.resize(out,(int(w*scale),int(h*scale)),interpolation=cv2.INTER_NEAREST)
    return _draw_heatmap_legend(out,near_mm,far_mm),near_mm,far_mm


class ROSThread(QThread):
    scan_signal = pyqtSignal(dict)
    rgb_signal = pyqtSignal(QImage)
    depth_signal = pyqtSignal(QImage, int, int, int)
    ai_signal = pyqtSignal(QImage)
    targets_signal = pyqtSignal(str)
    rtk_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.last_rgb_time = 0
        self.last_depth_time = 0
        self.last_ai_time = 0
        self.display_mode = 'ai'
        self.latest_scan = None
        self.latest_rgb = None
        self.latest_depth = None

    def run(self):
        try:
            rclpy.init()
        except Exception:
            pass
        node = Node('board_radar_gui_node')

        def scan_callback(msg):
            age = (node.get_clock().now().nanoseconds / 1e9 -
                   msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)
            payload = scan_payload(msg.ranges, msg.range_min, msg.range_max,
                                   msg.angle_min, msg.angle_increment, msg.scan_time, age)
            payload['received'] = time.monotonic()
            # A single latest-value mailbox prevents queued scan signal backlog.
            self.latest_scan = payload

        def rgb_callback(msg):
            now = time.time()
            try:
                rgb_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
                self.latest_rgb = rgb_arr
            except Exception:
                rgb_arr = None
            if self.display_mode not in ('rgb', 'ai'):
                return
            if now - self.last_rgb_time < 0.015:
                return
            self.last_rgb_time = now
            if rgb_arr is None:
                return
            try:
                qimg = QImage(rgb_arr.data, msg.width, msg.height, msg.width * 3, QImage.Format_RGB888).copy()
                self.rgb_signal.emit(qimg)
            except Exception:
                pass

        def depth_callback(msg):
            if self.display_mode != 'depth':return
            try:
                depth=decode_depth_mm(msg)
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                age=node.get_clock().now().nanoseconds/1e9-stamp
                self.latest_depth=dict(array=depth,received=time.monotonic(),age=max(0,age))
            except (ValueError,TypeError) as exc:
                node.get_logger().warn('depth: '+str(exc),throttle_duration_sec=5.0)

        def ai_callback(msg):
            if self.display_mode != 'ai':
                return
            now = time.time()
            if now - self.last_ai_time < 0.04:
                return
            self.last_ai_time = now
            try:
                qimg = QImage(msg.data, msg.width, msg.height, msg.width * 3, QImage.Format_RGB888).copy()
                self.ai_signal.emit(qimg)
            except Exception:
                pass

        def targets_callback(msg):
            try:
                self.targets_signal.emit(msg.data)
            except Exception:
                pass

        def rtk_callback(msg):
            try:
                self.rtk_signal.emit(json.loads(msg.data))
            except Exception:
                pass

        node.create_subscription(String, '/rtk/status', rtk_callback, 10)
        node.create_subscription(LaserScan, '/scan', scan_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Image, '/camera/rgb/image_raw', rgb_callback, 10)
        node.create_subscription(Image, '/camera/depth_raw/image', depth_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Image, '/camera/ai_detection/image', ai_callback, 10)
        node.create_subscription(String, '/camera/ai_detection/targets', targets_callback, 10)

        try:
            rclpy.spin(node)
        except Exception:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

class RadarCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ranges = []
        self.max_range = 5.0
        self.angle_min = 0.0
        self.angle_increment = math.pi / 360
        self.rtk_data = None
        self.paused = False
        self.stale = True
        self.range_min = 0.15
        self._grid_key = None
        self.setMinimumSize(360, 340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_rtk(self, data):
        # RTK belongs in its own card, not in the metric LiDAR plane.
        self.rtk_data = data

    def set_range(self, meters):
        self.max_range = float(meters)
        self.update()

    def set_scan(self, data):
        self.stale = data.get('stale', False)
        if not self.paused or self.stale:
            self.ranges = [] if self.stale else data.get('ranges', [])
            self.angle_min = data.get('angle_min', 0.0)
            self.angle_increment = data.get('angle_increment', math.pi / 360)
            self.range_min = data.get('range_min', 0.15)
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        radius = max(20, min(w, h) / 2 - 34)
        key = (w, h, self.max_range, self.devicePixelRatioF())
        if key != self._grid_key:
            dpr = self.devicePixelRatioF()
            self._grid = QPixmap(round(w*dpr), round(h*dpr))
            self._grid.setDevicePixelRatio(dpr)
            self._grid.fill(QColor('#0b1220'))
            g = QPainter(self._grid)
            g.setRenderHint(QPainter.Antialiasing)
            g.setFont(QFont('sans-serif', 10))
            for i in range(1, 6):
                rr = radius * i / 5
                g.setPen(QPen(QColor('#385167' if i == 5 else '#25364a'), 1,
                              Qt.SolidLine if i == 5 else Qt.DashLine))
                g.drawEllipse(QPointF(cx, cy), rr, rr)
                if i < 5:
                    g.setPen(QColor('#aebfd0'))
                    g.drawText(int(cx + 9), int(cy - rr - 5), f'{self.max_range*i/5:g} m')
            for deg in range(0, 360, 30):
                angle = math.radians(deg)
                x, y = project_point(angle, 1, cx, cy, radius)
                g.setPen(QPen(QColor('#354b62' if deg % 90 == 0 else '#1d2c3e'), 1))
                g.drawLine(QPointF(cx, cy), QPointF(x, y))
            g.setPen(QColor('#d0deea'))
            labels = [(cx-60, cy-radius-27, '前 0°'),
                      (cx-60, cy+radius+8, '后 180°')]
            for x, y, label in labels:
                g.drawText(QRectF(x, y, 120, 20), Qt.AlignCenter, label)
            # Keep horizontal labels outside the circle, rotated for narrow panes.
            for x, text in [(cx-radius-20, '左 90°'), (cx+radius+20, '右 270°')]:
                g.save(); g.translate(x, cy); g.rotate(-90 if x < cx else 90)
                g.drawText(QRectF(-50, -10, 100, 20), Qt.AlignCenter, text); g.restore()
            g.end()
            self._grid_key = key
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._grid)
        painter.setRenderHint(QPainter.Antialiasing)
        # No glow, connecting lines or temporal accumulation: preserve geometry.
        groups = [[], [], []]
        for i, distance in enumerate(self.ranges):
            if not math.isfinite(distance) or not self.range_min <= distance <= self.max_range:
                continue
            x, y = project_point(self.angle_min + i*self.angle_increment,
                                 distance, cx, cy, radius/self.max_range)
            groups[0 if distance < 0.6 else 1 if distance < 1.2 else 2].append(QPointF(x, y))
        for color, points in zip(['#ff747f', '#ffc66a', '#61d8e8'], groups):
            painter.setPen(QPen(QColor(color), 3.2, Qt.SolidLine, Qt.RoundCap))
            if points:
                painter.drawPoints(*points)
        painter.setPen(QPen(QColor('#e3eef9'), 2))
        painter.drawLine(QPointF(cx-5, cy+4), QPointF(cx, cy-7))
        painter.drawLine(QPointF(cx, cy-7), QPointF(cx+5, cy+4))
        painter.drawLine(QPointF(cx-5, cy+4), QPointF(cx+5, cy+4))
        if self.stale or self.paused:
            painter.setPen(QColor('#ffc66a'))
            painter.setFont(QFont('sans-serif', 12, QFont.Bold))
            painter.drawText(QRectF(0, h/2+20, w, 30), Qt.AlignCenter,
                             '数据中断 · 等待新扫描' if self.stale else '点云已暂停 · 测距仍在更新')
        painter.end()

class CorsConfigDialog(QDialog):
    def __init__(self, parent=None, ros_thread=None):
        super().__init__(parent)
        self.ros_thread = ros_thread
        self.setWindowTitle("CORS 厘米级差分设置")
        self.setFixedSize(560, 600)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setStyleSheet("""
            QDialog {
                background: #181f29;
                border: 2px solid #00f2fe;
                border-radius: 12px;
            }
            QLabel {
                color: #ecf2f8;
                font-family: sans-serif;
            }
            QLineEdit, QComboBox {
                background: #111923;
                border: 1px solid #303c4b;
                border-radius: 6px;
                color: #ecf2f8;
                font-size: 13px;
                padding: 7px 10px;
                font-family: monospace;
            }
            QLineEdit:focus, QComboBox:focus {
                border: 1px solid #00f2fe;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        # 标题栏
        top_bar = QHBoxLayout()
        title = QLabel("🛰️ CORS 厘米级差分设置")
        title.setFont(QFont("sans-serif", 13, QFont.Bold))
        title.setStyleSheet("color: #69dec4;")
        top_bar.addWidget(title)

        btn_close = QPushButton("✕")
        btn_close.setFixedSize(30, 30)
        btn_close.setStyleSheet("background: transparent; color: #a1afc0; font-size: 18px; border: none; font-weight: bold;")
        btn_close.clicked.connect(self.close)
        top_bar.addWidget(btn_close)
        layout.addLayout(top_bar)

        desc = QLabel("注入 RTCM3 差分流消除电离层延迟，左右天线实时独立解算 单点 / 浮点 / 固定。")
        desc.setStyleSheet("color: #94a3b8; font-size: 11px;")
        desc.setWordWrap(True)
        layout.addWidget(desc)

        # 预设
        layout.addWidget(QLabel("服务商快速预设:"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItem("千寻位置 (rtk.ntrip.qxwz.com:8002 / AUTO)", "qxwz")
        self.preset_combo.addItem("六分科技 (rtk.sixents.com:8002 / RTCM32_GGB)", "liufen")
        self.preset_combo.addItem("中国移动高精度 (221.178.251.100:8002 / RTCM33_GRCEJ)", "cmcc")
        self.preset_combo.addItem("自定义私有 CORS / NTRIP 基准站", "custom")
        self.preset_combo.currentIndexChanged.connect(self.on_preset_changed)
        layout.addWidget(self.preset_combo)

        # 服务器 & 端口
        grid = QGridLayout()
        grid.addWidget(QLabel("服务器地址 (Host):"), 0, 0)
        grid.addWidget(QLabel("端口 (Port):"), 0, 1)
        self.input_host = QLineEdit()
        self.input_host.setPlaceholderText("rtk.ntrip.qxwz.com")
        self.input_port = QLineEdit()
        self.input_port.setPlaceholderText("8002")
        grid.addWidget(self.input_host, 1, 0)
        grid.addWidget(self.input_port, 1, 1)
        layout.addLayout(grid)

        # 挂载点
        layout.addWidget(QLabel("挂载点 (MountPoint):"))
        self.input_mount = QLineEdit()
        self.input_mount.setPlaceholderText("AUTO")
        layout.addWidget(self.input_mount)

        # 账号
        layout.addWidget(QLabel("差分账号 (Username):"))
        self.input_user = QLineEdit()
        self.input_user.setPlaceholderText("输入 CORS 账号")
        layout.addWidget(self.input_user)

        # 密码
        layout.addWidget(QLabel("差分密码 (Password):"))
        self.input_pwd = QLineEdit()
        self.input_pwd.setEchoMode(QLineEdit.Password)
        self.input_pwd.setPlaceholderText("输入密码")
        layout.addWidget(self.input_pwd)

        # 实时状态显示卡片
        self.stat_frame = QFrame()
        self.stat_frame.setStyleSheet("background: #111923; border: 1px solid #283440; border-radius: 6px; padding: 8px 12px;")
        stat_l = QVBoxLayout(self.stat_frame)
        stat_l.setSpacing(4)
        stat_l.setContentsMargins(4, 4, 4, 4)
        self.lbl_status = QLabel("运行状态: 未启用")
        self.lbl_status.setStyleSheet("color: #f6c879; font-weight: bold; font-size: 11px;")
        self.lbl_rate = QLabel("差分速率: 0.0 KB/s")
        self.lbl_rate.setStyleSheet("color: #94a3b8; font-size: 11px;")
        stat_l.addWidget(self.lbl_status)
        stat_l.addWidget(self.lbl_rate)
        layout.addWidget(self.stat_frame)

        # 底部按钮
        btn_layout = QHBoxLayout()
        self.btn_save = QPushButton("💾 保存并连接差分")
        self.btn_save.setStyleSheet("""
            QPushButton {
                background: #10b981;
                border: none;
                border-radius: 6px;
                color: #ffffff;
                font-weight: bold;
                font-size: 12px;
                padding: 10px 16px;
            }
            QPushButton:hover { background: #059669; }
        """)
        self.btn_save.clicked.connect(lambda: self.save_config(True))
        btn_layout.addWidget(self.btn_save)

        self.btn_disconnect = QPushButton("⏹ 断开差分")
        self.btn_disconnect.setStyleSheet("""
            QPushButton {
                background: #38242c;
                border: 1px solid #9f525a;
                border-radius: 6px;
                color: #ff8c91;
                font-weight: bold;
                font-size: 12px;
                padding: 10px 16px;
            }
            QPushButton:hover { background: #4a2f39; }
        """)
        self.btn_disconnect.clicked.connect(lambda: self.save_config(False))
        btn_layout.addWidget(self.btn_disconnect)

        self.btn_cancel = QPushButton("关闭")
        self.btn_cancel.setStyleSheet("""
            QPushButton {
                background: #242f3d;
                border: 1px solid #3d4f63;
                border-radius: 6px;
                color: #ecf2f8;
                font-size: 12px;
                padding: 10px 16px;
            }
        """)
        self.btn_cancel.clicked.connect(self.close)
        btn_layout.addWidget(self.btn_cancel)
        layout.addLayout(btn_layout)

        self.load_config()

    def on_preset_changed(self, idx):
        preset_key = self.preset_combo.currentData()
        presets = {
            'qxwz': ('rtk.ntrip.qxwz.com', '8002', 'AUTO'),
            'liufen': ('rtk.sixents.com', '8002', 'RTCM32_GGB'),
            'cmcc': ('221.178.251.100', '8002', 'RTCM33_GRCEJ'),
            'custom': ('', '8002', '')
        }
        if preset_key in presets:
            host, port, mount = presets[preset_key]
            if host: self.input_host.setText(host)
            if port: self.input_port.setText(port)
            if mount: self.input_mount.setText(mount)

    def load_config(self):
        cfg = {}
        if os.path.exists(CORS_CONFIG_PATH):
            try:
                with open(CORS_CONFIG_PATH, 'r', encoding='utf-8') as f:
                    cfg = json.load(f)
            except Exception:
                pass
        preset = cfg.get('preset', 'qxwz')
        idx = self.preset_combo.findData(preset)
        if idx >= 0:
            self.preset_combo.setCurrentIndex(idx)
        self.input_host.setText(cfg.get('server', 'rtk.ntrip.qxwz.com'))
        self.input_port.setText(str(cfg.get('port', 8002)))
        self.input_mount.setText(cfg.get('mountpoint', 'AUTO'))
        self.input_user.setText(cfg.get('username', ''))
        self.input_pwd.setText(cfg.get('password', ''))

    def save_config(self, enabled=True):
        cfg = {
            'enabled': bool(enabled),
            'preset': self.preset_combo.currentData() or 'custom',
            'server': self.input_host.text().strip(),
            'port': int(self.input_port.text().strip() or 8002),
            'mountpoint': self.input_mount.text().strip(),
            'username': self.input_user.text().strip(),
            'password': self.input_pwd.text().strip()
        }
        try:
            with open(CORS_CONFIG_PATH, 'w', encoding='utf-8') as f:
                json.dump(cfg, f, indent=2, ensure_ascii=False)
            if self.ros_thread:
                self.ros_thread.send_cors_cmd("reload")
            self.lbl_status.setText("运行状态: 已发送连接指令" if enabled else "运行状态: 差分已断开")
            self.lbl_status.setStyleSheet("color: #10b981; font-weight: bold;" if enabled else "color: #ff8c91; font-weight: bold;")
        except Exception as e:
            self.lbl_status.setText(f"保存配置失败: {e}")

    def update_cors_status(self, cors_data):
        if not self.isVisible():
            return
        status = cors_data.get('status', '未启用')
        conn = cors_data.get('connected', False)
        rate = cors_data.get('rate_kbs', 0.0)
        bytes_recv = round(cors_data.get('bytes_received', 0) / 1024)
        self.lbl_status.setText(f"运行状态: {status}")
        self.lbl_status.setStyleSheet("color: #10b981; font-weight: bold;" if conn else "color: #f6c879; font-weight: bold;")
        self.lbl_rate.setText(f"差分速率: {rate} KB/s (累计接收 {bytes_recv} KB)")

class BoardRadarMainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RK3588 激光雷达 + 3D 深度相机智能感知控制台")
        self.setStyleSheet("background-color: #0b0f19; color: #e2e8f0;")
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setGeometry(0, 0, 1920, 1080)
        
        self.cam_mode = 'ai'  # 默认 'ai' 模式：实时显示 AI 3D 识别与测距
        self.latest_rgb_pixmap = None
        self.latest_depth_pixmap = None
        self.latest_ai_pixmap = None
        self.center_depth_mm = 0
        self.heat_near_mm = 0
        self.heat_far_mm = 0
        self.ros_thread = ROSThread()
        self.cors_dialog = CorsConfigDialog(self, self.ros_thread)

        self.init_ui()

        self._shown_scan = None
        self._last_scan_received = 0.0
        self._lidar_timer = QTimer(self)
        self._lidar_timer.timeout.connect(self.refresh_lidar)
        self._lidar_timer.start(50)
        self.ros_thread.rgb_signal.connect(self.on_rgb_frame)
        self._depth_shown = None
        self._depth_timer = QTimer(self)
        self._depth_timer.timeout.connect(self.refresh_depth)
        self._depth_timer.start(40)
        self.ros_thread.ai_signal.connect(self.on_ai_frame)
        self.ros_thread.targets_signal.connect(self.on_targets_data)
        self.ros_thread.rtk_signal.connect(self.on_rtk_data)
        self.ros_thread.display_mode = self.cam_mode
        self.ros_thread.start()
        self._fps_times = []
        self.last_targets = []

        if '--open-cors' in sys.argv:
            QTimer.singleShot(800, self.open_cors_dialog)
        for arg in sys.argv:
            if arg.startswith('--cam-mode='):
                mode = arg.split('=', 1)[1]
                if mode in ('ai', 'rgb', 'depth'):
                    QTimer.singleShot(200, lambda m=mode: self.switch_cam_mode(m))

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(20)

        # 左侧面板：雷达盘 + C-RTK 2HP 卫星数据监控卡片
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)
        lidar_title = QLabel('N10P  /  360° 激光雷达')
        lidar_title.setStyleSheet('font-size: 18px; font-weight: bold; color: #e3eef9; padding: 4px;')
        left_panel.addWidget(lidar_title)
        self.lidar_status = QLabel('等待 N10P 扫描数据…')
        self.lidar_status.setStyleSheet('font-size: 13px; color: #aebfd0; padding: 4px;')
        left_panel.addWidget(self.lidar_status)
        self.canvas = RadarCanvas(self)
        left_panel.addWidget(self.canvas, 1)
        lidar_tools = QHBoxLayout()
        legend = QLabel('<span style="color:#ff747f">● &lt;0.6m</span> 近距　<span style="color:#ffc66a">● &lt;1.2m</span> 注意　<span style="color:#61d8e8">● 回波</span> / 无拖影')
        legend.setStyleSheet('color: #aebfd0; font-size: 12px;')
        lidar_tools.addWidget(legend, 1)
        self.pause_lidar = QPushButton('暂停点云')
        self.pause_lidar.setCheckable(True)
        self.pause_lidar.setMinimumHeight(36)
        self.pause_lidar.setStyleSheet('QPushButton {background:#162435; color:#d0deea; border:1px solid #385167; border-radius:6px; padding:4px 12px; font-size:13px;} QPushButton:checked {background:#155e75;}')
        self.pause_lidar.clicked.connect(self.toggle_lidar_pause)
        lidar_tools.addWidget(self.pause_lidar)
        left_panel.addLayout(lidar_tools)

        self.rtk_card = self.create_rtk_card()
        left_panel.addWidget(self.rtk_card)

        main_layout.addLayout(left_panel, 38)

        # 右侧相机与控制面板
        right_panel = QVBoxLayout()
        right_panel.setSpacing(10)

        # 1. 顶部标题栏
        header_frame = QFrame()
        header_frame.setStyleSheet("background: rgba(0, 242, 254, 0.08); border: 1px solid rgba(0, 242, 254, 0.3); border-radius: 10px; padding: 6px 12px;")
        h_layout = QHBoxLayout(header_frame)
        h_layout.setContentsMargins(4, 2, 4, 2)
        
        title_box = QVBoxLayout()
        title = QLabel("RK3588 多源智能感知大屏")
        title.setFont(QFont("sans-serif", 15, QFont.Bold))
        title.setStyleSheet("color: #00f2fe;")
        subtitle = QLabel("LiDAR 激光雷达 + Astra S 深度相机 + C-RTK 2HP 卫星定位定向")
        subtitle.setStyleSheet("font-size: 11px; color: #94a3b8;")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        h_layout.addLayout(title_box)

        # 设备状态徽章
        self.dev_badge = QLabel("RTK 等待数据")
        self.dev_badge.setStyleSheet("background: rgba(16, 185, 129, 0.2); border: 1px solid #10b981; color: #10b981; font-weight: bold; border-radius: 6px; padding: 6px 10px; font-size: 12px;")
        h_layout.addWidget(self.dev_badge)
        right_panel.addWidget(header_frame)

        # 2. 相机画中画视频监视窗
        cam_card = QFrame()
        cam_card.setStyleSheet("background: rgba(15, 23, 42, 0.95); border: 1px solid rgba(0, 242, 254, 0.3); border-radius: 12px; padding: 10px;")
        cam_layout = QVBoxLayout(cam_card)
        cam_layout.setContentsMargins(8, 8, 8, 8)
        cam_layout.setSpacing(8)

        # 视频窗口顶栏信息
        cam_top = QHBoxLayout()
        cam_title = QLabel("📷 Astra S 3D 深度相机实时流")
        cam_title.setFont(QFont("sans-serif", 12, QFont.Bold))
        cam_title.setStyleSheet("color: #38bdf8;")
        cam_top.addWidget(cam_title)

        self.cam_dist_badge = QLabel("中心物距: -- mm")
        self.cam_dist_badge.setStyleSheet("color: #fbbf24; font-weight: bold; font-size: 12px; background: rgba(251, 191, 36, 0.15); border: 1px solid rgba(251, 191, 36, 0.3); border-radius: 4px; padding: 3px 8px;")
        cam_top.addWidget(self.cam_dist_badge)

        self.cam_fps_badge = QLabel("-- FPS")
        self.cam_fps_badge.setStyleSheet("color: #34d399; font-weight: bold; font-size: 11px; background: rgba(52, 211, 153, 0.15); border-radius: 4px; padding: 3px 6px;")
        cam_top.addWidget(self.cam_fps_badge)
        cam_layout.addLayout(cam_top)

        # 视频画面显示容器 (固定比例自适应缩放)
        self.video_box = QLabel("正在接收相机画面...")
        self.video_box.setAlignment(Qt.AlignCenter)
        self.video_box.setMinimumHeight(280)
        self.video_box.setMinimumWidth(1)
        self.video_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.video_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_box.setStyleSheet("background: #020617; border: 1px solid rgba(255, 255, 255, 0.1); border-radius: 8px; color: #64748b; font-size: 13px;")
        cam_layout.addWidget(self.video_box, 1)

        # 视频模式触控切换条 (3 档一键切换)
        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(8)
        
        self.btn_ai = QPushButton("🤖 AI 3D 测距")
        self.btn_ai.setFont(QFont("sans-serif", 11, QFont.Bold))
        self.btn_ai.clicked.connect(lambda: self.switch_cam_mode('ai'))
        mode_layout.addWidget(self.btn_ai)

        self.btn_rgb = QPushButton("🌈 彩色实景")
        self.btn_rgb.setFont(QFont("sans-serif", 11, QFont.Bold))
        self.btn_rgb.clicked.connect(lambda: self.switch_cam_mode('rgb'))
        mode_layout.addWidget(self.btn_rgb)

        self.btn_depth = QPushButton("深度距离图 · Z / m")
        self.btn_depth.setFont(QFont("sans-serif", 11, QFont.Bold))
        self.btn_depth.clicked.connect(lambda: self.switch_cam_mode('depth'))
        mode_layout.addWidget(self.btn_depth)
        
        cam_layout.addLayout(mode_layout)
        depth_controls=QHBoxLayout()
        self.depth_hint=QLabel('深度距离图 · 光轴深度 Z / m · 暗灰表示无效或超范围')
        self.depth_hint.setStyleSheet('font-size:12px; color:#c5d2e0; background:transparent; border:none; padding:2px;')
        depth_controls.addWidget(self.depth_hint,1)
        self.depth_range=QComboBox()
        for label,value in [('近景 0.2–2.0 m',2000),('室内 0.2–4.5 m',4500),('全程 0.2–5.5 m',5500)]:
            self.depth_range.addItem(label,value)
        self.depth_range.setStyleSheet('font-size:12px; color:#e2e8f0; background:#162435; border:1px solid #385167; border-radius:6px; padding:5px;')
        self.depth_range.currentIndexChanged.connect(self.change_depth_range)
        depth_controls.addWidget(self.depth_range)
        cam_layout.addLayout(depth_controls)
        right_panel.addWidget(cam_card, 1)
        self.update_mode_buttons()

        # 3. 四向避障仪表网格 (紧凑横排)
        grid = QGridLayout()
        grid.setSpacing(8)
        self.front_lbl = self._create_card("前向 (0°)", "-- m")
        self.left_lbl = self._create_card("左向 (90°)", "-- m")
        self.right_lbl = self._create_card("右向 (270°)", "-- m")
        self.back_lbl = self._create_card("后向 (180°)", "-- m")

        grid.addWidget(self.front_lbl, 0, 0)
        grid.addWidget(self.back_lbl, 0, 1)
        grid.addWidget(self.left_lbl, 0, 2)
        grid.addWidget(self.right_lbl, 0, 3)
        right_panel.addLayout(grid)

        # 4. 危险状态横幅
        self.alarm_box = QLabel("雷达等待数据 · 环境状态未知")
        self.alarm_box.setAlignment(Qt.AlignCenter)
        self.alarm_box.setFont(QFont("sans-serif", 12, QFont.Bold))
        self.alarm_box.setStyleSheet("padding: 9px; border-radius: 8px; background: rgba(16, 185, 129, 0.2); border: 1px solid #10b981; color: #10b981;")
        right_panel.addWidget(self.alarm_box)

        # 5. 底部快捷控制栏 (雷达量程 + 全屏/退出)
        bottom_bar = QHBoxLayout()
        bottom_bar.setSpacing(8)

        self.range_buttons = {}
        for r in [3, 5, 8, 12]:
            b = QPushButton(f"{r}m")
            b.setFont(QFont("sans-serif", 10, QFont.Bold))
            b.setStyleSheet("background: rgba(0, 242, 254, 0.12); border: 1px solid rgba(0, 242, 254, 0.35); color: #fff; padding: 7px 12px; border-radius: 6px;")
            b.setCheckable(True)
            b.setChecked(r == 5)
            b.setStyleSheet('QPushButton {background:#162435; color:#d0deea; border:1px solid #385167; border-radius:6px; padding:8px;} QPushButton:checked {background:#155e75; border:2px solid #61d8e8; color:white;}')
            self.range_buttons[r] = b
            b.clicked.connect(lambda _, val=r: self.set_lidar_range(val))
            bottom_bar.addWidget(b)

        self.btn_fs = QPushButton("🖥️ 退出全屏")
        self.btn_fs.setFont(QFont("sans-serif", 11, QFont.Bold))
        self.btn_fs.setStyleSheet("background: #0284c7; color: white; padding: 8px 14px; border-radius: 6px;")
        self.btn_fs.clicked.connect(self.toggle_fullscreen)
        bottom_bar.addWidget(self.btn_fs)

        btn_exit = QPushButton("❌ 退出")
        btn_exit.setFont(QFont("sans-serif", 11, QFont.Bold))
        btn_exit.setStyleSheet("background: #dc2626; color: white; padding: 8px 14px; border-radius: 6px;")
        btn_exit.clicked.connect(self.close)
        bottom_bar.addWidget(btn_exit)

        right_panel.addLayout(bottom_bar)
        main_layout.addLayout(right_panel, 62)

    def _create_card(self, title, val):
        frame = QFrame()
        frame.setStyleSheet("background: rgba(16, 24, 40, 0.85); border: 1px solid rgba(0, 242, 254, 0.25); border-radius: 8px; padding: 6px;")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(4, 4, 4, 4)
        t = QLabel(title)
        t.setStyleSheet("font-size: 11px; color: #94a3b8; font-weight: bold;")
        v = QLabel(val)
        v.setStyleSheet("font-size: 17px; font-weight: bold; color: #00f2fe; font-family: monospace;")
        layout.addWidget(t)
        layout.addWidget(v)
        frame.val_label = v
        return frame

    def open_cors_dialog(self):
        self.cors_dialog.load_config()
        self.cors_dialog.exec_()

    def create_rtk_card(self):
        card = QFrame()
        card.setStyleSheet("""
            QFrame {
                background: rgba(15, 23, 42, 0.95);
                border: 1px solid rgba(0, 242, 254, 0.35);
                border-radius: 10px;
                padding: 6px 10px;
            }
        """)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # 顶栏：标题与左右卫星独立固定状态胶囊 + CORS配置按钮 (不要显示统一固定解)
        top_layout = QHBoxLayout()
        title = QLabel("🛰️ C-RTK 2HP 定位定向")
        title.setFont(QFont("sans-serif", 11, QFont.Bold))
        title.setStyleSheet("color: #00f2fe;")
        top_layout.addWidget(title)

        self.rtk_ant1_badge = QLabel("左: 搜星中")
        self.rtk_ant1_badge.setStyleSheet("color: #fbbf24; font-weight: bold; font-size: 11px; background: rgba(251, 191, 36, 0.15); border: 1px solid rgba(251, 191, 36, 0.35); border-radius: 4px; padding: 2px 6px;")
        top_layout.addWidget(self.rtk_ant1_badge)

        self.rtk_ant2_badge = QLabel("右: 搜星中")
        self.rtk_ant2_badge.setStyleSheet("color: #fbbf24; font-weight: bold; font-size: 11px; background: rgba(251, 191, 36, 0.15); border: 1px solid rgba(251, 191, 36, 0.35); border-radius: 4px; padding: 2px 6px;")
        top_layout.addWidget(self.rtk_ant2_badge)

        self.rtk_cors_btn = QPushButton("⚙️ CORS 设置")
        self.rtk_cors_btn.setStyleSheet("""
            QPushButton {
                background: rgba(0, 242, 254, 0.18);
                border: 1px solid #00f2fe;
                border-radius: 4px;
                color: #00f2fe;
                font-size: 10px;
                font-weight: bold;
                padding: 2px 8px;
            }
            QPushButton:hover {
                background: rgba(0, 242, 254, 0.35);
            }
        """)
        self.rtk_cors_btn.clicked.connect(self.open_cors_dialog)
        top_layout.addWidget(self.rtk_cors_btn)
        layout.addLayout(top_layout)

        # 六格指标 (3x2 网格)
        grid = QGridLayout()
        grid.setSpacing(6)

        self.rtk_sats_ant1_box = self._create_sub_metric("左天线 (主 ANT1)", "-- 颗")
        self.rtk_sats_ant2_box = self._create_sub_metric("右天线 (辅 ANT2)", "-- 颗")
        self.rtk_sats_sky_box = self._create_sub_metric("机体天空左右分布", "左: -- | 右: --")
        self.rtk_heading_box = self._create_sub_metric("双天线航向 / 共视", "--°")
        self.rtk_coord_box = self._create_sub_metric("空间经纬坐标", "--, --")
        self.rtk_alt_box = self._create_sub_metric("海拔 / HDOP", "-- m")

        grid.addWidget(self.rtk_sats_ant1_box, 0, 0)
        grid.addWidget(self.rtk_sats_ant2_box, 0, 1)
        grid.addWidget(self.rtk_sats_sky_box, 1, 0)
        grid.addWidget(self.rtk_heading_box, 1, 1)
        grid.addWidget(self.rtk_coord_box, 2, 0)
        grid.addWidget(self.rtk_alt_box, 2, 1)
        layout.addLayout(grid)

        return card

    def _create_sub_metric(self, title, val):
        frame = QFrame()
        frame.setStyleSheet("background: rgba(11, 17, 30, 0.85); border: 1px solid rgba(0, 242, 254, 0.15); border-radius: 6px; padding: 4px 6px;")
        l = QVBoxLayout(frame)
        l.setContentsMargins(2, 2, 2, 2)
        l.setSpacing(2)
        t = QLabel(title)
        t.setStyleSheet("font-size: 10px; color: #94a3b8; font-weight: bold;")
        v = QLabel(val)
        v.setStyleSheet("font-size: 11px; font-weight: bold; color: #e2e8f0; font-family: monospace;")
        l.addWidget(t)
        l.addWidget(v)
        frame.val_lbl = v
        return frame

    def on_rtk_data(self, data):
        self.canvas.set_rtk(data)
        sats_used = data.get('sats_used', 0)
        sats_tracked = data.get('sats_tracked', 0)
        ant1_used = data.get('ant1_used', sats_used)
        ant1_trk = data.get('ant1_tracked', sats_tracked)
        ant2_used = data.get('ant2_used', sats_used)
        ant2_trk = data.get('ant2_tracked', sats_tracked)
        common_sats = data.get('heading_sats_common', 0)
        sats_left = data.get('sats_left', 0)
        sats_right = data.get('sats_right', 0)
        fix_str = data.get('fix_type_str', '未定位')
        fix_q = data.get('fix_quality', 0)
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        alt = data.get('alt', 0.0)
        hdop = data.get('hdop', 99.0)
        has_heading = data.get('has_heading', False)
        heading = data.get('heading', 0.0)

        # 左右天线单独固定情况：单点 / 浮点 / 固定 (不显示统一固定解)
        ant1_fix = data.get('ant1_fix', '')
        if not ant1_fix:
            ant1_fix = "固定" if fix_q == 4 else ("浮点" if fix_q == 5 else ("单点" if fix_q > 0 else "搜星中"))
        ant2_fix = data.get('ant2_fix', '')
        if not ant2_fix:
            h_stat = str(data.get('heading_type', '')).upper()
            if 'INT' in h_stat or 'FIX' in h_stat:
                ant2_fix = "固定"
            elif 'FLOAT' in h_stat:
                ant2_fix = "浮点"
            elif ant2_used >= 4 or has_heading:
                ant2_fix = "单点"
            else:
                ant2_fix = "搜星中"

        # 更新左天线独立状态胶囊
        if ant1_fix == "固定":
            self.rtk_ant1_badge.setText("💎 左: 固定")
            self.rtk_ant1_badge.setStyleSheet("color: #10b981; font-weight: bold; font-size: 11px; background: rgba(16, 185, 129, 0.2); border: 1px solid #10b981; border-radius: 4px; padding: 2px 6px;")
        elif ant1_fix == "浮点":
            self.rtk_ant1_badge.setText("🟡 左: 浮点")
            self.rtk_ant1_badge.setStyleSheet("color: #fbbf24; font-weight: bold; font-size: 11px; background: rgba(251, 191, 36, 0.2); border: 1px solid #fbbf24; border-radius: 4px; padding: 2px 6px;")
        elif ant1_fix == "单点":
            self.rtk_ant1_badge.setText("🟢 左: 单点")
            self.rtk_ant1_badge.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px; background: rgba(56, 189, 248, 0.2); border: 1px solid #38bdf8; border-radius: 4px; padding: 2px 6px;")
        else:
            self.rtk_ant1_badge.setText("⚪ 左: 搜星中")
            self.rtk_ant1_badge.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 11px; background: rgba(148, 163, 184, 0.15); border: 1px solid rgba(148, 163, 184, 0.35); border-radius: 4px; padding: 2px 6px;")

        # 更新右天线独立状态胶囊
        if ant2_fix == "固定":
            self.rtk_ant2_badge.setText("💎 右: 固定")
            self.rtk_ant2_badge.setStyleSheet("color: #10b981; font-weight: bold; font-size: 11px; background: rgba(16, 185, 129, 0.2); border: 1px solid #10b981; border-radius: 4px; padding: 2px 6px;")
        elif ant2_fix == "浮点":
            self.rtk_ant2_badge.setText("🟡 右: 浮点")
            self.rtk_ant2_badge.setStyleSheet("color: #fbbf24; font-weight: bold; font-size: 11px; background: rgba(251, 191, 36, 0.2); border: 1px solid #fbbf24; border-radius: 4px; padding: 2px 6px;")
        elif ant2_fix == "单点":
            self.rtk_ant2_badge.setText("🟢 右: 单点")
            self.rtk_ant2_badge.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px; background: rgba(56, 189, 248, 0.2); border: 1px solid #38bdf8; border-radius: 4px; padding: 2px 6px;")
        else:
            self.rtk_ant2_badge.setText("⚪ 右: 搜星中")
            self.rtk_ant2_badge.setStyleSheet("color: #94a3b8; font-weight: bold; font-size: 11px; background: rgba(148, 163, 184, 0.15); border: 1px solid rgba(148, 163, 184, 0.35); border-radius: 4px; padding: 2px 6px;")

        # 更新六格卡片中左右天线的单点/浮点/固定详情
        self.rtk_sats_ant1_box.val_lbl.setText(f"[{ant1_fix}] 解算 {ant1_used} / 跟踪 {ant1_trk}")
        self.rtk_sats_ant2_box.val_lbl.setText(f"[{ant2_fix}] 解算 {ant2_used} / 跟踪 {ant2_trk}")
        self.rtk_sats_sky_box.val_lbl.setText(f"◀ 左 {sats_left} 颗 | 右 {sats_right} 颗 ▶")

        if has_heading:
            self.rtk_heading_box.val_lbl.setText(f"{heading:.1f}° (共视 {common_sats}颗)")
            self.rtk_heading_box.val_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #10b981; font-family: monospace;")
        else:
            self.rtk_heading_box.val_lbl.setText("未锁定 (需双天线对空)")
            self.rtk_heading_box.val_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8; font-family: monospace;")

        if fix_q > 0 and (lat != 0 or lon != 0):
            self.rtk_coord_box.val_lbl.setText(f"{lat:.6f}°N, {lon:.6f}°E")
        else:
            self.rtk_coord_box.val_lbl.setText("等待定位解算...")

        cors = data.get('cors', {})
        cors_conn = cors.get('connected', False)
        cors_str = "已连" if cors_conn else "未连"
        self.rtk_alt_box.val_lbl.setText(f"{alt:.1f}m (HD:{hdop:.1f}) | CORS:{cors_str}")

        # 联动弹窗状态
        if hasattr(self, 'cors_dialog'):
            self.cors_dialog.update_cors_status(cors)

        # 顶部标题栏徽章联动 (单独显示左右天线单点/浮点/固定)
        self.dev_badge.setText(f"🟢 雷达 | 🟢 3D相机 | 🛰️ 左天线: {ant1_fix}({ant1_used}星) · 右天线: {ant2_fix}({ant2_used}星)")

    def _note_fps(self):
        now = time.time()
        self._fps_times.append(now)
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        fps = len(self._fps_times)
        self.cam_fps_badge.setText(f"{fps} FPS")

    def switch_cam_mode(self, mode):
        self.cam_mode = mode
        self.ros_thread.display_mode = mode
        self.depth_range.setEnabled(mode=='depth')
        self._depth_shown=None
        if mode=='depth':
            self.latest_depth_pixmap=None
            self.video_box.clear()
            self.video_box.setText('等待新的深度帧…')
        self._fps_times = []
        self.update_mode_buttons()
        if mode == 'ai':
            if self.latest_ai_pixmap:
                self.video_box.setPixmap(self.latest_ai_pixmap)
            self.cam_dist_badge.setText(getattr(self, 'ai_badge_text', 'AI 空间目标检测就绪'))
        elif mode == 'rgb':
            if self.latest_rgb_pixmap:
                self.video_box.setPixmap(self.latest_rgb_pixmap)
            self.cam_dist_badge.setText("🌈 彩色模式: 实时实景")
        elif mode == 'depth':
            if self.latest_depth_pixmap:
                self.video_box.setPixmap(self.latest_depth_pixmap)
            self._set_depth_badge(self.center_depth_mm, getattr(self, 'heat_near_mm', 0), getattr(self, 'heat_far_mm', 0))

    def update_mode_buttons(self):
        active_style = "background: #0284c7; border: 1px solid #38bdf8; color: white; padding: 7px; border-radius: 6px; font-weight: bold;"
        inactive_style = "background: rgba(255, 255, 255, 0.05); border: 1px solid rgba(255, 255, 255, 0.2); color: #94a3b8; padding: 7px; border-radius: 6px;"
        self.btn_ai.setStyleSheet(active_style if self.cam_mode == 'ai' else inactive_style)
        self.btn_rgb.setStyleSheet(active_style if self.cam_mode == 'rgb' else inactive_style)
        self.btn_depth.setStyleSheet(active_style if self.cam_mode == 'depth' else inactive_style)

    def on_targets_data(self, json_str):
        try:
            items = json.loads(json_str)
            self.last_targets = items if isinstance(items, list) else []
            if items:
                closest = min(items, key=lambda t: t.get('distance', 99))
                label = closest.get('label', 'Obj')
                d = closest.get('distance', 0)
                x = closest.get('x', 0)
                z = closest.get('z', 0)
                self.ai_badge_text = f"🎯 AI锁定: [{label}] {d:.2f}m (X:{x:+.2f} Z:{z:.2f}m)"
            else:
                self.ai_badge_text = "AI 空间检测: 未发现目标"
            if self.cam_mode == 'ai':
                self.cam_dist_badge.setText(self.ai_badge_text)
        except Exception:
            pass

    def _scale_camera_pixmap(self, qimage):
        pix = QPixmap.fromImage(qimage)
        box = self.video_box.size()
        if box.width() < 2 or box.height() < 2:
            return pix
        return pix.scaled(box, Qt.KeepAspectRatio, Qt.SmoothTransformation)

    def change_depth_range(self, index):
        self._depth_shown=None

    def refresh_depth(self):
        if self.cam_mode!='depth':return
        record=self.ros_thread.latest_depth
        now=time.monotonic()
        if record is None or now-record['received']+record['age']>.6:
            self.video_box.clear();self.video_box.setText('深度数据中断 · 等待新帧')
            self.cam_dist_badge.setText('测距 -- · 深度数据未就绪')
            self.depth_hint.setText('当前无新深度帧；旧颜色与距离已清除')
            self.cam_fps_badge.setText('-- FPS')
            self._depth_shown=None
            return
        if record is self._depth_shown:return
        self._depth_shown=record
        depth=record['array'];far=float(self.depth_range.currentData())
        colored,near,far=render_depth_heatmap(depth,near_mm=200,far_mm=far)
        h,w=colored.shape[:2]
        qimage=QImage(colored.data,w,h,w*3,QImage.Format_RGB888).copy()
        quality=depth_quality(depth,far)
        self.depth_hint.setText(f"深度 Z/m · 有效 {quality['valid']:.0%} · 无效 {quality['missing']:.0%} · 超范围 {quality['outside']:.0%} · 色标饱和 {quality['clipped']:.0%}")
        self.on_depth_frame(qimage,depth_center_mm(depth),int(near),int(far))

    def _set_depth_badge(self, center_val_mm, near_mm=0, far_mm=0):
        text=f'中心区域 Z {center_val_mm/1000:.2f} m' if center_val_mm>0 else '中心区域 -- · 无回波或跨物体边缘'
        self.cam_dist_badge.setText(text)

    def on_ai_frame(self, qimage):
        # AI 标注图仅作备份；实时画面走 RGB 叠加，避免被 3FPS 推理拖慢
        self.latest_ai_pixmap = self._scale_camera_pixmap(qimage)

    def on_rgb_frame(self, qimage):
        if self.cam_mode == 'ai' and self.last_targets:
            img = qimage.copy()
            painter = QPainter(img)
            painter.setRenderHint(QPainter.Antialiasing, False)
            for t in self.last_targets:
                x1, y1 = int(t.get('x1', 8)), int(t.get('y1', 8))
                x2, y2 = int(t.get('x2', x1 + 40)), int(t.get('y2', y1 + 40))
                label = t.get('label', 'Obj')
                d = t.get('distance')
                text = f"[{label}] {d:.2f}m" if d is not None else f"[{label}]"
                painter.setPen(QPen(QColor(0, 242, 254), 2))
                painter.drawRect(x1, y1, max(1, x2 - x1), max(1, y2 - y1))
                painter.setFont(QFont("sans-serif", 11, QFont.Bold))
                painter.drawText(x1 + 4, max(16, y1 - 6), text)
            painter.end()
            qimage = img
        scaled_pix = self._scale_camera_pixmap(qimage)
        self.latest_rgb_pixmap = scaled_pix
        if self.cam_mode in ('rgb', 'ai'):
            self.video_box.setPixmap(scaled_pix)
            self._note_fps()

    def on_depth_frame(self, qimage, center_val_mm, near_mm=0, far_mm=0):
        self.center_depth_mm = center_val_mm
        self.heat_near_mm = near_mm
        self.heat_far_mm = far_mm
        available=QSize(max(1,self.video_box.width()-24),max(1,self.video_box.height()-24))
        scaled_pix = QPixmap.fromImage(qimage).scaled(available, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.latest_depth_pixmap = scaled_pix
        if self.cam_mode == 'depth':
            self._set_depth_badge(center_val_mm, near_mm, far_mm)
            self.video_box.setPixmap(scaled_pix)
            self._note_fps()

    def set_lidar_range(self, meters):
        self.canvas.set_range(meters)
        for value, button in self.range_buttons.items():
            button.setChecked(value == meters)

    def toggle_lidar_pause(self, checked):
        self.canvas.paused = checked
        self.pause_lidar.setText('继续点云' if checked else '暂停点云')
        if not checked and self._shown_scan:
            self.canvas.set_scan(self._shown_scan)
        self.canvas.update()

    def refresh_lidar(self):
        data = self.ros_thread.latest_scan
        now = time.monotonic()
        if data is not None and data is not self._shown_scan:
            self._shown_scan = data
            self._last_scan_received = data['received']
            self.on_scan_data(data)
        age = now-self._last_scan_received if self._last_scan_received else math.inf
        if age > 0.5 or (data and age+data.get('source_age', 0) > 0.65):
            self.canvas.set_scan({'stale': True})
            self.lidar_status.setText('N10P 数据中断 · 请检查 USB / 雷达供电')
            for card in [self.front_lbl, self.left_lbl, self.right_lbl, self.back_lbl]:
                card.val_label.setText('--')
            self.alarm_box.setText('雷达数据过期 · 环境状态未知')
            self.alarm_box.setStyleSheet('padding:9px; background:#3d301d; color:#ffc66a; border-radius:8px;')
        elif data:
            period = data.get('scan_time', 0)
            hz = 1/period if period > 0 else 0
            delay_ms = (age + data.get('source_age', 0)) * 1000
            self.lidar_status.setText(f"实时  {hz:.1f} Hz   ·   有效 {data['count']}/{data['total']} 点   ·   帧龄 {delay_ms:.0f} ms")

    def on_scan_data(self, data):
        self.canvas.set_scan(data)
        for name, card in [('front', self.front_lbl), ('left', self.left_lbl),
                           ('right', self.right_lbl), ('back', self.back_lbl)]:
            card.val_label.setText(f"{data[name]:.2f} m" if math.isfinite(data[name]) else '--')
        if data['count'] == 0:
            text, color, bg = '本圈无有效回波 · 环境状态未知', '#ffc66a', '#3d301d'
        elif data['min'] < 0.6:
            text, color, bg = f"近距回波 {data['min']:.2f} m · 小于 0.6 m", '#ff8790', '#41232d'
        elif data['min'] < 1.2:
            text, color, bg = f"注意近障 {data['min']:.2f} m · 小于 1.2 m", '#ffc66a', '#3d301d'
        else:
            text, color, bg = f"最近有效回波 {data['min']:.2f} m · 当前未见近距回波", '#73dcc3', '#173a36'
        self.alarm_box.setText(text)
        self.alarm_box.setStyleSheet(f'padding:9px; border-radius:8px; background:{bg}; color:{color}; font-weight:bold;')

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.btn_fs.setText("🖥️ 全屏显示")
        else:
            self.showFullScreen()
            self.btn_fs.setText("🖥️ 退出全屏")

def main():
    app = QApplication(sys.argv)
    win = BoardRadarMainWindow()
    win.showFullScreen()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
