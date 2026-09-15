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
import threading
import atexit
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
from std_msgs.msg import String, Float32
import numpy as np
import cv2
cv2.setNumThreads(2)
cv2.ocl.setUseOpenCL(False)

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


def prepare_display_depth(raw_mm):
    """Independent display copy. None of these values feed metric readouts.

    3x3 bilateral averaging suppresses small within-surface noise. Fill only
    enclosed components <=9 pixels with >=5 valid neighbors on one surface.
    The returned estimate mask stays distinct from the original validity mask.
    """
    raw=np.asarray(raw_mm,dtype=np.float32)
    valid=np.isfinite(raw)&(raw>=DEPTH_VALID_MIN_MM)&(raw<=DEPTH_VALID_MAX_MM)
    clean=np.where(valid,raw,0).astype(np.float32)
    h,w=raw.shape
    # Native OpenCV implementation; zero samples are separated by >=200mm,
    # many range sigmas, then invalid locations are masked again below.
    display=cv2.bilateralFilter(clean,3,35.,1.0)
    # Connected-component gate excludes large holes and frame-edge voids.
    count,labels,stats,_=cv2.connectedComponentsWithStats((~valid).astype(np.uint8),8)
    small=np.zeros(count,dtype=bool)
    if count>1:
        area=stats[:,cv2.CC_STAT_AREA];xs=stats[:,cv2.CC_STAT_LEFT];ys=stats[:,cv2.CC_STAT_TOP]
        widths=stats[:,cv2.CC_STAT_WIDTH];heights=stats[:,cv2.CC_STAT_HEIGHT]
        small=(area<=9)&(xs>0)&(ys>0)&(xs+widths<w)&(ys+heights<h);small[0]=False
    eligible=small[labels]&(~valid)
    kernel=np.ones((3,3),np.uint8)
    neighbors=cv2.boxFilter(valid.astype(np.float32),-1,(3,3),normalize=False)
    total=cv2.boxFilter(clean,-1,(3,3),normalize=False)
    local_min=cv2.erode(np.where(valid,clean,100000).astype(np.float32),kernel)
    local_max=cv2.dilate(clean,kernel)
    estimated=eligible&(neighbors>=5)&(local_max-local_min<=100)
    display[estimated]=total[estimated]/neighbors[estimated]
    display[~(valid|estimated)]=0
    return display,valid,estimated


def render_depth_display(raw_mm,near_mm=200.,far_mm=2000.,smooth=True):
    if not smooth:
        image,near,far=render_depth_heatmap(raw_mm,near_mm=near_mm,far_mm=far_mm)
        return image,dict(estimated_pixels=0,mode='raw')
    if not 0<=near_mm<far_mm<=DEPTH_VALID_MAX_MM:raise ValueError('invalid metric range')
    display,measured,estimated=prepare_display_depth(raw_mm)
    supported=measured|estimated;h,w=display.shape
    normalized=np.clip((display-near_mm)/(far_mm-near_mm),0,1)*255
    colors=HEAT_LUT[np.rint(normalized).astype(np.uint8)]
    # Qt interpolates this display image at the actual viewport size, avoiding
    # a wasteful full-frame 2x intermediate; the metric field is not resampled.
    out=colors.copy()
    coverage=supported
    out[~coverage]=HEAT_BG_RGB
    est=estimated
    yy,xx=np.ogrid[:h,:w]
    hatch=est&(((xx+yy)%3)==0)
    out[hatch]=(210,215,225)  # visible estimate marking, never counted as measured
    legend=_draw_heatmap_legend(np.empty((0,w,3),np.uint8),near_mm,far_mm)
    return np.vstack([out,legend]),dict(estimated_pixels=int(estimated.sum()),mode='smooth')


class DepthDisplayWorker:
    """Latest-only background renderer; never blocks Qt paint/input handling."""
    def __init__(self):
        self.request=None;self.output=None;self.event=threading.Event()
        self.running=True
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()
        atexit.register(self.stop)

    def stop(self):
        self.running=False;self.event.set()
        if threading.current_thread() is not self.thread:self.thread.join(timeout=3.)

    def submit(self,record,far,smooth):
        key=(id(record),far,smooth)
        if self.request is None or self.request[0]!=key:
            self.request=(key,record,far,smooth);self.event.set()

    def run(self):
        previous=None
        while self.running:
            self.event.wait(.1);self.event.clear()
            request=self.request
            if request is None or request is previous:continue
            previous=request
            key,record,far,smooth=request
            try:
                started=time.monotonic()
                colored,stats=render_depth_display(record['array'],200.,far,smooth)
                self.output=dict(record=record,far=far,smooth=smooth,colored=colored,
                                 stats=stats,render_ms=(time.monotonic()-started)*1000)
            except Exception:
                self.output=None


class ROSThread(QThread):
    scan_signal = pyqtSignal(dict)
    rgb_signal = pyqtSignal(QImage)
    depth_signal = pyqtSignal(QImage, int, int, int)
    ai_signal = pyqtSignal(QImage)
    targets_signal = pyqtSignal(str)
    rtk_signal = pyqtSignal(dict)
    voltage_signal = pyqtSignal(float)

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

        def voltage_callback(msg):
            try:
                self.voltage_signal.emit(float(msg.data))
            except Exception:
                pass

        def rtk_callback(msg):
            try:
                self.rtk_signal.emit(json.loads(msg.data))
            except Exception:
                pass

        node.create_subscription(String, '/rtk/status', rtk_callback, 10)
        node.create_subscription(Float32, '/voltage', voltage_callback, 10)
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
        self.voltage = None
        self.battery_pct = None
        self.setMinimumSize(360, 340)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_voltage(self, v):
        try:
            self.voltage = float(v)
            self.battery_pct = max(0, min(100, int(round((self.voltage - 21.0) / 4.2 * 100))))
        except Exception:
            pass
        self.update()

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
            self._grid.fill(QColor('#182D3D'))
            g = QPainter(self._grid)
            g.setRenderHint(QPainter.Antialiasing)
            g.setFont(QFont('sans-serif', 10))
            for i in range(1, 6):
                rr = radius * i / 5
                g.setPen(QPen(QColor('#7892A7' if i == 5 else '#355368'), 1,
                              Qt.SolidLine if i == 5 else Qt.DashLine))
                g.drawEllipse(QPointF(cx, cy), rr, rr)
                if i < 5:
                    g.setPen(QColor('#B8CAD8'))
                    g.drawText(int(cx + 9), int(cy - rr - 5), f'{self.max_range*i/5:g} m')
            for deg in range(0, 360, 30):
                angle = math.radians(deg)
                x, y = project_point(angle, 1, cx, cy, radius)
                g.setPen(QPen(QColor('#668397' if deg % 90 == 0 else '#2D495D'), 1))
                g.drawLine(QPointF(cx, cy), QPointF(x, y))
            g.setPen(QColor('#CEE0EB'))
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
        for color, points in zip(['#FF7A86', '#F6CA78', '#67DFD8'], groups):
            painter.setPen(QPen(QColor(color), 3.8, Qt.SolidLine, Qt.RoundCap))
            if points:
                painter.drawPoints(*points)
        painter.setPen(QPen(QColor('#F0F6F8'), 2))
        painter.drawLine(QPointF(cx-5, cy+4), QPointF(cx, cy-7))
        painter.drawLine(QPointF(cx, cy-7), QPointF(cx+5, cy+4))
        painter.drawLine(QPointF(cx-5, cy+4), QPointF(cx+5, cy+4))
        if self.stale or self.paused:
            painter.setPen(QColor('#F6CA78'))
            painter.setFont(QFont('sans-serif', 12, QFont.Bold))
            painter.drawText(QRectF(0, h/2+20, w, 30), Qt.AlignCenter,
                             '数据中断 · 等待新扫描' if self.stale else '点云已暂停 · 测距仍在更新')
        self.draw_battery_hud(painter, w, h)
        painter.end()

    def draw_battery_hud(self, painter, w, h):
        card_w, card_h = 168, 44
        margin_x, margin_y = 14, 12
        card_x = w - card_w - margin_x
        card_y = margin_y

        painter.save()
        card_rect = QRectF(card_x, card_y, card_w, card_h)
        painter.setPen(QPen(QColor(53, 83, 104, 200), 1.2))
        painter.setBrush(QBrush(QColor(16, 28, 40, 220)))
        painter.drawRoundedRect(card_rect, 8.0, 8.0)

        if self.voltage is not None and self.voltage > 0:
            v = self.voltage
            pct = self.battery_pct if self.battery_pct is not None else max(0, min(100, int(round((v - 21.0) / 4.2 * 100))))
            if pct > 50:
                theme_color = QColor('#4ADE80')
            elif pct > 20:
                theme_color = QColor('#F6CA78')
            else:
                theme_color = QColor('#FF7A86')
            pct_text = f"{pct}%"
            volt_text = f"{v:.2f} V"
        else:
            pct = 0
            theme_color = QColor('#7892A7')
            pct_text = "-- %"
            volt_text = "等待电压"

        # Battery icon shell
        icon_x = card_x + 12
        icon_y = card_y + 14
        icon_w = 26
        icon_h = 15
        shell_rect = QRectF(icon_x, icon_y, icon_w, icon_h)
        painter.setPen(QPen(QColor('#9EB2C2'), 1.5))
        painter.setBrush(QBrush(QColor(24, 45, 61, 150)))
        painter.drawRoundedRect(shell_rect, 2.5, 2.5)

        # Terminal bump
        knob_rect = QRectF(icon_x + icon_w, icon_y + 4.5, 2.5, 6.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor('#9EB2C2')))
        painter.drawRoundedRect(knob_rect, 1.0, 1.0)

        # Fill bar
        if pct > 0:
            pad = 2.0
            max_fill_w = icon_w - 2 * pad
            fill_w = max(1.5, max_fill_w * (pct / 100.0))
            fill_rect = QRectF(icon_x + pad, icon_y + pad, fill_w, icon_h - 2 * pad)
            painter.setBrush(QBrush(theme_color))
            painter.drawRoundedRect(fill_rect, 1.5, 1.5)

        # Percentage text
        painter.setFont(QFont('sans-serif', 12, QFont.Bold))
        painter.setPen(theme_color)
        painter.drawText(QRectF(card_x + 48, card_y + 4, 60, 20), Qt.AlignLeft | Qt.AlignVCenter, pct_text)

        # Subtitle tag
        painter.setFont(QFont('sans-serif', 9, QFont.Normal))
        painter.setPen(QColor('#849EB2'))
        painter.drawText(QRectF(card_x + 104, card_y + 5, 52, 18), Qt.AlignRight | Qt.AlignVCenter, "6S 动力")

        # Voltage readout
        painter.setFont(QFont('sans-serif', 11, QFont.Normal))
        painter.setPen(QColor('#D6E5EF'))
        painter.drawText(QRectF(card_x + 48, card_y + 22, 108, 18), Qt.AlignLeft | Qt.AlignVCenter, volt_text)

        painter.restore()

class CorsConfigDialog(QDialog):
    def __init__(self, parent=None, ros_thread=None):
        super().__init__(parent)
        self.ros_thread = ros_thread
        self.setWindowTitle("CORS 厘米级差分设置")
        self.setFixedSize(620, 660)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setStyleSheet("""
            QDialog {
                background: #DFE7ED;
                border: 2px solid #087F83;
                border-radius: 12px;
            }
            QLabel {
                color: #243B50;
                font-family: sans-serif;
            }
            QLineEdit, QComboBox {
                background: #DFE7ED;
                border: 1px solid #CAD8E2;
                border-radius: 6px;
                color: #243B50;
                font-size: 13px;
                padding: 7px 10px;
                font-family: monospace;
            }
            QLineEdit:focus, QComboBox:focus {
                border: 1px solid #087F83;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        # 标题栏
        top_bar = QHBoxLayout()
        title = QLabel("🛰️ CORS 厘米级差分设置")
        title.setFont(QFont("sans-serif", 13, QFont.Bold))
        title.setStyleSheet("color: #087F83;")
        top_bar.addWidget(title)

        btn_close = QPushButton("✕")
        btn_close.setFixedSize(30, 30)
        btn_close.setStyleSheet("background: transparent; color: #607488; font-size: 18px; border: none; font-weight: bold;")
        btn_close.clicked.connect(self.close)
        top_bar.addWidget(btn_close)
        layout.addLayout(top_bar)

        desc = QLabel("注入 RTCM3 差分流消除电离层延迟，左右天线实时独立解算 单点 / 浮点 / 固定。")
        desc.setStyleSheet("color: #60768A; font-size: 11px;")
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
        self.stat_frame.setStyleSheet("background: #DFE7ED; border: 1px solid #B5C4D0; border-radius: 6px; padding: 8px 12px;")
        stat_l = QVBoxLayout(self.stat_frame)
        stat_l.setSpacing(4)
        stat_l.setContentsMargins(4, 4, 4, 4)
        self.lbl_status = QLabel("运行状态: 未启用")
        self.lbl_status.setStyleSheet("color: #8B621A; font-weight: bold; font-size: 11px;")
        self.lbl_rate = QLabel("差分速率: 0.0 KB/s")
        self.lbl_rate.setStyleSheet("color: #60768A; font-size: 11px;")
        stat_l.addWidget(self.lbl_status)
        stat_l.addWidget(self.lbl_rate)
        layout.addWidget(self.stat_frame)

        # 底部按钮
        btn_layout = QHBoxLayout()
        self.btn_save = QPushButton("💾 保存并连接差分")
        self.btn_save.setStyleSheet("""
            QPushButton {
                background: #247451;
                border: none;
                border-radius: 6px;
                color: #ffffff;
                font-weight: bold;
                font-size: 12px;
                padding: 10px 16px;
            }
            QPushButton:hover { background: #087F83; }
        """)
        self.btn_save.clicked.connect(lambda: self.save_config(True))
        btn_layout.addWidget(self.btn_save)

        self.btn_disconnect = QPushButton("⏹ 断开差分")
        self.btn_disconnect.setStyleSheet("""
            QPushButton {
                background: #FFF0F1;
                border: 1px solid #E6ABB2;
                border-radius: 6px;
                color: #B9404A;
                font-weight: bold;
                font-size: 12px;
                padding: 10px 16px;
            }
            QPushButton:hover { background: #FFE5E8; }
        """)
        self.btn_disconnect.clicked.connect(lambda: self.save_config(False))
        btn_layout.addWidget(self.btn_disconnect)

        self.btn_cancel = QPushButton("关闭")
        self.btn_cancel.setStyleSheet("""
            QPushButton {
                background: #EDF2F6;
                border: 1px solid #CAD8E2;
                border-radius: 6px;
                color: #243B50;
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
            self.lbl_status.setStyleSheet("color: #247451; font-weight: bold;" if enabled else "color: #B9404A; font-weight: bold;")
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
        self.lbl_status.setStyleSheet("color: #247451; font-weight: bold;" if conn else "color: #8B621A; font-weight: bold;")
        self.lbl_rate.setText(f"差分速率: {rate} KB/s (累计接收 {bytes_recv} KB)")


LIGHT_STYLE = """
QWidget#perceptionRoot, QDialog { background:#CBD6DF; color:#21394D; }
QLabel { background:transparent; border:none; color:#21394D; }
QFrame#panel { background:#DFE7ED; border:1px solid #B5C4D0; border-radius:14px; }
QPushButton { background:#D5E0E8; color:#425B70; border:1px solid #D8E2EA;
              border-radius:8px; padding:9px 15px; min-height:24px; font-size:15px; }
QPushButton:hover { background:#E5EEF3; border-color:#A8BAC7; }
QPushButton:checked { background:#DFF2F0; color:#086E74; border:1px solid #77B5B3; font-weight:600; }
QPushButton:focus { border:2px solid #087F83; }
QPushButton:disabled { color:#8697A6; background:#D6DFE6; }
QComboBox, QLineEdit { background:#DFE7ED; color:#304C63; border:1px solid #CBD9E4;
                      border-radius:7px; padding:8px 12px; min-height:24px; font-size:14px; }
QComboBox:focus, QLineEdit:focus { border:2px solid #087F83; }
QComboBox QAbstractItemView { background:white; color:#304C63; selection-background-color:#DFF2F0; selection-color:#086E74; }
QToolTip { background:#20394B; color:white; border:none; padding:8px; }
"""

def chip_style(state):
    colors = {'good': ('#E6F4EC', '#247451'), 'warn': ('#FFF3DA', '#8B621A'),
              'muted': ('#C3D1DC', '#5D7183')}
    background, foreground = colors[state]
    return f'background:{background};color:{foreground};font-size:14px;border-radius:8px;padding:9px 12px;'

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
        self.depth_renderer=DepthDisplayWorker()
        self._depth_timer = QTimer(self)
        self._depth_timer.timeout.connect(self.refresh_depth)
        self._depth_timer.start(40)
        self.ros_thread.ai_signal.connect(self.on_ai_frame)
        self.ros_thread.targets_signal.connect(self.on_targets_data)
        self.ros_thread.rtk_signal.connect(self.on_rtk_data)
        self.ros_thread.voltage_signal.connect(self.on_voltage_data)
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
        self.setObjectName('perceptionRoot')
        self.setFont(QFont('Noto Sans CJK SC', 11))
        self.setStyleSheet(LIGHT_STYLE)
        self._camera_received = 0.0
        self._rtk_connected = False
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        root.setSpacing(16)

        header = QHBoxLayout()
        header.setSpacing(16)
        mark = QLabel('RK')
        mark.setFixedSize(52, 52)
        mark.setAlignment(Qt.AlignCenter)
        mark.setStyleSheet('background:#DFF2F0;color:#087F83;border-radius:12px;font-size:22px;font-weight:700;')
        header.addWidget(mark)
        titles = QVBoxLayout()
        titles.setSpacing(3)
        title = QLabel('环境感知')
        title.setStyleSheet('font-size:27px;font-weight:700;color:#193044;')
        subtitle = QLabel('RK3588  /  实时监测')
        subtitle.setStyleSheet('font-size:14px;color:#66788A;')
        titles.addWidget(title)
        titles.addWidget(subtitle)
        header.addLayout(titles)
        header.addStretch()
        self.lidar_badge = QLabel('雷达 · 等待数据')
        self.camera_badge = QLabel('相机 · 等待数据')
        self.dev_badge = QLabel('定位 · 未连接')
        self.battery_badge = QLabel('电量 · -- V')
        for badge in (self.lidar_badge, self.camera_badge, self.dev_badge, self.battery_badge):
            badge.setStyleSheet(chip_style('muted'))
            header.addWidget(badge)
        self.rtk_details_btn = QPushButton('定位详情')
        self.rtk_details_btn.clicked.connect(self.open_rtk_details)
        header.addWidget(self.rtk_details_btn)
        self.btn_fs = QPushButton('窗口模式')
        self.btn_fs.clicked.connect(self.toggle_fullscreen)
        header.addWidget(self.btn_fs)
        self.btn_close = QPushButton('×')
        self.btn_close.setFixedSize(44, 44)
        self.btn_close.setAccessibleName('关闭界面')
        self.btn_close.setToolTip('关闭界面')
        self.btn_close.setStyleSheet('font-size:23px;padding:0;color:#607488;')
        self.btn_close.clicked.connect(self.close)
        header.addWidget(self.btn_close)
        root.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(18)
        camera = QFrame()
        camera.setObjectName('panel')
        cam = QVBoxLayout(camera)
        cam.setContentsMargins(20, 18, 20, 18)
        cam.setSpacing(12)
        top = QHBoxLayout()
        caption = QVBoxLayout()
        caption.setSpacing(3)
        title = QLabel('实时画面')
        title.setStyleSheet('font-size:22px;font-weight:700;')
        subtitle = QLabel('Astra S · 3D 深度相机')
        subtitle.setStyleSheet('font-size:14px;color:#66788A;')
        caption.addWidget(title)
        caption.addWidget(subtitle)
        top.addLayout(caption)
        top.addStretch()
        self.cam_fps_badge = QLabel('-- FPS')
        self.cam_fps_badge.setStyleSheet(chip_style('muted'))
        top.addWidget(self.cam_fps_badge)
        cam.addLayout(top)

        modes = QHBoxLayout()
        modes.setSpacing(8)
        self.btn_ai = QPushButton('AI 目标测距')
        self.btn_rgb = QPushButton('彩色画面')
        self.btn_depth = QPushButton('深度距离图')
        for mode, button in [('ai', self.btn_ai), ('rgb', self.btn_rgb), ('depth', self.btn_depth)]:
            button.setCheckable(True)
            button.setMinimumHeight(44)
            button.clicked.connect(lambda _, m=mode: self.switch_cam_mode(m))
            modes.addWidget(button)
        cam.addLayout(modes)
        self.video_box = QLabel('等待相机画面')
        self.video_box.setAlignment(Qt.AlignCenter)
        self.video_box.setMinimumSize(1, 280)
        self.video_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.video_box.setStyleSheet('background:#C7D4DE;color:#677A8C;border:1px solid #B5C4D0;border-radius:10px;font-size:17px;')
        cam.addWidget(self.video_box, 1)
        self.cam_dist_badge = QLabel('等待目标信息')
        self.cam_dist_badge.setStyleSheet('color:#344E63;font-size:16px;padding:4px 0;')
        self.cam_dist_badge.setWordWrap(True)
        cam.addWidget(self.cam_dist_badge)

        self.depth_controls = QWidget()
        controls = QVBoxLayout(self.depth_controls)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(8)
        options = QHBoxLayout()
        self.depth_range = QComboBox()
        for label, value in [('近景 0.2–2.0 m', 2000), ('室内 0.2–4.5 m', 4500), ('全程 0.2–5.5 m', 5500)]:
            self.depth_range.addItem(label, value)
        self.depth_range.currentIndexChanged.connect(self.change_depth_range)
        self.depth_style = QComboBox()
        self.depth_style.addItem('平滑展示（非测量）', 'smooth')
        self.depth_style.addItem('原始测量图', 'raw')
        self.depth_style.currentIndexChanged.connect(self.change_depth_range)
        options.addWidget(self.depth_range)
        options.addWidget(self.depth_style)
        controls.addLayout(options)
        self.depth_hint = QLabel('光轴深度 Z / m · 暗灰表示无效或超范围')
        self.depth_hint.setStyleSheet('font-size:13px;color:#64788A;')
        self.depth_hint.setWordWrap(True)
        controls.addWidget(self.depth_hint)
        cam.addWidget(self.depth_controls)
        body.addWidget(camera, 60)

        radar = QFrame()
        radar.setObjectName('panel')
        lidar = QVBoxLayout(radar)
        lidar.setContentsMargins(20, 18, 20, 18)
        lidar.setSpacing(10)
        radar_top = QHBoxLayout()
        title = QLabel('周围环境')
        title.setStyleSheet('font-size:22px;font-weight:700;')
        radar_top.addWidget(title)
        radar_top.addStretch()
        self.pause_lidar = QPushButton('暂停点云')
        self.pause_lidar.setCheckable(True)
        self.pause_lidar.clicked.connect(self.toggle_lidar_pause)
        radar_top.addWidget(self.pause_lidar)
        lidar.addLayout(radar_top)
        subtitle = QLabel('N10P · 360° 激光扫描')
        subtitle.setStyleSheet('font-size:14px;color:#66788A;')
        lidar.addWidget(subtitle)
        self.canvas = RadarCanvas(self)
        lidar.addWidget(self.canvas, 1)
        self.lidar_status = QLabel('等待 N10P 扫描数据')
        self.lidar_status.setStyleSheet('font-size:13px;color:#607488;')
        lidar.addWidget(self.lidar_status)
        legend = QLabel('<span style="color:#CA4653">●</span> &lt; 0.6 m　 <span style="color:#A16B12">●</span> &lt; 1.2 m　 <span style="color:#087F83">●</span> 正常回波')
        legend.setStyleSheet('color:#607488;font-size:13px;padding:4px 0;')
        lidar.addWidget(legend)
        ranges = QHBoxLayout()
        label = QLabel('显示范围')
        label.setStyleSheet('font-size:14px;color:#607488;')
        ranges.addWidget(label)
        self.range_buttons = {}
        for radius in (3, 5, 8, 12):
            button = QPushButton(f'{radius} m')
            button.setCheckable(True)
            button.setChecked(radius == 5)
            button.clicked.connect(lambda _, r=radius: self.set_lidar_range(r))
            self.range_buttons[radius] = button
            ranges.addWidget(button)
        lidar.addLayout(ranges)
        body.addWidget(radar, 40)
        root.addLayout(body, 1)

        metrics = QHBoxLayout()
        metrics.setSpacing(14)
        self.front_lbl = self._create_card('前方 · 0°', '--')
        self.left_lbl = self._create_card('左侧 · 90°', '--')
        self.right_lbl = self._create_card('右侧 · 270°', '--')
        self.back_lbl = self._create_card('后方 · 180°', '--')
        for card in (self.front_lbl, self.left_lbl, self.right_lbl, self.back_lbl):
            metrics.addWidget(card)
        root.addLayout(metrics)
        self.alarm_box = QLabel('雷达等待数据 · 环境状态未知')
        self.alarm_box.setMinimumHeight(48)
        self.alarm_box.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.alarm_box.setStyleSheet('background:#FFF6E5;color:#8B5D16;border-radius:10px;padding:8px 18px;font-size:16px;')
        root.addWidget(self.alarm_box)

        self.rtk_dialog = QDialog(self)
        self.rtk_dialog.setWindowTitle('定位与设备详情')
        self.rtk_dialog.resize(850, 570)
        self.rtk_dialog.setStyleSheet(LIGHT_STYLE)
        layout = QVBoxLayout(self.rtk_dialog)
        layout.setContentsMargins(24, 24, 24, 24)
        self.rtk_card = self.create_rtk_card()
        layout.addWidget(self.rtk_card, 1)
        close = QPushButton('完成')
        close.clicked.connect(self.rtk_dialog.close)
        layout.addWidget(close, 0, Qt.AlignRight)
        self.update_mode_buttons()
        self.depth_range.setEnabled(False)
        self.depth_style.setEnabled(False)
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self.refresh_connection_badges)
        self._status_timer.start(300)

    def _create_card(self, title, val):
        frame = QFrame()
        frame.setObjectName('panel')
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(22, 16, 22, 16)
        text = QLabel(title)
        text.setStyleSheet('font-size:16px;color:#64788A;')
        value = QLabel(val)
        value.setStyleSheet('font-size:30px;font-weight:700;color:#213D51;')
        value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(text)
        layout.addStretch()
        layout.addWidget(value)
        frame.val_label = value
        frame.setMinimumHeight(90)
        return frame

    def open_cors_dialog(self):
        self.cors_dialog.load_config()
        self.cors_dialog.exec_()

    def create_rtk_card(self):
        card = QFrame()
        layout = QVBoxLayout(card)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(16)
        title = QLabel('卫星定位')
        title.setStyleSheet('font-size:24px;font-weight:700;')
        layout.addWidget(title)
        subtitle = QLabel('C-RTK 2HP · 双天线定位与航向')
        subtitle.setStyleSheet('font-size:15px;color:#64788A;')
        layout.addWidget(subtitle)
        statuses = QHBoxLayout()
        self.rtk_ant1_badge = QLabel('左天线 · 等待数据')
        self.rtk_ant2_badge = QLabel('右天线 · 等待数据')
        for badge in (self.rtk_ant1_badge, self.rtk_ant2_badge):
            badge.setStyleSheet(chip_style('muted'))
            statuses.addWidget(badge)
        statuses.addStretch()
        self.rtk_cors_btn = QPushButton('差分连接设置')
        self.rtk_cors_btn.clicked.connect(self.open_cors_dialog)
        statuses.addWidget(self.rtk_cors_btn)
        layout.addLayout(statuses)
        grid = QGridLayout()
        grid.setSpacing(12)
        specs = [('rtk_sats_ant1_box', '左天线 · 主 ANT1'), ('rtk_sats_ant2_box', '右天线 · 辅 ANT2'),
                 ('rtk_sats_sky_box', '天空卫星分布'), ('rtk_heading_box', '航向 / 共视卫星'),
                 ('rtk_coord_box', '经纬度'), ('rtk_alt_box', '海拔 / 精度')]
        for i, (name, title) in enumerate(specs):
            metric = self._create_sub_metric(title, '--')
            setattr(self, name, metric)
            grid.addWidget(metric, i // 2, i % 2)
        layout.addLayout(grid, 1)
        return card

    def _create_sub_metric(self, title, val):
        frame = QFrame()
        frame.setObjectName('panel')
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(8)
        text = QLabel(title)
        text.setStyleSheet('font-size:14px;color:#64788A;')
        value = QLabel(val)
        value.setWordWrap(True)
        value.setStyleSheet('font-size:16px;font-weight:600;color:#213D51;')
        layout.addWidget(text)
        layout.addWidget(value)
        frame.val_lbl = value
        return frame

    def on_rtk_data(self, data):
        self.canvas.set_rtk(data)
        connected = bool(data.get('connected', False))
        fix_q = data.get('fix_quality', 0)
        ant1_used = data.get('ant1_used', data.get('sats_used', 0))
        ant2_used = data.get('ant2_used', 0)
        ant1_fix = data.get('ant1_fix') or ('固定' if fix_q == 4 else '浮点' if fix_q == 5 else '单点' if fix_q > 0 else '搜星中')
        ant2_fix = data.get('ant2_fix') or '搜星中'
        for side, badge, fix in [('左天线', self.rtk_ant1_badge, ant1_fix), ('右天线', self.rtk_ant2_badge, ant2_fix)]:
            badge.setText(f'{side} · {fix}' if connected else f'{side} · 未连接')
            badge.setStyleSheet(chip_style('good' if connected and fix == '固定' else 'warn' if connected else 'muted'))
        self.rtk_sats_ant1_box.val_lbl.setText(f'{ant1_fix}  ·  解算 {ant1_used} / 跟踪 {data.get("ant1_tracked", 0)}')
        self.rtk_sats_ant2_box.val_lbl.setText(f'{ant2_fix}  ·  解算 {ant2_used} / 跟踪 {data.get("ant2_tracked", 0)}')
        self.rtk_sats_sky_box.val_lbl.setText(f'左侧 {data.get("sats_left", 0)} 颗  /  右侧 {data.get("sats_right", 0)} 颗')
        self.rtk_heading_box.val_lbl.setText(f'{data.get("heading", 0):.1f}°  ·  共视 {data.get("heading_sats_common", 0)} 颗' if data.get('has_heading') else '等待航向解算')
        lat, lon = data.get('lat', 0), data.get('lon', 0)
        self.rtk_coord_box.val_lbl.setText(f'{lat:.6f}, {lon:.6f}' if connected and fix_q > 0 else '等待定位解算')
        self.rtk_alt_box.val_lbl.setText(f'{data.get("alt", 0):.1f} m  ·  HDOP {data.get("hdop", 99):.1f}' if connected and fix_q > 0 else '--')
        cors = data.get('cors', {})
        self.cors_dialog.update_cors_status(cors)
        self.dev_badge.setText(f'定位 · {ant1_fix}' if connected else '定位 · 未连接')
        self.dev_badge.setStyleSheet(chip_style('good' if connected and fix_q == 4 else 'warn' if connected else 'muted'))

    def on_voltage_data(self, voltage):
        self.canvas.set_voltage(voltage)
        if hasattr(self, 'battery_badge'):
            pct = max(0, min(100, int(round((voltage - 21.0) / 4.2 * 100))))
            self.battery_badge.setText(f'电量 · {voltage:.1f}V ({pct}%)')
            style_type = 'good' if pct > 50 else 'warn' if pct > 20 else 'danger'
            self.battery_badge.setStyleSheet(chip_style(style_type))

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
        self.depth_style.setEnabled(mode=='depth')
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
            self.cam_dist_badge.setText("彩色实景 · 实时画面")
        elif mode == 'depth':
            if self.latest_depth_pixmap:
                self.video_box.setPixmap(self.latest_depth_pixmap)
            self._set_depth_badge(self.center_depth_mm, getattr(self, 'heat_near_mm', 0), getattr(self, 'heat_far_mm', 0))

    def update_mode_buttons(self):
        for mode, button in [('ai', self.btn_ai), ('rgb', self.btn_rgb), ('depth', self.btn_depth)]:
            button.setChecked(self.cam_mode == mode)
        self.depth_controls.setVisible(self.cam_mode == 'depth')

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
                self.ai_badge_text = f"最近目标 · {label}  {d:.2f} m   |   横向 {x:+.2f} m · 深度 {z:.2f} m"
            else:
                self.ai_badge_text = "当前画面未发现可识别目标"
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

    def closeEvent(self,event):
        self.depth_renderer.stop()
        super().closeEvent(event)

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
        far=float(self.depth_range.currentData());smooth=self.depth_style.currentData()=='smooth'
        self.depth_renderer.submit(record,far,smooth)
        result=self.depth_renderer.output
        if result is None or result['far']!=far or result['smooth']!=smooth:return
        if result is self._depth_shown:return
        frame=result['record']
        if now-frame['received']+frame['age']>.6:return
        self._depth_shown=result
        depth=frame['array'];near=200.
        colored=result['colored'];display_stats=result['stats']
        h,w=colored.shape[:2]
        qimage=QImage(colored.data,w,h,w*3,QImage.Format_RGB888).copy()
        quality=depth_quality(depth,far)
        self.depth_hint.setToolTip('平滑层与原始测距分离；灰白斜纹表示局部估算，暗灰表示缺失。所有测距和有效率都来自原始数据。')
        self.depth_hint.setText(f"原始有效 {quality['valid']:.0%} · 超范围 {quality['outside']:.0%}")
        suffix=f" · 展示估算 {display_stats['estimated_pixels']} px（灰纹）" if smooth else ' · 原始图'
        self.depth_hint.setText(self.depth_hint.text()+suffix)
        self.on_depth_frame(qimage,depth_center_mm(depth),int(near),int(far))

    def _set_depth_badge(self, center_val_mm, near_mm=0, far_mm=0):
        text=f'原始测距 Z {center_val_mm/1000:.2f} m' if center_val_mm>0 else '原始测距 -- · 无回波或跨物体边缘'
        self.cam_dist_badge.setText(text)

    def on_ai_frame(self, qimage):
        # AI 标注图仅作备份；实时画面走 RGB 叠加，避免被 3FPS 推理拖慢
        self.latest_ai_pixmap = self._scale_camera_pixmap(qimage)

    def on_rgb_frame(self, qimage):
        self._camera_received = time.monotonic()
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
        scaled_pix = QPixmap.fromImage(qimage).scaled(available, Qt.KeepAspectRatio, Qt.SmoothTransformation if self.depth_style.currentData()=='smooth' else Qt.FastTransformation)
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
            self.alarm_box.setStyleSheet('padding:8px 18px;background:#FFF3DA;color:#8B621A;border-radius:10px;font-size:16px;')
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
            text, color, bg = '本圈无有效回波 · 环境状态未知', '#8B621A', '#FFF3DA'
        elif data['min'] < 0.6:
            text, color, bg = f"近距回波 {data['min']:.2f} m · 小于 0.6 m", '#B53E4B', '#FFF0F1'
        elif data['min'] < 1.2:
            text, color, bg = f"注意近障 {data['min']:.2f} m · 小于 1.2 m", '#8B621A', '#FFF3DA'
        else:
            text, color, bg = f"最近有效回波 {data['min']:.2f} m · 当前未见近距回波", '#247451', '#EAF6EF'
        self.alarm_box.setText(text)
        self.alarm_box.setStyleSheet(f'padding:8px 18px;border-radius:10px;background:{bg};color:{color};font-size:16px;font-weight:600;')

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.btn_fs.setText("全屏显示")
        else:
            self.showFullScreen()
            self.btn_fs.setText("窗口模式")

    def open_rtk_details(self):
        self.rtk_dialog.exec_()

    def refresh_connection_badges(self):
        now = time.monotonic()
        scan = self.ros_thread.latest_scan
        live = scan is not None and now - scan['received'] + scan.get('source_age', 0) < 0.65
        self.lidar_badge.setText('雷达 · 在线' if live else '雷达 · 等待数据')
        self.lidar_badge.setStyleSheet(chip_style('good' if live else 'muted'))
        if self.cam_mode == 'depth':
            record = self.ros_thread.latest_depth
            live_cam = record is not None and now - record['received'] + record['age'] < 0.6
        else:
            live_cam = self._camera_received > 0 and now - self._camera_received < 1.0
        self.camera_badge.setText('相机 · 在线' if live_cam else '相机 · 等待数据')
        self.camera_badge.setStyleSheet(chip_style('good' if live_cam else 'muted'))
        self.cam_fps_badge.setStyleSheet(chip_style('good' if live_cam else 'muted'))
        if not live_cam:
            self.cam_fps_badge.setText('-- FPS')

def main():
    app = QApplication(sys.argv)
    win = BoardRadarMainWindow()
    win.showFullScreen()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
