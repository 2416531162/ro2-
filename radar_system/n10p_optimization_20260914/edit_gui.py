from pathlib import Path
p=Path(__file__).parent/'MODIFIED_FILE/board_radar_gui.py'
s=p.read_text()
s=s.replace('QThread, QPointF, QSize', 'QThread, QPointF, QSize, QTimer, QRectF')
s=s.replace('from rclpy.node import Node', 'from rclpy.node import Node\nfrom rclpy.qos import qos_profile_sensor_data\nfrom n10p_pipeline import scan_payload, project_point')
s=s.replace("self.display_mode = 'ai'", "self.display_mode = 'ai'\n        self.latest_scan = None",1)
a=s.index('            n = len(msg.ranges)'); b=s.index('        def rgb_callback',a)
s=s[:a]+'''            age = (node.get_clock().now().nanoseconds / 1e9 -
                   msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9)
            payload = scan_payload(msg.ranges, msg.range_min, msg.range_max,
                                   msg.angle_min, msg.angle_increment, msg.scan_time, age)
            payload['received'] = time.monotonic()
            # A single latest-value mailbox prevents queued scan signal backlog.
            self.latest_scan = payload

'''+s[b:]
s=s.replace("node.create_subscription(LaserScan, '/scan', scan_callback, 10)","node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)")
a=s.index('class RadarCanvas'); b=s.index('class BoardRadarMainWindow',a)
s=s[:a]+'''class RadarCanvas(QWidget):
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

'''+s[b:]
s=s.replace('self.ros_thread.scan_signal.connect(self.on_scan_data)', '''self._shown_scan = None
        self._last_scan_received = 0.0
        self._lidar_timer = QTimer(self)
        self._lidar_timer.timeout.connect(self.refresh_lidar)
        self._lidar_timer.start(50)''')
s=s.replace('        self.canvas = RadarCanvas(self)', '''        lidar_title = QLabel('N10P  /  360° 激光雷达')
        lidar_title.setStyleSheet('font-size: 18px; font-weight: bold; color: #e3eef9; padding: 4px;')
        left_panel.addWidget(lidar_title)
        self.lidar_status = QLabel('等待 N10P 扫描数据…')
        self.lidar_status.setStyleSheet('font-size: 13px; color: #aebfd0; padding: 4px;')
        left_panel.addWidget(self.lidar_status)
        self.canvas = RadarCanvas(self)''')
s=s.replace('        left_panel.addWidget(self.canvas, 1)', '''        left_panel.addWidget(self.canvas, 1)
        lidar_tools = QHBoxLayout()
        legend = QLabel('● <0.6m 近距   ● <1.2m 注意   · 原始回波 / 无拖影')
        legend.setStyleSheet('color: #aebfd0; font-size: 12px;')
        lidar_tools.addWidget(legend, 1)
        self.pause_lidar = QPushButton('暂停点云')
        self.pause_lidar.setCheckable(True)
        self.pause_lidar.setMinimumHeight(32)
        self.pause_lidar.clicked.connect(self.toggle_lidar_pause)
        lidar_tools.addWidget(self.pause_lidar)
        left_panel.addLayout(lidar_tools)''')
s=s.replace('main_layout.addLayout(left_panel, 35)', 'main_layout.addLayout(left_panel, 44)')
s=s.replace('main_layout.addLayout(right_panel, 65)', 'main_layout.addLayout(right_panel, 56)')
s=s.replace('self.video_box.setMinimumHeight(520)', 'self.video_box.setMinimumHeight(360)\n        self.video_box.setMinimumWidth(1)\n        self.video_box.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)')
s=s.replace('🟢 雷达 | 🟢 3D相机 | 🛰️ 左天线', 'RTK | 左天线')
s=s.replace('self.dev_badge = QLabel("🟢 双传感器在线")','self.dev_badge = QLabel("RTK 等待数据")')
s=s.replace('self.alarm_box = QLabel("✅ 安全状态: 周围障碍物正常")','self.alarm_box = QLabel("雷达等待数据 · 环境状态未知")')
s=s.replace('        for r in [3, 5, 8, 12]:','        self.range_buttons = {}\n        for r in [3, 5, 8, 12]:')
s=s.replace('b.clicked.connect(lambda _, val=r: self.canvas.set_range(val))','''b.setCheckable(True)
            b.setChecked(r == 5)
            b.setStyleSheet('QPushButton {background:#162435; color:#d0deea; border:1px solid #385167; border-radius:6px; padding:8px;} QPushButton:checked {background:#155e75; border:2px solid #61d8e8; color:white;}')
            self.range_buttons[r] = b
            b.clicked.connect(lambda _, val=r: self.set_lidar_range(val))''')
a=s.index('    def on_scan_data(self, data):'); b=s.index('    def toggle_fullscreen',a)
s=s[:a]+'''    def set_lidar_range(self, meters):
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

'''+s[b:]
p.write_text(s)
