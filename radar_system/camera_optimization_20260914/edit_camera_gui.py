from pathlib import Path
p=Path(__file__).parent/'MODIFIED_FILE/board_radar_gui.py'
s=p.read_text().replace('QDialog, QLineEdit, QComboBox)','QDialog, QLineEdit, QComboBox, QShortcut)')
s=s.replace('QFontMetrics','QFontMetrics, QKeySequence',1)
s=s.replace('import cv2','import cv2\nfrom camera_pipeline import image_array, stamp_seconds, heatmap_rgb, surface_sample\nfrom camera_view import CameraViewport',1)
s=s.replace('        self.latest_scan = None','        self.latest_scan = None\n        self.camera_frames = {}\n        self.camera_status = {}\n        self.observations = {}',1)
a=s.index('        def rgb_callback');b=s.index('        def rtk_callback',a)
s=s[:a]+'''        def camera_frame(msg, mode):
            if mode != self.display_mode:
                return
            try:
                array=image_array(msg)
                age=node.get_clock().now().nanoseconds/1e9-stamp_seconds(msg)
                if mode=='depth':
                    image=None
                else:
                    h,w=array.shape[:2]
                    image=QImage(array.data,w,h,w*3,QImage.Format_RGB888).copy()
                self.camera_frames[mode]=dict(array=array if mode=='depth' else None,
                    image=image,received=time.monotonic(),age=max(0,age),stamp=stamp_seconds(msg))
            except (ValueError,TypeError):
                pass

        def rgb_callback(msg): camera_frame(msg,'rgb')
        def depth_callback(msg): camera_frame(msg,'depth')
        def ai_callback(msg): camera_frame(msg,'ai')

        def observations_callback(msg):
            try:self.observations=json.loads(msg.data)
            except (ValueError,TypeError):pass

        def status_callback(msg):
            try:self.camera_status=json.loads(msg.data)
            except (ValueError,TypeError):pass

'''+s[b:]
s=s.replace("node.create_subscription(Image, '/camera/rgb/image_raw', rgb_callback, 10)","node.create_subscription(Image, '/camera/rgb/image_raw', rgb_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))")
s=s.replace("node.create_subscription(Image, '/camera/depth_raw/image', depth_callback, 10)","node.create_subscription(Image, '/camera/depth_raw/image', depth_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))")
s=s.replace("node.create_subscription(Image, '/camera/ai_detection/image', ai_callback, 10)","node.create_subscription(Image, '/camera/ai_detection/image', ai_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))")
s=s.replace("node.create_subscription(String, '/camera/ai_detection/targets', targets_callback, 10)","node.create_subscription(String, '/camera/ai_detection/observations', observations_callback, 1)\n        node.create_subscription(String, '/camera/ai_detection/status', status_callback, 1)")
s=s.replace('        self.ros_thread.rgb_signal.connect(self.on_rgb_frame)\n        self.ros_thread.depth_signal.connect(self.on_depth_frame)\n        self.ros_thread.ai_signal.connect(self.on_ai_frame)\n        self.ros_thread.targets_signal.connect(self.on_targets_data)\n','')
s=s.replace('        self.last_targets = []','''        self.last_targets = []
        self._camera_shown = None
        self._camera_depth = None
        self._camera_timer = QTimer(self)
        self._camera_timer.timeout.connect(self.refresh_camera)
        self._camera_timer.start(40)
        self.camera_shortcuts = []
        for key,mode in [('1','ai'),('2','rgb'),('3','depth')]:
            shortcut=QShortcut(QKeySequence(key),self)
            shortcut.activated.connect(lambda m=mode:self.switch_cam_mode(m))
            self.camera_shortcuts.append(shortcut)''',1)
s=s.replace('self.video_box = QLabel("正在接收相机画面...")','self.video_box = CameraViewport(self)\n        self.video_box.picked.connect(self.pick_depth)')
s=s.replace('        cam_layout.addLayout(mode_layout)\n', '''        cam_layout.addLayout(mode_layout)
        depth_tools=QHBoxLayout()
        depth_tools.addWidget(QLabel('热图量程'))
        self.heat_range=QComboBox()
        for title,value in [('近景 0.2–2m',2.0),('室内 0.2–4.5m',4.5),('全量程 0.2–5.5m',5.5)]:
            self.heat_range.addItem(title,value)
        self.heat_range.setCurrentIndex(1)
        self.heat_range.currentIndexChanged.connect(self.change_heat_range)
        self.heat_range.setMinimumHeight(34)
        depth_tools.addWidget(self.heat_range)
        self.depth_center_btn=QPushButton('测点回中')
        self.depth_center_btn.setMinimumHeight(34)
        self.depth_center_btn.clicked.connect(self.reset_depth_pick)
        depth_tools.addWidget(self.depth_center_btn)
        depth_tools.addStretch()
        self.camera_detail=QLabel('AI: 同帧标注 · Z 光轴深度 / R 直线距离')
        self.camera_detail.setWordWrap(True)
        self.camera_detail.setStyleSheet('font-size:12px; color:#b9cadb; background:transparent; border:none; padding:4px;')
        cam_layout.addLayout(depth_tools)
        cam_layout.addWidget(self.camera_detail)
''')
a=s.index('    def switch_cam_mode');b=s.index('    def update_mode_buttons',a)
s=s[:a]+'''    def switch_cam_mode(self, mode):
        self.cam_mode=mode;self.ros_thread.display_mode=mode
        self._camera_shown=None;self._fps_times=[]
        self.video_box.mode=mode
        self.video_box.setText('等待新画面…')
        self.update_mode_buttons()
        self.heat_range.setEnabled(mode=='depth');self.depth_center_btn.setEnabled(mode=='depth')
        self.cam_fps_badge.setText('-- FPS')
        self.cam_dist_badge.setText('正在接收…')

'''+s[b:]
a=s.index('    def on_targets_data');b=s.index('    def set_lidar_range',a)
s=s[:a]+'''    def change_heat_range(self, index):
        self.video_box.far=float(self.heat_range.currentData())
        self._camera_shown=None

    def reset_depth_pick(self):
        self.video_box.pixel=None
        self._camera_shown=None

    def pick_depth(self, x, y):
        self.video_box.pixel=(x,y)
        self._camera_shown=None

    def refresh_camera(self):
        record=self.ros_thread.camera_frames.get(self.cam_mode)
        now=time.monotonic()
        if record is None or now-record['received']+record['age']>.8:
            self.video_box.setText('相机数据中断 · 等待新画面' if record else '等待相机画面…')
            self.cam_dist_badge.setText('测距 -- · 数据未就绪')
            self.cam_fps_badge.setText('-- FPS')
            self.camera_detail.setText('旧检测框和旧距离已清除，等待当前画面。')
            self._camera_shown=None
            return
        if record is self._camera_shown:return
        self._camera_shown=record
        self.video_box.mode=self.cam_mode
        if self.cam_mode=='depth':
            depth=record['array'];self._camera_depth=depth
            rgb=heatmap_rgb(depth,self.video_box.far)
            h,w=depth.shape
            image=QImage(rgb.data,w,h,w*3,QImage.Format_RGB888).copy()
            u,v=self.video_box.pixel or (w//2,h//2)
            sample=surface_sample(depth,u,v,5,5)
            if sample['valid']:
                self.cam_dist_badge.setText(f"测点 Z {sample['z']:.2f} m · 有效 {sample['valid_ratio']:.0%}")
            else:
                self.cam_dist_badge.setText('测点 -- · 深度不足或跨物体边缘')
            valid=np.isfinite(depth)
            self.camera_detail.setText(f"点击图像测深 · 像素 ({u}, {v}) · 全图有效 {np.mean(valid):.0%} · 暖色近 / 冷色远 · 黑色无效")
        else:
            image=record['image']
            if self.cam_mode=='ai':
                obs=self.ros_thread.observations
                # Show metadata only when it belongs to this annotated frame.
                matched=abs(obs.get('source_stamp',0)-record['stamp'])<.001
                items=obs.get('targets',[]) if matched else []
                valid=[t for t in items if t.get('valid') and t.get('distance') is not None]
                if valid:
                    nearest=min(valid,key=lambda t:t['distance'])
                    self.cam_dist_badge.setText(f"{nearest['label']} · Z {nearest['z']:.2f} m / R {nearest['distance']:.2f} m")
                    self.camera_detail.setText(f"框中心表面测距 · X {nearest['x']:+.2f} / Y {nearest['y']:+.2f} m · 有效 {nearest.get('valid_ratio',0):.0%} · 同步差 {obs.get('sync_ms',0):.0f} ms")
                else:
                    self.cam_dist_badge.setText('AI 同帧画面 · 当前无有效3D目标')
                    self.camera_detail.setText('Z=光轴深度，R=直线距离；深度不足/未对齐/不同步时只保留2D框。')
            else:
                self.cam_dist_badge.setText('彩色实景 · 原始比例')
                self.camera_detail.setText('实时 RGB 画面；AI 测距在独立同帧模式显示，按 1 / 2 / 3 切换。')
        self.video_box.setPixmap(QPixmap.fromImage(image))
        self._note_fps()
        frame_age=(now-record['received']+record['age'])*1000
        self.cam_fps_badge.setText(self.cam_fps_badge.text()+f' · {frame_age:.0f} ms')

'''+s[b:]
# Shorter metric labels avoid header minimum-size pressure at native width.
s=s.replace('cam_title = QLabel("📷 Astra S 3D 深度相机实时流")','cam_title = QLabel("Astra S · RGB-D")')
s=s.replace('self.cam_dist_badge = QLabel("中心物距: -- mm")','self.cam_dist_badge = QLabel("测距 --")')
p.write_text(s)
