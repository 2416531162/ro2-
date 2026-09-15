from pathlib import Path
D=Path(__file__).parent;s=(D/'BASELINE.py').read_text()
a=s.index('HEAT_BG_RGB =');b=s.index('class ROSThread',a)
s=s[:a]+'''HEAT_BG_RGB = (20, 25, 34)
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
    cv2.putText(canvas,'NEAR / RED    DEPTH Z (m)    FAR / BLUE    DARK = NO RETURN',
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


'''+s[b:]
s=s.replace('        self.heat_near = None\n        self.heat_far = None','        self.latest_depth = None')
a=s.index('        def depth_callback');b=s.index('        def ai_callback',a)
s=s[:a]+'''        def depth_callback(msg):
            if self.display_mode != 'depth':return
            try:
                depth=decode_depth_mm(msg)
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                age=node.get_clock().now().nanoseconds/1e9-stamp
                self.latest_depth=dict(array=depth,received=time.monotonic(),age=max(0,age))
            except (ValueError,TypeError) as exc:
                node.get_logger().warn('depth: '+str(exc),throttle_duration_sec=5.0)

'''+s[b:]
s=s.replace("node.create_subscription(Image, '/camera/depth_raw/image', depth_callback, 10)","node.create_subscription(Image, '/camera/depth_raw/image', depth_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))")
s=s.replace('        self.ros_thread.depth_signal.connect(self.on_depth_frame)','''        self._depth_shown = None
        self._depth_timer = QTimer(self)
        self._depth_timer.timeout.connect(self.refresh_depth)
        self._depth_timer.start(40)''')
s=s.replace('        cam_layout.addLayout(mode_layout)', '''        cam_layout.addLayout(mode_layout)
        depth_controls=QHBoxLayout()
        self.depth_hint=QLabel('深度距离图 · 非温度图 · 暗灰表示无回波')
        self.depth_hint.setStyleSheet('font-size:12px; color:#c5d2e0; background:transparent; border:none; padding:2px;')
        depth_controls.addWidget(self.depth_hint,1)
        self.depth_range=QComboBox()
        for label,value in [('近景 0.2–2.0 m',2000),('室内 0.2–4.5 m',4500),('全程 0.2–5.5 m',5500)]:
            self.depth_range.addItem(label,value)
        self.depth_range.setStyleSheet('font-size:12px; color:#e2e8f0; background:#162435; border:1px solid #385167; border-radius:6px; padding:5px;')
        self.depth_range.currentIndexChanged.connect(self.change_depth_range)
        depth_controls.addWidget(self.depth_range)
        cam_layout.addLayout(depth_controls)''')
s=s.replace('self.video_box.setMinimumHeight(520)','self.video_box.setMinimumHeight(360)')
s=s.replace('self.video_box.setMinimumHeight(360)','self.video_box.setMinimumHeight(280)')
s=s.replace('        self.cam_mode = mode\n        self.ros_thread.display_mode = mode', '''        self.cam_mode = mode
        self.ros_thread.display_mode = mode
        self.depth_range.setEnabled(mode=='depth')
        self._depth_shown=None
        if mode=='depth':
            self.latest_depth_pixmap=None
            self.video_box.clear()
            self.video_box.setText('等待新的深度帧…')''')
a=s.index('    def _set_depth_badge');b=s.index('    def on_ai_frame',a)
s=s[:a]+'''    def change_depth_range(self, index):
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
        valid=np.isfinite(depth)&(depth>=DEPTH_VALID_MIN_MM)&(depth<=DEPTH_VALID_MAX_MM)
        ratio=float(np.mean(valid))
        self.depth_hint.setText(f'纯深度 Z · 有效 {ratio:.0%} / 无回波 {1-ratio:.0%} · 红近蓝远 · 超色标截断')
        self.on_depth_frame(qimage,depth_center_mm(depth),int(near),int(far))

    def _set_depth_badge(self, center_val_mm, near_mm=0, far_mm=0):
        text=f'中心区域 Z {center_val_mm/1000:.2f} m' if center_val_mm>0 else '中心区域 -- · 无回波或跨物体边缘'
        self.cam_dist_badge.setText(text)

'''+s[b:]
# No bilinear color bleed when scaling a measurement visualization.
s=s.replace('        scaled_pix = self._scale_camera_pixmap(qimage)\n        self.latest_depth_pixmap = scaled_pix','        scaled_pix = QPixmap.fromImage(qimage).scaled(self.video_box.size(), Qt.KeepAspectRatio, Qt.FastTransformation)\n        self.latest_depth_pixmap = scaled_pix')
(D/'MODIFIED_FILE.py').write_text(s)
