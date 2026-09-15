#!/usr/bin/env python3
import sys
import pathlib
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage

sys.path.insert(0, sys.argv[1])
import board_radar_gui as gui

app = QApplication([])
gui.ROSThread.start = lambda self: None
w = gui.BoardRadarMainWindow()
w.resize(1920, 1080)
w.show()
app.processEvents()

depth = np.frombuffer(pathlib.Path('/tmp/camera-baseline/depth.bin').read_bytes(), dtype='<u2').reshape(480, 640)
rgb = np.frombuffer(pathlib.Path('/tmp/camera-baseline/rgb.bin').read_bytes(), dtype=np.uint8).reshape(480, 640, 3)
img, near, far = gui.render_depth_heatmap(depth, rgb)
h, width = img.shape[:2]
qimg = QImage(img.data, width, h, width * 3, QImage.Format_RGB888).copy()
center = int(depth[240, 320])
w.switch_cam_mode('depth')
w.on_depth_frame(qimg, center, int(near), int(far))
app.processEvents()
assert w.cam_mode == 'depth'
assert w.latest_depth_pixmap is not None
assert '中心物距' in w.cam_dist_badge.text()
assert '色标' in w.cam_dist_badge.text()
assert w.latest_depth_pixmap.width() <= w.video_box.width()
assert w.latest_depth_pixmap.height() <= w.video_box.height()
out = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else '/tmp/heatmap-gui.png')
assert w.grab().save(str(out))
print('GUI checks=6 passed=6 depth_mode=ok badge=ok aspect=ok render=ok')
w.hide()
