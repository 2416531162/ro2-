import sys,importlib.util,pathlib,time,json,types
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage
from PyQt5.QtCore import Qt
spec=importlib.util.spec_from_file_location('heat_gui',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.ROSThread.start=lambda self:None
app=QApplication([]);w=m.BoardRadarMainWindow();w.resize(1920,1080);w.show();app.processEvents()
base=pathlib.Path('/tmp/camera-baseline');meta=json.loads((base/'meta.json').read_text());x=meta['images']['depth']
depth=np.frombuffer((base/'depth.bin').read_bytes(),np.uint16).reshape(x['h'],x['w'])
w.switch_cam_mode('depth');w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic(),age=.01)
w.refresh_depth();app.processEvents()
assert w.latest_depth_pixmap is not None
assert '无回波' in w.depth_hint.text()
assert w.heat_near_mm==200 and w.heat_far_mm==2000
w.depth_range.setCurrentIndex(2);w.refresh_depth();assert w.heat_far_mm==5500
w.depth_range.setCurrentIndex(0);w.refresh_depth()
w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic()-1,age=.01);w.refresh_depth();assert '中断' in w.video_box.text()
w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic(),age=.01);w.refresh_depth();app.processEvents()
assert w.video_box.pixmap() and not w.video_box.pixmap().isNull()
w.grab().save('/tmp/heatmap-corrected-preview.png')
w.switch_cam_mode('rgb');assert w.ros_thread.display_mode=='rgb'
w.switch_cam_mode('ai');assert w.ros_thread.display_mode=='ai'
assert hasattr(w,'cors_dialog') and hasattr(w,'canvas')
w._depth_timer.stop();w._lidar_timer.stop();w.hide()
print('GUI checks=8 passed=8 range=ok stale_recovery=ok mode_switch=ok radar_cors=preserved')
