import sys,pathlib,importlib.util,time,json
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
spec=importlib.util.spec_from_file_location('gui_smooth',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.ROSThread.start=lambda self:None
app=QApplication([]);w=m.BoardRadarMainWindow();w.resize(1920,1080);w.show();app.processEvents()
base=pathlib.Path('/tmp/heatmap-live-data');meta=json.loads((base/'meta.json').read_text());x=meta['images']['depth'];depth=np.frombuffer((base/'depth.bin').read_bytes(),np.uint16).reshape(x['h'],x['w'])
def update():
    for i in range(70):
        w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic(),age=.005)
        w.refresh_depth();app.processEvents();time.sleep(.01)
        if w.latest_depth_pixmap and w.depth_renderer.output and w.depth_renderer.output['smooth']==(w.depth_style.currentData()=='smooth') and w.heat_far_mm==int(w.depth_range.currentData()) and w._depth_shown is w.depth_renderer.output:return
    raise AssertionError('render timeout')
w.switch_cam_mode('depth');update();assert w.depth_style.currentData()=='smooth'
smooth_center=w.center_depth_mm;raw_quality=m.depth_quality(depth,2000)
w.grab().save('/tmp/depth-smooth-preview.png')
w.depth_style.setCurrentIndex(1);update();assert w.center_depth_mm==smooth_center
assert m.depth_quality(depth,2000)==raw_quality
w.grab().save('/tmp/depth-raw-preview.png')
w.depth_range.setCurrentIndex(2);update();assert w.heat_far_mm==5500
w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic()-1,age=.01);w.refresh_depth();assert '中断' in w.video_box.text()
w.depth_style.setCurrentIndex(0);w.latest_depth_pixmap=None;update();assert w.video_box.pixmap() and not w.video_box.pixmap().isNull()
w.switch_cam_mode('rgb');assert not w.depth_style.isEnabled();w.switch_cam_mode('depth');assert w.depth_style.isEnabled()
assert hasattr(w,'canvas') and hasattr(w,'cors_dialog')
w.depth_renderer.stop();w._depth_timer.stop();w._lidar_timer.stop();w.hide()
print('GUI checks=8 passed=8 smooth_raw_toggle=ok raw_metric_invariant=ok worker=ok stale_recovery=ok')
