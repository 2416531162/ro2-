import sys,pathlib,json,time,types,math
sys.path.insert(0,sys.argv[1])
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt,QPoint
from PyQt5.QtGui import QImage
from PyQt5.QtTest import QTest
import board_radar_gui as g
from camera_pipeline import image_array,surface_sample
app=QApplication([]);g.ROSThread.start=lambda self:None
w=g.BoardRadarMainWindow();w.resize(1920,1080);w.show();app.processEvents()
meta=json.load(open('/tmp/camera-baseline/meta.json'));arrays={}
for key,m in meta['images'].items():
    message=types.SimpleNamespace(width=m['w'],height=m['h'],step=m['step'],encoding=m['encoding'],is_bigendian=m['bigendian'],data=pathlib.Path('/tmp/camera-baseline/'+key+'.bin').read_bytes())
    arrays[key]=image_array(message)
rgb=arrays['rgb'];h,ww=rgb.shape[:2];image=QImage(rgb.data,ww,h,ww*3,QImage.Format_RGB888).copy()
def record(mode,array=None):return dict(array=array,image=image if mode!='depth' else None,received=time.monotonic(),age=.03,stamp=1.)
w.switch_cam_mode('depth');w.ros_thread.camera_frames['depth']=record('depth',arrays['depth']);w.refresh_camera();app.processEvents()
assert w.video_box.raw and w.video_box.mode=='depth'
rect=w.video_box.image_rect;assert abs(rect.width()/rect.height()-4/3)<.001
QTest.mouseClick(w.video_box,Qt.LeftButton,pos=QPoint(int(rect.center().x()),int(rect.center().y())))
w.refresh_camera();assert abs(w.video_box.pixel[0]-320)<=1 and abs(w.video_box.pixel[1]-240)<=1
w.heat_range.setCurrentIndex(0);w.refresh_camera();assert w.video_box.far==2.
w.reset_depth_pick();assert w.video_box.pixel is None
w.heat_range.setCurrentIndex(1);w.refresh_camera();app.processEvents();w.grab().save('/tmp/camera-depth-preview.png')
w.switch_cam_mode('ai');w.ros_thread.camera_frames['ai']=record('ai');w.refresh_camera()
# Later metadata for the same frame updates badge without waiting for a new image.
w.ros_thread.observations=dict(source_stamp=1.,sync_ms=7,targets=[dict(label='Chair',valid=True,distance=2.,z=1.9,x=.2,y=.1,valid_ratio=.8)])
w.refresh_camera();assert '1.90' in w.cam_dist_badge.text()
w.ros_thread.camera_frames['ai']=dict(record('ai'),received=time.monotonic()-2);w.refresh_camera()
assert w.video_box.raw is None and '未就绪' in w.cam_dist_badge.text()
w.ros_thread.camera_frames['ai']=record('ai');w.refresh_camera();assert w.video_box.raw is not None
w.switch_cam_mode('rgb');w.ros_thread.camera_frames['rgb']=record('rgb');w.refresh_camera();app.processEvents()
assert '原始比例' in w.cam_dist_badge.text()
w.grab().save('/tmp/camera-rgb-preview.png')
for size in [(1920,1080),(1600,900)]:
    w.resize(*size);app.processEvents();rect=w.video_box.image_rect
    assert abs(rect.width()/rect.height()-4/3)<.001
    assert w.video_box.geometry().right()<w.width()
assert hasattr(w,'canvas') and hasattr(w,'cors_dialog')
w._camera_timer.stop();w._lidar_timer.stop();w.hide()
print('GUI checks=12 passed=12 aspect=ok pick=ok range=ok same_frame_metadata=ok stale_recovery=ok radar_cors=preserved')
