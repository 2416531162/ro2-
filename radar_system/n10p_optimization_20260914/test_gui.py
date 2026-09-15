import sys, pathlib, json, math, time
sys.path.insert(0, sys.argv[1])
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
import board_radar_gui as gui
from n10p_pipeline import scan_payload
app=QApplication([])
gui.ROSThread.start=lambda self:None
w=gui.BoardRadarMainWindow(); w.resize(1920,1080); w.show(); app.processEvents()
r=[math.inf]*720; r[181]=.4; r[540]=2.0
p=scan_payload(r,.15,12,scan_time=.108); p['received']=time.monotonic()
w.ros_thread.latest_scan=p; w.refresh_lidar()
assert w.left_lbl.val_label.text()=='0.40 m'
assert w.right_lbl.val_label.text()=='2.00 m'
assert '2/720' in w.lidar_status.text()
w.pause_lidar.click(); assert w.canvas.paused
frozen=list(w.canvas.ranges); p2=dict(p,ranges=[3.0]*720,received=time.monotonic())
w.ros_thread.latest_scan=p2; w.refresh_lidar(); assert w.canvas.ranges==frozen
w.pause_lidar.click(); assert w.canvas.ranges==p2['ranges']
w.range_buttons[8].click(); assert w.canvas.max_range==8 and sum(b.isChecked() for b in w.range_buttons.values())==1
w.ros_thread.latest_scan=dict(p,received=time.monotonic()-1); w.refresh_lidar()
assert w.canvas.stale and not w.canvas.ranges and w.front_lbl.val_label.text()=='--'
assert '未知' in w.alarm_box.text()
empty=scan_payload([math.inf]*720,.15,12); empty['received']=time.monotonic()
w.ros_thread.latest_scan=empty; w.refresh_lidar(); assert '未知' in w.alarm_box.text()
p['received']=time.monotonic(); w.ros_thread.latest_scan=p; w.refresh_lidar(); assert not w.canvas.stale
sample=json.load(open('/tmp/n10p-scan.json')); sample['ranges']=[math.inf if r is None else r for r in sample['ranges']]
sample=scan_payload(**sample); sample['received']=time.monotonic()
w.ros_thread.latest_scan=sample; w.set_lidar_range(5); w.refresh_lidar()
app.processEvents()
assert w.grab().save('/tmp/n10p-preview.png')
# Basic geometry containment at native and smaller desktop resolution.
for width,height in [(1920,1080),(1600,900)]:
    w.resize(width,height); app.processEvents()
    assert w.canvas.width()>=360 and w.canvas.height()>=340
    assert w.canvas.geometry().right()<w.width()
print('GUI checks=12 passed=12 pause_resume=ok stale_recovery=ok empty_unknown=ok ranges=ok render=ok')
w._lidar_timer.stop(); w.hide()
