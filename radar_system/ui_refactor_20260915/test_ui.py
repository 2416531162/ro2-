#!/usr/bin/env python3
"""Qt interactions with synthetic sensor fixtures; no physical device writes."""
import sys
import importlib.util
import math
import time
import json
import ast
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]).resolve().parent.parent))
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage
from n10p_pipeline import scan_payload

spec=importlib.util.spec_from_file_location('gui_under_test',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.ROSThread.start=lambda self:None
app=QApplication([])
w=m.BoardRadarMainWindow();w.resize(1920,1080);w.show();app.processEvents()
checks=[]

def check(name, condition):
    assert condition,name
    checks.append(name)

try:
    if '--contrast' in sys.argv:
        def luminance(c):
            channels=[v/255 for v in (c.red(),c.green(),c.blue())]
            linear=[v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4 for v in channels]
            return sum(v*k for v,k in zip(linear,(.2126,.7152,.0722)))
        page_luma=luminance(w.grab().toImage().pixelColor(2,2))
        w.canvas.set_scan(dict(ranges=[.4,.9,2.],angle_min=math.pi/4,
                              angle_increment=math.pi/2,range_min=.15,stale=False))
        radar=w.canvas.grab().toImage()
        radar_luma=luminance(radar.pixelColor(2,2))
        cx,cy=w.canvas.width()/2,w.canvas.height()/2
        radius=max(20,min(w.canvas.width(),w.canvas.height())/2-34)
        ratios=[]
        for i,distance in enumerate((.4,.9,2.)):
            x,y=m.project_point(math.pi/4+i*math.pi/2,distance,cx,cy,radius/w.canvas.max_range)
            point_luma=max(luminance(radar.pixelColor(round(x)+dx,round(y)+dy))
                           for dx in range(-2,3) for dy in range(-2,3))
            ratios.append((point_luma+.05)/(radar_luma+.05))
        muted=.4<=page_luma<=.8
        dark=radar_luma<.08
        details=hasattr(w,'rtk_dialog') and not w.rtk_card.isVisible()
        passed=muted and dark and min(ratios)>=4.5 and details
        print(f"{'PASS' if passed else 'FAIL'} contrast: muted_background={str(muted).lower()} dark_radar={str(dark).lower()} minimum_return_contrast={min(ratios):.2f}:1 rtk_details_collapsed={str(details).lower()}")
        sys.exit(0 if passed else 1)
    if '--appearance' in sys.argv:
        c=w.grab().toImage().pixelColor(2,2)
        light=(c.red()+c.green()+c.blue())/3 > 200
        details=hasattr(w,'rtk_dialog') and not w.rtk_card.isVisible()
        print(f"{'PASS' if light and details else 'FAIL'} appearance: light_background={str(light).lower()} rtk_details_collapsed={str(details).lower()}")
        sys.exit(0 if light and details else 1)
    ranges=[math.inf]*720;ranges[181]=.4;ranges[540]=2.0
    p=scan_payload(ranges,.15,12,scan_time=.1);p['received']=time.monotonic()
    w.ros_thread.latest_scan=p;w.refresh_lidar()
    check('left_right_distances',w.left_lbl.val_label.text()=='0.40 m' and w.right_lbl.val_label.text()=='2.00 m')
    check('valid_count','2/720' in w.lidar_status.text())
    w.pause_lidar.click();frozen=list(w.canvas.ranges)
    p2=dict(p,ranges=[3.0]*720,received=time.monotonic())
    w.ros_thread.latest_scan=p2;w.refresh_lidar()
    check('pause_preserves_scan',w.canvas.paused and w.canvas.ranges==frozen)
    w.pause_lidar.click()
    check('resume_latest',w.canvas.ranges==p2['ranges'])
    w.range_buttons[8].click()
    check('range_selection',w.canvas.max_range==8 and sum(b.isChecked() for b in w.range_buttons.values())==1)
    w.ros_thread.latest_scan=dict(p,received=time.monotonic()-1);w.refresh_lidar()
    check('stale_clears_data',w.canvas.stale and w.front_lbl.val_label.text()=='--' and '未知' in w.alarm_box.text())
    empty=scan_payload([math.inf]*720,.15,12);empty['received']=time.monotonic()
    w.ros_thread.latest_scan=empty;w.refresh_lidar()
    check('empty_is_unknown','未知' in w.alarm_box.text())
    depth=np.full((240,320),1200,dtype=np.float32);depth[10:20,20:30]=0
    original=depth.copy()
    def update_depth():
        for _ in range(80):
            w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic(),age=.005)
            w.refresh_depth();app.processEvents();time.sleep(.01)
            if w.latest_depth_pixmap and w.heat_far_mm==int(w.depth_range.currentData()) and w._depth_shown and w._depth_shown['smooth']==(w.depth_style.currentData()=='smooth'):
                return
        raise AssertionError('depth render timeout')
    w.switch_cam_mode('depth');update_depth()
    check('metric_depth',w.center_depth_mm==1200)
    w.depth_style.setCurrentIndex(1);update_depth()
    check('raw_smooth_metric_invariant',w.center_depth_mm==1200 and np.array_equal(depth,original))
    w.depth_range.setCurrentIndex(2);update_depth()
    check('depth_range',w.heat_far_mm==5500)
    w.ros_thread.latest_depth=dict(array=depth,received=time.monotonic()-1,age=.005);w.refresh_depth()
    check('depth_stale_clear','中断' in w.video_box.text())
    w.switch_cam_mode('rgb')
    img=QImage(640,480,QImage.Format_RGB888);img.fill(0x748596)
    w.on_rgb_frame(img)
    check('rgb_display',w.video_box.pixmap() is not None and not w.depth_style.isEnabled())
    w.switch_cam_mode('ai');w.on_targets_data('[{"label":"Chair","distance":1.2,"x":0.1,"z":1.1}]');w.on_rgb_frame(img)
    check('ai_readout','1.20' in w.cam_dist_badge.text())
    for width,height in [(1920,1080),(1600,900)]:
        w.resize(width,height);app.processEvents()
        check(f'geometry_{width}',w.width()==width and w.height()==height and w.canvas.width()>=360 and w.canvas.height()>=340 and w.video_box.width()>300)
    if hasattr(w,'rtk_dialog'):
        check('depth_controls_contextual',not w.depth_controls.isVisible())
        w.rtk_dialog.show();app.processEvents()
        check('rtk_details_available',w.rtk_card.isVisible() and w.rtk_cors_btn.isVisible())
        w.on_rtk_data({'connected':False})
        check('offline_rtk_truthful','未连接' in w.dev_badge.text())
        w.rtk_dialog.close()
    print('GUI_INTERACTIONS passed='+str(len(checks))+'/'+str(len(checks))+' raw_measurements_unchanged=true')
finally:
    w.depth_renderer.stop();w._depth_timer.stop();w._lidar_timer.stop()
    if hasattr(w,'_status_timer'):w._status_timer.stop()
    w.hide()
