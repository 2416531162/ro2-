#!/usr/bin/env python3
"""3D-first native PyQt5 screen. Same binary scene as WebGL; no Chromium needed.

Perspective point rasterization and z-buffer run on a latest-only worker.
Qt paints one image plus inexpensive annotation lines, not 60000 individual
points. This backend is CPU/NumPy, not a claim of native GPU acceleration.
"""
import json
import math
import os
import sys
import threading
import time
import numpy as np
from PyQt5.QtCore import Qt,QTimer,QUrl,QPointF
from PyQt5.QtGui import QPainter,QImage,QColor,QPen,QFont,QLinearGradient
from PyQt5.QtNetwork import QNetworkAccessManager,QNetworkRequest,QNetworkReply
from PyQt5.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,
                             QPushButton,QComboBox,QSlider,QCheckBox,QDoubleSpinBox,QTabWidget)
from cloud_scene import raster,project,RAMP

BASE=os.environ.get('RO2_MAP_URL','http://127.0.0.1:8088').rstrip('/')


class RasterWorker:
    def __init__(self):
        self.event=threading.Event(); self.running=True
        self.request=None; self.output=None
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()

    def run(self):
        while self.running:
            self.event.wait(.2); self.event.clear()
            request=self.request
            if request is None:
                continue
            number,points,kwargs=request
            try:
                started=time.monotonic()
                image=raster(points,**kwargs)
                self.output=(number,kwargs,image,(time.monotonic()-started)*1000)
            except (ValueError,MemoryError) as exc:
                self.output=(number,kwargs,None,str(exc))
            if self.request is request:
                self.request=None

    def submit(self,number,points,kwargs):
        self.request=(number,points,kwargs); self.event.set()

    def stop(self):
        self.running=False; self.event.set(); self.thread.join(timeout=3.)


class CloudCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320,280)
        self.points=np.empty((0,4),np.float32); self.meta={}; self.state={}
        self.target=[0.,0.,.5]; self.distance=14.; self.azimuth=-2.1; self.elevation=.9
        self.color_mode='height'; self.z_low=-.2; self.z_high=3.; self.point_size=2
        self.light=False; self.follow=True; self.rings=True; self.grid=True; self.view='orbit'
        self.online=False; self.drag=None
        self.worker=RasterWorker(); self.revision=0; self.generation=0; self.shown=-1; self.image=None
        self.render_ms=0.; self.render_state=None; self.pending=True
        self.timer=QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(80)

    def clear(self):
        self.points=np.empty((0,4),np.float32); self.meta={}; self.state={}
        self.image=None; self.render_state=None; self.generation+=1
        self.invalidate()

    def invalidate(self):
        self.revision+=1; self.pending=True; self.update()

    def set_cloud(self,points,meta):
        if (points.shape!=(meta['count'],4) or meta['count']>120000 or
                meta.get('format')!='xyzi-f32le' or not np.isfinite(points).all()):
            raise ValueError('invalid scene packet')
        first=not len(self.points)
        self.points=points.copy(); self.meta=meta
        if first and meta.get('bounds'):
            self.fit()
        self.invalidate()

    def fit(self):
        if not self.meta.get('bounds'):
            return
        a,b=np.array(self.meta['bounds'],dtype=float)
        self.target=((a+b)/2).tolist()
        self.distance=max(4.,float(np.linalg.norm(b-a))*1.4)
        self.follow=False; self.invalidate()

    def set_view(self,name):
        self.view=name
        self.elevation,self.azimuth={'top':(math.pi/2,-math.pi/2),'front':(.18,-math.pi/2),'orbit':(.9,-2.1)}[name]
        self.invalidate()

    def set_state(self,state,online=True):
        self.state=state; self.online=online
        if self.follow and online and state.get('localized') and state.get('robot'):
            self.target=[state['robot'][0],state['robot'][1],.4]
        self.invalidate()

    def options(self):
        # Bound pixel work independently of the native panel resolution.
        ratio=min(1.,1280/max(self.width(),1),900/max(self.height(),1))
        return dict(width=max(1,int(self.width()*ratio)),height=max(1,int(self.height()*ratio)),
                    target=tuple(self.target),distance=self.distance,azimuth=self.azimuth,elevation=self.elevation,
                    color_mode=self.color_mode,z_low=self.z_low,z_high=self.z_high,point_size=self.point_size,
                    light=self.light,robot=tuple((self.state.get('robot') or [0,0])[:2]+[0]),
                    intensity_range=self.meta.get('intensity_range'))

    def refresh(self):
        out=self.worker.output
        if out:
            (generation,number),options,image,ms=out
            # Accept intermediate camera frames while dragging, but never cross
            # map epochs. Annotation projection uses exactly this rendered view.
            if generation==self.generation and number>self.shown and image is not None:
                self.image=QImage(image.data,image.shape[1],image.shape[0],image.strides[0],QImage.Format_RGBA8888).copy()
                self.render_ms=ms; self.shown=number; self.render_state=options; self.update()
        if self.pending:
            self.pending=False
            self.worker.submit((self.generation,self.revision),self.points,self.options())

    def xy(self,points):
        o=self.render_state or self.options()
        p=project(points,o['target'],o['distance'],o['azimuth'],o['elevation'],o['width'],o['height'])
        p[:,0]*=self.width()/o['width'];p[:,1]*=self.height()/o['height']
        return p

    def draw_lines(self,painter,points,color,width=1):
        if len(points)<2:
            return
        xy=self.xy(points); painter.setPen(QPen(QColor(color),width))
        for i in range(0,len(xy)-1,2):
            a,b=xy[i],xy[i+1]
            if min(a[2],b[2])>.1:
                painter.drawLine(QPointF(float(a[0]),float(a[1])),QPointF(float(b[0]),float(b[1])))

    def paintEvent(self,event):
        p=QPainter(self); p.fillRect(self.rect(),QColor('#ebf1f6' if self.light else '#0f1620'))
        # The point image and annotation camera must match, not old camera/new overlays.
        if self.image is not None:
            p.drawImage(self.rect(),self.image)
        p.setRenderHint(QPainter.Antialiasing)
        if self.grid:
            target=(self.render_state or self.options())['target']
            cx,cy=round(target[0]),round(target[1]); lines=[]
            for i in range(-10,11):
                lines.extend(((cx+i,cy-10,-.03),(cx+i,cy+10,-.03),(cx-10,cy+i,-.03),(cx+10,cy+i,-.03)))
            self.draw_lines(p,lines,'#c4d1db' if self.light else '#293846')
        s=self.state; valid=self.online and s.get('localized') and s.get('robot')
        for key,color in [('trajectory','#18a9c5'),('plan','#ab80ef')]:
            points=s.get(key,[]) if key=='trajectory' or valid else []
            lines=[]
            for a,b in zip(points,points[1:]):
                lines.extend(((a[0],a[1],.035),(b[0],b[1],.035)))
            self.draw_lines(p,lines,color,2)
        p.setFont(QFont('Noto Sans CJK SC',10))
        if valid:
            x,y,a=s['robot']
            if self.rings:
                for radius in (1,2,3,5):
                    lines=[]
                    for i in range(96):
                        lines.extend(((x+radius*math.cos(i*math.pi/48),y+radius*math.sin(i*math.pi/48),.02),
                                      (x+radius*math.cos((i+1)*math.pi/48),y+radius*math.sin((i+1)*math.pi/48),.02)))
                    self.draw_lines(p,lines,'#899fac' if not self.light else '#8096a7')
                    at=self.xy([[x+radius,y,.02]])[0]
                    if at[2]>.1:
                        p.drawText(QPointF(float(at[0]+4),float(at[1]-5)),f'{radius} m')
            local=lambda u,v,z:(x+u*math.cos(a)-v*math.sin(a),y+u*math.sin(a)+v*math.cos(a),z)
            corners=[(-.18,-.335),(.67,-.335),(.67,.335),(-.18,.335)]; lines=[]
            for i in range(4):
                lines.extend((local(*corners[i],.05),local(*corners[(i+1)%4],.05)))
            lines.extend((local(0,0,.05),local(.9,0,.05),local(.9,0,.05),local(.7,.13,.05),local(.9,0,.05),local(.7,-.13,.05)))
            self.draw_lines(p,lines,'#f5d337',2)
            for person in s.get('people',[]):
                point=self.xy([[person['x'],person['y'],.05]])[0]
                if point[2]>.1:
                    p.setPen(QPen(QColor('#ff963d'),2)); pos=QPointF(float(point[0]),float(point[1]))
                    p.drawEllipse(pos,7,7); p.drawText(pos+QPointF(12,-8),'跟随目标' if person.get('locked') else '人体观测')
            goal=s.get('goal')
            if goal:
                gx,gy=goal['x'],goal['y']
                self.draw_lines(p,[(gx-.15,gy-.15,.06),(gx+.15,gy+.15,.06),(gx-.15,gy+.15,.06),(gx+.15,gy-.15,.06)],'#ff963d',2)
            # /scan is a ground-plane projection, not fabricated 3D geometry.
            scan=s.get('scan',[])
            if scan:
                xy=self.xy([[v[0],v[1],.025] for v in scan])
                p.setPen(QPen(QColor('#19d7d7'),2))
                for v in xy:
                    if v[2]>.1:
                        p.drawPoint(QPointF(float(v[0]),float(v[1])))
        p.setPen(QColor('#405d73' if self.light else '#bdd2df'))
        frame_name=s.get('scene',{}).get('frame','map')
        frame_desc='车体局部' if frame_name=='base_link' else '全局建图'
        p.drawText(16,25,f'3D 实时点云 · {frame_name} ({frame_desc}) · 原生 NumPy 深度缓冲')
        p.drawText(16,self.height()-18,'左键旋转 / 右键或 Shift 平移 / 滚轮缩放 · 网格 1 m')
        p.drawText(16,self.height()-40,f'点数 {len(self.points):,} · 栅格化 {self.render_ms:.0f} ms · '+('实时' if self.online and s.get('scene',{}).get('live') else '历史 / 等待'))
        # Same metric palette as WebGL; intensity is only enabled when present.
        w=min(190,self.width()//3); x=self.width()-w-18; y=self.height()-68
        gradient=QLinearGradient(x,y,x+w,y)
        for i,rgb in enumerate(RAMP):
            gradient.setColorAt(i/5,QColor.fromRgbF(*map(float,rgb)))
        p.fillRect(x,y,w,8,gradient)
        mode=self.color_mode
        bounds=([0,10] if mode=='distance' else (self.meta.get('intensity_range') or [0,1])
                if mode=='intensity' else [self.z_low,self.z_high])
        p.drawText(x,y-8,{'height':'高度 Z / m','distance':'距机器人 / m','intensity':'真实强度 / 原始单位'}[mode])
        p.drawText(x,y+24,f'{bounds[0]:.1f}')
        p.drawText(x+w-36,y+24,f'{bounds[1]:.1f}')
        if not len(self.points):
            message=s.get('scene',{}).get('error') or '等待深度数据、匹配内参与真实传感器 TF'
            p.drawText(self.rect().adjusted(25,50,-25,-50),Qt.AlignCenter|Qt.TextWordWrap,'等待真实三维点云\n'+message)
        if s.get('test_fixture'):
            p.setPen(QColor('#f5be45'));p.drawText(16,50,s['test_fixture'])
        p.end()

    def resizeEvent(self,event):
        self.invalidate()

    def wheelEvent(self,event):
        self.distance=max(1.,min(150.,self.distance*math.exp(-event.angleDelta().y()*.001)))
        self.invalidate()

    def mousePressEvent(self,event):
        self.drag=(event.pos(),self.azimuth,self.elevation,list(self.target),
                   event.button()!=Qt.LeftButton or bool(event.modifiers()&Qt.ShiftModifier) or self.view=='top')

    def mouseMoveEvent(self,event):
        if not self.drag:
            return
        pos,a,e,t,pan=self.drag; dx,dy=event.x()-pos.x(),event.y()-pos.y()
        if pan:
            right=np.array([-math.sin(a),math.cos(a),0.])
            up=np.array([-math.sin(e)*math.cos(a),-math.sin(e)*math.sin(a),math.cos(e)])
            scale=self.distance*2*math.tan(math.pi/8)/max(self.height(),1)
            self.target=(np.array(t)-dx*scale*right+dy*scale*up).tolist();self.follow=False
        else:
            self.azimuth=a-dx*.006;self.elevation=max(.08,min(math.pi/2,e+dy*.006))
        self.invalidate()

    def mouseReleaseEvent(self,event):
        self.drag=None


class CloudPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.net=QNetworkAccessManager(self);self.polling=False;self.binary_pending=False
        self.epoch='';self.loaded=None;self.last_ok=0.;self.last_state={};self.closed=False
        root=QVBoxLayout(self)
        self.status=QLabel('等待三维地图服务 :8088');self.status.setWordWrap(True);root.addWidget(self.status)
        tools=QHBoxLayout();self.canvas=CloudCanvas()
        for label,callback in [('三维视角',lambda:self.canvas.set_view('orbit')),('俯视',lambda:self.canvas.set_view('top')),
                               ('低视角',lambda:self.canvas.set_view('front')),('全图',self.canvas.fit)]:
            b=QPushButton(label);b.clicked.connect(callback);tools.addWidget(b)
        self.follow=QPushButton('跟随车位');self.follow.setCheckable(True);self.follow.setChecked(True)
        self.follow.clicked.connect(self.set_follow);tools.addWidget(self.follow)
        self.theme=QPushButton('浅色模式');self.theme.clicked.connect(self.set_theme);tools.addWidget(self.theme)
        root.addLayout(tools);root.addWidget(self.canvas,1)
        options=QHBoxLayout();self.color=QComboBox();self.color.addItems(['高度 Z','距机器人距离','真实强度'])
        self.color.currentIndexChanged.connect(self.settings);options.addWidget(self.color)
        options.addWidget(QLabel('高度 / m'))
        self.low,self.high=QDoubleSpinBox(),QDoubleSpinBox()
        for spin,value in [(self.low,-.2),(self.high,3.)]:
            spin.setRange(-100.,100.);spin.setSingleStep(.1);spin.setValue(value)
            spin.valueChanged.connect(self.settings);options.addWidget(spin)
        self.size=QSlider(Qt.Horizontal);self.size.setRange(1,4);self.size.setValue(2)
        self.size.setMaximumWidth(120);self.size.valueChanged.connect(self.settings)
        options.addWidget(QLabel('点大小'));options.addWidget(self.size)
        self.rings=QCheckBox('距离环');self.rings.setChecked(True);self.rings.toggled.connect(self.settings);options.addWidget(self.rings)
        root.addLayout(options)
        self.hint=QLabel('这是有限窗口的实测三维观测，不是完整三维 SLAM；建图/保存和遥控在其他标签中。')
        self.hint.setWordWrap(True);root.addWidget(self.hint)
        self.timer=QTimer(self);self.timer.timeout.connect(self.poll);self.timer.start(300)

    def set_follow(self,checked):
        self.canvas.follow=checked;self.canvas.set_state(self.last_state,self.canvas.online)

    def set_theme(self):
        self.canvas.light=not self.canvas.light;self.theme.setText('深色模式' if self.canvas.light else '浅色模式');self.canvas.invalidate()

    def settings(self,*args):
        if self.low.value()>=self.high.value():
            return
        self.canvas.color_mode=['height','distance','intensity'][self.color.currentIndex()]
        self.canvas.z_low,self.canvas.z_high=self.low.value(),self.high.value()
        self.canvas.point_size=self.size.value();self.canvas.rings=self.rings.isChecked();self.canvas.invalidate()

    def get(self,path,done):
        reply=self.net.get(QNetworkRequest(QUrl(BASE+path)))
        timer=QTimer(reply);timer.setSingleShot(True);timer.timeout.connect(reply.abort);timer.start(4000)
        def finished():
            try:
                if not self.closed:
                    done(bytes(reply.readAll()) if reply.error()==QNetworkReply.NoError else None)
            finally:
                reply.deleteLater()
        reply.finished.connect(finished)

    def poll(self):
        if self.closed:
            return
        if time.monotonic()-self.last_ok>1.8:
            self.canvas.set_state(self.last_state,False)
            self.status.setText('连接超时 · 历史点云，不显示旧车位为实时车位')
        if not self.polling:
            self.polling=True;self.get('/api/live_map',self.receive)

    def receive(self,body):
        self.polling=False
        if not body:
            return
        try:
            state=json.loads(body);meta=state['scene']
            if meta['format']!='xyzi-f32le':
                raise ValueError('wrong scene format')
            self.last_ok=time.monotonic();self.last_state=state
            if self.epoch!=meta['epoch']:
                self.epoch=meta['epoch'];self.loaded=None;self.canvas.clear()
            self.canvas.set_state(state,True)
            self.status.setText(f"三维点云 {meta['count']:,} 点 · {meta['source']} · "+
                                ('实时' if meta.get('live') else '历史 / 等待')+'\n'+(meta.get('error') or state.get('error','')))
            available=meta.get('intensity_available',False)
            self.color.model().item(2).setEnabled(available)
            self.color.model().item(1).setEnabled(bool(state.get('localized')))
            if (self.color.currentIndex()==2 and not available) or (self.color.currentIndex()==1 and not state.get('localized')):
                self.color.setCurrentIndex(0)
            key=(meta['epoch'],meta['revision'])
            if self.loaded!=key and not self.binary_pending:
                self.binary_pending=True
                self.get(f'/api/live_map/scene.bin?epoch={key[0]}&v={key[1]}',lambda b:self.receive_cloud(b,meta))
        except (ValueError,KeyError,TypeError):
            self.status.setText('地图服务未升级或数据格式错误')

    def receive_cloud(self,body,meta):
        self.binary_pending=False
        if body is None or self.epoch!=meta['epoch']:
            return
        try:
            if len(body)!=meta['count']*16:
                raise ValueError('点云字节数错误')
            self.canvas.set_cloud(np.frombuffer(body,dtype='<f4').reshape(-1,4),meta)
            self.loaded=(meta['epoch'],meta['revision']);self.follow.setChecked(self.canvas.follow)
        except (ValueError,KeyError,TypeError) as exc:
            self.status.setText(str(exc))

    def stop(self):
        self.closed=True;self.timer.stop();self.canvas.timer.stop();self.canvas.worker.stop()


def _prewarm_ros():
    try:
        import rclpy
        from rclpy.node import Node
        import rcl_interfaces.msg
        import sensor_msgs.msg
        import std_msgs.msg
        if not rclpy.ok():
            rclpy.init()
        _node = Node('_prewarm')
        _node.destroy_node()
    except Exception:
        pass


def main():
    # --map-only also supports headless/desktop testing without ROS or cameras.
    if '--map-only' not in sys.argv:
        _prewarm_ros()
    app=QApplication(sys.argv)
    win=QTabWidget();win.setWindowTitle('RK3588 · 三维点云地图')
    win.setStyleSheet('QWidget{background:#eef3f6;color:#284354;font-size:15px}QPushButton,QComboBox,QDoubleSpinBox{padding:9px;background:white;border:1px solid #c4d4df;border-radius:6px}QTabBar::tab{padding:14px 25px}QTabBar::tab:selected{background:white;color:#087f89}')
    panel=CloudPanel();win.addTab(panel,'三维建模 / 点云地图')
    board=None
    if '--map-only' not in sys.argv:
        from live_map_gui import MapPanel
        from board_radar_gui import BoardRadarMainWindow
        win.addTab(MapPanel(),'二维导航 / 建图管理')
        board=BoardRadarMainWindow();board.setWindowFlags(Qt.Widget)
        board.btn_close.clicked.disconnect();board.btn_close.clicked.connect(win.close)
        board.btn_fs.clicked.disconnect();board.btn_fs.clicked.connect(lambda:win.showNormal() if win.isFullScreen() else win.showFullScreen())
        win.addTab(board,'相机 / 雷达感知')
    win.resize(1280,800)
    win.show() if '--windowed' in sys.argv else win.showFullScreen()
    result=app.exec_();panel.stop()
    if board:
        import rclpy
        board.depth_renderer.stop()
        if rclpy.ok():
            rclpy.shutdown()
        board.ros_thread.wait(2000)
    return result

if __name__=='__main__':
    sys.exit(main())
