#!/usr/bin/env python3
"""Native Qt map tab, consuming the SAME 8088 snapshot/PNG as the website.

No QtWebEngine, browser subprocess or duplicate SLAM required on a 4GB board.
"""
import json
import math
import os
import sys
import time
from PyQt5.QtCore import Qt,QTimer,QUrl,QPointF,QRectF
from PyQt5.QtGui import QPainter,QImage,QColor,QPen,QTransform,QPolygonF,QFont
from PyQt5.QtNetwork import QNetworkAccessManager,QNetworkRequest,QNetworkReply
from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QPushButton,QLabel,QLineEdit,QTabWidget,QComboBox

BASE=os.environ.get('RO2_MAP_URL','http://127.0.0.1:8088').rstrip('/')

class MapCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320,250)
        self.data={}
        self.meta=None
        self.image=None
        self.cloud=[]
        self.scale=50.
        self.cx=self.cy=0.
        self.iso=False
        self.online=False
        self.drag=None

    def project(self,x,y,z=0.):
        if self.iso:
            return QPointF(self.width()/2+((x-self.cx)-(y-self.cy))*.7071*self.scale,
                           self.height()/2-((x-self.cx)+(y-self.cy))*.35355*self.scale-z*.8*self.scale)
        return QPointF(self.width()/2+(x-self.cx)*self.scale,self.height()/2-(y-self.cy)*self.scale)

    def world(self,gx,gy):
        m=self.meta
        x,y=gx*m['resolution'],gy*m['resolution']
        ox,oy,a=m['origin']
        return ox+x*math.cos(a)-y*math.sin(a),oy+x*math.sin(a)+y*math.cos(a)

    def fit(self):
        if self.meta:
            self.cx,self.cy=self.world(self.meta['width']/2,self.meta['height']/2)
            self.scale=.8*min(self.width(),self.height())/max(self.meta['width']*self.meta['resolution'],self.meta['height']*self.meta['resolution'],1.)
            self.update()

    def paintEvent(self,event):
        p=QPainter(self)
        p.fillRect(self.rect(),QColor('#e4ebf0'))
        p.setRenderHint(QPainter.Antialiasing)
        if self.image is not None and self.meta:
            m=self.meta
            o,u,v=self.project(*self.world(0,0)),self.project(*self.world(m['step'],0)),self.project(*self.world(0,m['step']))
            p.save()
            p.setTransform(QTransform(u.x()-o.x(),u.y()-o.y(),v.x()-o.x(),v.y()-o.y(),o.x(),o.y()))
            p.setClipRect(QRectF(0,0,m['width']/m['step'],m['height']/m['step']))
            p.drawImage(QPointF(0,0),self.image)
            p.restore()
        def line(points,color,width=2,closed=False):
            if not points:
                return
            poly=QPolygonF([self.project(*point) for point in points])
            p.setPen(QPen(QColor(color),width))
            if closed:
                p.drawPolygon(poly)
            else:
                p.drawPolyline(poly)
        if self.iso:
            p.setPen(QPen(QColor('#397ca2'),3))
            for point in self.cloud:
                p.drawPoint(self.project(*point))
        line(self.data.get('trajectory',[]),'#1686a0')
        if self.online and self.data.get('localized'):
            line(self.data.get('plan',[]),'#9763cf',3)
            p.setPen(QPen(QColor('#08a098'),2))
            for point in self.data.get('scan',[]):
                p.drawPoint(self.project(*point))
            p.setFont(QFont('Noto Sans CJK SC',11))
            for person in self.data.get('people',[]):
                point=self.project(person['x'],person['y'])
                p.setPen(QPen(QColor('#ce752a'),3))
                p.drawEllipse(point,6,6)
                p.drawText(point+QPointF(10,-8),'锁定目标' if person.get('locked') else '人体')
            goal=self.data.get('goal')
            if goal:
                point=self.project(goal['x'],goal['y'])
                p.drawText(point,'× 候选点（未校验）')
            robot=self.data.get('robot')
            if robot:
                x,y,a=robot
                shape=[[x+u*math.cos(a)-v*math.sin(a),y+u*math.sin(a)+v*math.cos(a)]
                       for u,v in [(-.18,-.335),(.67,-.335),(.67,.335),(-.18,.335)]]
                line(shape,'#16798a',3,True)
                line([[x,y],[x+.6*math.cos(a),y+.6*math.sin(a)]],'#16798a',4)
        p.setPen(QColor('#536d80'))
        p.drawText(16,self.height()-18,'map · 米   |   拖动平移 / 滚轮缩放')
        text=''
        if not self.meta:
            text='等待 /map\n请启动雷达、底盘里程计，再开始建图'
        elif self.iso and not self.cloud:
            text='未收到 3D 实测体素\n当前底图仍是 2D 地图；需校准传感器 TF 并启动 mapping3d'
        if text:
            p.drawText(self.rect(),Qt.AlignCenter,text)
        p.end()

    def wheelEvent(self,event):
        self.scale=max(2.,min(1000.,self.scale*math.exp(event.angleDelta().y()*.001)))
        self.update()

    def mousePressEvent(self,event):
        self.drag=(event.pos(),self.cx,self.cy)

    def mouseMoveEvent(self,event):
        if not self.drag:
            return
        start,x,y=self.drag
        u=(event.x()-start.x())/self.scale
        v=(event.y()-start.y())/self.scale
        if self.iso:
            self.cx=x-(u/.7071-v/.35355)/2
            self.cy=y+(u/.7071+v/.35355)/2
        else:
            self.cx,self.cy=x-u,y+v
        self.update()

    def mouseReleaseEvent(self,event):
        self.drag=None


class MapPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.net=QNetworkAccessManager(self)
        self.active=None
        self.active_since=0.
        self.last_ok=0.
        self.image_pending=False
        self.cloud_pending=False
        self.cloud_revision=-1
        root=QVBoxLayout(self)
        title=QLabel('实时室内地图  /  屏幕与网页同源')
        title.setStyleSheet('font-size:25px;font-weight:600;padding:12px')
        root.addWidget(title)
        self.status=QLabel('等待地图服务 :8088')
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        bar=QHBoxLayout()
        self.canvas=MapCanvas()
        for label,callback in [('2D 导航地图',lambda:self.set_iso(False)),('3D 实测点云',lambda:self.set_iso(True)),('适应地图',self.canvas.fit)]:
            button=QPushButton(label)
            button.clicked.connect(callback)
            bar.addWidget(button)
        bar.addStretch()
        root.addLayout(bar)
        root.addWidget(self.canvas,1)
        controls=QHBoxLayout()
        self.name=QLineEdit()
        self.name.setPlaceholderText('地图名：office_01')
        self.maps=QComboBox()
        self.action_buttons=[]
        for label,callback in [('开始建图',lambda:self.action('start',dict(mode='mapping'))),
                               ('停止建图/定位',lambda:self.action('stop',{}))]:
            b=QPushButton(label)
            b.clicked.connect(callback)
            self.action_buttons.append(b)
            controls.addWidget(b)
        controls.addWidget(self.name)
        for label,callback in [('保存地图',lambda:self.action('save',dict(name=self.name.text()))),
                               ('加载并定位',lambda:self.action('start',dict(mode='localization',name=self.maps.currentText())))]:
            b=QPushButton(label)
            b.clicked.connect(callback)
            self.action_buttons.append(b)
            controls.addWidget(b)
        controls.addWidget(self.maps)
        root.addLayout(controls)
        self.message=QLabel('首次建图请遥控慢行；灰色为未知。切换前停止跟随。加载地图后请在网页设置 AMCL 初始位置。')
        self.message.setWordWrap(True)
        root.addWidget(self.message)
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(300)

    def set_iso(self,value):
        self.canvas.iso=value
        self.canvas.update()

    def get(self,path,done):
        reply=self.net.get(QNetworkRequest(QUrl(BASE+path)))
        timeout=QTimer(reply)
        timeout.setSingleShot(True)
        timeout.timeout.connect(reply.abort)
        timeout.start(4000)
        def finished():
            try:
                done(bytes(reply.readAll()) if reply.error()==QNetworkReply.NoError else None)
            finally:
                reply.deleteLater()
        reply.finished.connect(finished)
        return reply

    def poll(self):
        if time.monotonic()-self.last_ok>2.:
            self.canvas.online=False
            self.status.setText('连接超时 / 等待地图服务 :8088 · 保留历史地图，不显示旧位置为实时位置')
            self.canvas.update()
        if self.active is None:
            self.active=self.get('/api/live_map',self.receive)

    def receive(self,body):
        self.active=None
        if not body:
            return
        try:
            data=json.loads(body)
            if not isinstance(data,dict):
                return
            self.last_ok=time.monotonic()
            self.canvas.data=data
            self.canvas.online=True
            meta=data.get('map')
            text=('定位 TF 在线' if data.get('localized') else '定位未就绪')+' · '+('雷达实时' if data.get('scan_live') else '雷达无新数据')
            if meta:
                text+=f" · 已知面积 {meta['known_area_m2']} m² · {meta['resolution']} m/格"
            self.status.setText(text+'\n'+data.get('error',''))
            names=data.get('session',{}).get('maps',[])
            if names!=[self.maps.itemText(i) for i in range(self.maps.count())]:
                old=self.maps.currentText()
                self.maps.clear()
                self.maps.addItems(names)
                self.maps.setCurrentText(old)
            if meta and (self.canvas.meta or {}).get('revision')!=meta['revision'] and not self.image_pending:
                self.image_pending=True
                self.get('/api/live_map/image?v='+str(meta['revision']),lambda b:self.receive_image(b,meta))
            if data.get('cloud_revision',0)!=self.cloud_revision and not self.cloud_pending:
                self.cloud_pending=True
                self.get('/api/live_map/cloud',self.receive_cloud)
            self.canvas.update()
        except (ValueError,KeyError,TypeError):
            self.status.setText('地图服务返回格式错误')

    def receive_image(self,body,meta):
        self.image_pending=False
        if body:
            image=QImage.fromData(body)
            if not image.isNull():
                first=self.canvas.meta is None
                self.canvas.image,self.canvas.meta=image,meta
                if first:
                    self.canvas.fit()
                self.canvas.update()

    def receive_cloud(self,body):
        self.cloud_pending=False
        try:
            if body:
                data=json.loads(body)
                self.cloud_revision=data['revision']
                self.canvas.cloud=data['points']
                self.canvas.update()
        except (ValueError,KeyError,TypeError):
            pass

    def action(self,name,data):
        for button in self.action_buttons:
            button.setEnabled(False)
        request=QNetworkRequest(QUrl(BASE+'/api/live_map/'+name))
        request.setHeader(QNetworkRequest.ContentTypeHeader,'application/json')
        reply=self.net.post(request,json.dumps(data).encode())
        timer=QTimer(reply)
        timer.setSingleShot(True)
        timer.timeout.connect(reply.abort)
        timer.start(20000)
        def finished():
            try:
                result=json.loads(bytes(reply.readAll()))
                self.message.setText(result.get('message') or result.get('error','请求失败'))
            except (ValueError,TypeError):
                self.message.setText('请求失败，请确认网页服务已启动')
            finally:
                for button in self.action_buttons:
                    button.setEnabled(True)
                reply.deleteLater()
        reply.finished.connect(finished)


def main():
    from board_radar_gui import BoardRadarMainWindow
    import rclpy
    app=QApplication(sys.argv)
    win=QTabWidget()
    win.setWindowTitle('RK3588 · 实时地图与环境感知')
    win.setStyleSheet('QWidget{background:#eef3f6;color:#223a4e;font-size:16px}QPushButton,QLineEdit,QComboBox{background:white;border:1px solid #c5d5df;padding:12px;border-radius:7px}QTabBar::tab{padding:15px 35px}QTabBar::tab:selected{background:white;color:#087f86}')
    panel=MapPanel()
    win.addTab(panel,'实时建模地图')
    board=BoardRadarMainWindow()
    board.setWindowFlags(Qt.Widget)
    board.btn_close.clicked.disconnect()
    board.btn_close.clicked.connect(win.close)
    board.btn_fs.clicked.disconnect()
    board.btn_fs.clicked.connect(lambda:win.showNormal() if win.isFullScreen() else win.showFullScreen())
    win.addTab(board,'相机 / 雷达感知')
    win.resize(1280,800)
    win.showFullScreen() if '--windowed' not in sys.argv else win.show()
    result=app.exec_()
    board.depth_renderer.stop()
    if rclpy.ok():
        rclpy.shutdown()
    board.ros_thread.wait(2000)
    return result

if __name__=='__main__':
    sys.exit(main())
