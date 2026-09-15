"""Aspect-preserving camera viewport with a metric depth legend and pick point."""
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt,QRectF,QPointF,pyqtSignal
from PyQt5.QtGui import QPainter,QColor,QPen,QFont,QPixmap,QImage
from camera_pipeline import color_lut,DEPTH_MIN


class CameraViewport(QLabel):
    picked=pyqtSignal(int,int)
    def __init__(self,parent=None):
        super().__init__(parent)
        self.raw=None;self.mode='ai';self.far=4.5;self.pixel=None
        self.image_rect=QRectF();self.empty_text='正在接收相机画面…'
        self.setMinimumSize(320,240)
        self.setMouseTracking(True)
        lut=color_lut().reshape(1,256,3)
        self.legend=QImage(lut.data,256,1,768,QImage.Format_RGB888).copy()
        self.setToolTip('热力图：点击有效图像区域测深；Z为沿相机光轴的距离。')

    def setPixmap(self,pixmap):
        self.raw=pixmap;self.update()

    def setText(self,text):
        self.empty_text=text;self.raw=None;self.update()

    def paintEvent(self,event):
        p=QPainter(self);p.fillRect(self.rect(),QColor('#080f1b'))
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        legend_h=44 if self.mode=='depth' else 0
        if self.raw and not self.raw.isNull():
            scale=min(self.width()/self.raw.width(),max(1,self.height()-legend_h)/self.raw.height())
            w,h=self.raw.width()*scale,self.raw.height()*scale
            self.image_rect=QRectF((self.width()-w)/2,(self.height()-legend_h-h)/2,w,h)
            p.drawPixmap(self.image_rect,self.raw,QRectF(self.raw.rect()))
            if self.mode=='depth':
                u,v=self.pixel or (self.raw.width()//2,self.raw.height()//2)
                x=self.image_rect.left()+u*scale;y=self.image_rect.top()+v*scale
                p.setPen(QPen(QColor('#ffffff'),1.5))
                p.drawRect(QRectF(x-5*scale,y-5*scale,10*scale,10*scale))
                p.drawLine(QPointF(x-12,y),QPointF(x+12,y));p.drawLine(QPointF(x,y-12),QPointF(x,y+12))
        else:
            self.image_rect=QRectF();p.setPen(QColor('#b9cadb'));p.setFont(QFont('sans-serif',12))
            p.drawText(self.rect(),Qt.AlignCenter,self.empty_text)
        if self.mode=='depth':
            x=16;w=max(100,self.width()-150);y=self.height()-38
            p.drawImage(QRectF(x,y,w,10),self.legend)
            p.setFont(QFont('sans-serif',10));p.setPen(QColor('#cdd8e5'))
            for fraction in [0,.25,.5,.75,1]:
                val=DEPTH_MIN+(self.far-DEPTH_MIN)*fraction
                label=f'{val:.2g}m'
                tx=x+fraction*w
                if fraction==1:tx-=35
                p.drawText(QRectF(tx,y+13,60,20),Qt.AlignLeft,label)
            p.fillRect(self.width()-116,y,14,10,QColor(18,23,32))
            p.drawText(QRectF(self.width()-96,y-4,96,30),Qt.AlignLeft,'无效 / 盲区')
        p.end()

    def mousePressEvent(self,event):
        if self.mode=='depth' and self.raw and self.image_rect.contains(event.localPos()):
            u=int((event.x()-self.image_rect.left())/self.image_rect.width()*self.raw.width())
            v=int((event.y()-self.image_rect.top())/self.image_rect.height()*self.raw.height())
            self.pixel=(min(u,self.raw.width()-1),min(v,self.raw.height()-1))
            self.picked.emit(*self.pixel);self.update()
        super().mousePressEvent(event)
