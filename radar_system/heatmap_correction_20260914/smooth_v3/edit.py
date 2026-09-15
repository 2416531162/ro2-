from pathlib import Path
D=Path(__file__).parent;s=(D/'BASELINE.py').read_text()
a=s.index('class ROSThread')
s=s[:a]+'''def prepare_display_depth(raw_mm):
    """Independent display copy. None of these values feed metric readouts.

    Masked 3x3 bilateral averaging preserves >100mm discontinuities. Fill only
    enclosed components <=9 pixels with >=5 valid neighbors on one surface.
    The returned estimate mask stays distinct from the original validity mask.
    """
    raw=np.asarray(raw_mm,dtype=np.float32)
    valid=np.isfinite(raw)&(raw>=DEPTH_VALID_MIN_MM)&(raw<=DEPTH_VALID_MAX_MM)
    clean=np.where(valid,raw,0).astype(np.float32)
    padded=np.pad(clean,1,mode='edge');vp=np.pad(valid,1,mode='edge')
    h,w=raw.shape;num=np.zeros_like(clean);den=np.zeros_like(clean)
    for dy in range(3):
        for dx in range(3):
            neighbor=padded[dy:dy+h,dx:dx+w]
            delta=neighbor-clean
            weight=np.exp(-.5*(delta/35.)**2)*np.exp(-.5*((dy-1)**2+(dx-1)**2))
            weight*=vp[dy:dy+h,dx:dx+w]&valid&(np.abs(delta)<=100)
            num+=neighbor*weight;den+=weight
    display=np.divide(num,den,out=clean.copy(),where=den>0)
    # Connected-component gate excludes large holes and frame-edge voids.
    count,labels,stats,_=cv2.connectedComponentsWithStats((~valid).astype(np.uint8),8)
    small=np.zeros(count,dtype=bool)
    if count>1:
        area=stats[:,cv2.CC_STAT_AREA];xs=stats[:,cv2.CC_STAT_LEFT];ys=stats[:,cv2.CC_STAT_TOP]
        widths=stats[:,cv2.CC_STAT_WIDTH];heights=stats[:,cv2.CC_STAT_HEIGHT]
        small=(area<=9)&(xs>0)&(ys>0)&(xs+widths<w)&(ys+heights<h);small[0]=False
    eligible=small[labels]&(~valid)
    kernel=np.ones((3,3),np.uint8)
    neighbors=cv2.boxFilter(valid.astype(np.float32),-1,(3,3),normalize=False)
    total=cv2.boxFilter(clean,-1,(3,3),normalize=False)
    local_min=cv2.erode(np.where(valid,clean,100000).astype(np.float32),kernel)
    local_max=cv2.dilate(clean,kernel)
    estimated=eligible&(neighbors>=5)&(local_max-local_min<=100)
    display[estimated]=total[estimated]/neighbors[estimated]
    display[~(valid|estimated)]=0
    return display,valid,estimated


def render_depth_display(raw_mm,near_mm=200.,far_mm=2000.,smooth=True):
    if not smooth:
        image,near,far=render_depth_heatmap(raw_mm,near_mm=near_mm,far_mm=far_mm)
        return image,dict(estimated_pixels=0,mode='raw')
    if not 0<=near_mm<far_mm<=DEPTH_VALID_MAX_MM:raise ValueError('invalid metric range')
    display,measured,estimated=prepare_display_depth(raw_mm)
    supported=measured|estimated;h,w=display.shape;size=(w*2,h*2)
    # Continuous resampling of a masked scalar field, then colorize. This avoids
    # mixing dark invalid pixels into real surface colors at silhouettes.
    weights=cv2.resize(supported.astype(np.float32),size,interpolation=cv2.INTER_LINEAR)
    values=cv2.resize(display,size,interpolation=cv2.INTER_LINEAR)
    values=np.divide(values,weights,out=np.zeros_like(values),where=weights>1e-5)
    coverage=cv2.resize(supported.astype(np.uint8),size,interpolation=cv2.INTER_NEAREST).astype(bool)
    u=np.clip((values-near_mm)/(far_mm-near_mm),0,1)*255
    lo=np.floor(u).astype(np.uint8);hi=np.minimum(lo.astype(np.int16)+1,255)
    t=(u-lo)[...,None]
    out=(HEAT_LUT[lo]*(1-t)+HEAT_LUT[hi]*t).round().astype(np.uint8)
    out[~coverage]=HEAT_BG_RGB
    est=cv2.resize(estimated.astype(np.uint8),size,interpolation=cv2.INTER_NEAREST).astype(bool)
    yy,xx=np.indices(est.shape)
    hatch=est&(((xx+yy)%5)<2)
    out[hatch]=(210,215,225)  # visible estimate marking, never counted as measured
    return _draw_heatmap_legend(out,near_mm,far_mm),dict(estimated_pixels=int(estimated.sum()),mode='smooth')


'''+s[a:]
s=s.replace('        depth_controls.addWidget(self.depth_range)', '''        depth_controls.addWidget(self.depth_range)
        self.depth_style=QComboBox()
        self.depth_style.addItem('平滑展示（非测量）','smooth')
        self.depth_style.addItem('原始测量图','raw')
        self.depth_style.setStyleSheet(self.depth_range.styleSheet())
        self.depth_style.currentIndexChanged.connect(self.change_depth_range)
        depth_controls.addWidget(self.depth_style)''')
s=s.replace("        colored,near,far=render_depth_heatmap(depth,near_mm=200,far_mm=far)","        near=200.\n        smooth=self.depth_style.currentData()=='smooth'\n        colored,display_stats=render_depth_display(depth,near_mm=near,far_mm=far,smooth=smooth)")
s=s.replace('        quality=depth_quality(depth,far)', '''        quality=depth_quality(depth,far)
        self.depth_hint.setToolTip('平滑层与原始测距分离；灰白斜纹表示局部估算，暗灰表示缺失。所有测距和有效率都来自原始数据。')''')
s=s.replace('        self.on_depth_frame(qimage,depth_center_mm(depth),int(near),int(far))', '''        suffix=f" · 展示估算 {display_stats['estimated_pixels']} px（灰纹）" if smooth else ' · 原始图'
        self.depth_hint.setText(self.depth_hint.text()+suffix)
        self.on_depth_frame(qimage,depth_center_mm(depth),int(near),int(far))''')
s=s.replace("        text=f'中心区域 Z {center_val_mm/1000:.2f} m'", "        text=f'原始测距 Z {center_val_mm/1000:.2f} m'")
s=s.replace("else '中心区域 -- · 无回波或跨物体边缘'", "else '原始测距 -- · 无回波或跨物体边缘'")
s=s.replace('        self.depth_range.setEnabled(mode==\'depth\')',"        self.depth_range.setEnabled(mode=='depth')\n        self.depth_style.setEnabled(mode=='depth')")
s=s.replace('scaled_pix = QPixmap.fromImage(qimage).scaled(available, Qt.KeepAspectRatio, Qt.FastTransformation)',"scaled_pix = QPixmap.fromImage(qimage).scaled(available, Qt.KeepAspectRatio, Qt.SmoothTransformation if self.depth_style.currentData()=='smooth' else Qt.FastTransformation)")
# Longer explanatory legend for 2x display with enough readable pixel height.
s=s.replace('    h,w=rgb.shape[:2];canvas=np.empty((h+54,w,3),np.uint8)', '    h,w=rgb.shape[:2];canvas=np.empty((h+54,w,3),np.uint8)')
(D/'MODIFIED_FILE.py').write_text(s)
