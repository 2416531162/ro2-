"""霍尔累计位置的因果速度估计；纯计算，不打开总线或发送指令。"""
import math
import threading
from collections import deque


class HallSpeed:
    def __init__(self, step_m, window_s=.06):
        self.step, self.window = step_m, window_s
        self.rows = deque(maxlen=256)
        self.events = deque(maxlen=8)
        self.reference = None
        self._lock = threading.Lock()

    def reset(self):
        with self._lock:
            self.rows.clear()
            self.events.clear()
            self.reference = None

    def add(self, t, position):
        with self._lock:
            self._add(t, position)

    def _add(self, t, position):
        if self.rows and t <= self.rows[-1][0]:
            return
        self.rows.append((t, position))
        if self.reference is None:
            self.reference = (t, position)
        elif abs(position-self.reference[1]) >= .5*self.step:
            # 小于半格的寄存器抖动不刷新运动时刻；采用累计差，避免慢动每帧都不到门限。
            self.events.append((t, position))
            self.reference = (t, position)

    def at(self, t, *, use_window=True):
        # I/O 线程写入、ROS 遥测线程读取：只在复制时持锁，计算不阻塞收发。
        with self._lock:
            rows, events = tuple(self.rows), tuple(self.events)
        rows = [r for r in rows if r[0] <= t]
        events = [r for r in events if r[0] <= t]
        if not rows:
            return None
        last_motion = events[-1][0] if events else rows[0][0]
        age = max(0., t-last_motion)
        period,v,span = None,None,None
        if len(events) >= 2:
            a,b = events[-2:]
            period = b[0]-a[0]
            if 0 < period <= 1.:
                span=period
                v=(b[1]-a[1])/span
                if len(events)>=3:
                    first=events[-3]
                    before=a[0]-first[0]
                    if 0<before<=1. and (a[1]-first[1])*(b[1]-a[1])>0:
                        # 接收节拍会令单格周期一长一短。合并两段完整跨格周期，
                        # 避免把上一段短周期的偏大速度保持到下一段长周期造成正偏。
                        span=b[0]-first[0]
                        distance=b[1]-first[1]
                        v=distance/span
                        period=span/max(1.,abs(distance)/self.step)
        # 高速用时间窗；低速只合并完整跨格周期，不用固定窗的未满格距离。
        window = [r for r in rows if t-r[0] <= self.window+1e-9]
        if use_window and len(window) >= 2:
            dt,d = window[-1][0]-window[0][0],window[-1][1]-window[0][1]
            # 按周期速度选高速模式，不能按窗口本帧刚好跨过几格选择，否则会只留下偏大的窗口。
            if dt > 0 and v is not None and abs(v)>=.35:
                v,period,span = d/dt,dt,dt
        timeout = max(.3,min(1.,2.5*(period or .12)))
        quiet=age>=timeout
        bounded=False
        if len(events)>=3:
            a,b,c=events[-3:]
            da,db=b[1]-a[1],c[1]-b[1]
            # 快速格边抖动与原有慢速往返都可确认；单向跨格不能通过。
            for window_s, coverage_s in ((.6,.6),(2.,1.5)):
                recent=[r for r in rows if t-r[0]<=window_s+1e-9]
                if (recent and recent[-1][0]-recent[0][0]>=coverage_s-1e-9
                        and da*db<0 and (window_s>1. or a[0]>=t-window_s) and t-c[0]<=min(1.,window_s)
                        and max(r[1] for r in recent)-min(r[1] for r in recent)<=1.25*self.step):
                    bounded=True
                    break
        if quiet:
            return (0., self.step/max(age,.1), True, age)
        if v is None:
            # 首次跨格尚不能知道完整周期；明确标成初始化，不伪造低延迟速度。
            return (None,self.step/max(age,.02),False if events else None,age)
        if age > period:
            v *= max(0.,(timeout-age)/max(timeout-period,1e-6))
        # 周期端点有接收周期不确定度，高速窗口有一格量化误差。
        dt_rx = rows[-1][0]-rows[-2][0] if len(rows)>1 else .02
        sigma = max(.015,abs(v)*dt_rx/max(period,.02),self.step/(math.sqrt(6)*max(span or period,.02)))
        return (v,sigma,bounded,age)
