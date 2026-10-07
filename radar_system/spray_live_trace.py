"""随实车遥测推算落料轨迹；只接收观测数据，永不调用车辆或水泵接口。"""
import math
import threading
from spray_simulation import SpraySettings, SimulatedSpray, polyline, project, tangent, stroke_length, closed


def finite(value,name,low,high):
    if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError(name+'无效')
    return float(value)


class LiveSprayTrace:
    def __init__(self):
        self.lock=threading.RLock();self.ident=None;self.spray=None;self.previous=None;self.finished=False
        self.valid=False;self.offset=None;self.good_traces=[];self.bad_traces=[];self.trace_kind=None

    def receive(self,request):
        if not isinstance(request,dict):raise ValueError('喷线观测须为对象')
        ident=request.get('run')
        if not isinstance(ident,str) or not 1<=len(ident)<=256:raise ValueError('作业编号无效')
        samples=request.get('samples')
        if not isinstance(samples,list) or not 1<=len(samples)<=20:raise ValueError('喷线观测数量无效')
        checked=[]
        for raw in samples:
            if not isinstance(raw,dict):raise ValueError('喷线观测格式无效')
            value=dict(raw)
            for key,lo,hi in [('time',0,1e12),('speed',0,3),('heading',-1000,1000),('body_speed',-3,3),('remaining',0,100000)]:
                value[key]=finite(raw.get(key,0),key,lo,hi)
            point=raw.get('point')
            if not isinstance(point,list) or len(point)!=2:raise ValueError('喷头位置无效')
            value['point']=[finite(v,'喷头位置',-10000,10000) for v in point]
            for key in ('valid','painting','eligible','finished'):
                if type(raw.get(key)) is not bool:raise ValueError('观测状态无效')
            value['allowed']=raw.get('allowed',True)
            if type(value['allowed']) is not bool:raise ValueError('喷线资格无效')
            value['repair_only']=raw.get('repair_only',False)
            if type(value['repair_only']) is not bool:raise ValueError('补喷状态无效')
            windows=raw.get('paint_windows',[])
            if not isinstance(windows,list) or len(windows)>512:raise ValueError('补喷区间过多')
            value['paint_windows']=[]
            for entry in windows:
                if not isinstance(entry,list) or len(entry)!=2:raise ValueError('补喷区间无效')
                a,b=[finite(v,'补喷区间',0,1) for v in entry]
                if b<=a:raise ValueError('补喷区间无效')
                value['paint_windows'].append((a,b))
            checked.append(value)
        if any(b['time']<=a['time'] for a,b in zip(checked,checked[1:])):raise ValueError('观测时序无效')
        with self.lock:
            if ident!=self.ident:
                target=request.get('target')
                if not isinstance(target,list) or not 1<=len(target)<=1024:raise ValueError('请重新发送喷线图纸')
                strokes=[];total=0;total_length=0.0
                for i,points in enumerate(target):
                    if not isinstance(points,list) or not 2<=len(points)<=5000:raise ValueError('喷线图纸点数无效')
                    clean=[]
                    for point in points:
                        if not isinstance(point,list) or len(point)!=2:raise ValueError('喷线图纸坐标无效')
                        p=[finite(v,'图纸位置',-10000,10000) for v in point]
                        if not clean or math.dist(clean[-1],p)>=.001:clean.append(p)
                    if len(clean)<2:continue
                    total+=len(clean)
                    if total>12000:raise ValueError('喷线图纸过密，请选择部分边')
                    total_length+=sum(math.dist(a,b) for a,b in zip(clean,clean[1:]))
                    if total_length>3000:raise ValueError('喷线图纸过长，请选择部分边')
                    strokes.append(polyline(str(i),clean))
                if not strokes:raise ValueError('没有可计算的喷线')
                settings=SpraySettings.parse(request.get('settings'))
                speed=finite(request.get('paint_speed',.8),'划线速度',.05,1.2)
                spray=SimulatedSpray(strokes,settings,speed)
                self.ident=ident;self.spray=spray;self.previous=None;self.finished=False
                self.good_traces=[];self.bad_traces=[];self.trace_kind=None
            for sample in checked:self.update(sample)
            return self.view()

    def update(self,sample):
        now=sample['time'];previous=self.previous
        if previous is not None and now<=previous:return
        dt=.1 if previous is None else now-previous;self.previous=now
        spray=self.spray
        if not sample['valid'] or dt>.6:
            # 观测断档不能用最后位置继续积料，也不能在恢复时跨断档连线。
            for valve in spray.valves.values():valve.queue.clear();valve.flow=valve.target=valve.command=0.;valve.next_command=0.
            for coverage in spray.coverage.values():coverage.previous=None;coverage.observe(None)
            spray.joined=False;spray.last_point=None;self.valid=False;self.trace_kind=None
            return
        self.valid=True;point=sample['point'];speed=sample['speed']
        row=min(spray.coverage['optimized'].rows.values(),key=lambda r:project(point,r['stroke'])[2])
        stroke=row['stroke'];s,lateral,_=project(point,stroke);length=stroke_length(stroke)
        reverse=math.cos(sample['heading']-tangent(stroke,s))<0
        progress=length-s if reverse else s
        if closed(stroke):
            ratio=speed/max(.05,abs(sample['body_speed']))
            progress=length-sample['remaining']*min(3.,max(.1,ratio))
        # 正常圈使用全图，补喷圈只请求欠喷窗口；剂量统计仍累计在原图上，不清空已有覆盖。
        for target in spray.coverage['optimized'].rows.values():
            item=target['stroke']
            if sample['repair_only']:
                ranges=[(a*stroke_length(item),b*stroke_length(item)) for a,b in sample['paint_windows']]
                item['paint_ranges']=ranges
            else:item.pop('paint_ranges',None)
            if 'paint_ranges' in item:
                from spray_simulation import Pattern
                spray.patterns[item['id']]=Pattern(item)
        eligible=sample['eligible'] and not sample['finished']
        if not sample['painting']:
            # 仅允许接近线头的助跑提前开阀；经过图形中部的转场不能被误当成划线。
            lead=speed*(spray.settings.open_delay_s+spray.settings.response_s+spray.settings.command_period_s)
            eligible=eligible and not closed(stroke) and -.5<=progress<=lead and lateral<=spray.settings.offset_limit_m
        spray.update(now,min(.6,dt),point=point,speed=speed,painting=sample['painting'],eligible=eligible,
            progress=progress,paint_from=0.,length=length,stroke=stroke,estimate=point,heading=sample['heading'],allowed=sample['allowed'])
        for coverage in spray.coverage.values():coverage.observe(point if sample['painting'] or spray.last['spraying'] else None)
        if spray.last['spraying']:
            distance=project(point,stroke)[2]
            good=sample['allowed'] and distance<=spray.settings.offset_limit_m
            lines=self.good_traces if good else self.bad_traces
            if self.trace_kind!=good:lines.append([list(point)])
            elif math.dist(lines[-1][-1],point)>=.03:lines[-1].append(list(point))
            self.trace_kind=good
            while len(lines)>512 or sum(map(len,lines))>24000:
                if len(lines[0])>1:lines[0].pop(0)
                else:lines.pop(0)
        else:self.trace_kind=None
        self.finished=sample['finished']
        # 显示历史有界；喷量统计独立于轨迹裁剪，不会因裁剪减少积料统计。
        while len(spray.trails)>512 or sum(map(len,spray.trails))>24000:
            if len(spray.trails[0])>1:spray.trails[0].pop(0)
            else:spray.trails.pop(0)

    def view(self):
        if not self.spray:return dict(estimate_only=True,hardware_output=False,run=None,traces=[],missing=[],excess=[])
        result=self.spray.coverage['optimized'].result(visited_only=not self.finished)
        return dict(estimate_only=True,hardware_output=False,run=self.ident,valid=self.valid,finished=self.finished,
            flow_pct=self.spray.last['flow_pct'],command_pct=self.spray.last['command_pct'],
            traces=self.good_traces,missing=result.pop('missing_segments'),
            excess=result.pop('excess_segments')+self.bad_traces,metrics=result)


def handle(handler,trace):
    if handler.path.split('?')[0]!='/app/spray-trace':return False
    if handler.headers.get('Origin'):
        handler._send_json(dict(error='不接受网页跨源请求'),403);return True
    try:
        if handler.command=='GET':
            with trace.lock:value=trace.view()
        else:
            size=int(handler.headers.get('Content-Length',0))
            if not 0<size<=400000:raise ValueError('喷线观测数据过大')
            value=trace.receive(__import__('json').loads(handler.rfile.read(size)))
        handler._send_json(value)
    except (ValueError,TypeError,KeyError) as exc:handler._send_json(dict(error=str(exc)),400)
    return True
