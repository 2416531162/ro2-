"""纯数值模拟喷水：虚拟指令、延迟喷阀与落料覆盖；不导入任何硬件输出模块。"""
from collections import deque
from dataclasses import dataclass
from bisect import bisect_right
import math


@dataclass(frozen=True)
class SpraySettings:
    line_width_m: float = .05
    spray_width_m: float = .05
    open_delay_s: float = .15
    close_delay_s: float = .10
    response_s: float = .05
    command_period_s: float = .20
    base_pressure: float = 50.0
    sample_m: float = .02
    position_margin_m: float = 0.0
    offset_limit_m: float = .03

    @classmethod
    def parse(cls, raw):
        if raw is None: raw = {}
        if not isinstance(raw, dict): raise ValueError('模拟喷水设置须为对象')
        limits = dict(line_width_m=(.01,.5),spray_width_m=(.01,.5),open_delay_s=(0,2),
                      close_delay_s=(0,2),response_s=(0,1),command_period_s=(.02,1),
                      base_pressure=(1,100),sample_m=(.01,.10),position_margin_m=(0,.02),offset_limit_m=(.002,.10))
        unknown = set(raw) - set(limits)
        if unknown: raise ValueError('未知模拟喷水设置：'+','.join(sorted(unknown)))
        for key, value in raw.items():
            low, high = limits[key]
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
                raise ValueError('模拟喷水参数超出范围：'+key)
        return cls(**raw)


class VirtualValve:
    """虚拟百分比指令经过开/关延迟和一阶喷量响应；仅改变内存。"""
    def __init__(self, settings):
        self.settings=settings;self.queue=deque();self.command=self.target=self.flow=0.
        self.next_command=0.;self.sequence=0;self.applied_sequence=0

    def update(self, now, requested, dt):
        requested=max(0.,min(100.,float(requested)))
        if (requested==0. or now>=self.next_command-1e-9) and abs(requested-self.command)>.01:
            delay=self.settings.close_delay_s if requested==0 else self.settings.open_delay_s
            self.sequence+=1
            self.queue.append((now+delay,self.sequence,requested))
            self.command=requested;self.next_command=now+self.settings.command_period_s
        # 开关延迟不同，按生效时刻排序；迟到旧指令不能覆盖更新的关闭动作。
        due=[item for item in self.queue if item[0]<=now+1e-9]
        self.queue=deque(item for item in self.queue if item[0]>now+1e-9)
        for _,sequence,value in sorted(due):
            if sequence>self.applied_sequence:
                self.target=value;self.applied_sequence=sequence
        weight=1. if self.settings.response_s==0 else 1-math.exp(-dt/self.settings.response_s)
        self.flow+=(self.target-self.flow)*weight
        return self.flow


def stroke_length(stroke):
    if stroke['kind']=='polyline':return stroke['cumulative'][-1]
    return math.dist(stroke['a'],stroke['b']) if stroke['kind']=='line' else stroke['r']*abs(stroke['sweep'])


def polyline(ident,points):
    points=[list(p) for p in points]
    cumulative=[0.];grid={}
    for index,(a,b) in enumerate(zip(points,points[1:])):
        length=math.dist(a,b)
        if length<.001:raise ValueError('喷线采样点重复或过密')
        cumulative.append(cumulative[-1]+length)
        for k in range(max(1,math.ceil(length/.5))+1):
            u=min(1,k/max(1,math.ceil(length/.5)))
            cell=(math.floor(a[0]+u*(b[0]-a[0])),math.floor(a[1]+u*(b[1]-a[1])))
            grid.setdefault(cell,set()).add(index)
    return dict(id=ident,name=ident,kind='polyline',points=points,cumulative=cumulative,grid=grid,
                closed=math.dist(points[0],points[-1])<.001)


def closed(stroke):return stroke['kind']=='circle' or stroke.get('closed',False)


def tangent(stroke,s):
    if stroke['kind']=='polyline':
        i=min(len(stroke['points'])-2,max(0,bisect_right(stroke['cumulative'],s)-1))
        a,b=stroke['points'][i:i+2]
        return math.atan2(b[1]-a[1],b[0]-a[0])
    if stroke['kind']=='line':
        a,b=stroke['a'],stroke['b'];return math.atan2(b[1]-a[1],b[0]-a[0])
    angle=stroke['start']+(1 if stroke['sweep']>=0 else -1)*s/stroke['r']
    return angle+(math.pi/2 if stroke['sweep']>=0 else -math.pi/2)


class Pattern:
    def __init__(self,stroke):
        length=stroke_length(stroke);style=stroke.get('dash')
        if 'paint_ranges' in stroke:self.ranges=sorted(stroke['paint_ranges'])
        elif not style:self.ranges=[(0.,length)]
        else:
            if not isinstance(style,dict) or style.get('mode') not in ('length','count'):raise ValueError('虚线模式无效')
            if style['mode']=='count':
                count=style.get('count')
                if type(count) is not int or not 2<=count<=500:raise ValueError('虚线段数须为 2–500')
                dash=gap=length/(2*count-1)
            else:dash,gap=style.get('dash'),style.get('gap')
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<.05 for v in (dash,gap)):
                raise ValueError('虚线太短或间隔无效')
            n=math.ceil(length/(dash+gap))
            if n>500:raise ValueError('虚线过密，请缩小范围')
            self.ranges=[(i*(dash+gap),min(length,i*(dash+gap)+dash)) for i in range(n)]
        self.starts=[a for a,_ in self.ranges];self.ends=[b for _,b in self.ranges]
        self.prefix=[];total=0.
        for a,b in self.ranges:self.prefix.append(total);total+=b-a
        self.length=total

    def contains(self,s):
        index=bisect_right(self.starts,s)-1
        return index>=0 and s<self.ends[index]

    def before(self,s):
        index=bisect_right(self.starts,s)-1
        return 0. if index<0 else self.prefix[index]+min(max(0.,s-self.starts[index]),self.ends[index]-self.starts[index])

    def overlap(self,a,b):return max(0.,self.before(b)-self.before(a))


def project(point, stroke):
    x,y=point
    if stroke['kind']=='polyline':
        gx,gy=math.floor(x),math.floor(y);indices=set()
        for dx in (-1,0,1):
            for dy in (-1,0,1):indices.update(stroke['grid'].get((gx+dx,gy+dy),()))
        if not indices:indices=range(len(stroke['points'])-1)
        best=None
        for i in indices:
            a,b=stroke['points'][i:i+2];dx,dy=b[0]-a[0],b[1]-a[1];length=stroke['cumulative'][i+1]-stroke['cumulative'][i]
            u=((x-a[0])*dx+(y-a[1])*dy)/(length*length)
            bounded=min(1,max(0,u));distance=math.hypot(x-a[0]-bounded*dx,y-a[1]-bounded*dy)
            along=u if (i==0 and u<0 or i==len(stroke['points'])-2 and u>1) else bounded
            value=(stroke['cumulative'][i]+along*length,abs((x-a[0])*dy-(y-a[1])*dx)/length,distance)
            if best is None or distance<best[2]:best=value
        return best
    if stroke['kind']=='line':
        a,b=stroke['a'],stroke['b'];dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        s=((x-a[0])*dx+(y-a[1])*dy)/length
        lateral=abs((x-a[0])*dy-(y-a[1])*dx)/length
        distance=math.hypot(lateral,max(0.,-s,s-length))
        return s,lateral,distance
    cx,cy=stroke['c'];r=stroke['r'];sign=1 if stroke['sweep']>=0 else -1
    angle=math.atan2(y-cy,x-cx)
    rel=((angle-stroke['start'])*sign)%(2*math.pi)
    if stroke['kind']=='circle':return rel*r,abs(math.hypot(x-cx,y-cy)-r),abs(math.hypot(x-cx,y-cy)-r)
    sweep=abs(stroke['sweep'])
    if rel>sweep:
        if rel>math.pi+sweep/2:rel-=2*math.pi
    lateral=abs(math.hypot(x-cx,y-cy)-r)
    distance=math.hypot(lateral,r*max(0.,-rel,rel-sweep))
    return rel*r,lateral,distance


def point_at(stroke, s):
    if stroke['kind']=='polyline':
        i=min(len(stroke['points'])-2,max(0,bisect_right(stroke['cumulative'],s)-1));a,b=stroke['points'][i:i+2]
        u=(s-stroke['cumulative'][i])/(stroke['cumulative'][i+1]-stroke['cumulative'][i])
        return [a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1])]
    if stroke['kind']=='line':
        a,b=stroke['a'],stroke['b'];u=s/stroke_length(stroke)
        return [a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u]
    sign=1 if stroke['sweep']>=0 else -1;angle=stroke['start']+sign*s/stroke['r']
    return [stroke['c'][0]+stroke['r']*math.cos(angle),stroke['c'][1]+stroke['r']*math.sin(angle)]


class Coverage:
    """按原图几何分箱累加模拟料量；停车积料、越界料量及重复通过均不会被删掉。"""
    def __init__(self, strokes, settings):
        self.settings=settings;self.rows={};self.outside=0.;self.total=0.;self.previous=None;self.observed=None
        for stroke in strokes:
            length=stroke_length(stroke);n=max(1,math.ceil(length/settings.sample_m))
            pattern=Pattern(stroke);step=length/n
            self.rows[stroke['id']]={'stroke':stroke,'length':length,'step':step,'dose':[0.]*n,
                'seen':[False]*n,'pattern':pattern,'desired':[pattern.overlap(i*step,(i+1)*step)/step for i in range(n)]}
        if sum(len(row['dose']) for row in self.rows.values())>150000:raise ValueError('模拟图纸过长，请缩小范围')

    def deposit(self, point, amount):
        previous=self.previous;self.previous=point
        if amount<=0 or not self.rows:return
        self.total+=amount
        row=None;projection=None
        for candidate in self.rows.values():
            p=project(point,candidate['stroke'])
            if projection is None or p[2]<projection[2]:row,projection=candidate,p
        s,lateral,_=projection
        desired_half=self.settings.line_width_m/2;spray_half=self.settings.spray_width_m/2
        overlap=max(0.,min(desired_half,lateral+spray_half)-max(-desired_half,lateral-spray_half))
        fraction=min(1.,overlap/self.settings.spray_width_m)
        s0=project(previous,row['stroke'])[0] if previous is not None else s
        if closed(row['stroke']):
            delta=s-s0
            if delta>row['length']/2:s0+=row['length']
            elif delta<-row['length']/2:s0-=row['length']
        lo,hi=sorted((s0,s));inside=0.
        if hi-lo<1e-8:
            if 0<=s<row['length']:
                index=min(len(row['dose'])-1,int(s/row['step']))
                row['dose'][index]+=amount*fraction/row['step'];inside=fraction if row['pattern'].contains(s) else 0.
        else:
            # 将一帧内的连续喷线积分到所有穿过的分箱，不用单点计数冒充覆盖。
            for shift in ((-row['length'],0.,row['length']) if closed(row['stroke']) else (0.,)):
                a,b=max(0.,lo+shift),min(row['length'],hi+shift)
                if b<=a:continue
                first,last=int(a/row['step']),min(len(row['dose'])-1,int(b/row['step']))
                for index in range(first,last+1):
                    overlap_s=max(0.,min(b,(index+1)*row['step'])-max(a,index*row['step']))
                    part=fraction*overlap_s/(hi-lo)
                    row['dose'][index]+=amount*part/row['step']
                inside+=fraction*row['pattern'].overlap(a,b)/(hi-lo)
        self.outside+=amount*max(0.,1-inside)

    def observe(self,point):
        if point is None:self.observed=None;return
        previous=self.observed;self.observed=point
        if not self.rows:return
        row=min(self.rows.values(),key=lambda r:project(point,r['stroke'])[2])
        s=project(point,row['stroke'])[0];s0=project(previous,row['stroke'])[0] if previous is not None else s
        if closed(row['stroke']):
            if s-s0>row['length']/2:s0+=row['length']
            elif s-s0<-row['length']/2:s0-=row['length']
        lo,hi=sorted((s0,s))
        for shift in ((-row['length'],0.,row['length']) if closed(row['stroke']) else (0.,)):
            a,b=max(0.,lo+shift),min(row['length'],hi+shift)
            if b<a:continue
            first=min(len(row['seen'])-1,int(a/row['step']));last=min(len(row['seen'])-1,int(b/row['step']))
            for i in range(first,last+1):row['seen'][i]=True

    def result(self,visited_only=False):
        missing=excess=length=under=over=0.;rows=[];miss_segments=[];excess_segments=[]
        for row in self.rows.values():
            dose=row['dose'];desired=row['desired'];step=row['step'];stroke=row['stroke']
            seen=row['seen'] if visited_only else [True]*len(dose)
            length+=sum(w for w,on in zip(desired,seen) if on)*step
            miss=sum(w for v,w,on in zip(dose,desired,seen) if on and v<w*.85)*step
            extra=sum(v>max(.15,w*1.15) for v,w in zip(dose,desired))*step
            missing+=miss;excess+=extra;under+=sum(max(0.,w-v) for v,w in zip(dose,desired))*step;over+=sum(max(0.,v-w) for v,w in zip(dose,desired))*step
            rows.append(dict(id=stroke['id'],name=stroke['name'],length_m=row['pattern'].length,missing_m=miss,excess_m=extra,
                             mean_dose=sum(dose)/len(dose),min_dose=min(dose),max_dose=max(dose)))
            for predicate, output in ((lambda v,w:v<w*.85,miss_segments),(lambda v,w:v>max(.15,w*1.15),excess_segments)):
                start=None
                for index,value in enumerate([*dose,1.]):
                    on=index<len(dose) and (seen[index] or value>0) and predicate(value,desired[index])
                    if on and start is None:start=index
                    elif not on and start is not None:
                        lo,hi=start*step,index*step;n=max(1,math.ceil((hi-lo)/.2))
                        if len(output)<500:output.append([point_at(stroke,lo+(hi-lo)*k/n) for k in range(n+1)])
                        start=None
        return dict(target_length_m=length,missing_length_m=missing,excess_length_m=excess,
                    coverage_pct=max(0.,min(100.,100*(1-missing/length))) if length else 0.,under_material_m=under,
                    over_material_m=over,off_line_material_m=self.outside,total_material_m=self.total,
                    elements=rows,missing_segments=miss_segments,excess_segments=excess_segments)


class SimulatedSpray:
    """同一虚拟轨迹并行比较原始时机与补偿；控制决策只用当前位置/速度和规划进度。"""
    def __init__(self, strokes, settings, paint_speed):
        self.settings=settings;self.paint_speed=paint_speed;self.previous_speed=0.
        self.valves={name:VirtualValve(settings) for name in ('baseline','optimized')}
        self.coverage={name:Coverage(strokes,settings) for name in self.valves}
        self.patterns={s['id']:Pattern(s) for s in strokes}
        self.trails=[];self.joined=False;self.last_point=None
        self.last=dict(command_pct=0.,flow_pct=0.,spraying=False)

    def update(self, now, dt, *, point, speed, painting, eligible, progress, paint_from, length, scale=1.,stroke=None,estimate=None,heading=0.,allowed=True):
        speed=abs(speed)*scale
        requested=0.
        if eligible and speed>.02:
            # 提前发虚拟开/关指令；量随喷头地面速度变化，停顿不沿用巡航喷量。
            lead=self.settings.open_delay_s+self.settings.response_s+self.settings.command_period_s/2
            stop=self.settings.close_delay_s+self.settings.response_s+self.settings.command_period_s/2
            if progress+speed/scale*lead>=paint_from and progress+speed/scale*stop<length:
                acceleration=max(-2.,min(2.,(speed-self.previous_speed)/dt))
                predicted=max(0.,speed+acceleration*(self.settings.open_delay_s+self.settings.response_s))
                requested=min(100.,self.settings.base_pressure*predicted/self.paint_speed)
        if requested>0 and stroke and estimate is not None:
            # 偏离目标时先关指令；不能通过增大喷量补偿已经偏出线的喷头。
            if project(estimate,stroke)[1]>self.settings.offset_limit_m:
                requested=0.
        if stroke and (stroke.get('dash') or 'paint_ranges' in stroke):
            pattern=self.patterns[stroke['id']];s=project(estimate if estimate is not None else point,stroke)[0]
            tangent_yaw=tangent(stroke,s)
            direction=1 if math.cos(heading-tangent_yaw)>=0 else -1
            lead=(self.settings.close_delay_s if self.valves['optimized'].command>0 else self.settings.open_delay_s)+self.settings.response_s+self.settings.command_period_s/2
            future=s+direction*speed*lead
            if closed(stroke):future%=stroke_length(stroke)
            if not pattern.contains(future):requested=0.
            painting=painting and pattern.contains(s)
        if not allowed:requested=0.
        self.previous_speed=speed
        commands={'baseline':self.settings.base_pressure if painting else 0.,'optimized':requested}
        for name,valve in self.valves.items():
            flow=valve.update(now,commands[name],dt)
            amount=flow/self.settings.base_pressure*self.paint_speed*dt
            self.coverage[name].deposit(point,amount)
        flow=self.valves['optimized'].flow
        spraying=flow>.5
        if spraying:
            if not self.joined:self.trails.append([list(point)]);self.joined=True
            elif self.last_point is None or math.dist(point,self.last_point)>=.03:self.trails[-1].append(list(point))
            self.last_point=point
        else:self.joined=False;self.last_point=None
        self.last=dict(command_pct=requested,flow_pct=flow,spraying=spraying)
        return self.last

    def result(self):
        target=[]
        for row in self.coverage['optimized'].rows.values():
            for a,b in row['pattern'].ranges:
                n=max(1,min(2048,math.ceil((b-a)/.2)))
                target.append([point_at(row['stroke'],a+(b-a)*i/n) for i in range(n+1)])
        return dict(simulation_only=True,hardware_output=False,model='虚拟延迟喷阀＋规划位置和速度补偿',
                    baseline=self.coverage['baseline'].result(),optimized=self.coverage['optimized'].result(),
                    traces=self.trails,target=target,settings=vars(self.settings))
