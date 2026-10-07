"""Position-aware field planning for a differential chassis with a right-hand nozzle.

The objective prefers complete-body interior travel. Where outside space is
unavoidable, it balances exposure and elapsed time. Wet paint is not an input. This produces geometric
vehicle paths; it does not operate a pump or claim measured paint coverage.
"""
import math
from itertools import product
from dataclasses import dataclass, replace
import route_plan as rp
from path_tracker import Path, Section
from motion_speeds import DEFAULT_PAINT_SPEED, DEFAULT_TRANSIT_SPEED, MAX_SECTION_SPEED

TAU = 2 * math.pi
# 每条刷线段在线头前接一段助跑，助跑不喷（motion 的 paint_from）。差速车每段都从停车起步、先慢走（万向轮翻过来、
# 低速轮速不平顺），2026-10-03 带万向轮的整场仿真：线头 1 m 内 95% 横偏 2.05 cm、占 >2 cm 的 81%；路线作业倒车
# 助跑按喷头实际行程2.5m（route_plan.RUN_UP_M），先0.3m/s对正（path_tracker SectionTracker），再在线头前接到作业速度。
# 圆弧的助跑后一半（PRE_ARC_FRACTION）沿同一个圆先转起来：直线进弧曲率一步跳上去，进弧误差全看执行延迟估得准不准
# （角球弧 R1 m：直线助跑入口均值 −0.6 ~ −2.5 cm 随底盘延迟变），先转起来到线头时已是稳态（入口 0.9 cm、各延迟一致）
RUN_IN_M = rp.RUN_UP_M
PRE_ARC_FRACTION = 0.5
# 场外可用空地（米）：2026-10-03 用户「有 2 m 空地，可以拔掉（角旗）」。车身伸出场地在这以内只作轻微偏好（不再为几十
# 厘米外伸绕成三段停车倒车，角上一次转场 25 s → 一段倒车）；超出的仍按原来的重罚，规划仍报告最大外伸供操作员确认
OUTSIDE_SPACE_M = 2.0
# 转场的掉头方式（2026-10-03 用户：原地掉头太僵硬，app 可选，再加一种像阿克曼那样方向打满的掉头）：
#   pivot  原地转：停车原地转向 → 直线 → 原地转（默认，原来的做法）
#   arc    圆弧：始终向前，按 turn_radius 绕 Dubins 弧线接入；相切段连续交接，实际外伸仍进入预览确认。
#   lock   打满方向：用车能稳定跟踪的最小半径（cfg.radius），前进 / 倒车弧线（Reeds–Shepp，可三点掉头），不原地转
# arc / lock 转场最后一段往前开时与「助跑 + 刷线」合成一段（paint_from 往后挪），进线不停车。
# 默认仍是 pivot：10-03 Gen5 标定整场仿真（6 场）arc R1.0 用时 +1%、95% 与全场最大误差相当，但线头 1 m 内最大中位
# 2.0 cm（pivot 1.6），每场超 1.5 cm 的线头 2.5 条（pivot 1.2）——角上只能倒车进助跑，往前起步万向轮翻转甩车头。
# 试过不合段停稳再进、往前摆正 1–2 m、沿线倒车进线、补转门槛 1.5°、加长慢走 / 锁车头，都没追平。
TURN_STYLES = ('pivot', 'arc', 'lock')
RS_CANDIDATES = 6     # 弧线转场每次取最短的几条走法按用时 / 外伸细比（全算会让整场规划慢很多）
RS_MIN_RUN_M = 0.3    # 换向之间至少走这么远：更短的一小段前进 / 倒车实车跟不准（万向轮翻转、低速不平顺、停车容差）
# 弧线转场最后沿下一条线的方向往前直开这么远再进助跑；场边放不下就缩短、不加
TURN_ALIGN_M = (0.8, 0.4, 0.0)
# 圆弧方式接进圆弧（角球弧等）：从上一条线的终点只往前开出去、与弧同向绕一圈切进这条弧自己的圆，沿圆先走一段
# （≥ 助跑里沿圆的那一半）再喷——不倒车、不原地转，转场和刷弧一段开完（2026-10-03 18:32 用户手绘：底线画到头继续往前，
# 右转绕出去一圈接上角球弧接着画，要看着丝滑；「不用担心右边空间」）。绕圈的车身外伸在 LOOP_SPACE_M 以内按场外空地以内排，
# 规划报告的最大外伸照实给
LOOP_SPACE_M = 4.0
LOOP_STEP_DEG = 5
# 进线前最后一段尽量往前开：倒车到助跑起点再往前起步，万向轮要翻转 100° 以上，车头甩 1.4–1.9°，到线头纠不回来
# （2026-10-03 Gen5 标定仿真：边线、角球弧线头 1.5–1.9 cm 都是这样来的；原地转的做法是倒完再原地转，万向轮先转到侧面）


def number(v, name, lo, hi):
    return rp._number(v, name, lo, hi)


def point(v):
    return rp._pair(v, '场地坐标', (-1000, -1000), (1000, 1000))


def polygon(raw):
    if not isinstance(raw, list) or not 3 <= len(raw) <= 64:
        raise ValueError('请提供 3–64 个场地边界顶点')
    p = [point(q) for q in raw]
    if p[0] == p[-1]:
        p.pop()
    area = sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(p, p[1:]+p[:1]))
    if abs(area) < .01:
        raise ValueError('场地边界没有有效面积')
    if area < 0:
        p.reverse()
    planes = []
    for a,b in zip(p,p[1:]+p[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length < .05:
            raise ValueError('场地边界有重复或过近的点')
        nx,ny=-dy/length,dx/length;c=nx*a[0]+ny*a[1]
        if any(nx*q[0]+ny*q[1]<c-1e-6 for q in p):
            raise ValueError('当前内场规划需要凸形场地边界；请先分区')
        planes.append((nx,ny,c))
    return p,planes


@dataclass(frozen=True)
class Settings:
    forward: float
    right: float
    body: tuple
    radius: float
    paint_speed: float = DEFAULT_PAINT_SPEED
    transit_speed: float = DEFAULT_TRANSIT_SPEED
    yaw_rate: float = .8
    accel: float = .6
    yaw_accel: float = 1.0
    settle: float = .3
    margin: float = .0
    lateral_accel: float = .05
    run_out: float = rp.LINE_RUN_OUT_M
    run_in: float = RUN_IN_M
    # 边线、中线、角球弧的助跑朝场外（全长 1.2 m 时车身外伸 0.78 → 1.98 m、角球弧预转弯 1.03 → 2.05 m）。
    # 助跑让车身伸出场地不超过 run_in_outside_max（米，2026-10-03 用户：场外有 2 m 空地、角旗可拔），这条线本身
    # 已超过的只许再多 run_in_outside；放不下就缩短助跑（2/3、1/3），再不行不接
    run_in_outside: float = .05
    run_in_outside_max: float = OUTSIDE_SPACE_M
    # 用时估算：跟踪器每段起步慢走（path_tracker start_creep_m / reverse_section_creep_m、start_creep_mps，
    # Gen5 0.3 / 0.15 m、0.1 m/s），不算的话整场实际比估算多约 13%
    creep_m: float = .3
    reverse_creep_m: float = .15
    creep_mps: float = .1
    # 每段起步 / 到点停车比按加速度算的多花的时间、原地转（>5°）连停车停稳多花的时间（2026-10-03 按 Gen5 实车标定底盘
    # 整场仿真校准：前进段 +1.1–1.5 s、倒车段 +0.7 s、原地转 +1.7 s）；不计的话规划会把多停几次车当成很便宜
    section_s: float = 1.3
    reverse_section_s: float = .7
    pivot_s: float = 1.7
    turn: str = 'pivot'          # 转场掉头方式，见 TURN_STYLES
    turn_radius: float = 1.0     # turn='arc' 的弧线半径（米，app 1.0 / 1.5 / 2.0）；lock 用 radius

    def validate(self):
        number(self.forward,'喷头前伸',-3,3);number(self.right,'喷头右偏',.001,2)
        number(self.radius,'最小转弯半径',.1,20)
        if len(self.body)!=3:raise ValueError('车身尺寸无效')
        for v in self.body:number(v,'车身尺寸',.01,5)
        number(self.paint_speed,'划线速度',.05,1.2)
        number(self.transit_speed,'转场速度',.05,MAX_SECTION_SPEED)
        number(self.yaw_rate,'转向速度',.1,2);number(self.accel,'加速度',.05,2)
        number(self.yaw_accel,'转向加速度',.05,3);number(self.margin,'边界余量',0,1)
        number(self.lateral_accel,'弯道加速度',.01,2)
        number(self.run_out,'开出长度',0,.5);number(self.run_in,'助跑长度',0,5);number(self.run_in_outside,'助跑外伸',0,10);number(self.run_in_outside_max,'助跑外伸上限',0,10);number(self.creep_m,'起步慢走',0,2);number(self.reverse_creep_m,'倒车起步慢走',0,2)
        number(self.creep_mps,'慢走车速',0,1)
        number(self.section_s,'每段附加时间',0,10);number(self.reverse_section_s,'倒车段附加时间',0,10);number(self.pivot_s,'原地转附加时间',0,10)
        if self.turn not in TURN_STYLES:raise ValueError('掉头方式无效：'+str(self.turn))
        number(self.turn_radius,'掉头半径',self.radius,20)


def duration(distance,speed,accel):
    return 2*math.sqrt(distance/accel) if distance<speed*speed/accel else distance/speed+speed/accel


def corner_positions(pose,body):
    x,y,h=pose;c,s=math.cos(h),math.sin(h)
    f,r,w=body
    return [(x+a*c-b*s,y+a*s+b*c) for a,b in ((f,w),(f,-w),(-r,-w),(-r,w))]


def violation(pose,planes,body,margin=0):
    return max(0.0,max(c+margin-nx*x-ny*y for x,y in corner_positions(pose,body) for nx,ny,c in planes))


def angular_min(a,b,start,delta):
    """Exact minimum of a*cos(h)+b*sin(h) on a signed angular interval."""
    end=start+delta;lo,hi=sorted((start,end))
    v=min(a*math.cos(start)+b*math.sin(start),a*math.cos(end)+b*math.sin(end))
    theta=math.atan2(b,a)+math.pi
    k=math.ceil((lo-theta)/TAU-1e-12)
    if theta+k*TAU<=hi+1e-12:v=min(v,-math.hypot(a,b))
    return v


def arc_violation(center,start,delta,vectors,planes,margin=0):
    return max(0.0,max(c+margin-nx*center[0]-ny*center[1]-angular_min(nx*a+ny*b,-nx*b+ny*a,start,delta)
                       for a,b in vectors for nx,ny,c in planes))


def path_violation(section,planes,cfg):
    """Continuous footprint check, including between sampled preview points."""
    body=cfg.body;f,r,w=body;corners=((f,w),(f,-w),(-r,-w),(-r,w));worst=0.0
    for p in section.path.pieces:
        if abs(p.kappa)<1e-12:
            poses=[p.pose(0),p.pose(p.length)]
            worst=max(worst,*(violation((x,y,h+(math.pi if section.direction<0 else 0)),planes,body,cfg.margin) for x,y,h in poses))
        else:
            x,y,h=p.pose(0);rad=1/p.kappa
            center=(x-rad*math.sin(h),y+rad*math.cos(h))
            # Path heading is travel heading. Reverse corners are rotated by pi.
            vectors=[(section.direction*a,section.direction*b-rad) for a,b in corners]
            worst=max(worst,arc_violation(center,h,p.kappa*p.length,vectors,planes,cfg.margin))
    return worst


def pivot_violation(pose,heading,planes,cfg):
    f,r,w=cfg.body
    return arc_violation(pose[:2],pose[2],rp.wrap(heading-pose[2]),((f,w),(f,-w),(-r,-w),(-r,w)),planes,cfg.margin)


def transit_limit(section,cfg):
    # 执行器按前方曲率动态减速，不把一段中的短弯道速度套到整段长直线上。
    return min(cfg.transit_speed,.3) if section.direction<0 else cfg.transit_speed


def speed_for(section,cfg,paint=False,nozzle_radius=None):
    speed=cfg.paint_speed if paint else min(cfg.transit_speed,.3) if section.direction<0 else cfg.transit_speed
    k=section.path.kappa_abs_max
    if paint and nozzle_radius and k>1e-9:speed*=1/k/nozzle_radius
    return max(.02,min(MAX_SECTION_SPEED,speed,cfg.yaw_rate/k,math.sqrt(cfg.lateral_accel/k))) if k>1e-9 else min(MAX_SECTION_SPEED,speed)


def creep_time(section,cfg,speed):
    """每段起步慢走比按巡航加速多花的时间。"""
    creep=min(section.path.length,cfg.creep_m if section.direction>0 else cfg.reverse_creep_m)
    if creep<=0 or cfg.creep_mps<=0 or speed<=cfg.creep_mps:return 0.0
    return creep/cfg.creep_mps-duration(creep,speed,cfg.accel)


def pivot_time(angle,cfg):
    """原地转 angle（弧度）连停车停稳的时间；小于跟踪器 pivot_min（5°）不原地转。"""
    if angle<=1e-6:return 0.0
    return duration(angle,cfg.yaw_rate,cfg.yaw_accel)+cfg.settle+(cfg.pivot_s if angle>math.radians(5) else 0.0)


def path_score(section,planes,cfg,speed):
    length=section.path.length;peak=path_violation(section,planes,cfg)
    t=(duration(length,speed,cfg.accel)+cfg.settle+creep_time(section,cfg,speed)
       +(cfg.section_s if section.direction>0 else cfg.reverse_section_s))
    if peak<=1e-7:return (0.0,0.0,t)
    # Exposure is an optimization estimate; zero/maximum comes from the exact check.
    n=max(2,min(160,math.ceil(length/.3)))
    depth=sum(violation(section.car_pose(length*(i+.5)/n),planes,cfg.body,cfg.margin) for i in range(n))/n
    return (max(depth,peak/n)*t,peak,t)


def add_scores(*scores):
    return (sum(s[0] for s in scores),max((s[1] for s in scores),default=0),sum(s[2] for s in scores))


def score_key(score):
    # 车身伸出场地不超过场外可用空地（OUTSIDE_SPACE_M）的走法优先；空地以内的外伸只轻罚（1 倍暴露），不为几十厘米绕远；
    # 超出空地的按原来的重罚（8 倍暴露、每米峰值 30 s）。这些是排序罚分，不是预估的实际耗时或速度指令。
    over=score[1]>OUTSIDE_SPACE_M+1e-6
    penalty=(8*score[0]+30*(score[1]-OUTSIDE_SPACE_M)) if over else score[0]
    return (over,round(score[2]+penalty,5),round(score[1],5),round(score[2],5))


def footprint_key(score):
    return (score[1]>1e-6,round(score[1],5),round(score[0],5))


def parse_selection(raw):
    if not isinstance(raw,list) or not 1<=len(raw)<=80:
        raise ValueError('请至少选择一条边，一次最多选择 80 条')
    selected=set()
    for entry in raw:
        if not isinstance(entry,dict) or set(entry)!= {'element_id','edge_index'}:
            raise ValueError('选边格式无效，请重新选择')
        ident,index=entry['element_id'],entry['edge_index']
        if not isinstance(ident,str) or not 1<=len(ident)<=100:
            raise ValueError('选中的图形编号无效')
        if index is not None and (type(index) is not int or not 0<=index<400):
            raise ValueError('选中的边编号无效')
        key=(ident,index)
        if key in selected:raise ValueError('选中的边重复')
        selected.add(key)
    return selected


def parse_strokes(raw,selected=None,max_strokes=80):
    if not isinstance(raw,list) or not 1<=len(raw)<=80:raise ValueError('场地须有 1–80 个图形')
    strokes=[];manual=[];ids=set();matched=set()
    def add(stroke,ident,index):
        key=(ident,index)
        if selected is not None and key not in selected:return
        matched.add(key)
        stroke.update(key=key,selection=dict(element_id=ident,edge_index=index))
        strokes.append(stroke)
        if len(strokes)>max_strokes:raise ValueError('图纸分段过多，请选择部分边')
    for item in raw:
        if not isinstance(item,dict):raise ValueError('图形格式无效')
        ident=item.get('id');name=item.get('name',ident)
        if not isinstance(ident,str) or not 1<=len(ident)<=100 or ident in ids:raise ValueError('图形 ID 缺失或重复')
        if not isinstance(name,str) or len(name)>100:raise ValueError('图形名称无效')
        ids.add(ident);kind=item.get('kind')
        if kind=='polyline':
            points=item.get('points')
            if not isinstance(points,list) or not 2<=len(points)<=400:raise ValueError('折线点数无效')
            points=[point(p) for p in points]
            if item.get('closed',False) and points[0]!=points[-1]:points.append(points[0])
            for i,(a,b) in enumerate(zip(points,points[1:])):
                if math.dist(a,b)<.05:raise ValueError('划线分段太短')
                add(dict(id=ident+'/'+str(i),name=name+' · '+str(i+1),kind='line',a=a,b=b),ident,i)
        elif kind in ('circle','arc'):
            c=point(item.get('center'));radius=number(item.get('radius'),'圆弧半径',.05,200)
            start=math.radians(number(item.get('start_deg',0),'圆弧起角',-3600,3600))
            sweep=TAU if kind=='circle' else math.radians(number(item.get('sweep_deg'),'圆弧范围',-360,360))
            if abs(sweep)*radius<.05:raise ValueError('圆弧太短')
            add(dict(id=ident,name=name,kind=kind,c=c,r=radius,start=start,sweep=sweep),ident,None)
        elif kind=='point':
            mark=dict(id=ident,name=name,point=point(item.get('center')),reason='点位需人工标记')
            if selected is None:manual.append(mark)
        else:raise ValueError('不支持的场地图形：'+str(kind))
    if selected is not None and matched!=selected:raise ValueError('选中的边已不存在，请重新选择；点位需人工标记')
    if not strokes or len(strokes)>max_strokes:raise ValueError('一次规划支持 1–%d 条线或圆弧' % max_strokes)
    return strokes,manual


def lead_in(axle,kappa,run):
    """线头前的助跑：圆弧（kappa≠0）后 PRE_ARC_FRACTION 沿同一个圆，前面直线；直线全是直线。返回 (起点位姿, pieces)。"""
    pre=run*PRE_ARC_FRACTION if abs(kappa)>1e-9 else 0.0
    x,y,h=axle
    if pre>0:
        # 从线头沿同一个圆倒着走 pre：车头转 180°、曲率反号前进 pre，再把车头转回来
        x,y,h=Path((x,y,h+math.pi),[(pre,-kappa)]).pose(pre);h=rp.wrap(h-math.pi)
    st=run-pre
    return (x-st*math.cos(h),y-st*math.sin(h),h),([(st,0.0)] if st>1e-9 else [])+([(pre,kappa)] if pre>0 else [])


def with_run_in(axle,pieces,cfg,planes):
    """后轴从 axle 开始走 pieces；前面接助跑（lead_in）。助跑让车身伸出场地超过 run_in_outside_max（这条线本身已超过的，
    超过本身 + run_in_outside）就缩短（全长、2/3、1/3），都不行不接。返回 (Section, 助跑长度)。"""
    bare=Section(1,Path(axle,pieces))
    if cfg.run_in<=0:return bare,0.0
    limit=max(path_violation(bare,planes,cfg)+cfg.run_in_outside,cfg.run_in_outside_max)+1e-9
    for run in (cfg.run_in,cfg.run_in*2/3,cfg.run_in/3):
        start,lead=lead_in(axle,pieces[0][1],run)
        sec=Section(1,Path(start,lead+list(pieces)))
        if path_violation(sec,planes,cfg)<=limit:return sec,run
    return bare,0.0


def stroke_options(stroke,cfg,planes):
    a,b=cfg.forward,-cfg.right;out=[]
    if stroke['kind']=='line':
        for reverse in (False,True):
            q0,q1=(stroke['b'],stroke['a']) if reverse else (stroke['a'],stroke['b'])
            h=math.atan2(q1[1]-q0[1],q1[0]-q0[0]);c,s=math.cos(h),math.sin(h)
            axle=(q0[0]-a*c+b*s,q0[1]-a*s-b*c,h)
            sec,run=with_run_in(axle,[(math.dist(q0,q1),0)]+([(cfg.run_out,0)] if cfg.run_out>0 else []),cfg,planes)
            out.append(dict(section=sec,reverse=reverse,target=[q0,q1],nozzle_length=math.dist(q0,q1),paint_from=run,paint_to=run+math.dist(q0,q1)))
    else:
        r=stroke['r']
        if r<=abs(a)+1e-6:return []
        root=math.sqrt(r*r-a*a)
        starts=[i*TAU/8 for i in range(8)] if stroke['kind']=='circle' else [stroke['start']]
        for start in starts:
            for reverse in (False,True):
                sweep=stroke['sweep']*(-1 if reverse else 1)
                angle=start+(stroke['sweep'] if reverse and stroke['kind']=='arc' else 0)
                sign=1 if sweep>0 else -1;rear_radius=root+sign*b
                if rear_radius<cfg.radius-1e-6:continue
                h=angle+sign*math.pi/2-math.atan2(sign*a,root)
                cx,cy=stroke['c'];qx=cx+r*math.cos(angle);qy=cy+r*math.sin(angle)
                axle=(qx-a*math.cos(h)+b*math.sin(h),qy-a*math.sin(h)-b*math.cos(h),h)
                sec,run=with_run_in(axle,[(rear_radius*abs(sweep),sign/rear_radius),(rp.CIRCLE_RUN_OUT_M,sign/rear_radius)],cfg,planes)
                out.append(dict(section=sec,reverse=reverse,target_arc=(cx,cy,r,angle,sweep),
                                nozzle_length=r*abs(sweep),nozzle_radius=r,paint_from=run,paint_to=run+rear_radius*abs(sweep)))
                if stroke['kind']=='arc':
                    out[-1]['circle']=dict(pose=axle,kappa=sign/rear_radius,length=rear_radius*abs(sweep))
    circle_scores={}
    for option in out:
        sec=option['section'];speed=speed_for(sec,cfg,True,option.get('nozzle_radius'))
        # A full circle's footprint is independent of its chosen starting angle.
        key=option['reverse']
        if stroke['kind']!='circle' or key not in circle_scores:
            circle_scores[key]=path_score(sec,planes,cfg,speed)
        option.update(id=stroke['id'],key=stroke.get('key',(stroke['id'],None)),selection=stroke.get('selection'),
                      name=stroke['name'],speed=speed,start=sec.car_pose(0),end=sec.car_pose(sec.path.length),
                      score=circle_scores[key])
    return out


def target_points(option):
    if 'target' in option:return option['target']
    cx,cy,r,angle,sweep=option['target_arc']
    n=max(8,min(2048,math.ceil(r*abs(sweep)/.2)))
    return [(cx+r*math.cos(angle+sweep*i/n),cy+r*math.sin(angle+sweep*i/n)) for i in range(n+1)]


def clamp_inside(p,planes,inset):
    x,y=p
    for _ in range(20):
        changed=False
        for nx,ny,c in planes:
            d=c+inset-nx*x-ny*y
            if d>1e-8:x+=nx*d;y+=ny*d;changed=True
        if not changed:break
    return x,y


def merge_saving(cfg):
    """转场最后一段往前开、直接接进助跑（合成一段不停车）省下的时间：刷线段自己的起步停车和起步慢走。"""
    creep=cfg.creep_m/cfg.creep_mps-duration(cfg.creep_m,cfg.paint_speed,cfg.accel) if cfg.creep_mps>0 and cfg.paint_speed>cfg.creep_mps else 0.0
    return cfg.section_s+cfg.settle+max(0.0,creep)


def dubins_paths(start,goal,radius):
    """start → goal 只往前的全部 Dubins 走法（六种里成立、积分验算过终点的），每个是 [(1, 长度, 曲率), …]。
    Reeds–Shepp 每类只给最短的一条，绕一大圈只往前开的走法不在里面，要单独加。"""
    x0,y0,yaw0=start;dx,dy=goal[0]-x0,goal[1]-y0;dist=math.hypot(dx,dy)
    if dist<1e-9 and abs(rp.wrap(goal[2]-yaw0))<1e-9:return []
    theta=math.atan2(dy,dx);alpha,beta=rp._mod2pi(yaw0-theta),rp._mod2pi(goal[2]-theta)
    curv={'L':1/radius,'R':-1/radius,'S':0.0};out=[]
    for word,*lengths in rp._dubins_words(alpha,beta,dist/radius):
        segs=[(1,u*radius,curv[c]) for c,u in zip(word,lengths) if u*radius>1e-6]
        x,y,yaw=start
        for _,length,kappa in segs:x,y,yaw=rp.advance(x,y,yaw,length,kappa)
        if math.hypot(x-goal[0],y-goal[1])<=1e-6*max(1.0,dist) and abs(rp.wrap(yaw-goal[2]))<=1e-6:out.append(segs)
    return out


def turn_routes(start,goal,planes,cfg,align=0.0):
    """不原地转的转场候选 [(sections, score, [])]：arc 按 turn_radius，lock 按最小半径；Reeds–Shepp 全部走法里
    先挑最短的几条（arc 只往前的排前面），再按用时 / 外伸算分。最后一段往前开的会与刷线段合段，分里先扣掉省下的停车。"""
    radius=cfg.turn_radius if cfg.turn=='arc' else cfg.radius
    gx,gy,gh=goal
    pre=(gx-align*math.cos(gh),gy-align*math.sin(gh),gh)
    try:paths=rp.reeds_shepp_paths(start,pre,radius)+dubins_paths(start,pre,radius)
    except (ValueError,ZeroDivisionError):paths=[]
    paths=[segs+([(1,align,0.0)] if align>0 else []) for segs in paths]
    ranked={True:[],False:[],None:[]}
    for segs in paths:
        runs=rp._runs(segs)
        if any(length<RS_MIN_RUN_M for _,length in runs):continue
        # 只往前 / 最后往前（中间可倒一把）/ 倒车进线 三类各取最短几条；圆弧方式少换向：每次换向按 2 m 排后（方向打满 0.5 m）
        kind=True if runs[0][0]>0 and len(runs)==1 else False if runs[-1][0]>0 else None
        ranked[kind].append((sum(length for _,length,_ in segs)+(2.0 if cfg.turn=='arc' else .5)*(len(runs)-1),segs))
    routes=[]
    for kind in (True,False,None):
        for _,segs in sorted(ranked[kind],key=lambda v:v[0])[:RS_CANDIDATES]:
            builder=rp._Sections(start)
            for direction,length,kappa in segs:builder.add(direction,length,kappa)
            sections=[Section(d,Path(p,pieces)) for d,p,pieces in builder.out]
            cost=(0,0,0)
            for sec in sections:cost=add_scores(cost,path_score(sec,planes,cfg,speed_for(sec,cfg)))
            if sections and sections[-1].direction>0:cost=(cost[0],cost[1],cost[2]-merge_saving(cfg))
            routes.append((sections,cost,[]))
    return routes


def transit(start,goal,planes,cfg):
    if cfg.turn=='arc':
        # 圆弧走法始终向前；外伸照实进入预览，由既有场外范围确认把关，不暗中降级为倒车/原地转。
        if math.dist(start[:2],goal[:2])<1e-6 and abs(rp.wrap(start[2]-goal[2]))<1e-6:
            return [],(0.,0.,0.),[]
        routes=[]
        for pieces in dubins_paths(start,goal,cfg.turn_radius):
            sec=Section(1,Path(start,[(length,k) for _,length,k in pieces]))
            routes.append(([sec],path_score(sec,planes,cfg,speed_for(sec,cfg)),[]))
        if not routes:raise ValueError('圆弧转场无法连续向前接入，请调整车位或转弯半径')
        return min(routes,key=lambda r:score_key(r[1]))
    if cfg.turn!='pivot':
        # 摆正直线从长到短试；arc 放得下就只往前开，lock 前进 / 倒车里挑最快的。
        # 都放不下（车身超出场外空地）才回到原地转的走法
        # 放得下的里面：arc 先要只往前的，再要最后一段往前进线的（两种方式都是），最后才允许倒车进线；
        # 摆正直线从长到短试。都放不下（车身超出场外空地）才回到原地转的走法
        tried=[];fallback=[]
        for align in TURN_ALIGN_M:
            smooth=turn_routes(start,goal,planes,cfg,align);tried+=smooth
            fits=[r for r in smooth if not score_key(r[1])[0]]
            forward=[r for r in fits if all(sec.direction>0 for sec in r[0])]
            ends_forward=[r for r in fits if not r[0] or r[0][-1].direction>0]
            if cfg.turn=='arc' and forward:return min(forward,key=lambda r:score_key(r[1]))
            if ends_forward:return min(ends_forward,key=lambda r:score_key(r[1]))
            fallback+=fits
        if fallback:return min(fallback,key=lambda r:score_key(r[1]))
        return min(tried+[transit(start,goal,planes,replace(cfg,turn='pivot'))],key=lambda r:score_key(r[1]))
    if math.dist(start[:2],goal[:2])<1e-6:
        angle=abs(rp.wrap(goal[2]-start[2]));peak=pivot_violation(start,goal[2],planes,cfg)
        t=pivot_time(angle,cfg)
        return [],(t*peak,peak,t),[(start,goal[2])] if angle>1e-6 else []
    routes=[]
    inset=math.hypot(max(cfg.body[0],cfg.body[1]),cfg.body[2])+cfg.margin+.05
    p=clamp_inside(start[:2],planes,inset);q=clamp_inside(goal[:2],planes,inset)
    alternatives=[[],[p],[q],[p,q]]
    distance=max(1.0,inset*1.5)
    for da in (-distance,distance):
        a=(start[0]+da*math.cos(start[2]),start[1]+da*math.sin(start[2]))
        for db in (-distance,distance):
            b=(goal[0]+db*math.cos(goal[2]),goal[1]+db*math.sin(goal[2]))
            alternatives.extend(([a,b],[a,clamp_inside(a,planes,inset),clamp_inside(b,planes,inset),b]))
    for via in alternatives:
        states=[(start,[],(0,0,0),[])]
        for end in via+[goal[:2]]:
            next_states=[]
            for pose,sections,score,pivots in states:
                dist=math.dist(pose[:2],end)
                if dist<1e-5:
                    next_states.append((pose,sections,score,pivots));continue
                travel=math.atan2(end[1]-pose[1],end[0]-pose[0])
                for direction in (1,-1):
                    heading=rp.wrap(travel+(math.pi if direction<0 else 0));angle=abs(rp.wrap(heading-pose[2]))
                    cost=score;turns=pivots
                    if angle>1e-5:
                        peak=pivot_violation(pose,heading,planes,cfg);t=pivot_time(angle,cfg)
                        cost=add_scores(cost,(t*peak,peak,t));turns=pivots+[(pose,heading)]
                    sec=Section(direction,Path((pose[0],pose[1],travel),[(dist,0)]))
                    cost=add_scores(cost,path_score(sec,planes,cfg,speed_for(sec,cfg)))
                    next_states.append(((end[0],end[1],heading),sections+[sec],cost,turns))
            states=sorted(next_states,key=lambda v:score_key(v[2]))[:4]
        for pose,sections,score,pivots in states:
            angle=abs(rp.wrap(goal[2]-pose[2]))
            if angle>1e-5:
                peak=pivot_violation(pose,goal[2],planes,cfg);t=pivot_time(angle,cfg)
                score=add_scores(score,(t*peak,peak,t));pivots=pivots+[(pose,goal[2])]
            routes.append((sections,score,pivots))
    # Smooth connectors can remain inside where a stationary turn would swing outside.
    for radius in (cfg.radius,max(cfg.radius,1.0),max(cfg.radius,2.0)):
        try:
            pieces=rp.dubins(start,goal,radius)
            if pieces:
                sec=Section(1,Path(start,pieces));routes.append(([sec],path_score(sec,planes,cfg,speed_for(sec,cfg)),[]))
        except ValueError:pass
    return min(routes,key=lambda r:score_key(r[1]))


def loop_entry(start,option,planes,cfg):
    """圆弧方式接进圆弧（见 LOOP_SPACE_M）：先直行 d、与弧同向按 turn_radius 转、再直行 L，正好切进弧自己的圆，沿圆走 pre 再喷。
    切入点在圆上每 LOOP_STEP_DEG 试一个，取总路程最短的几个算分。返回 ([转场 Section], 分, [], 刷线 Section, paint_from)；
    不是圆弧方式、不是圆弧、几何上接不上或外伸超过 LOOP_SPACE_M 时返回 None。"""
    circle=option.get('circle')
    if cfg.turn!='arc' or not circle:return None
    kappa=circle['kappa'];s=1 if kappa>0 else -1;rho=1/abs(kappa);radius=cfg.turn_radius
    ax,ay,ah=circle['pose'];cx,cy=ax-math.sin(ah)/kappa,ay+math.cos(ah)/kappa
    x0,y0,h0=start
    normal=lambda h:(-s*math.sin(h),s*math.cos(h))     # 指向转弯圆心
    pre_min=max(cfg.run_in*PRE_ARC_FRACTION,.05)
    pres=[pre_min]+[rho*math.radians(d) for d in range(LOOP_STEP_DEG,181,LOOP_STEP_DEG) if rho*math.radians(d)>pre_min+1e-9]
    found=[]
    for pre in pres:
        h=ah-kappa*pre;n0,n=normal(h0),normal(h)
        # 转弯圆心 c1 = 起点 + d·u0 + R·n0；离开点 c1 − R·n，切入点 圆心 − ρ·n，两点之间沿车头 h 直行 L
        wx=cx-x0-radius*n0[0]-(rho-radius)*n[0];wy=cy-y0-radius*n0[1]-(rho-radius)*n[1]
        u0=(math.cos(h0),math.sin(h0));u=(math.cos(h),math.sin(h));det=u0[0]*u[1]-u0[1]*u[0]
        if abs(det)<1e-9:continue
        d=(wx*u[1]-wy*u[0])/det;run=(u0[0]*wy-u0[1]*wx)/det
        if d<-1e-9 or run<-1e-9:continue
        turn=(s*(h-h0))%TAU
        pieces=[(length,k) for length,k in ((d,0.0),(radius*turn,s/radius),(run,0.0)) if length>1e-6]
        if pieces:found.append((sum(length for length,_ in pieces)+pre,pre,pieces))
    best=None
    for _,pre,pieces in sorted(found,key=lambda v:v[0])[:3]:
        lead=Section(1,Path(start,pieces));x,y,h=lead.path.pose(lead.path.length)
        paint=Section(1,Path((x,y,h),[(pre,kappa),(circle['length'],kappa),(rp.CIRCLE_RUN_OUT_M,kappa)]))
        cost=add_scores(path_score(lead,planes,cfg,speed_for(lead,cfg)),path_score(paint,planes,cfg,option['speed']))
        if cost[1]>LOOP_SPACE_M+1e-6:continue
        cost=(cost[0],min(cost[1],OUTSIDE_SPACE_M),cost[2]-merge_saving(cfg))
        if best is None or score_key(cost)<score_key(best[1]):best=([lead],cost,[],paint,pre)
    return best


def entry(start,option,planes,cfg):
    """从 start 接进 option：([转场 Section], 转场 + 刷线的分, 原地转, 刷线 Section, paint_from)。
    圆弧方式进圆弧：现有转场要倒车 / 原地转、或绕圈切进弧的圆更省时就绕圈（loop_entry）。"""
    path,cost,turns=transit(start,option['start'],planes,cfg)
    base=(path,add_scores(cost,option['score']),turns,option['section'],option.get('paint_from',0.0))
    loop=loop_entry(start,option,planes,cfg)
    if loop is None:return base
    forward=not turns and all(sec.direction>0 for sec in path) and not score_key(base[1])[0]
    return base if forward and score_key(base[1])<=score_key(loop[1]) else loop


ORDER_EVALS=4000   # 排序局部搜索的近似评估次数上限：按次数不按时间，车上和电脑上排出同一个顺序


def approx_transition(start,goal,cfg):
    """排序用的转场估算：走直线 + 两次原地转（每次转都算上停稳和起步，与 pivot_time 一致）。"""
    distance=math.dist(start[:2],goal[:2])
    if distance<1e-6:
        turns=(abs(rp.wrap(goal[2]-start[2])),)
    else:
        heading=math.atan2(goal[1]-start[1],goal[0]-start[0])
        turns=(abs(rp.wrap(heading-start[2])),abs(rp.wrap(goal[2]-heading)))
    if cfg.turn!='pivot':
        # 弧线掉头：每个转角按弧长、弯道限速走
        radius=cfg.turn_radius if cfg.turn=='arc' else cfg.radius
        speed=min(cfg.transit_speed,math.sqrt(cfg.lateral_accel*radius))
        return distance/cfg.transit_speed+sum(turns)*radius/speed
    return distance/cfg.transit_speed+sum(pivot_time(t,cfg) for t in turns)


def optimize_order(groups,start,cfg,evals=None):
    """多个最近邻起手 + 按顺序选方向的 DP + or-opt / 2-opt 局部搜索（不保证全局最优）。

    起手包括：从车的位置最近邻、先画离车最近的几条线再最近邻、图纸原顺序；车放在哪，路线就从哪附近开始。
    """
    # Discard a direction only if its own footprint is demonstrably worse.
    eligible=[]
    for group in groups:
        best=min(footprint_key(v['score']) for v in group)
        eligible.append([v for v in group if footprint_key(v['score'])==best])
    def choose(order):
        states=[(approx_transition(start,v['start'],cfg)+v['score'][2],[v]) for v in eligible[order[0]]]
        for i in order[1:]:
            next_states=[]
            for v in eligible[i]:
                cost,path=min(((c+approx_transition(p[-1]['end'],v['start'],cfg),p) for c,p in states),key=lambda q:q[0])
                next_states.append((cost+v['score'][2],path+[v]))
            states=next_states
        return min(states,key=lambda p:p[0])
    n=len(groups)
    def greedy(first=None):
        remaining=set(range(n));order=[];pose=start
        if first is not None:
            order.append(first);remaining.remove(first)
            pose=min(eligible[first],key=lambda v:approx_transition(start,v['start'],cfg))['end']
        while remaining:
            i,v=min(((i,v) for i in remaining for v in eligible[i]),key=lambda iv:approx_transition(pose,iv[1]['start'],cfg)+iv[1]['score'][2])
            order.append(i);remaining.remove(i);pose=v['end']
        return order
    near=sorted(range(n),key=lambda i:min(approx_transition(start,v['start'],cfg) for v in eligible[i]))[:4]
    original=list(range(n));order=best=None
    for candidate in [greedy()]+[greedy(i) for i in near]+[original]:
        score=choose(candidate)
        if best is None or score[0]<best[0]-1e-9:order,best=candidate,score
    budget=ORDER_EVALS if evals is None else evals;improved=True
    while improved and budget>0:
        improved=False
        # or-opt：把连续 1–3 条线整体挪到别的位置
        for length in (1,2,3):
            for i in range(n-length+1):
                segment=order[i:i+length];rest=order[:i]+order[i+length:]
                for j in range(len(rest)+1):
                    if j==i:continue
                    if budget<=0:break
                    trial=rest[:j]+segment+rest[j:];score=choose(trial);budget-=1
                    if score[0]<best[0]-1e-6:
                        order,best,improved=trial,score,True;break
        # 2-opt：整段倒过来画
        for i in range(n-1):
            for j in range(i+2,n+1):
                if budget<=0:break
                trial=order[:i]+order[i:j][::-1]+order[j:];score=choose(trial);budget-=1
                if score[0]<best[0]-1e-6:order,best,improved=trial,score,True
    return best[1],choose(original)[1]


def assemble(selected,start,planes,cfg,connect=None):
    motions=[];pivots=[];score=(0,0,0);pose=start
    for option in selected:
        path,cost,turns,section,paint_from=connect(pose,option) if connect else entry(pose,option,planes,cfg)
        # 绕圈切入可改变助跑长度，划线终点必须随新的起喷点平移。
        paint_to=option.get('paint_to')
        if paint_to is not None:paint_to+=paint_from-option.get('paint_from',0.)
        entry_speed=None
        score=add_scores(score,cost);pivots.extend(turns);path=list(path)
        if cfg.turn!='pivot' and path and path[-1].direction>0 and not turns:
            # 弧线转场最后一段往前开，直接接进助跑：合成一段不停车，从 paint_from 起喷
            last=path.pop()
            entry_speed=transit_limit(last,cfg)
            section=Section(1,Path(last.path.pose(0),[(p.length,p.kappa) for p in last.path.pieces+section.path.pieces]))
            paint_from+=last.path.length
            if paint_to is not None:paint_to+=last.path.length
        motions.extend(dict(section=s,role='transit',speed=transit_limit(s,cfg),id=option['id']) for s in path)
        motions.append(dict(section=section,role='paint',speed=option['speed'],id=option['id'],paint_from=paint_from,paint_to=paint_to,entry_speed=entry_speed))
        pose=option['end']
    return motions,score,pivots


def improve_actual_order(selected,baseline,groups,start,planes,cfg):
    """Refine against real connectors, including reversing and body sweeps.

    A deterministic bound on new connectors keeps a large field responsive. Both
    original and distance-optimized orders remain candidates; no segment is lost.
    """
    cache={}
    def connect(a,v):
        key=(a,v['start'])
        if key not in cache:cache[key]=entry(a,v,planes,cfg)
        return cache[key]
    def evaluate(order,limit=None):
        pose=start;score=(0,0,0)
        for v in order:
            if limit is not None and len(cache)>=limit and (pose,v['start']) not in cache:return None
            score=add_scores(score,connect(pose,v)[1]);pose=v['end']
        return score
    best_score=evaluate(selected);baseline_score=evaluate(baseline)
    # 车在场外或贴边起步时，近似代价看不出头一段转场会不会把车身甩出允许范围：换离车近的几条线打头再用真实转场比。
    for j in sorted(range(1,len(selected)),key=lambda j:math.dist(start[:2],selected[j]['start'][:2]))[:3]:
        trial=[selected[j]]+selected[:j]+selected[j+1:];score=evaluate(trial)
        if score_key(score)<score_key(best_score):selected,best_score=trial,score
    if score_key(baseline_score)<score_key(best_score):selected,best_score=baseline,baseline_score
    limit=len(cache)+160
    variants={g[0]['key']:[v for v in g if footprint_key(v['score'])==min(footprint_key(x['score']) for x in g)] for g in groups}
    for _ in range(2):
        improved=False
        for i in range(len(selected)):
            # Try alternate direction/circle start, then move near geometrically close tasks.
            trials=[selected[:i]+[v]+selected[i+1:] for v in variants[selected[i]['key']] if v is not selected[i]]
            near=sorted((j for j in range(len(selected)) if j!=i),
                        key=lambda j:math.dist(selected[i]['start'][:2],selected[j]['end'][:2]))[:4]
            for j in near:
                trial=selected.copy();v=trial.pop(i);trial.insert(j,v);trials.append(trial)
            for trial in trials:
                score=evaluate(trial,limit)
                if score is not None and score_key(score)<score_key(best_score):
                    selected,best_score,improved=trial,score,True
            if len(cache)>=limit:break
        if not improved or len(cache)>=limit:break
    return selected,assemble(selected,start,planes,cfg,connect)


def serialize_motion(m):
    s=m['section'];x,y,h=s.path.pose(0)
    return dict(direction=s.direction,start=[x,y,h],pieces=[[p.length,p.kappa] for p in s.path.pieces],
                speed=m['speed'],role=m['role'],element_id=m['id'],paint_from=m.get('paint_from',0.0),paint_to=m.get('paint_to'),entry_speed=m.get('entry_speed'),continuous_after=m.get('continuous_after',False))


def corner_best(candidates):
    """角弧图纸不是行驶禁区；相近外伸范围内优先用时，不为少量外场停留多兜一圈。"""
    peak=min(v[1][1] for v in candidates)
    # 原有空地范围内优先短路；都必须外伸时，在最小外伸附近 10 cm 内比较用时。
    # 精确外伸仍报告给操作员，不把这个排序容差当作实际场地许可。
    limit=OUTSIDE_SPACE_M if peak<=OUTSIDE_SPACE_M else peak+.1
    return min((v for v in candidates if v[1][1]<=limit+1e-9),key=lambda v:(v[1][2],v[1][0]))


def corner_entry(start,option,planes,cfg):
    """角弧流程的圆弧转场只向前；实际外伸仍进入预览并要求现场确认。"""
    if cfg.turn!='arc':return entry(start,option,planes,cfg)
    goal=option['start'];candidates=[]
    if math.dist(start[:2],goal[:2])<1e-6 and abs(rp.wrap(start[2]-goal[2]))<1e-6:
        return [],option['score'],[],option['section'],option.get('paint_from',0.)
    for segments in dubins_paths(start,goal,cfg.turn_radius):
        sec=Section(1,Path(start,[(length,k) for _,length,k in segments]))
        score=add_scores(path_score(sec,planes,cfg,speed_for(sec,cfg)),option['score'])
        candidates.append(([sec],score,[],option['section'],option.get('paint_from',0.)))
    loop=loop_entry(start,option,planes,cfg)
    if loop is not None:
        paths,cost,turns,section,paint_from=loop
        # 通用绕圈候选会对外伸降罚；此处要用实际包络与其他候选作同口径比较。
        peak=max(path_violation(sec,planes,cfg) for sec in paths+[section])
        candidates.append((paths,(cost[0],peak,cost[2]),turns,section,paint_from))
    if not candidates:raise ValueError('圆弧转弯无法向前接入，请调整车位或转弯半径')
    return corner_best(candidates)


def corner_flow(groups,start,planes,cfg):
    """最近的外端起步，一条边→角弧→另一条边；选边时按相同流程只取所选部分。"""
    by_id={}
    for group in groups:
        ref=group[0]['selection'];ident=ref['element_id']
        if ident not in ('corner-leg-1','corner-leg-2','corner-arc') or ident in by_id:
            raise ValueError('角弧流程需要两条完整直角边和一条角弧，不支持拆分图形')
        if ref['edge_index']!=(None if ident=='corner-arc' else 0):
            raise ValueError('角弧流程中的直角边格式无效')
        by_id[ident]=group
    tip=rp.nozzle_at(*start,cfg.forward,-cfg.right)
    candidates=[]
    for chain,reverse in ((('corner-leg-1','corner-arc','corner-leg-2'),False),
                          (('corner-leg-2','corner-arc','corner-leg-1'),True)):
        variants=[[v for v in by_id[ident] if ident=='corner-arc' or v['reverse']==reverse]
                  for ident in chain if ident in by_id]
        if any(not v for v in variants):continue
        for choices in product(*variants):
            selected=list(choices)
            # 从真实划线端点比较，而不是比较被助跑挪远的后轴起点。
            distance=math.dist(tip,target_points(selected[0])[0])
            motions,score,pivots=assemble(selected,start,planes,cfg,
                                         lambda pose,option:corner_entry(pose,option,planes,cfg))
            candidates.append((round(distance,3),score,selected,motions,score,pivots))
    if not candidates:raise ValueError('当前喷头和底盘无法完成角弧流程')
    nearest=min(v[0] for v in candidates)
    _,_,selected,motions,score,pivots=corner_best([v for v in candidates if v[0]==nearest])
    return selected,(motions,score,pivots)


def plan(job,start,cfg):
    cfg.validate()
    if not isinstance(job,dict):raise ValueError('场地图纸无效')
    # 作业顺序（可选 job.execution_order）：optimized（默认）内场优先、按用时排序，方向也由规划挑；
    # drawing 照图纸顺序与方向；corner 从最近的外端画边→角弧→另一边。
    execution_order=job.get('execution_order','optimized')
    if execution_order not in ('optimized','drawing','corner'):raise ValueError('作业顺序无效')
    outline,planes=polygon(job.get('boundary'))
    if len(start)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in start):
        raise ValueError('车辆位置无效')
    # The entire interior is reachable; outside the field retain the existing entry-distance limit.
    if any(nx*start[0]+ny*start[1]<c for nx,ny,c in planes):
        def distance_to_edge(a,b):
            dx,dy=b[0]-a[0],b[1]-a[1]
            t=max(0,min(1,((start[0]-a[0])*dx+(start[1]-a[1])*dy)/(dx*dx+dy*dy)))
            return math.hypot(start[0]-a[0]-t*dx,start[1]-a[1]-t*dy)
        if min(distance_to_edge(a,b) for a,b in zip(outline,outline[1:]+outline[:1]))>rp.MAX_ENTRY_M:
            raise ValueError('车辆离场地太远，请先手动开到场地附近')
    requested=parse_selection(job['selected_strokes']) if 'selected_strokes' in job else None
    strokes,manual=parse_strokes(job.get('elements'),requested)
    groups=[stroke_options(s,cfg,planes) for s in strokes]
    if execution_order=='drawing':groups=[[v for v in group if not v['reverse']] for group in groups]
    bad=[s['name'] for s,g in zip(strokes,groups) if not g]
    if bad:raise ValueError('当前喷头/底盘尺寸无法准确跟踪：'+'、'.join(bad))
    if execution_order=='corner':
        selected,(motions,score,pivots)=corner_flow(groups,start,planes,cfg)
    elif execution_order=='drawing':
        selected=[group[0] for group in groups]
        motions,score,pivots=assemble(selected,start,planes,cfg)
    else:
        selected,baseline=optimize_order(groups,start,cfg)
        selected,(motions,score,pivots)=improve_actual_order(selected,baseline,groups,start,planes,cfg)
    # 分段仍保留独立的喷线范围，控制器只把相切前进段无停顿交接。
    if cfg.turn=='arc':
        for i,m in enumerate(motions):
            m['continuous_after']=i<len(motions)-1
    preview=[];outside=[];outside_seconds=0.0
    for motion in motions:
        sec=motion['section'];n=max(1,min(2048,math.ceil(sec.path.length/.5)))
        poses=[sec.car_pose(sec.path.length*i/n) for i in range(n+1)]
        points=[[round(p[0],4),round(p[1],4)] for p in poses]
        spans=[];span=[]
        for i,pose in enumerate(poses):
            if violation(pose,planes,cfg.body,cfg.margin)>1e-7:
                if not span and i:span.append(points[i-1])
                span.append(points[i])
            elif span:
                span.append(points[i]);spans.append(span);span=[]
        if span:spans.append(span)
        preview.append(dict(role=motion['role'],points=points,outside_spans=spans,
                            paint_from_m=round(motion.get('paint_from',0.0),3),paint_to_m=motion.get('paint_to'),length_m=round(sec.path.length,3)))
        peak=path_violation(sec,planes,cfg)
        if peak>1e-7:
            outside_seconds+=duration(sec.path.length,motion['speed'],cfg.accel)*sum(violation(p,planes,cfg.body,cfg.margin)>1e-7 for p in poses)/len(poses)
            witness=max(poses,key=lambda p:violation(p,planes,cfg.body,cfg.margin))
            outside.append(dict(element_id=motion['id'],role=motion['role'],outside_m=peak,body=corner_positions(witness,cfg.body)))
    for pose,heading in pivots:
        peak=pivot_violation(pose,heading,planes,cfg)
        if peak>1e-7:
            outside_seconds+=duration(abs(rp.wrap(heading-pose[2])),cfg.yaw_rate,cfg.yaw_accel)
            delta=rp.wrap(heading-pose[2]);poses=[(*pose[:2],pose[2]+delta*i/32) for i in range(33)]
            witness=max(poses,key=lambda p:violation(p,planes,cfg.body,cfg.margin))
            outside.append(dict(element_id='turn',role='turn',outside_m=peak,body=corner_positions(witness,cfg.body)))
    # 排序时绕圈切进圆弧的外伸按空地以内算（loop_entry），报告按实际最大外伸
    peak=max([score[1]]+[o['outside_m'] for o in outside])
    return dict(version=1,execution_order=execution_order,objective='inside_then_time',wet_paint_considered=False,global_optimum=False,
                turn=cfg.turn,turn_radius=cfg.turn_radius if cfg.turn=='arc' else cfg.radius if cfg.turn=='lock' else None,
                pivot_count=len(pivots),reverse_count=sum(m['section'].direction<0 for m in motions),
                selection_mode='selected' if requested is not None else 'all',
                inside=peak<=1e-7,max_outside_m=peak,outside_time_estimate_s=outside_seconds,outside_penalty_m_s=score[0],duration_s=score[2],
                paint_speed=cfg.paint_speed,transit_speed=cfg.transit_speed,body=list(cfg.body),nozzle_forward_m=cfg.forward,nozzle_right_m=cfg.right,boundary=outline,
                distance_m=sum(m['section'].path.length for m in motions),paint_length_m=sum(v['nozzle_length'] for v in selected),
                start=list(start),order=[dict(id=v['id'],name=v['name'],reversed=v['reverse'],selection=v['selection']) for v in selected],
                target=[target_points(v) for v in selected],preview=preview,outside=outside,manual=manual,
                motions=[serialize_motion(m) for m in motions])
