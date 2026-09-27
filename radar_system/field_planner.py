"""Position-aware field planning for a differential chassis with a right-hand nozzle.

The objective prefers complete-body interior travel. Where outside space is
unavoidable, it balances exposure and elapsed time. Wet paint is not an input. This produces geometric
vehicle paths; it does not operate a pump or claim measured paint coverage.
"""
import math
from dataclasses import dataclass
import route_plan as rp
from path_tracker import Path, Section

TAU = 2 * math.pi


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
    paint_speed: float = .5
    transit_speed: float = .8
    yaw_rate: float = .8
    accel: float = .6
    yaw_accel: float = 1.0
    settle: float = .3
    margin: float = .0
    lateral_accel: float = .05

    def validate(self):
        number(self.forward,'喷头前伸',-3,3);number(self.right,'喷头右偏',.001,2)
        number(self.radius,'最小转弯半径',.1,20)
        if len(self.body)!=3:raise ValueError('车身尺寸无效')
        for v in self.body:number(v,'车身尺寸',.01,5)
        for v in (self.paint_speed,self.transit_speed):number(v,'速度',.05,1.2)
        number(self.yaw_rate,'转向速度',.1,2);number(self.accel,'加速度',.05,2)
        number(self.yaw_accel,'转向加速度',.05,3);number(self.margin,'边界余量',0,1)
        number(self.lateral_accel,'弯道加速度',.01,2)


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


def speed_for(section,cfg,paint=False,nozzle_radius=None):
    speed=cfg.paint_speed if paint else min(cfg.transit_speed,.3) if section.direction<0 else cfg.transit_speed
    k=section.path.kappa_abs_max
    if paint and nozzle_radius and k>1e-9:speed*=1/k/nozzle_radius
    return max(.02,min(speed,cfg.yaw_rate/k,math.sqrt(cfg.lateral_accel/k))) if k>1e-9 else speed


def path_score(section,planes,cfg,speed):
    length=section.path.length;peak=path_violation(section,planes,cfg)
    t=duration(length,speed,cfg.accel)+cfg.settle
    if peak<=1e-7:return (0.0,0.0,t)
    # Exposure is an optimization estimate; zero/maximum comes from the exact check.
    n=max(2,min(160,math.ceil(length/.3)))
    depth=sum(violation(section.car_pose(length*(i+.5)/n),planes,cfg.body,cfg.margin) for i in range(n))/n
    return (max(depth,peak/n)*t,peak,t)


def add_scores(*scores):
    return (sum(s[0] for s in scores),max((s[1] for s in scores),default=0),sum(s[2] for s in scores))


def score_key(score):
    # Prefer a fully-inside option. If both require outside space, treat that space
    # as expensive ground instead of taking arbitrarily long detours for millimetres.
    # These are ranking penalties, not predicted physical delays or speed commands.
    return (score[1]>1e-6,round(score[2]+8*score[0]+30*score[1],5),round(score[1],5),round(score[2],5))


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


def parse_strokes(raw,selected=None):
    if not isinstance(raw,list) or not 1<=len(raw)<=80:raise ValueError('场地须有 1–80 个图形')
    strokes=[];manual=[];ids=set();matched=set()
    def add(stroke,ident,index):
        key=(ident,index)
        if selected is not None and key not in selected:return
        matched.add(key)
        stroke.update(key=key,selection=dict(element_id=ident,edge_index=index))
        strokes.append(stroke)
        if len(strokes)>80:raise ValueError('一次规划支持 1–80 条线或圆弧，请选择部分边')
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
    if not strokes or len(strokes)>80:raise ValueError('一次规划支持 1–80 条线或圆弧')
    return strokes,manual


def stroke_options(stroke,cfg,planes):
    a,b=cfg.forward,-cfg.right;out=[]
    if stroke['kind']=='line':
        for reverse in (False,True):
            q0,q1=(stroke['b'],stroke['a']) if reverse else (stroke['a'],stroke['b'])
            h=math.atan2(q1[1]-q0[1],q1[0]-q0[0]);c,s=math.cos(h),math.sin(h)
            axle=(q0[0]-a*c+b*s,q0[1]-a*s-b*c,h)
            sec=Section(1,Path(axle,[(math.dist(q0,q1),0)]))
            out.append(dict(section=sec,reverse=reverse,target=[q0,q1],nozzle_length=math.dist(q0,q1)))
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
                sec=Section(1,Path(axle,[(rear_radius*abs(sweep),sign/rear_radius)]))
                out.append(dict(section=sec,reverse=reverse,target_arc=(cx,cy,r,angle,sweep),
                                nozzle_length=r*abs(sweep),nozzle_radius=r))
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


def transit(start,goal,planes,cfg):
    if math.dist(start[:2],goal[:2])<1e-6:
        angle=abs(rp.wrap(goal[2]-start[2]));peak=pivot_violation(start,goal[2],planes,cfg)
        t=duration(angle,cfg.yaw_rate,cfg.yaw_accel) if angle>1e-6 else 0
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
                        peak=pivot_violation(pose,heading,planes,cfg);t=duration(angle,cfg.yaw_rate,cfg.yaw_accel)+cfg.settle
                        cost=add_scores(cost,(t*peak,peak,t));turns=pivots+[(pose,heading)]
                    sec=Section(direction,Path((pose[0],pose[1],travel),[(dist,0)]))
                    cost=add_scores(cost,path_score(sec,planes,cfg,speed_for(sec,cfg)))
                    next_states.append(((end[0],end[1],heading),sections+[sec],cost,turns))
            states=sorted(next_states,key=lambda v:score_key(v[2]))[:4]
        for pose,sections,score,pivots in states:
            angle=abs(rp.wrap(goal[2]-pose[2]))
            if angle>1e-5:
                peak=pivot_violation(pose,goal[2],planes,cfg);t=duration(angle,cfg.yaw_rate,cfg.yaw_accel)+cfg.settle
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


def approx_transition(start,goal,cfg):
    distance=math.dist(start[:2],goal[:2])
    heading=math.atan2(goal[1]-start[1],goal[0]-start[0]) if distance>1e-8 else start[2]
    turns=abs(rp.wrap(heading-start[2]))+abs(rp.wrap(goal[2]-heading))
    return distance/cfg.transit_speed+turns/cfg.yaw_rate


def optimize_order(groups,start,cfg):
    """Nearest-neighbour, option DP, and bounded 2-opt (not a global optimum proof)."""
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
                cost,path=min(states,key=lambda pair:pair[0]+approx_transition(pair[1][-1]['end'],v['start'],cfg))
                next_states.append((cost+approx_transition(path[-1]['end'],v['start'],cfg)+v['score'][2],path+[v]))
            states=next_states
        return min(states,key=lambda p:p[0])
    remaining=set(range(len(groups)));order=[];pose=start
    while remaining:
        i,v=min(((i,v) for i in remaining for v in eligible[i]),key=lambda iv:approx_transition(pose,iv[1]['start'],cfg)+iv[1]['score'][2])
        order.append(i);remaining.remove(i);pose=v['end']
    best=choose(order)
    # Preserve the input order as a candidate, and improve deterministic bounded alternatives.
    original=list(range(len(groups)));candidate=choose(original)
    if candidate[0]<best[0]:order,best=original,candidate
    budget=100
    for span in range(len(order)-1,1,-1):
        for i in range(len(order)-span+1):
            if budget<=0:break
            trial=order[:i]+list(reversed(order[i:i+span]))+order[i+span:];score=choose(trial);budget-=1
            if score[0]<best[0]-1e-6:order,best=trial,score
        if budget<=0:break
    return best[1],choose(original)[1]


def assemble(selected,start,planes,cfg,connect=None):
    motions=[];pivots=[];score=(0,0,0);pose=start
    for option in selected:
        path,cost,turns=connect(pose,option['start']) if connect else transit(pose,option['start'],planes,cfg)
        score=add_scores(score,cost,option['score']);pivots.extend(turns)
        motions.extend(dict(section=s,role='transit',speed=speed_for(s,cfg),id=option['id']) for s in path)
        motions.append(dict(section=option['section'],role='paint',speed=option['speed'],id=option['id']))
        pose=option['end']
    return motions,score,pivots


def improve_actual_order(selected,baseline,groups,start,planes,cfg):
    """Refine against real connectors, including reversing and body sweeps.

    A deterministic bound on new connectors keeps a large field responsive. Both
    original and distance-optimized orders remain candidates; no segment is lost.
    """
    cache={}
    def connect(a,b):
        key=(a,b)
        if key not in cache:cache[key]=transit(a,b,planes,cfg)
        return cache[key]
    def evaluate(order,limit=None):
        pose=start;score=(0,0,0)
        for v in order:
            if limit is not None and len(cache)>=limit and (pose,v['start']) not in cache:return None
            score=add_scores(score,connect(pose,v['start'])[1],v['score']);pose=v['end']
        return score
    best_score=evaluate(selected);baseline_score=evaluate(baseline)
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
                speed=m['speed'],role=m['role'],element_id=m['id'])


def plan(job,start,cfg):
    cfg.validate()
    if not isinstance(job,dict):raise ValueError('场地图纸无效')
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
    bad=[s['name'] for s,g in zip(strokes,groups) if not g]
    if bad:raise ValueError('当前喷头/底盘尺寸无法准确跟踪：'+'、'.join(bad))
    selected,baseline=optimize_order(groups,start,cfg)
    selected,(motions,score,pivots)=improve_actual_order(selected,baseline,groups,start,planes,cfg)
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
        preview.append(dict(role=motion['role'],points=points,outside_spans=spans))
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
    return dict(version=1,objective='inside_then_time',wet_paint_considered=False,global_optimum=False,
                selection_mode='selected' if requested is not None else 'all',
                inside=score[1]<=1e-7,max_outside_m=score[1],outside_time_estimate_s=outside_seconds,outside_penalty_m_s=score[0],duration_s=score[2],
                paint_speed=cfg.paint_speed,transit_speed=cfg.transit_speed,body=list(cfg.body),nozzle_forward_m=cfg.forward,nozzle_right_m=cfg.right,boundary=outline,
                distance_m=sum(m['section'].path.length for m in motions),paint_length_m=sum(v['nozzle_length'] for v in selected),
                start=list(start),order=[dict(id=v['id'],name=v['name'],reversed=v['reverse'],selection=v['selection']) for v in selected],
                target=[target_points(v) for v in selected],preview=preview,outside=outside,manual=manual,
                motions=[serialize_motion(m) for m in motions])
