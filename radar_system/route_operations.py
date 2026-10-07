"""Route checkpoints, operator holds, and previewed resume/rework operations.

Distances here describe vehicle tracking, not confirmed paint coverage. This robot has
no spray controller. A checkpoint owns immutable geometry so crossings/laps never
resume by searching the nearest line in a newly generated route.
"""
from robot_core.standstill import stationary
import collections
import copy
import hashlib
import json
import math
import os
import time
import uuid

import route_plan as rp
import paint_quality
import circle_spray_repair
import paint_gaps
from path_tracker import Path, Section, SectionTracker
from motion_speeds import DEFAULT_PAINT_SPEED, MAX_SECTION_SPEED
from robot_core.config import CONTROL_DT

HELD = ('PAUSING', 'PAUSED', 'INTERRUPTED')
# 开/关喷标记留最近几条：App 5 Hz 收状态，偶尔丢几帧也接得上
PAINT_MARKS_KEEP = 16
ACTIONS = ['pause', 'preview', 'resume', 'rework', 'field_start', 'start_current', 'end', 'speed']


def signature(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def cut(path, start=0.0, end=None):
    end = path.length if end is None else end
    if not 0 <= start < end <= path.length + 1e-7:
        raise ValueError('补画范围超出线段')
    pieces = []
    for p in path.pieces:
        lo, hi = max(start, p.s0), min(end, p.s0 + p.length)
        if hi - lo > 1e-8:
            pieces.append((hi - lo, p.kappa))
    return Path(path.pose(start), pieces)


def tail(sections, distance):
    out, offset = [], 0.0
    for section in sections:
        end = offset + section.path.length
        if end - distance > 1e-5:
            remaining=Section(section.direction, cut(section.path, max(0.0, distance-offset)))
            remaining.cruise_mps=getattr(section,'cruise_mps',None)
            remaining.entry_speed_mps=getattr(section,'entry_speed_mps',None)
            remaining.field_role=getattr(section,'field_role',None)
            remaining.continuous_after=section.continuous_after
            remaining.paint_from_m=max(0.0,getattr(section,'paint_from_m',0.0)-max(0.0,distance-offset))
            consumed=max(0.,distance-offset)
            remaining.paint_to_m=None if section.paint_to_m is None else max(0.,section.paint_to_m-consumed)
            remaining.paint_spans=None if section.paint_spans is None else [(max(0.,a-consumed),b-consumed) for a,b in section.paint_spans if b>consumed]
            if remaining.paint_spans:
                remaining.paint_from_m,remaining.paint_to_m=remaining.paint_spans[0][0],remaining.paint_spans[-1][1]
            if remaining.paint_spans == []:remaining.field_role='transit'
            out.append(remaining)
        offset = end
    if not out:
        raise ValueError('当前任务已到终点，无需继续')
    return out


def painting_start(sections):
    """已保存路径第一次真正划线的位置；接入和助跑不算已划线。旧记录缺标记时不猜。"""
    offset=0.0
    for sec in sections:
        if getattr(sec,'field_role',None)=='paint':
            return offset+getattr(sec,'paint_from_m',0.0)
        offset+=sec.path.length
    return None


def samples(sections, limit=500):
    total = sum(s.path.length for s in sections)
    step = max(0.1, total / max(1, limit - len(sections)))
    out = []
    for sec in sections:
        n = max(1, math.ceil(sec.path.length / step))
        out.append(dict(direction=sec.direction,
                        points=[[round(v, 4) for v in sec.path.pose(sec.path.length * i / n)[:2]]
                                for i in range(n + 1)]))
    return out


def entry_to(sections, car, radius):
    """Connect to the exact remaining section, including a reverse section/cusp."""
    first = sections[0]
    goal = first.car_pose(0.0)
    if math.hypot(car[0] - goal[0], car[1] - goal[1]) < 0.03 and abs(rp.wrap(car[2] - goal[2])) < math.radians(2):
        return sections, 0.0, []
    way = rp.reeds_shepp(car, goal, radius, end_forward=first.direction > 0)
    if way is None:
        way = [(1, length, k) for length, k in rp.dubins(car, goal, radius)]
    length = sum(v[1] for v in way)
    if length > rp.MAX_ENTRY_M:
        raise ValueError('接入路线 %.1f m，超过 %.0f m；请先手动开近断点' % (length, rp.MAX_ENTRY_M))
    builder = rp._Sections(car)
    for direction, dist, kappa in way:
        builder.add(direction, dist, kappa)
    entry = [Section(d, Path(p, pieces)) for d, p, pieces in builder.out]
    for sec in entry:
        sec.field_role='transit'
    # A stop at the join makes the handover (including a gear change) explicit.
    return entry + sections, length, entry


def pivot_entry_to(sections, car, radius, nozzle):
    """差速车接入剩下的第一段（与开始作业同一套，route_plan.plan_current）：水泵离起点 NEAR_START_M 以内倒车助跑，
    更远原地转向、直线开到起点前 ENTRY_LEAD_M 对正再开进来。助跑与第一段接成一段（中间不停车，水泵到起点时已稳住），
    返回 (整趟分段, 接入路程米, 只含接入的分段（预览画接入用）)。第一段是倒车时退回 entry_to。"""
    first = sections[0]
    if first.direction < 0:
        return entry_to(sections, car, radius)
    f, l = rp.nozzle_point(nozzle)
    x0, y0, h0 = first.path.pose(0.0)
    gap = math.dist(rp.nozzle_at(*car, f, l), rp.nozzle_at(x0, y0, h0, f, l))
    entry_fn = rp.runup_entry if gap <= rp.NEAR_START_M else rp.pivot_entry
    pieces = [(p.length, p.kappa) for p in first.path.pieces]
    joined, length = entry_fn((1, (x0, y0, h0), pieces), car, nozzle)
    if length > rp.MAX_ENTRY_M:
        raise ValueError('接入路线 %.1f m，超过 %.0f m；请先手动开近断点' % (length, rp.MAX_ENTRY_M))
    only, _ = entry_fn((1, (x0, y0, h0), []), car, nozzle)
    entry = [Section(d, Path(p, pc)) for d, p, pc in only if pc]
    head = [Section(d, Path(p, pc)) for d, p, pc in joined if pc]
    from live_cruise import entry_speed
    for sec in head:
        # 接入（不喷）按 entry_speed（同开始作业，2026-10-07 用户）；离正式路线 3 m 内由 SectionTracker 收回画线速度
        sec.cruise_mps=(entry_speed(getattr(first,'cruise_mps',None) or 0.) if sec.direction>0
                        else getattr(first,'cruise_mps',None))
        sec.field_role='transit'
    sec=head[-1];shift=sec.path.length-first.path.length
    sec.cruise_mps=getattr(first,'cruise_mps',None)
    sec.field_role=getattr(first,'field_role',None) or 'paint'
    sec.continuous_after=first.continuous_after
    sec.paint_from_m=getattr(first,'paint_from_m',0.)+shift
    sec.paint_to_m=None if first.paint_to_m is None else first.paint_to_m+shift
    sec.paint_spans=None if first.paint_spans is None else [(a+shift,b+shift) for a,b in first.paint_spans]
    return head + sections[1:], length, entry


def elements(spec, radius):
    """Stable IDs refer to drawing legs, never to generated reversing manoeuvres."""
    if spec.kind == 'path':
        points = list(spec.points)
        if spec.closed and points and points[-1] != points[0]:
            points.append(points[0])
        return [dict(id=i, name='第 %d 条线（%d → %d）' % (i + 1, i + 1, (i + 1) % len(spec.points) + 1),
                     length_m=math.dist(a, b)) for i, (a, b) in enumerate(zip(points, points[1:]))]
    if spec.kind == 'circle':
        return [dict(id=0, name='圆形（一圈）', length_m=2 * math.pi * spec.radius)]
    start, pieces = rp.loop_pieces(spec, radius)
    path = Path(start, pieces)
    return [dict(id=i, name='第 %d 段%s' % (i + 1, '圆弧' if abs(p.kappa) > 1e-9 else '直线'),
                 length_m=p.length) for i, p in enumerate(path.pieces)]


def rework_sections(spec, element, start, end, radius, body, track_x=0.0, nozzle=None):
    """补画图纸上第 element 条线 / 圆的 [start, end] 米。nozzle 不为 None（差速车，水泵在车体系 (前, 左)，可以在侧面）：
    让水泵正好压过这一段——连线的一条边后轴与边平行、按水泵横移；圆按水泵算后轴的圆（route_plan.circle_axle_radius），
    圆的 s 从图纸圆的起点（最南点或 start_angle）顺着画的方向量。阿克曼车按车身中心。"""
    choices = elements(spec, radius)
    if isinstance(element, bool) or not isinstance(element, int) or not 0 <= element < len(choices):
        raise ValueError('补画线段不存在，请重新选择')
    length = choices[element]['length_m']
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (start, end)):
        raise ValueError('补画范围必须为有限数值')
    if start < 0 or end > length + 1e-4 or end - start < rp.MIN_LEG_M:
        raise ValueError('补画范围须在线段内且至少 %.1f m' % rp.MIN_LEG_M)
    end = min(end, length)
    if nozzle is not None and spec.kind in ('path', 'circle'):
        # 补画也必须保留图纸终点，再沿原切线/圆弧增加不喷缓停段；预览、快照和执行同一份几何。
        path=_nozzle_rework_path(spec, element, start, end, radius, nozzle)
        last=path.pieces[-1];run=rp.LINE_RUN_OUT_M if last.straight else rp.CIRCLE_RUN_OUT_M
        sec=Section(1,Path(path.pose(0.),[(p.length,p.kappa) for p in path.pieces]+[(run,last.kappa)]))
        sec.paint_to_m=path.length
        return [sec], choices[element]['name']
    if spec.kind == 'path':
        a, b = spec.points[element], spec.points[(element + 1) % len(spec.points)]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        centre = (body[0] - body[1]) / 2 - track_x
        # Exact drawing range, without the normal route's terminal overrun.
        pose = (a[0] + (start - centre) * math.cos(yaw), a[1] + (start - centre) * math.sin(yaw), yaw)
        path = Path(pose, [(end - start, 0.0)])
    else:
        p0, pieces = rp.loop_pieces(spec, radius)
        full = Path(p0, pieces)
        offset = 0.0 if spec.kind == 'circle' else full.pieces[element].s0
        path = cut(full, offset + start, offset + end)
    return [Section(1, path)], choices[element]['name']


def _nozzle_rework_path(spec, element, start, end, radius, nozzle):
    """水泵（车体系前 f、左 l）压过图纸第 element 条线 / 圆的 [start, end] 时后轴要走的路径。"""
    f, l = rp.nozzle_point(nozzle)
    if spec.kind == 'path':
        a, b = spec.points[element], spec.points[(element + 1) % len(spec.points)]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        q = (a[0] + start * math.cos(yaw), a[1] + start * math.sin(yaw))
        return Path(rp.axle_for(q, yaw, f, l) + (yaw,), [(end - start, 0.0)])
    cx, cy = spec.center
    sign = -1.0 if spec.clockwise else 1.0
    r = rp.circle_axle_radius(spec.radius, (f, l), radius, sign)
    # 后轴在圆心正东（方位角 0）、车头顺着画的方向时，水泵的方位角；水泵总比后轴超前这么多
    px, py = rp.nozzle_at(r, 0.0, sign * math.pi / 2, f, l)
    lead = math.atan2(py, px)
    theta0 = -math.pi / 2 if spec.start_angle is None else spec.start_angle
    axle = theta0 + sign * start / spec.radius - lead
    pose = (cx + r * math.cos(axle), cy + r * math.sin(axle), rp.wrap(axle + sign * math.pi / 2))
    return Path(pose, [((end - start) / spec.radius * r, sign / r)])


class RouteOperations:
    def __init__(self, mission):
        self.m = mission
        self.path = (mission.route_path + '.task.json') if mission.route_path else None
        self.record = self.parent = self.preview = None
        self.history = []
        self.last_saved = -math.inf
        self.error = None
        self.runtime_base = self.runtime_entry = 0.0
        self.still_since = None
        self.last_token = None
        self.execution = None
        self._elements_key = self._elements = None
        self._field_key = self._field_elements = self._field_context = None
        # 仅运行时保存：每个执行分段独立锁存关泵，定位回摆不能在该段尾重新开喷。
        self._gap_previous = None
        self._paint_tail_key = None
        self._paint_tail_armed = self._paint_tail_closed = False
        self._paint_tail_closed_at = None
        # 开/关喷那一拍的实际喷头位置（只给 App 画喷涂线，不存盘、不影响行驶和开泵）；
        # run 每次车端进程启动换一个，App 据此认出车重启后序号重新从 1 数
        self._paint_marks = collections.deque(maxlen=PAINT_MARKS_KEEP)
        self._paint_marks_run = uuid.uuid4().hex[:8]
        self._paint_mark_seq = 0
        self._paint_mark_job = None
        self._paint_mark_on = False
        self._load()

    @property
    def held(self):
        return bool(self.record and self.record['state'] in HELD)

    def fingerprint(self):
        return signature(self.m.route.to_dict(digits=9)) if self.m.route else None

    def profile(self):
        return signature(dict(body=self.m.body, radius=self.m.min_route_radius,
                              track_x=self.m.path_cfg.track_x_m, line_track_x=self.m.line_cfg.track_x_m))

    def field_profile(self, record):
        """原图纸与喷头设置属于任务快照；几何参数改变后须重新规划；已验证的在线调速会更新速度签名。"""
        return signature(dict(profile=self.profile(), field_context=record.get('field_context'),
                              cruise=record.get('cruise'), yaw_rate=self.m.geo.pivot_rate_rad_s,
                              yaw_accel=self.m.geo.pivot_accel_rad_s2, accel=self.m.path_cfg.accel_mps2,
                              lateral_accel=self.m.path_cfg.lat_accel_mps2))

    def field_info(self, record):
        from field_rework import decode_source, catalog
        key = (record['field_source_hash'], self.m._origin())
        if key != self._field_key:
            job = decode_source(record['field_source'], self.m.fusion)
            self._field_elements = catalog(job)
            self._field_context = [dict(direction=1, points=e['points']) for e in self._field_elements]
            self._field_key = key
        return self._field_elements, self._field_context

    def field_source_usable(self, record):
        try:
            return bool(record.get('field_source') and
                        signature(record['field_source']) == record.get('field_source_hash') and
                        self.field_profile(record) == record.get('field_profile'))
        except (ValueError, TypeError, KeyError):
            return False

    def encode(self, sections):
        f = self.m.fusion
        out = []
        for sec in sections:
            x, y, yaw = sec.path.pose(0)
            out.append(dict(direction=sec.direction, point=f.to_geodetic(x, y),
                            heading_point=f.to_geodetic(x + math.cos(yaw), y + math.sin(yaw)),
                            pieces=[[p.length, p.kappa] for p in sec.path.pieces],speed=getattr(sec,'cruise_mps',None),
                            field_role=getattr(sec,'field_role',None),paint_from=getattr(sec,'paint_from_m',0.0),
                            paint_to=sec.paint_to_m,paint_spans=sec.paint_spans,continuous_after=sec.continuous_after,
                            entry_speed=getattr(sec,'entry_speed_mps',None)))
        return out

    def decode(self, record):
        out = []
        for sec in record['plan']:
            x, y = self.m.fusion.to_local(*sec['point'])
            hx, hy = self.m.fusion.to_local(*sec['heading_point'])
            restored=Section(sec['direction'], Path((x, y, math.atan2(hy - y, hx - x)), sec['pieces']))
            restored.cruise_mps=sec.get('speed')
            restored.entry_speed_mps=sec.get('entry_speed')
            restored.field_role=sec.get('field_role')
            restored.paint_from_m=float(sec.get('paint_from') or 0.0)
            restored.paint_to_m=sec.get('paint_to')
            restored.paint_spans=sec.get('paint_spans')
            restored.continuous_after=sec.get('continuous_after',False)
            out.append(restored)
        return out

    def new_record(self, sections, kind='route', label='路线作业'):
        return dict(id=str(uuid.uuid4()), route_rev=self.m.route_rev, route_hash=self.fingerprint(),
                    profile=self.profile(), kind=kind, label=label, state='RUNNING', reason='',
                    cursor_m=0.0, total_m=sum(s.path.length for s in sections), plan=self.encode(sections),
                    started_at=time.time(), updated_at=time.time(), rework=None)

    def begin(self, sections, entry_m=0.0, **fields):
        old = self.record, self.parent, self.preview, self.execution
        self.record = self.new_record(sections)
        self.record.update(fields)
        self.execution = dict(sections=samples(sections), entry_m=entry_m)
        self.parent = self.preview = None
        try:
            self.save()
        except ValueError:
            self.record, self.parent, self.preview, self.execution = old
            raise
        self.runtime_base = self.runtime_entry = 0.0

    def save(self):
        if not self.path:
            return
        tmp = self.path + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(dict(version=1, active=self.record, parent=self.parent, history=self.history[-20:]),
                          fh, ensure_ascii=False, allow_nan=False)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self.path)
            self.error = None
        except OSError as exc:
            self.error = '任务进度保存失败：' + str(exc)
            raise ValueError(self.error) from exc

    def _validate_record(self, r):
        if r is None:
            return
        if not isinstance(r, dict) or not isinstance(r.get('id'), str) or r.get('state') not in (*HELD, 'RUNNING', 'DONE', 'CANCELLED'):
            raise ValueError('无效任务记录')
        if r.get('kind') not in ('route', 'rework') or not isinstance(r.get('plan'), list) or not 0 < len(r['plan']) <= 10000:
            raise ValueError('无效任务路径')
        paint_gaps.validate(r)
        if r.get('cruise') is not None:
            rp._number(r['cruise'],'任务巡航速度',.1,1.2)
        total = 0.0
        for s in r['plan']:
            if type(s.get('continuous_after',False)) is not bool:
                raise ValueError('连续前进标记必须为布尔值')
            if s.get('direction') not in (-1, 1):
                raise ValueError('无效行驶方向')
            if s.get('speed') is not None:
                rp._number(s['speed'], '分段速度', .02, MAX_SECTION_SPEED)
            if s.get('entry_speed') is not None:
                rp._number(s['entry_speed'],'接入速度',.02,MAX_SECTION_SPEED)
            for key in ('point', 'heading_point'):
                rp._pair(s[key], '任务坐标', (-90, -180), (90, 180))
            if not isinstance(s.get('pieces'), list) or not 0 < len(s['pieces']) <= 20000:
                raise ValueError('无效路径分段')
            for p in s['pieces']:
                length, _ = rp._pair(p, '任务分段', (1e-9, -10), (1e7, 10))
                total += length
        rp._number(r.get('cursor_m'), '任务进度', 0, total + 1e-5)
        if abs(total - r['total_m']) > 1e-4:
            raise ValueError('任务长度不一致')
        repair=r.get('circle_spray')
        if repair is not None:
            if not isinstance(repair,dict) or type(repair.get('bins')) is not int or not 16<=repair['bins']<=20000:
                raise ValueError('圆补喷记录格式无效')
            for key in ('lap_m','paint_start_m','original_end_m','last_s','start_fraction'):
                if type(repair.get(key)) not in (int,float) or not math.isfinite(repair[key]):raise ValueError('圆补喷记录数值无效')
            if not 0<repair['lap_m'] or not 0<=repair['paint_start_m']<=repair['original_end_m']<=total+1e-4:
                raise ValueError('圆补喷参考范围无效')
            if not 0<=repair['last_s']<=total+1e-4 or not 0<=repair['start_fraction']<1:
                raise ValueError('圆补喷进度无效')
            good=repair.get('good')
            if not isinstance(good,list) or len(good)!=repair['bins'] or any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in good):
                raise ValueError('圆补喷覆盖记录无效')
            if type(repair.get('wanted')) is not int or not 1<=repair['wanted']<=rp.MAX_LAPS:
                raise ValueError('圆补喷圈数无效')


    def _load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            if os.path.getsize(self.path) > 8 * 1024 * 1024:
                raise ValueError('任务文件过大')
            with open(self.path, encoding='utf-8') as fh:
                data = json.load(fh)
            if data.get('version') != 1:
                raise ValueError('不支持的任务版本')
            for r in (data.get('active'), data.get('parent')):
                self._validate_record(r)
            self.record, self.parent = data.get('active'), data.get('parent')
            self.history = data.get('history', [])[-20:]
            if self.record and self.record['state'] in (*HELD, 'RUNNING'):
                self.record.update(state='PAUSED', reason='restart')
                self.m.phase, self.m.task, self.m.message = 'PAUSED', 'route', '已恢复任务断点，请检查后继续'
                self.m.cruise=self.record.get('cruise',self.m.cruise)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.record = self.parent = None
            self.error = '任务记录无法读取：' + str(exc)

    def check_identity(self, cmd, require_job=True):
        if isinstance(cmd.get('route_rev'), bool) or not isinstance(cmd.get('route_rev'), int) or cmd.get('route_rev') != self.m.route_rev:
            raise ValueError('路线已改变，请重新预览')
        if require_job and (not self.record or cmd.get('job_id') != self.record['id']):
            raise ValueError('任务已改变，请重新读取车辆状态')

    def capture(self, now, force=False):
        r = self.record
        if not r or self.m.task != 'route' or self.m.phase not in ('RUNNING', 'PAUSING'):
            return
        progress=self.runtime_base+max(0,self.m.tracker.st.s-self.runtime_entry)
        if isinstance(self.m.tracker,SectionTracker):
            t=self.m.tracker;child=t.trackers[t.index]
            guard=getattr(child,'paint_start_guard',None)
            if guard is not None and not guard.started:
                # 即使制动滑行或定位跳变跨过起点，也不能把尚未开喷的线头记成已完成。
                start=t.offsets[t.index]+child.paint_from_m
                progress=min(progress,self.runtime_base+max(0,start-self.runtime_entry))
        r['cursor_m'] = min(r['total_m'], max(r['cursor_m'], progress))
        r['updated_at'] = time.time()
        if self.m.phase=='RUNNING' and isinstance(self.m.tracker,SectionTracker):
            continuous_circle=(self.m.differential and self.m.route is not None and self.m.route.kind=='circle'
                               and self.m.path_cfg.arc_speed_mps is not None)
            if 'circle_spray' not in r and self.m.differential and self.m.route is not None and self.m.route.kind=='circle' and not continuous_circle:
                sections=self.decode(r)
                circle_spray_repair.initialise(r,self.m,sections,painting_start(sections))
            paint=self.paint_status()
            recording_phase=(paint['paint_phase'] and self.m.tracker.st.state!='DONE'
                             and paint.get('paint_blocked')!='repair_skip_covered')
            if not continuous_circle:
                circle_spray_repair.step(r,r['cursor_m'],recording_phase,paint['painting'])
            if not r.get('circle_spray'):
                self._gap_previous=paint_gaps.observe(r,r['cursor_m'],recording_phase,paint['painting'],self._gap_previous)
            data=r.get('circle_spray')
            if not continuous_circle and data and data['missing_seen'] and not data['extra_added'] and circle_spray_repair.summary(r,r['cursor_m'])['pending_windows']:
                self._append_spray_repair()
                r=self.record
        if force or now - self.last_saved >= 1.0:
            self.save()
            self.last_saved = now

    def _append_spray_repair(self):
        """先保存追加参考，随后仅延长原圆的尾弧，保持现有控制器进度、积分和起步状态。"""
        t=self.m.tracker;old=self.record;data=old['circle_spray'];lap=data['lap_m']
        if t.st.state not in ('RUNNING','DONE'):return
        source=self.decode(old);source[-1].path=circle_spray_repair.append_path(source[-1].path,lap)
        # 圆的停喷点（route_plan.CIRCLE_RUN_OUT_M 之前）跟着顺延一圈，多走不喷的那截仍在最后。
        if source[-1].paint_to_m is not None:source[-1].paint_to_m+=lap
        extended=circle_spray_repair.append_path(t.sections[-1].path,lap)
        candidate=copy.deepcopy(old);candidate.update(plan=self.encode(source),total_m=old['total_m']+lap)
        candidate['circle_spray']['extra_added']=True
        self.record=candidate
        try:self.save()
        except ValueError:
            self.record=old
            raise
        child=t.trackers[-1];t.sections[-1].path=child.path=extended
        if t.sections[-1].paint_to_m is not None:
            t.sections[-1].paint_to_m+=lap;child.paint_to_m+=lap
            if child.paint_spans:child.paint_spans=child.paint_spans[:-1]+[(child.paint_spans[-1][0],child.paint_spans[-1][1]+lap)]
        child.st.length=extended.length;t.length+=lap;t.st.length=t.length
        if child.st.state=='DONE':
            child.st.state='PAUSED'
            if not child.start(t._view(t.sections[-1],self.m.pose),preserve_startup=True):
                t.st.state,t.st.reason='PAUSED',child.st.reason
                self.record['circle_spray']['error']='追加圈起步条件未满足，欠喷位置保留'
            else:t.st.state,t.st.reason='RUNNING',''
        t._sync()
        entry_m=(self.execution or {}).get('entry_m',self.runtime_entry)
        self.execution=dict(sections=samples(t.sections),entry_m=entry_m)
        self.m.message='已记录欠喷位置，原圈后追加一圈定点补喷'

    def _paint_window(self):
        from path_tracker import paint_window
        t=self.m.tracker
        sec=t.sections[t.index]
        # 新规划越线后才截止，区间选择须保留这一小段，否则越线时会跳到下一喷线区间。
        extra=.05 if self.m.differential and sec.paint_to_m is not None else 0.
        return paint_window(sec,t.trackers[t.index].st.s,tail_extra=extra)

    def _paint_key(self):
        t=self.m.tracker
        return (t,t.index,self._paint_window()[0])

    def _paint_tail_section(self):
        """只接管 Gen5 前进直线段尾；场地直线同用，圆、尾弧、倒车和旧底盘保持原语义。"""
        t=self.m.tracker;r=self.record
        if not r or not self.m.differential or not isinstance(t,SectionTracker):return None
        if not r.get('field_plan') and self.m.route is not None and self.m.route.kind=='circle':return None
        sec=t.sections[t.index];tr=t.trackers[t.index]
        _,start,end=self._paint_window()
        last=next(p for p in sec.path.pieces if p.s0+p.length>=end-1e-8)
        if sec.direction!=1 or getattr(sec,'field_role',None)!='paint' or not last.straight:return None
        if tr.st.s<max(last.s0,start):return None
        return tr

    def _paint_tail_active(self):
        t=self.m.tracker
        return (self._paint_tail_key==self._paint_key() and self._paint_tail_armed
                and not self._paint_tail_closed and t.trackers[t.index].st.state=='DONE'
                and t.st.state in ('RUNNING','DONE') and t._pivot is None)

    def update_paint_tail(self, now):
        """每拍控制后更新一次；status/trace 只读取，避免发布频率改变截止时机。"""
        t=self.m.tracker
        if not isinstance(t,SectionTracker):return
        key=self._paint_key()
        if key!=self._paint_tail_key:
            self._paint_tail_key=key
            self._paint_tail_armed=self._paint_tail_closed=False
            self._paint_tail_closed_at=None
        tr=self._paint_tail_section()
        if tr is None or self.m.phase!='RUNNING' or tr.st.state not in ('RUNNING','DONE'):return
        current=self.paint_status()
        if tr.st.state=='RUNNING' and not t.switching:
            self._paint_tail_armed=current['painting']
        # DONE 后只延续上一拍仍合格的喷线；任何保护触发立即结束，不在滑行中重新开喷。
        terminal=current['paint_blocked']=='not_painting' and not current['paint_phase']
        if not self._paint_tail_closed and (terminal or
                (tr.st.state=='DONE' and not current['painting'])):
            self._paint_tail_closed=True
            self._paint_tail_closed_at=now

    def waiting_for_paint_tail(self, now):
        """最终 DONE 仍发零速；截止那一拍保留 RUNNING，使 50 Hz trace 先发停喷再结束作业。"""
        if self._paint_tail_section() is None:return False
        t=self.m.tracker
        return (self._paint_tail_key==self._paint_key() and
                (self._paint_tail_active() or self._paint_tail_closed_at==now))

    def paint_status(self):
        t=self.m.tracker;r=self.record
        empty=dict(paint_phase=False,painting=False,paint_blocked='not_painting',nozzle_deviation_m=None,
                   paint_offset_limit_m=paint_quality.offset_limit(),paint_geometry_allowed=False)
        if not r or not isinstance(t,SectionTracker):return empty
        sec=t.sections[t.index];role=getattr(sec,'field_role',None)
        _,start,end=self._paint_window()
        leading=not start<=t.trackers[t.index].st.s<end
        tr=t.trackers[t.index]
        tail=self._paint_tail_section()
        late_cutoff=tail is not None and sec.paint_to_m is not None
        if late_cutoff:leading=not start<=tr.st.s<end+.05
        coast=tail is not None and self._paint_tail_active()
        closed=self._paint_tail_key==self._paint_key() and self._paint_tail_closed
        phase=role=='paint' and (sec.direction>0 or not self.m.differential) and not leading and (not t.switching or coast) and self.m.phase=='RUNNING'
        phase=phase and not (tail is not None and closed)
        current=paint_quality.status(self.m,r,phase and (t.st.state=='RUNNING' or coast),sec,tr)
        if phase and tail is not None:
            delay=paint_quality.close_delay()
            if delay is None:
                current.update(painting=False,paint_blocked='paint_configuration_invalid')
            else:
                remaining,speed=tr.nozzle_end(self.m.pose,*paint_quality.nozzle(self.m,r),at=end)
                # 至少留 5 mm，或一拍实际喷头行程，避免本拍开着跨过终点；补偿只提前关，不延迟关。
                margin=max(.005,max(0.,speed)*CONTROL_DT)+max(0.,speed)*delay
                # 新直线规划必须越过线尾后才关闭，不使用提前采样余量或提前关泵补偿。
                # 保留 5 mm 的越线确认，避免定位在边界附近抖动时提前锁存。
                if late_cutoff:margin=-.005
                if not math.isfinite(remaining) or not math.isfinite(speed):
                    current.update(painting=False,paint_blocked='paint_position_invalid')
                elif remaining<=margin+1e-9 or (coast and speed<=0.):
                    phase=False
                    current.update(painting=False,paint_blocked='not_painting')
        current['paint_phase']=bool(phase)
        current['paint_geometry_allowed']=current['painting']
        cursor=self.runtime_base+max(0.,t.st.s-self.runtime_entry)
        if current['painting'] and not paint_quality.continuous_arc(self.m,sec,tr) and not circle_spray_repair.allowed(r,cursor):
            current.update(painting=False,paint_blocked='repair_skip_covered')
        return current

    def mark_paint(self, painting):
        """每拍控制后调一次，painting 与这一拍 /line/trace 发给水泵的喷线资格同值；开/关变化的那一拍记下实际喷头位置。

        2026-10-07：App 地图的喷涂线原来从第一次收到 painting=true 的状态（/line/status 5 Hz + 网络）画起，线头白一截、
        线尾早断一截；改成按这些点起止。暂停、结束、中止那一拍 painting 为 False，照样记「关」。"""
        job = self.record['id'] if self.record else None
        if job != self._paint_mark_job:
            self._paint_marks.clear()
            self._paint_mark_job, self._paint_mark_on = job, False
        painting = bool(painting) and job is not None
        if painting == self._paint_mark_on:
            return
        self._paint_mark_on = painting
        self._paint_mark_seq += 1
        x = y = None
        try:
            pose = self.m.pose
            f, l = paint_quality.nozzle(self.m, self.record)
            px, py = rp.nozzle_at(pose.x, pose.y, pose.yaw, f, l)
            if math.isfinite(px) and math.isfinite(py):
                x, y = round(px, 4), round(py, 4)
        except (AttributeError, TypeError, ValueError):
            pass   # 没有位姿或喷头配置时只记开/关，App 照样断线，不画这个点
        self._paint_marks.append(dict(seq=self._paint_mark_seq, on=painting, x=x, y=y))

    def pause(self, cmd, now):
        self.check_identity(cmd)
        if self.held:
            return '已暂停' if self.m.phase != 'PAUSING' else '正在停车'
        if self.m.task != 'route' or self.m.phase not in ('SELECTING', 'RUNNING'):
            raise ValueError('当前没有可暂停的路线任务')
        try:
            self.capture(now, force=True)
        except ValueError:
            pass  # A disk error must never prevent the requested stop.
        self.m.tracker.pause('operator')
        self.record.update(state='PAUSING', reason='operator')
        self.preview = None
        self.m.phase, self.m.message = 'PAUSING', '正在停车，任务进度已保留'
        self.m._usable_since = self.still_since = None
        try:
            self.save()
        except ValueError as exc:
            self.m.message += '；' + str(exc)
        return self.m.message

    def end(self, cmd, now):
        """迟到的补画结束请求不能结束刚恢复的原任务。"""
        self.check_identity(cmd, require_job=False)
        requested = cmd.get('job_id')
        if not self.record or requested != self.record['id']:
            if self.record and any(h.get('id') == requested and h.get('kind') == 'rework' and
                                   h.get('parent_job_id') == self.record['id'] for h in self.history):
                return '该补画已结束；原任务断点已保留'
            raise ValueError('任务已改变，未结束当前任务')
        if self.record['state'] in ('DONE', 'CANCELLED'):
            return '该任务已结束'
        self.m.tracker.stop('operator')
        self.m._finish('操作员结束作业', state='CANCELLED')
        if self.record.get('reason') == 'storage_error':
            raise ValueError(self.m.message)
        return self.m.message

    def hold_tick(self, now, pose, mode):
        # Keep updating the stop point inside the current traversal branch, without
        # advancing across a cusp or allowing a position jump to move the checkpoint.
        if pose and pose.usable_for_control and isinstance(self.m.tracker, SectionTracker):
            t = self.m.tracker
            tr, sec = t.trackers[t.index], t.sections[t.index]
            if not t.switching:
                s, e, _, _ = tr.locate(pose.x, pose.y, self._travel_yaw(pose.yaw, sec.direction), tr._hint)
                if abs(e) < 0.15:
                    t.st.s = t.offsets[t.index] + min(tr.path.length, max(0.0, s))
        try:
            self.capture(now)
        except ValueError:
            pass  # Keep commanding zero and wait for measured standstill.
        still = stationary(pose)
        self.still_since = (self.still_since if self.still_since is not None else now) if still else None
        if self.still_since is not None and now - self.still_since >= 0.3 - 1e-8:
            self.record.update(state='PAUSED')
            self.m.phase, self.m.message = 'PAUSED', '已暂停，可手动挪车；点继续作业恢复'
            self.m._close_log()
            try:
                self.save()
            except ValueError as exc:
                self.m.message += '；' + str(exc)
            return 0.0, 0.0, False if mode == 'NAVIGATION' else None
        return 0.0, 0.0, None

    @staticmethod
    def _travel_yaw(yaw, direction):
        return yaw if direction > 0 else rp.wrap(yaw + math.pi)

    def ready(self, now):
        m = self.m
        if m.phase not in ('IDLE', 'PAUSED'):
            raise ValueError('请先暂停并等待车辆停稳')
        if m.last_motion_mode not in ('IDLE', 'NAVIGATION'):
            raise ValueError('请先停车、退出手动并解除急停或故障')
        p = m._control_pose(now)
        if p is None or not p.usable_for_control:
            raise ValueError('定位不可用，请等待定位恢复')
        if abs(p.speed) > 0.02 or abs(p.yaw_rate) > 0.05:
            raise ValueError('车辆尚未停稳')
        return p

    def start_field(self,cmd,now):
        """Accept only a recent server-generated plan, with an explicit outside-space acknowledgement."""
        value=cmd.get('field_plan')
        if not isinstance(value,dict):raise ValueError('缺少车端计算的场地规划')
        token=value.get('token')
        if token and token==self.last_token:return '该场地规划已开始'
        if self.m.phase!='IDLE' or self.held:raise ValueError('请先结束当前任务')
        p=self.ready(now)
        if not self.m.differential:raise ValueError('场地规划需要差速底盘')
        if not 0<=now-value.get('created_at',-1e9)<=60:raise ValueError('规划已过期，请重新计算')
        if value.get('origin')!=self.m._origin() or value.get('route_rev')!=self.m.route_rev:
            raise ValueError('地图或车辆路线已变化，请重新规划')
        start=value.get('start',[])
        if len(start)!=3 or math.hypot(p.x-start[0],p.y-start[1])>.10 or abs(rp.wrap(p.yaw-start[2]))>math.radians(5):
            raise ValueError('车辆位置或朝向已变化，请重新规划')
        body=value.get('body',[])
        if len(body)!=3 or any(abs(a-b)>.001 for a,b in zip(body,self.m.body)):
            raise ValueError('车身参数已改变，请重新规划')
        if not value.get('inside') and cmd.get('allow_outside') is not True:
            raise ValueError('请先核对规划中标出的外场范围')
        raw=value.get('motions')
        if not isinstance(raw,list) or not 1<=len(raw)<=500:raise ValueError('规划分段数量无效')
        sections=[];last=start[:2]
        for item in raw:
            pose=item.get('start',[]);pieces=item.get('pieces',[])
            if len(pose)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose):raise ValueError('规划位姿无效')
            if math.dist(pose[:2],last)>.01:raise ValueError('规划分段不连续')
            if not isinstance(pieces,list) or not 1<=len(pieces)<=32:raise ValueError('规划曲线无效')
            for length,k in pieces:
                rp._number(length,'规划长度',1e-8,5000)
                rp._number(k,'规划曲率',-1/self.m.min_route_radius,1/self.m.min_route_radius)
            direction=item.get('direction')
            if type(direction) is not int or direction not in (-1,1):raise ValueError('规划方向无效')
            sec=Section(direction,Path(tuple(pose),pieces))
            sec.cruise_mps=rp._number(item.get('speed'),'分段速度',.02,MAX_SECTION_SPEED)
            sec.field_role=item.get('role')
            sec.continuous_after=item.get('continuous_after',False)
            sec.entry_speed_mps=rp._number(item['entry_speed'],'接入速度',.02,MAX_SECTION_SPEED) if item.get('entry_speed') is not None else None
            # 刷线段前面的直线助跑不喷（field_planner.RUN_IN_M）；旧规划没有这一项按 0
            sec.paint_from_m=rp._number(item.get('paint_from',0.0),'助跑长度',0,sec.path.length)
            sec.paint_to_m=rp._number(item['paint_to'],'划线终点',sec.paint_from_m,sec.path.length) if item.get('paint_to') is not None else None
            sections.append(sec);last=sec.car_pose(sec.path.length)[:2]
        from dataclasses import replace
        tracker=SectionTracker(sections,replace(self.m.path_cfg,track_x_m=0,start_min_left_m=0),self.m.geo,**self.m.section_options())
        ok,why=tracker.check_start(p)
        if not ok:raise ValueError('起步检查未通过：'+why)
        old=self.record,self.parent
        label=('选边作业 · %d 条' % len(value['order'])) if value.get('selection_mode')=='selected' else '场地作业 · 内场优先'
        self.record=self.new_record(sections,label=label)
        self.record.update(field_plan=True,cruise=value.get('paint_speed',DEFAULT_PAINT_SPEED))
        self.record['selected_strokes']=[v['selection'] for v in value['order']] if value.get('selection_mode')=='selected' else None
        self.record['field_context'] = dict(
            boundary=[self.m.fusion.to_geodetic(*q) for q in value['boundary']],
            forward=value['nozzle_forward_m'], right=value['nozzle_right_m'],
            transit_speed=value['transit_speed'],turn=value.get('turn','pivot'),turn_radius=value.get('turn_radius') or 1.)
        if isinstance(value.get('source_job'), dict):
            from field_rework import encode_source
            self.record['field_source'] = encode_source(value['source_job'], self.m.fusion)
            self.record['field_source_hash'] = signature(self.record['field_source'])
            self.record['field_profile'] = self.field_profile(self.record)
        self.parent=None
        try:self.save()
        except ValueError:
            self.record,self.parent=old
            raise
        self.runtime_base=self.runtime_entry=0.0
        self.m.tracker,self.m.task,self.m.cruise=tracker,'route',self.record['cruise']
        self.m.phase,self.m.select_since,self.m.release_pending='SELECTING',now,False
        self.m.message='申请控制权 · 内场优先场地路线'
        self.execution=dict(sections=samples(sections),entry_m=0.0,target=value['target'])
        self.last_token,self.preview=token,None
        return self.m.message

    def prepare(self, cmd, now):
        operation = cmd.get('operation')
        self.check_identity(cmd, require_job=operation == 'resume')
        p = self.ready(now)
        r = self.record
        if operation == 'resume':
            if not self.held:
                raise ValueError('没有可继续的任务')
            if r['route_hash'] != self.fingerprint() or r['profile'] != self.profile():
                raise ValueError('路线或车辆参数已改变，不能套用旧断点')
            overlap = rp._number(cmd.get('overlap_m', 0), '接缝重叠', 0, 0.5)
            base = max(0.0, r['cursor_m'] - overlap)
            rest = tail(self.decode(r), base)
            label = '从 %.2f m 继续%s' % (r['cursor_m'], '（重叠 %.2f m）' % overlap if overlap else '')
        elif operation == 'rework':
            if r and r['kind'] == 'rework' and r['state'] not in ('DONE', 'CANCELLED'):
                raise ValueError('请先结束当前补画')
            if r and r.get('field_plan'):
                self.check_identity(cmd)
                if not self.held:
                    raise ValueError('请先暂停并保留原场地任务')
                if not r.get('field_source'):
                    raise ValueError('旧任务没有完整图纸，仍可续画；局部补画需重新载入场地')
                if r['field_source_hash'] != signature(r['field_source']) or r['field_profile'] != self.field_profile(r):
                    raise ValueError('场地图纸或车辆参数已改变，不能套用旧任务')
                rest, label = None, ''  # 下方用原图纸和喷头几何生成区间。
            else:
                if 'element_id' in cmd:
                    raise ValueError('当前没有可局部补画的整场任务')
                if self.m.route is None:
                    raise ValueError('车辆没有路线')
                route_nozzle = self.rework_nozzle(cmd, r)
                rest, label = rework_sections(self.m._route_local(), cmd.get('element'), cmd.get('from_m'),
                                             cmd.get('to_m'), self.m.min_route_radius, self.m.body, self.m.line_cfg.track_x_m,
                                             nozzle=(route_nozzle['forward'], -route_nozzle['right']) if route_nozzle else None)
            base = 0.0
            label += ' · %.2f–%.2f m' % (cmd['from_m'], cmd['to_m'])
        else:
            raise ValueError('未知预览操作')
        k = 0.0 if (r or {}).get('field_plan') else self.m.line_cfg.track_x_m
        car = (p.x + k * math.cos(p.yaw), p.y + k * math.sin(p.yaw), p.yaw)
        if (r or {}).get('field_plan'):
            route_nozzle = None
        elif operation=='resume':
            # 普通差速路线续作业沿用原任务喷头，和开始作业用同一套接入；不能退回阿克曼倒车连接。
            route_nozzle=self.rework_nozzle({},r)
        outside = None
        target = context = None
        field = bool((r or {}).get('field_plan'))
        source_rest=None
        unpainted_circle=False
        if operation=='resume' and not field and route_nozzle and r.get('kind')=='route' and self.m.route is not None:
            original=self.decode(r);paint_at=painting_start(original)
            unpainted_circle=(self.m._route_local().kind=='circle' and paint_at is not None
                              and r['cursor_m']<=paint_at+1e-5)
        if unpainted_circle:
            # 圆还没开始划：挪车后废弃旧接入，从当前位置接同一个完整圆；圈的半径、方向和圈数不变。
            spec=self.m._route_local()
            plan,entry_m=rp.plan_current(spec,car,self.m.min_route_radius,body=self.m.body,
                nozzle_x=(route_nozzle['forward'],-route_nozzle['right']))
            from live_cruise import label_sections
            label_sections(plan,entry_m,r['cruise'])
            rest=tail(plan,entry_m);source_rest=rest;base=0.0
            entry=[];remaining=entry_m
            for sec in plan:
                amount=min(remaining,sec.path.length)
                if amount>1e-8:entry.append(Section(sec.direction,cut(sec.path,0.0,amount)))
                remaining-=amount
                if remaining<=1e-8:break
            label='未开始画圆，从当前位置接入整圆'
        elif operation == 'rework' and field:
            from field_rework import prepare
            rest, entry, entry_m, outside, target, label = prepare(r, self.m, cmd, car)
            label += ' · %.2f–%.2f m' % (cmd['from_m'], cmd['to_m'])
            plan = entry + rest
        elif operation == 'resume' and field:
            if r.get('field_profile') and r['field_profile'] != self.field_profile(r):
                raise ValueError('车辆参数已改变，不能套用旧断点')
            from field_planner import Settings, polygon, transit, speed_for, transit_limit, path_violation, pivot_violation
            saved=r.get('field_context')
            if not saved:raise ValueError('任务缺少场地边界，请重新规划')
            _,planes=polygon([list(self.m.fusion.to_local(*q)) for q in saved['boundary']])
            cfg=Settings(saved['forward'],saved['right'],self.m.body,self.m.min_route_radius+.001,
                         r['cruise'],saved['transit_speed'],yaw_rate=self.m.geo.pivot_rate_rad_s,
                         yaw_accel=self.m.geo.pivot_accel_rad_s2,accel=self.m.path_cfg.accel_mps2,
                         lateral_accel=self.m.path_cfg.lat_accel_mps2,turn=saved.get('turn','pivot'),turn_radius=saved.get('turn_radius',1.))
            first=rest[0]
            extra=max(0.0,cfg.run_in-first.paint_from_m) if first.field_role=='paint' and first.direction>0 else 0.0
            if extra>1e-6:
                # 断点在刷线段上：和开始作业一样在断点前接直线助跑，水泵到断点时已经开稳（助跑不喷）
                x,y,h=first.path.pose(0.0)
                lead=Section(1,Path((x-extra*math.cos(h),y-extra*math.sin(h),h),
                                    [(extra,0.0)]+[(q.length,q.kappa) for q in first.path.pieces]))
                lead.cruise_mps,lead.field_role,lead.paint_from_m=first.cruise_mps,'paint',first.paint_from_m+extra
                lead.paint_to_m=None if first.paint_to_m is None else first.paint_to_m+extra
                lead.paint_spans=None if first.paint_spans is None else [(a+extra,b+extra) for a,b in first.paint_spans]
                lead.entry_speed_mps=getattr(first,'entry_speed_mps',None)
                lead.continuous_after=first.continuous_after
                rest=[lead]+rest[1:]
            entry,cost,_=transit(car,rest[0].car_pose(0),planes,cfg)
            for sec in entry:
                sec.cruise_mps=transit_limit(sec,cfg);sec.field_role='transit'
            entry_m=sum(s.path.length for s in entry)
            if entry_m>rp.MAX_ENTRY_M:raise ValueError('请先手动开到断点附近，再预览接入')
            entry_m+=extra    # 助跑接在剩余路线前面，进度从断点算起
            plan=entry+rest
            if cfg.turn=='arc':
                for sec in entry:sec.continuous_after=True
            outside=cost[1];pose=rest[0].car_pose(0)
            for sec in rest:
                outside=max(outside,pivot_violation(pose,sec.car_pose(0)[2],planes,cfg),path_violation(sec,planes,cfg))
                pose=sec.car_pose(sec.path.length)
            label+=' · 内场优先接入'
        elif route_nozzle is not None:
            # 差速车补画：与开始作业同样接入，水泵在补画起点前已沿线助跑、对正
            plan, entry_m, entry = pivot_entry_to(rest, car, self.m.min_route_radius,
                                                  (route_nozzle['forward'], -route_nozzle['right']))
        else:
            plan, entry_m, entry = entry_to(rest, car, self.m.min_route_radius)
        if field and self.field_source_usable(r):
            _, context = self.field_info(r)
        self.preview = dict(token=str(uuid.uuid4()), request_id=cmd.get('id'), operation=operation,
                            created=now, origin=self.m._origin(), car=(p.x, p.y, p.yaw),
                            route_rev=self.m.route_rev, job_id=r['id'] if r else None,
                            cursor=r['cursor_m'] if r else None, profile=self.profile(),
                            plan=plan, rest=rest, entry=entry, entry_m=entry_m, base=base, label=label,
                            selection={k: cmd[k] for k in ('element', 'element_id', 'edge_index', 'from_m', 'to_m') if k in cmd},
                            max_outside_m=outside, field_plan=field, target=target, context=context,
                            field_source_hash=(r or {}).get('field_source_hash'), route_nozzle=route_nozzle, source_rest=source_rest)
        return '预览已就绪，请确认接入路径'

    def rework_nozzle(self, cmd, record):
        """连线路线 / 圆补画用的水泵 {forward, right}：补画指令带的 app 对准点，没带用原任务记的，再没有用共享配置的
        水泵；阿克曼车 None（按车身中心）。"""
        if not self.m.differential:
            return None
        for v in (cmd.get('nozzle'), (record or {}).get('route_nozzle')):
            if isinstance(v, dict) and all(isinstance(v.get(k), (int, float)) and not isinstance(v.get(k), bool)
                                           and math.isfinite(v[k]) for k in ('forward', 'right')):
                return dict(forward=float(v['forward']), right=float(v['right']))
        f, l = self.m.nozzle
        return dict(forward=f, right=-l)

    def commit(self, cmd, now, operation):
        if cmd.get('preview_token') and cmd['preview_token'] == self.last_token:
            return '该操作已接受'
        v = self.preview
        if not v or v['operation'] != operation or cmd.get('preview_token') != v['token'] or now - v['created'] > 60:
            raise ValueError('预览已过期，请重新预览')
        self.check_identity(cmd, require_job=operation == 'resume' or v.get('field_plan', False))
        p = self.ready(now)
        r = self.record
        if (self.m._origin() != v['origin'] or self.m.route_rev != v['route_rev'] or self.profile() != v['profile'] or
                (r['id'] if r else None) != v['job_id'] or (r['cursor_m'] if r else None) != v['cursor'] or
                math.hypot(p.x - v['car'][0], p.y - v['car'][1]) > 0.10 or abs(rp.wrap(p.yaw - v['car'][2])) > math.radians(5)):
            raise ValueError('车辆位置或任务已改变，请重新预览')
        if v.get('field_plan') and r.get('field_source') and operation == 'rework':
            if (v['field_source_hash'] != r.get('field_source_hash') or
                    signature(r['field_source']) != r['field_source_hash'] or self.field_profile(r) != r['field_profile']):
                raise ValueError('场地图纸或车辆参数已改变，请重新预览')
        if (v.get('max_outside_m') or 0)>1e-7 and cmd.get('allow_outside') is not True:
            raise ValueError('请先核对接入和剩余路线的外场范围')
        from dataclasses import replace
        cruise=(r or {}).get('cruise',self.m.route.cruise if self.m.route else DEFAULT_PAINT_SPEED)
        track_x=0.0 if (r or {}).get('field_plan') else self.m.line_cfg.track_x_m
        # tail 已保留原接入/助跑的 transit 和 paint_from；继续作业不能把这部分重标成已上线。
        # 新补画、以及缺少旧分段标记的任务仍按预览接入边界初始化。
        preserve_paint_roles = operation == 'resume' and all(
            getattr(sec, 'field_role', None) in ('transit', 'paint') for sec in v['plan'])
        if not v.get('field_plan') and not preserve_paint_roles:
            from live_cruise import label_sections
            label_sections(v['plan'],v['entry_m'],cruise)
            label_sections(v['rest'],0.,cruise)
        tracker = SectionTracker(v['plan'], replace(self.m.path_cfg, cruise_mps=cruise,
                                                   track_x_m=track_x, start_min_left_m=0.0), self.m.geo,
                                 **self.m.section_options(route=not v.get('field_plan')))
        ok, why = tracker.check_start(p)
        if not ok:
            raise ValueError('无法接入：' + why)
        old_record, old_parent = self.record, self.parent
        if operation == 'rework':
            self.parent = copy.deepcopy(r)
            self.record = self.new_record(v['rest'], 'rework', v['label'])
            self.record['cruise']=cruise
            self.record['rework'] = v['selection']
            self.record['parent_job_id'] = r['id'] if r else None
            if v.get('route_nozzle'):
                self.record['route_nozzle'] = dict(v['route_nozzle'])
            if v.get('field_plan'):
                for key in ('field_context', 'field_source', 'field_source_hash', 'field_profile', 'cruise', 'selected_strokes'):
                    if key in r:
                        self.record[key] = copy.deepcopy(r[key])
                self.record.update(field_plan=True, target_length_m=v['selection']['to_m']-v['selection']['from_m'])
        else:
            self.record = copy.deepcopy(r)
            if v.get('source_rest') is not None:
                self.record.pop('circle_spray',None)
                # 预览确认后才替换尚未划线的参考路径；新接入仍由 runtime_entry 排除在划线进度之外。
                self.record.update(plan=self.encode(v['source_rest']),cursor_m=0.0,
                    total_m=sum(sec.path.length for sec in v['source_rest']),updated_at=time.time())
        self.record.update(state='RUNNING', reason='')
        try:
            self.save()  # Persist acceptance before arming the controller.
        except ValueError:
            self.record, self.parent = old_record, old_parent
            raise
        self._gap_previous=None
        self.runtime_base, self.runtime_entry = v['base'], v['entry_m']
        self.m.tracker, self.m.task, self.m.cruise = tracker, 'route', cruise
        self.m.phase, self.m.select_since = 'SELECTING', now
        self.m.release_pending = False
        self.m.message = '申请控制权 · ' + ('继续作业' if operation == 'resume' else '补画')
        self.execution = dict(sections=samples(v['plan']), entry_m=v['entry_m'], target=v.get('target') or samples(v['rest']))
        self.last_token, self.preview = v['token'], None
        return self.m.message

    def finish(self, state, reason):
        r = self.record
        if not r or self.m.task != 'route':
            return
        before = copy.deepcopy((self.record, self.parent, self.history))
        if state == 'DONE':
            r['cursor_m'] = r['total_m']
            if r.get('circle_spray'):
                data=r['circle_spray'];data['stage']='finished'
                data['error']='仍有欠喷位置，未自动追加第二次补喷圈' if circle_spray_repair.summary(r,r['cursor_m'])['pending_windows'] else None
        if state=='DONE' and paint_gaps.has_gaps(r):
            reason=paint_gaps.completion_message(r)
            self.m.message=reason
        self._gap_previous=None
        r.update(state=state, reason=reason, updated_at=time.time())
        self.execution = None
        self.preview = None
        if state in ('DONE', 'CANCELLED'):
            self.history.append({k: v for k, v in r.items() if k not in ('plan', 'field_source')})
            self.history = self.history[-20:]
            if r['kind'] == 'rework' and self.parent:
                self.record, self.parent = self.parent, None
                if self.record['state'] in HELD:
                    self.m.phase, self.m.message = 'PAUSED', reason + '；原任务断点已保留，可继续'
                    self.m.cruise=self.record.get('cruise',self.m.route.cruise if self.m.route else DEFAULT_PAINT_SPEED)
        try:
            self.save()  # 完成记录和恢复父任务必须同一次原子保存。
        except ValueError:
            self.record, self.parent, self.history = before
            self.record.update(state='PAUSED', reason='storage_error')
            self.m.phase = 'PAUSED'
            raise

    def clear(self):
        self.record = self.parent = self.preview = self.execution = None
        self.save()

    def status(self):
        out = dict(actions=ACTIONS, durable=bool(self.path), spray_supported=False, error=self.error,
                   job=None, preview=None, elements=[], context=[], history=self.history[-20:], execution=self.execution)
        if self.record:
            out['job'] = {k: v for k, v in self.record.items() if k not in ('plan', 'profile', 'route_hash', 'field_context',
                                                                      'field_source', 'field_source_hash', 'field_profile','circle_spray')}
            # 本作业最近的开/关喷点（本地坐标，喷头位置）；车端有这个字段，App 的喷涂线就按它起止
            out['job']['paint_marks'] = dict(run=self._paint_marks_run, items=[dict(m) for m in self._paint_marks]
                                             if self._paint_mark_job == self.record['id'] else [])
            if isinstance(self.m.tracker,SectionTracker):
                t=self.m.tracker
                sec=t.sections[t.index];role=getattr(sec,'field_role',None)
                # 一条线画完后停车、原地转向、等停稳（switching）时段号还停在这条线上，车并没在压线；刷线段前面的直线
                # 助跑（段内路程 < paint_from_m）水泵还没到线头：都报 transit。
                # 作业阶段与喷线资格分开：知道在刷线段，不代表喷头已经压在线上。
                paint=self.paint_status()
                out['job']['current_role']='transit' if role=='paint' and (t.switching or not paint['paint_phase']) else role
                out['job'].update(paint)
                repair=circle_spray_repair.summary(self.record,self.record['cursor_m'])
                if repair is not None:out['job']['spray_repair']=repair
            out['job']['can_resume'] = self.held and self.record['route_hash'] == self.fingerprint()
            out['job']['parent_id'] = self.parent['id'] if self.parent else None
            out['job']['parent_job_id'] = self.parent['id'] if self.parent else None
            if self.field_source_usable(self.record):
                try:
                    field_elements, context = self.field_info(self.record)
                    out['context'] = context
                    if self.record['kind'] != 'rework':
                        out['elements'] = field_elements
                        out['actions'] = [*ACTIONS, 'field_rework']
                except (ValueError, KeyError, TypeError):
                    pass
        try:
            if self.m.route and not (self.record or {}).get('field_plan'):
                key=(self.m.route_rev, self.m.min_route_radius, self.m._origin())
                if key != self._elements_key:
                    self._elements=elements(self.m._route_local(), self.m.min_route_radius)
                    self._elements_key=key
                out['elements'] = self._elements
        except ValueError:
            pass
        v = self.preview
        if v:
            out['preview'] = {k: v[k] for k in ('token', 'request_id', 'operation', 'label', 'entry_m')}
            out['preview'].update(total_m=sum(s.path.length for s in v['plan']),
                                  sections=samples(v['plan']), entry=samples(v['entry']), target=v.get('target') or samples(v['rest']),
                                  max_outside_m=v.get('max_outside_m'), field_plan=v.get('field_plan',False),
                                  context=v.get('context') or [], start=list(v['car']))
        return out
