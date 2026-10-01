"""Route checkpoints, operator holds, and previewed resume/rework operations.

Distances here describe vehicle tracking, not confirmed paint coverage. This robot has
no spray controller. A checkpoint owns immutable geometry so crossings/laps never
resume by searching the nearest line in a newly generated route.
"""
import copy
import hashlib
import json
import math
import os
import time
import uuid

import route_plan as rp
from path_tracker import Path, Section, SectionTracker

HELD = ('PAUSING', 'PAUSED', 'INTERRUPTED')
ACTIONS = ['pause', 'preview', 'resume', 'rework', 'field_start', 'start_current']


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
            remaining.field_role=getattr(section,'field_role',None)
            out.append(remaining)
        offset = end
    if not out:
        raise ValueError('当前任务已到终点，无需继续')
    return out


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
    # A stop at the join makes the handover (including a gear change) explicit.
    return entry + sections, length, entry


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


def rework_sections(spec, element, start, end, radius, body, track_x=0.0):
    choices = elements(spec, radius)
    if isinstance(element, bool) or not isinstance(element, int) or not 0 <= element < len(choices):
        raise ValueError('补画线段不存在，请重新选择')
    length = choices[element]['length_m']
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (start, end)):
        raise ValueError('补画范围必须为有限数值')
    if start < 0 or end > length + 1e-4 or end - start < rp.MIN_LEG_M:
        raise ValueError('补画范围须在线段内且至少 %.1f m' % rp.MIN_LEG_M)
    end = min(end, length)
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
        self._load()

    @property
    def held(self):
        return bool(self.record and self.record['state'] in HELD)

    def fingerprint(self):
        return signature(self.m.route.to_dict(digits=9)) if self.m.route else None

    def profile(self):
        return signature(dict(body=self.m.body, radius=self.m.min_route_radius,
                              track_x=self.m.path_cfg.track_x_m, line_track_x=self.m.line_cfg.track_x_m))

    def encode(self, sections):
        f = self.m.fusion
        out = []
        for sec in sections:
            x, y, yaw = sec.path.pose(0)
            out.append(dict(direction=sec.direction, point=f.to_geodetic(x, y),
                            heading_point=f.to_geodetic(x + math.cos(yaw), y + math.sin(yaw)),
                            pieces=[[p.length, p.kappa] for p in sec.path.pieces],speed=getattr(sec,'cruise_mps',None),
                            field_role=getattr(sec,'field_role',None)))
        return out

    def decode(self, record):
        out = []
        for sec in record['plan']:
            x, y = self.m.fusion.to_local(*sec['point'])
            hx, hy = self.m.fusion.to_local(*sec['heading_point'])
            restored=Section(sec['direction'], Path((x, y, math.atan2(hy - y, hx - x)), sec['pieces']))
            restored.cruise_mps=sec.get('speed')
            restored.field_role=sec.get('field_role')
            out.append(restored)
        return out

    def new_record(self, sections, kind='route', label='路线作业'):
        return dict(id=str(uuid.uuid4()), route_rev=self.m.route_rev, route_hash=self.fingerprint(),
                    profile=self.profile(), kind=kind, label=label, state='RUNNING', reason='',
                    cursor_m=0.0, total_m=sum(s.path.length for s in sections), plan=self.encode(sections),
                    started_at=time.time(), updated_at=time.time(), rework=None)

    def begin(self, sections, entry_m=0.0):
        old = self.record, self.parent, self.preview, self.execution
        self.record = self.new_record(sections)
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
        total = 0.0
        for s in r['plan']:
            if s.get('direction') not in (-1, 1):
                raise ValueError('无效行驶方向')
            if s.get('speed') is not None:
                rp._number(s['speed'], '分段速度', .02, 1.2)
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
        r['cursor_m'] = min(r['total_m'], max(r['cursor_m'], self.runtime_base + max(0, self.m.tracker.st.s - self.runtime_entry)))
        r['updated_at'] = time.time()
        if force or now - self.last_saved >= 1.0:
            self.save()
            self.last_saved = now

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
        still = pose is not None and math.isfinite(pose.speed) and abs(pose.speed) <= 0.02 and abs(pose.yaw_rate) <= 0.05
        still = still and 'wheel_stale' not in getattr(pose, 'reasons', ())
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
            sec.cruise_mps=rp._number(item.get('speed'),'分段速度',.02,1.2)
            sec.field_role=item.get('role')
            sections.append(sec);last=sec.car_pose(sec.path.length)[:2]
        from dataclasses import replace
        tracker=SectionTracker(sections,replace(self.m.path_cfg,track_x_m=0,start_min_left_m=0),self.m.geo,**self.m.section_options())
        ok,why=tracker.check_start(p)
        if not ok:raise ValueError('起步检查未通过：'+why)
        old=self.record,self.parent
        label=('选边作业 · %d 条' % len(value['order'])) if value.get('selection_mode')=='selected' else '场地作业 · 内场优先'
        self.record=self.new_record(sections,label=label)
        self.record.update(field_plan=True,cruise=value.get('paint_speed',.5))
        self.record['selected_strokes']=[v['selection'] for v in value['order']] if value.get('selection_mode')=='selected' else None
        self.record['field_context'] = dict(
            boundary=[self.m.fusion.to_geodetic(*q) for q in value['boundary']],
            forward=value['nozzle_forward_m'], right=value['nozzle_right_m'],
            transit_speed=value['transit_speed'])
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
            if r and r.get('field_plan'):
                raise ValueError('整场任务请先结束，再选择需要重画的图形重新规划')
            if r and r['kind'] == 'rework' and r['state'] not in ('DONE', 'CANCELLED'):
                raise ValueError('请先结束当前补画')
            if self.m.route is None:
                raise ValueError('车辆没有路线')
            rest, label = rework_sections(self.m._route_local(), cmd.get('element'), cmd.get('from_m'),
                                         cmd.get('to_m'), self.m.min_route_radius, self.m.body, self.m.line_cfg.track_x_m)
            base = 0.0
            label += ' · %.2f–%.2f m' % (cmd['from_m'], cmd['to_m'])
        else:
            raise ValueError('未知预览操作')
        k = 0.0 if (r or {}).get('field_plan') else self.m.line_cfg.track_x_m
        car = (p.x + k * math.cos(p.yaw), p.y + k * math.sin(p.yaw), p.yaw)
        outside = None
        if operation == 'resume' and r.get('field_plan'):
            from field_planner import Settings, polygon, transit, speed_for, path_violation, pivot_violation
            saved=r.get('field_context')
            if not saved:raise ValueError('任务缺少场地边界，请重新规划')
            _,planes=polygon([list(self.m.fusion.to_local(*q)) for q in saved['boundary']])
            cfg=Settings(saved['forward'],saved['right'],self.m.body,self.m.min_route_radius+.001,
                         r['cruise'],saved['transit_speed'],yaw_rate=self.m.geo.pivot_rate_rad_s,
                         yaw_accel=self.m.geo.pivot_accel_rad_s2,accel=self.m.path_cfg.accel_mps2,
                         lateral_accel=self.m.path_cfg.lat_accel_mps2)
            entry,cost,_=transit(car,rest[0].car_pose(0),planes,cfg)
            for sec in entry:
                sec.cruise_mps=speed_for(sec,cfg);sec.field_role='transit'
            entry_m=sum(s.path.length for s in entry)
            if entry_m>rp.MAX_ENTRY_M:raise ValueError('请先手动开到断点附近，再预览接入')
            plan=entry+rest;outside=cost[1];pose=rest[0].car_pose(0)
            for sec in rest:
                outside=max(outside,pivot_violation(pose,sec.car_pose(0)[2],planes,cfg),path_violation(sec,planes,cfg))
                pose=sec.car_pose(sec.path.length)
            label+=' · 内场优先接入'
        else:
            plan, entry_m, entry = entry_to(rest, car, self.m.min_route_radius)
        self.preview = dict(token=str(uuid.uuid4()), request_id=cmd.get('id'), operation=operation,
                            created=now, origin=self.m._origin(), car=(p.x, p.y, p.yaw),
                            route_rev=self.m.route_rev, job_id=r['id'] if r else None,
                            cursor=r['cursor_m'] if r else None, profile=self.profile(),
                            plan=plan, rest=rest, entry=entry, entry_m=entry_m, base=base, label=label,
                            selection={k: cmd[k] for k in ('element', 'from_m', 'to_m') if k in cmd},
                            max_outside_m=outside)
        return '预览已就绪，请确认接入路径'

    def commit(self, cmd, now, operation):
        if cmd.get('preview_token') and cmd['preview_token'] == self.last_token:
            return '该操作已接受'
        v = self.preview
        if not v or v['operation'] != operation or cmd.get('preview_token') != v['token'] or now - v['created'] > 60:
            raise ValueError('预览已过期，请重新预览')
        self.check_identity(cmd, require_job=operation == 'resume')
        p = self.ready(now)
        r = self.record
        if (self.m._origin() != v['origin'] or self.m.route_rev != v['route_rev'] or self.profile() != v['profile'] or
                (r['id'] if r else None) != v['job_id'] or (r['cursor_m'] if r else None) != v['cursor'] or
                math.hypot(p.x - v['car'][0], p.y - v['car'][1]) > 0.10 or abs(rp.wrap(p.yaw - v['car'][2])) > math.radians(5)):
            raise ValueError('车辆位置或任务已改变，请重新预览')
        if (v.get('max_outside_m') or 0)>1e-7 and cmd.get('allow_outside') is not True:
            raise ValueError('请先核对接入和剩余路线的外场范围')
        from dataclasses import replace
        cruise=self.m.route.cruise if self.m.route else (r or {}).get('cruise',.5)
        track_x=0.0 if (r or {}).get('field_plan') else self.m.line_cfg.track_x_m
        tracker = SectionTracker(v['plan'], replace(self.m.path_cfg, cruise_mps=cruise,
                                                   track_x_m=track_x, start_min_left_m=0.0), self.m.geo, **self.m.section_options())
        ok, why = tracker.check_start(p)
        if not ok:
            raise ValueError('无法接入：' + why)
        old_record, old_parent = self.record, self.parent
        if operation == 'rework':
            self.parent = copy.deepcopy(r)
            self.record = self.new_record(v['rest'], 'rework', v['label'])
            self.record['rework'] = v['selection']
        else:
            self.record = copy.deepcopy(r)
        self.record.update(state='RUNNING', reason='')
        try:
            self.save()  # Persist acceptance before arming the controller.
        except ValueError:
            self.record, self.parent = old_record, old_parent
            raise
        self.runtime_base, self.runtime_entry = v['base'], v['entry_m']
        self.m.tracker, self.m.task, self.m.cruise = tracker, 'route', cruise
        self.m.phase, self.m.select_since = 'SELECTING', now
        self.m.release_pending = False
        self.m.message = '申请控制权 · ' + ('继续作业' if operation == 'resume' else '补画')
        self.execution = dict(sections=samples(v['plan']), entry_m=v['entry_m'], target=samples(v['rest']))
        self.last_token, self.preview = v['token'], None
        return self.m.message

    def finish(self, state, reason):
        r = self.record
        if not r or self.m.task != 'route':
            return
        if state == 'DONE':
            r['cursor_m'] = r['total_m']
        r.update(state=state, reason=reason, updated_at=time.time())
        self.execution = None
        self.preview = None
        if state in ('DONE', 'CANCELLED'):
            self.history.append({k: v for k, v in r.items() if k != 'plan'})
            self.history = self.history[-20:]
            if r['kind'] == 'rework' and self.parent:
                self.record, self.parent = self.parent, None
                if self.record['state'] in HELD:
                    self.m.phase, self.m.message = 'PAUSED', reason + '；原任务断点已保留，可继续'
        self.save()  # Completion, history, and restoring the parent are one atomic write.

    def clear(self):
        self.record = self.parent = self.preview = self.execution = None
        self.save()

    def status(self):
        out = dict(actions=ACTIONS, durable=bool(self.path), spray_supported=False, error=self.error,
                   job=None, preview=None, elements=[], history=self.history[-20:], execution=self.execution)
        if self.record:
            out['job'] = {k: v for k, v in self.record.items() if k not in ('plan', 'profile', 'route_hash', 'field_context')}
            if self.record.get('field_plan') and isinstance(self.m.tracker,SectionTracker):
                t=self.m.tracker
                role=getattr(t.sections[t.index],'field_role',None)
                # 一条线画完后停车、原地转向、等停稳（switching）时段号还停在这条线上，车并没在压线：报 transit。
                # painting 是喷头该不该喷的唯一依据：刷线段、行驶中、正在跟线（不在换段 / 原地转、没暂停）
                out['job']['current_role']='transit' if t.switching and role=='paint' else role
                out['job']['painting']=role=='paint' and not t.switching and t.st.state=='RUNNING' and self.m.phase=='RUNNING'
            out['job']['can_resume'] = self.held and self.record['route_hash'] == self.fingerprint()
            out['job']['parent_id'] = self.parent['id'] if self.parent else None
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
                                  sections=samples(v['plan']), entry=samples(v['entry']), target=samples(v['rest']),
                                  max_outside_m=v.get('max_outside_m'))
        return out
