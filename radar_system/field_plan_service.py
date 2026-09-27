"""Read-only planning cache. The motion node rechecks a plan before it can move."""
import copy
import math
import threading
import time
import uuid
from field_planner import Settings, plan


class FieldPlanService:
    def __init__(self):
        self.lock=threading.Lock();self.plans={}

    def prepare(self,request,line,now=None):
        now=time.monotonic() if now is None else now
        if not isinstance(request,dict) or not isinstance(line,dict):raise ValueError('没有车辆定位状态')
        age=now-line.get('_rx_monotonic',-1e9)
        pose=line.get('pose') or {};limits=line.get('route_limits') or {}
        if not 0<=age<=1.5 or not pose.get('usable'):raise ValueError('请等待新鲜、可用的 RTK 定位')
        if line.get('phase')!='IDLE' or line.get('motion_mode')!='IDLE':raise ValueError('请先停稳并结束当前作业')
        if abs(pose.get('speed',1))>.02:raise ValueError('车辆还在移动')
        if limits.get('drive')!='differential':raise ValueError('内场优先规划需要差速底盘')
        if request.get('origin')!=line.get('origin'):raise ValueError('地图原点已改变，请重新定位图纸')
        nozzle=request.get('nozzle')
        if not isinstance(nozzle,list) or len(nozzle)!=2:raise ValueError('缺少已配置的喷头位置')
        cfg=Settings(nozzle[0],nozzle[1],tuple(limits['body']),limits['min_radius']+.001,
                     request.get('paint_speed',.5),request.get('transit_speed',.8),
                     yaw_rate=limits.get('pivot_rate',.8),yaw_accel=limits.get('pivot_accel',1.0),
                     accel=limits.get('accel',.5),lateral_accel=limits.get('lateral_accel',.05))
        start=(pose['x'],pose['y'],math.radians(pose['yaw_deg']))
        result=plan(request.get('job'),start,cfg)
        token=str(uuid.uuid4())
        result.update(token=token,origin=line['origin'],route_rev=(line.get('route') or {}).get('rev',0),
                      created_at=now,source_job=copy.deepcopy(request['job']),spray_supported=False)
        with self.lock:
            self.plans={k:v for k,v in self.plans.items() if now-v['created_at']<=90}
            if len(self.plans)>=8:self.plans.pop(next(iter(self.plans)))
            self.plans[token]=copy.deepcopy(result)
        return {k:v for k,v in result.items() if k not in ('motions','source_job','created_at')}

    def get(self,token,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            result=self.plans.get(token)
            if result is None or not 0<=now-result['created_at']<=60:raise ValueError('规划已过期，请按当前车位重新计算')
            return copy.deepcopy(result)
