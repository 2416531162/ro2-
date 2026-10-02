#!/usr/bin/env python3
"""在电脑上只读回放已导出的 Gen5 传感器数据；不调用 ROS 或车辆接口。"""
import argparse
from collections import Counter
import gzip
import json
import math
import os
from pathlib import Path
import sys


def replay(record, fusion_module, mission_type, overrides):
    cfg = dict(record['fusion_config'])
    for key in ('baseline_m', 'baseline_tol_m', 'baseline_check'):
        cfg.pop(key, None)
    # 与 line_node.load_fusion_config 的 Gen5 默认值相同；不改航向融合参数。
    for key, value in dict(drive_differential=True, yaw_input='angle', yaw_scale_sigma=.03,
                           heading_sync_s=.02, yaw_walk_deg_sqrt_s=.1).items():
        cfg.setdefault(key, value)
    cfg.update(overrides)
    if 'antenna_xy' in cfg:
        cfg['antenna_xy'] = tuple(cfg['antenna_xy'])
    fusion = fusion_module.GnssFusion(fusion_module.FusionConfig(**cfg))
    mission = mission_type(fusion, fusion_module, body=(.8,.3,.4), nozzle_x=.3)
    events = record['events']
    begin = events[0][0]
    tick = begin
    rows = []
    online = [e for e in events if e[1] == 'status']
    active = [e[0] for e in online if e[2].get('phase') in ('SELECTING','RUNNING','PAUSING','PAUSED')]
    active_start, active_end = (min(active),max(active)) if active else (begin,events[-1][0])
    was_usable = False
    initial_cal = None
    pivot_outliers = 0
    previous_outliers = 0
    actual_status = None
    for received, kind, value in events:
        while tick <= received + 1e-10:
            pose = fusion.pose(tick)
            cal = fusion.calibration()
            if initial_cal is None and tick >= active_start:
                initial_cal = cal
            if pose is not None:
                was_usable |= pose.usable_for_control
                turning = bool((actual_status or {}).get('tracker', {}).get('pivoting'))
                if turning:
                    pivot_outliers += max(0, cal['rejected']['position_outliers']-previous_outliers)
                previous_outliers = cal['rejected']['position_outliers']
                rows.append({'t':tick-begin,'active':active_start<=tick<=active_end,'pivoting':turning,
                             'usable':pose.usable_for_control,'reasons':pose.reasons,'x':pose.x,'y':pose.y,
                             'yaw':pose.yaw,'sigma_xy':pose.sigma_xy,'resets':cal['position_resets'],
                             'position_outliers':cal['rejected']['position_outliers']})
            tick += .05
        if kind == 'gnss':
            mission.on_gnss(value)
        elif kind == 'imu':
            t, gz, yaw, roll, pitch = value
            mission.on_imu(t,gz,yaw,roll=roll,pitch=pitch)
        elif kind == 'wheel':
            mission.on_wheel(*value)
        elif kind == 'status':
            actual_status = value
    cal = fusion.calibration()
    active_rows = [r for r in rows if r['active']]
    bad = [r for r in active_rows if not r['usable']]
    online_resets = [e[2].get('calibration',{}).get('position_resets',0) for e in online]
    result = {'job':record['job'],'segment':record['segment'],'recorded_release':record['release'],
              'duration_s':round(events[-1][0]-begin,3),'ticks':len(rows),
              'online_reset_delta':max(online_resets)-min(online_resets),
              'position_resets':cal['position_resets'],'rejected':cal['rejected'],
              'unusable_all_s':round(sum(not r['usable'] for r in rows)*.05,3),
              'unusable_active_s':round(len(bad)*.05,3),
              'active_reasons_s':{k:round(v*.05,3) for k,v in Counter(k for r in bad for k in r['reasons']).items()},
              'pivot_position_outliers':pivot_outliers,
              'active_max_sigma_xy':max((r['sigma_xy'] for r in active_rows),default=None)}
    return result,rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--overrides',default='{}')
    parser.add_argument('--prefix',default='')
    args = parser.parse_args()
    source=args.source.resolve()
    os.environ['RK3588_ROBOT_CONFIG']=str(source/'robot_core/robot_gen5.json')
    sys.path[:0]=[str(source/'radar_system'),str(source)]
    import gnss_fusion
    from line_mission import LineMission
    with gzip.open(args.data,'rt',encoding='utf-8') as fh:
        records=json.load(fh)
    results=[]
    traces={}
    for record in records:
        if args.prefix and not record['job'].startswith(args.prefix):
            continue
        result,rows=replay(record,gnss_fusion,LineMission,json.loads(args.overrides))
        results.append(result);traces[record['job']+'/'+record['segment']]=rows
        print(record['job'],'重置',result['position_resets'],'作业中不可用',result['unusable_active_s'],
              '秒，转向离群',result['pivot_position_outliers'],result['active_reasons_s'],flush=True)
    args.out.write_text(json.dumps(results,ensure_ascii=False,indent=2))
    with gzip.open(str(args.out)+'.trace.gz','wt',encoding='utf-8') as fh:
        json.dump(traces,fh,ensure_ascii=False,separators=(',',':'))


if __name__=='__main__':
    main()
