#!/usr/bin/env python3
"""在临时目录加载故意破坏的模块，检查线尾测试能否检出；不改仓库文件、不连接车辆。
用仿真所用的 Python 环境运行，输出 JSON；全部检出返回 0。
"""
import subprocess,pathlib,tempfile,json,sys
py=sys.executable
root=pathlib.Path(__file__).resolve().parents[2]
mutations=[
 ('early_brake','path_tracker.py','if nozzle_remaining < 0.:','if nozzle_remaining < .15:'),
 ('wrong_decel_cap','path_tracker.py','run_out_decel_max_mps2: float = 1.0','run_out_decel_max_mps2: float = .5'),
 ('early_zero','path_tracker.py','and (self.run_out_m <= 0 or self._run_out_started and abs(pose.speed) <= cfg.min_mps*1.5)',''),
 ('no_run_out','route_plan.py','LINE_RUN_OUT_M = 0.40','LINE_RUN_OUT_M = 0.0'),
 ('transit_paints','route_plan.py','*, paint=False):','*, paint=True):'),
 ('labels_overwrite','live_cruise.py',"if getattr(sec,'field_role',None) is not None:", 'if False:'),
 ('tail_endpoint','path_tracker.py','self.path.length if at is None else at','self.path.length'),
 ('lost_resume_boundary','route_operations.py',"remaining.paint_to_m=None if section.paint_to_m is None else max(0.,section.paint_to_m-consumed)", 'remaining.paint_to_m=section.paint_to_m'),
 ('lost_span_key','route_operations.py','return (t,t.index,self._paint_window()[0])','return (t,t.index,0)'),
 ('no_field_run_out','field_planner.py', '[(cfg.run_out,0)] if cfg.run_out>0 else []','[]'),
 ('lost_serialized_end','field_planner.py',"paint_to=m.get('paint_to')", 'paint_to=None'),
 ('lost_record_end','route_operations.py', "restored.paint_to_m=sec.get('paint_to')", 'restored.paint_to_m=None'),
]
results=[]
for name,file,a,b in mutations:
 with tempfile.TemporaryDirectory(prefix='le-mut-') as d:
  p=root/'radar_system'/file;s=p.read_text();assert a in s,name
  pathlib.Path(d,file).write_text(s.replace(a,b))
  bootstrap="import sys;sys.path[:0]="+repr([d,str(root/'radar_system'),str(root),str(root/'tests')])+";import "+file[:-3]+";import pytest;raise SystemExit(pytest.main(['-q','tests/test_line_end_run_out.py','--tb=no']))"
  r=subprocess.run([py,'-c',bootstrap],cwd=root,capture_output=True,text=True)
  results.append(dict(name=name,killed=r.returncode==1 and 'failed' in r.stdout,summary=r.stdout.splitlines()[-1] if r.stdout else r.stderr[-300:]))
print(json.dumps(results,ensure_ascii=False,indent=2))
raise SystemExit(0 if all(item['killed'] for item in results) else 1)
