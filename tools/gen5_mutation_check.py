#!/usr/bin/env python3
"""在临时副本撤掉原地转向补偿，确认新增回归会失败；不修改工作区或车端。"""
from pathlib import Path
import argparse,json,shutil,subprocess,sys,tempfile

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 root=Path(__file__).resolve().parents[1]
 with tempfile.TemporaryDirectory(prefix='gen5-pivot-mutation-') as tmp:
  tmp=Path(tmp)
  for tree in ('radar_system','robot_core'):
   shutil.copytree(root/tree,tmp/tree,ignore=shutil.ignore_patterns('__pycache__','models','static','templates'))
  (tmp/'tools').mkdir();(tmp/'tests').mkdir()
  for name in ('tools/gen5_turn_eval.py','tests/gen5_line_sim.py','tests/test_pivot_delay.py'):
   shutil.copy2(root/name,tmp/name)
  path=tmp/'radar_system/path_tracker.py';s=path.read_text()
  line='            yaw += rate * self.cfg.steer_gain * max(0.0, end - max(start, stamp))'
  assert line in s
  path.write_text(s.replace(line,'            yaw += 0.0  # 变异：撤掉在途指令对车头的预估'))
  r=subprocess.run([sys.executable,'-m','pytest','tests/test_pivot_delay.py','-q','-k','settles_once'],cwd=tmp,capture_output=True,text=True,timeout=45)
  a.out.write_text(r.stdout+r.stderr)
  assert r.returncode==1 and '4 failed' in r.stdout,(r.returncode,r.stdout,r.stderr)
  print('变异检查通过：去掉预估后，4 个标称到位测试全部失败。')
if __name__=='__main__':main()
