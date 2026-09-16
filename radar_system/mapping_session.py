"""Own only the mapping subprocesses started here. Never arm or move a chassis."""
import os
import re
import signal
import subprocess
import threading
from pathlib import Path


class MappingSession:
    def __init__(self,root=None):
        self.root=Path(root or Path(__file__).resolve().parent)
        self.maps=Path(os.environ.get('RO2_MAP_DIR',str(self.root/'maps'))).resolve()
        self.maps.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()
        self.proc=None
        self.mode='idle'
        self.log=None

    def name(self,name):
        if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',name):
            raise ValueError('地图名称仅限 1–64 位英文字母、数字、下划线或横线')
        return name

    def map_file(self,name):
        path=self.maps/(self.name(name)+'.yaml')
        if path.is_symlink() or path.resolve().parent!=self.maps:
            raise ValueError('拒绝符号链接地图')
        return path

    def list_maps(self):
        return [p.stem for p in sorted(self.maps.glob('*.yaml'))
                if not p.is_symlink() and re.fullmatch(r'[A-Za-z0-9_-]{1,64}',p.stem)]

    def status(self):
        with self.lock:
            active=self.proc is not None and self.proc.poll() is None
            return dict(mode=self.mode if active else 'idle',running=active,maps=self.list_maps(),
                        exit_code=self.proc.poll() if self.proc else None,
                        log=str(self.maps/'mapping.log'))

    def start(self,mode,name=''):
        if mode not in ('mapping','localization'):
            raise ValueError('未知运行模式')
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                raise ValueError('请先停止当前建图/定位，再切换模式')
            # Reject duplicate TF/map owners rather than killing outside processes.
            result=subprocess.run(['ros2','node','list'],capture_output=True,text=True,timeout=5,check=True)
            if any(token in result.stdout for token in ('slam_toolbox','/amcl','fake_slam')):
                raise ValueError('已有外部 SLAM/AMCL/模拟建图节点，请先停止，避免重复 TF')
            cmd=['bash',str(self.root/'run_mapping.sh'),mode]
            if mode=='localization':
                path=self.map_file(name)
                if not path.is_file():
                    raise ValueError('地图不存在')
                cmd.append(str(path))
            if self.log:
                self.log.close()
            self.log=open(self.maps/'mapping.log','ab',buffering=0)
            self.proc=subprocess.Popen(cmd,cwd=self.root,stdout=self.log,stderr=subprocess.STDOUT,
                                       start_new_session=True)
            self.mode=mode
            return dict(ok=True,message='进程已启动；地图与定位就绪状态请看实时视图',**self.status())

    def stop(self):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                try:
                    os.killpg(self.proc.pid,signal.SIGINT)
                    self.proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(self.proc.pid,signal.SIGTERM)
                    try:
                        self.proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.proc.pid,signal.SIGKILL)
                        self.proc.wait(timeout=2)
                except ProcessLookupError:
                    pass
            if self.log:
                self.log.close()
                self.log=None
            self.mode='idle'
            return dict(ok=True,message='本界面启动的建图/定位已停止；这不是底盘急停')

    def save(self,name):
        with self.lock:
            path=self.map_file(name)
            prefix=path.with_suffix('')
            if any(prefix.with_suffix(ext).exists() for ext in ('.yaml','.pgm','.png')):
                raise ValueError('同名地图已存在，请使用新名称，避免覆盖')
            # Save the FULL /map, not the downsampled display PNG. No shell interpolation.
            result=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli',
                                   '-f',str(prefix),'--ros-args','-p','map_subscribe_transient_local:=true',
                                   '-p','save_map_timeout:=5.0'],capture_output=True,text=True,timeout=15)
            if result.returncode or not path.is_file():
                raise ValueError('地图保存失败: '+(result.stderr or result.stdout)[-1200:])
            return dict(ok=True,message='已保存完整导航地图（不是缩略图）',map=name,path=str(path))
