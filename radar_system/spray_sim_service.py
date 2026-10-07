"""模拟喷水任务管理；只启动受限制的纯数值进程，不使用运动或水泵控制接口。"""
import hashlib
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid


class SpraySimulations:
    def __init__(self,root=None,timeout=180):
        self.root=Path(root or Path.home()/'rtk/spray-simulations')
        self.timeout=timeout;self.lock=threading.RLock();self.process=None;self.current=None
        self.closed=False

    def start(self,request):
        from spray_sim_runner import normalize
        value=normalize(request)
        if value['settings']['plan_only']:raise ValueError('模拟喷水需要完整运行，不能只规划')
        if value['settings']['seeds']>3 or value['settings']['max_seconds']>3600:
            raise ValueError('板子模拟最多 3 个种子、3600 秒虚拟时间')
        key=hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
        with self.lock:
            if self.closed:raise ValueError('模拟服务已退出')
            if self.current and self.current['key']==key and self.current['state'] not in ('failed','cancelled'):
                return self.view()
            if self.process and self.process.poll() is None:self.process.terminate()
            self.root.mkdir(parents=True,exist_ok=True)
            # 仅保留最近 20 次模拟，保护板子存储；正在退出的任务至少留 5 分钟。
            old=sorted((p for p in self.root.iterdir() if p.is_dir() and len(p.name)==32),key=lambda p:p.stat().st_mtime,reverse=True)
            for path in old[19:]:
                if time.time()-path.stat().st_mtime>300:shutil.rmtree(path)
            ident=uuid.uuid4().hex;directory=self.root/ident;directory.mkdir(parents=True)
            (directory/'request.json').write_text(json.dumps(value,ensure_ascii=False))
            self.current=dict(id=ident,key=key,state='running',created=time.time(),name='模拟喷水',
                simulation_only=True,hardware_output=False,error=None)
            self._save(directory)
            threading.Thread(target=self._execute,args=(ident,directory),daemon=True).start()
            return dict(self.current)

    def _save(self,directory):
        temp=directory/'meta.tmp';temp.write_text(json.dumps(self.current,ensure_ascii=False));temp.replace(directory/'meta.json')

    def _execute(self,ident,directory):
        log=None;process=None
        try:
            with self.lock:
                if self.closed or self.current['id']!=ident:return
                log=(directory/'run.log').open('w')
                self.process=subprocess.Popen([sys.executable,'-B',str(Path(__file__).with_name('spray_sim_worker.py')),
                    '--directory',str(directory)],cwd=directory,stdout=log,stderr=log,
                    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','PYTHONHASHSEED':'0'})
                process=self.process
            process.wait(timeout=self.timeout)
            error=None
            if process.returncode or not (directory/'result.json').is_file():
                error='\n'.join((directory/'run.log').read_text().splitlines()[-4:])[-800:] or '模拟进程未生成结果'
            with self.lock:
                if self.current['id']!=ident or self.current['state']=='cancelled':return
                self.current.update(state='failed' if error else 'done',error=error,finished=time.time())
                self._save(directory)
        except (subprocess.TimeoutExpired,OSError) as exc:
            if process and process.poll() is None:process.kill();process.wait()
            with self.lock:
                if self.current['id']==ident and self.current['state']!='cancelled':
                    self.current.update(state='failed',error='模拟计算超过时间上限，请缩小图纸范围' if isinstance(exc,subprocess.TimeoutExpired) else str(exc))
                    self._save(directory)
        finally:
            if log:log.close()

    def view(self):
        with self.lock:
            if not self.current:return dict(state='idle',simulation_only=True,hardware_output=False)
            value=dict(self.current);directory=self.root/value['id']
        for name,key in (('progress.json','progress'),('live.json','live'),('result.json','result')):
            path=directory/name
            if path.is_file():
                try:value[key]=json.loads(path.read_text())
                except (ValueError,OSError):pass
        return value

    def cancel(self,ident):
        with self.lock:
            if not self.current or self.current['id']!=ident:raise ValueError('模拟编号已改变')
            if self.process and self.process.poll() is None:self.process.terminate()
            self.current['state']='cancelled';self._save(self.root/ident)

    def close(self):
        with self.lock:
            self.closed=True
            if self.process and self.process.poll() is None:self.process.terminate()


def handle(handler,simulations):
    path=handler.path.split('?')[0]
    if path!='/app/spray-simulation':return False
    if handler.headers.get('Origin'):
        handler._send_json(dict(error='不接受网页跨源请求'),status=403);return True
    try:
        if handler.command=='GET':value=simulations.view()
        elif handler.command=='POST':
            size=int(handler.headers.get('Content-Length',0))
            if not 0<size<=300000:raise ValueError('模拟图纸数据大小不对')
            request=json.loads(handler.rfile.read(size))
            if not isinstance(request,dict):raise ValueError('模拟请求须为对象')
            if request.get('action')=='cancel':simulations.cancel(request.get('id'));value=simulations.view()
            else:value=simulations.start(request)
        else:handler._send_json(dict(error='不支持的模拟方法'),status=405);return True
        handler._send_json(value)
    except (ValueError,TypeError,OSError) as exc:handler._send_json(dict(error=str(exc)),status=400)
    return True
