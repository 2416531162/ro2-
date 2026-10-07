"""隔离模拟喷水进程：禁用设备、网络、ROS 和任何子进程，不连接实车。"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import resource


def install_guard(directory):
    directory=Path(directory).resolve()
    def audit(event,args):
        if event in ('socket.__new__','subprocess.Popen','os.system','os.posix_spawn','os.posix_spawnp'):
            raise PermissionError('模拟喷水禁止网络、硬件和外部命令')
        if event=='import' and str(args[0]).split('.')[0] in {
                'rclpy','pump_control','pump_gpio','pump_worker','hc08_pump','hc08_pump_worker',
                'oid_driver','wheeltec_driver','can','serial','dbus'}:
            raise PermissionError('模拟喷水禁止导入硬件或 ROS 接口')
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            path=Path(os.fsdecode(args[0])).resolve()
            if str(path).startswith(('/dev/','/sys/')):
                raise PermissionError('模拟喷水禁止访问设备')
            flags=args[2] if len(args)>2 and isinstance(args[2],int) else 0
            if flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC) and not path.is_relative_to(directory):
                raise PermissionError('模拟喷水只允许写入本次模拟目录')
        if event in ('os.remove','os.rmdir','os.mkdir','os.rename','os.symlink','os.link','os.chmod','os.chown','os.truncate'):
            paths=args[:2] if event in ('os.rename','os.symlink','os.link') else args[:1]
            for value in paths:
                if isinstance(value,(str,bytes,os.PathLike)) and not Path(os.fsdecode(value)).resolve().is_relative_to(directory):
                    raise PermissionError('模拟喷水禁止修改本次模拟目录之外的文件')
    sys.addaudithook(audit)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',required=True)
    args=parser.parse_args();directory=Path(args.directory).resolve()
    source=Path(__file__).resolve().parents[1]
    temporary=directory/'tmp';temporary.mkdir(exist_ok=True)
    tempfile.tempdir=str(temporary)
    os.environ.update(HOME=str(directory),RK3588_PUMP_ENABLED='0',
                      RK3588_ROBOT_CONFIG=str(source/'robot_core/robot_gen5.json'))
    sys.dont_write_bytecode=True
    resource.setrlimit(resource.RLIMIT_CPU,(120,125))
    if sys.platform.startswith("linux"):
        resource.setrlimit(resource.RLIMIT_AS,(768*1024*1024,768*1024*1024))
    install_guard(directory)
    try:
        os.nice(10)
    except OSError:pass
    from spray_sim_runner import run
    request=json.loads((directory/'request.json').read_text())
    def save(name,value):
        path=directory/name;temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False));temp.replace(path)
    result=run(request,source,lambda value:save('progress.json',value),lambda value:save('live.json',value))
    save('result.json',result)


if __name__=='__main__':main()
