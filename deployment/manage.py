#!/usr/bin/env python3
"""Versioned full-stack staging, verification and explicit systemd activation.

stage/verify operate without root or hardware. activate/rollback run on the board,
stop the stack before switching the release symlink, and only start passive tasks.

发布包从 git 提交打（stage --git REV）：只含那个提交里的文件，清单记下 source_commit，车上跑的代码
随时能对回仓库（info 查）。不带 --git 从工作区打包只为兼容旧做法，会提示没有对应提交。
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TREES = ('robot_core', 'radar_system', 'wheeltec_protocol', 'oid_protocol', 'rion_protocol', 'deployment', 'docs')
# rtk：接收机、发状态；line：融合 + 作业（路线、定点画线、场地），只有操作员在 app 上点「开始」才申请 NAVIGATION；
# imu：Gen5 是 HDA536T 驱动（发融合要的 /imu 和姿态 /imu/data），旧车是 imu_filter_madgwick；
# jobrec：作业自动记录（radar_system/job_recorder.py），点开始作业就把原始数据存到 ~/rtk/jobs，排查问题用；
# web：网页 + 平板 app 的 WebSocket。底盘驱动（rk3588-wheeltec）另起。
COMPONENTS = ('web', 'rtk', 'line', 'imu', 'jobrec')
PROFILES = {'rtk': COMPONENTS}
DEFAULT_PROFILE = 'rtk'
# 服务用哪个账号跑：activate 默认用执行 sudo 的账号（Jetson 车上就是 wheeltec，正点原子 RK3588 上是板子的登录账号）
DEFAULT_USER = 'wheeltec'
# Gen5：两台欧艾迪驱动器和 HDA536T IMU 挂在同一条 CAN（RK3588 板载 CAN1，系统里叫 can0）上，统一 500K
CAN_BITRATE = 500000


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(release):
    manifest = json.loads((release / 'manifest.json').read_text())
    expected_id = hashlib.sha256(json.dumps(manifest['files'], sort_keys=True).encode()).hexdigest()[:16]
    if manifest.get('release_id') != expected_id:
        raise ValueError('manifest identity mismatch')
    actual = {str(p.relative_to(release)) for tree in TREES for p in (release/tree).rglob('*')
              if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    if (release/'README.md').is_file():
        actual.add('README.md')
    if actual != set(manifest['files']):
        raise ValueError('release file set mismatch')
    for name, digest in manifest['files'].items():
        p = release / name
        if not p.resolve().is_relative_to(release.resolve()) or not p.is_file() or sha(p) != digest:
            raise ValueError('release verification failed: ' + name)
        if p.suffix == '.py':
            compile(p.read_text(), str(p), 'exec')
    # Validate the staged profile with the staged validator, not host cached state.
    env = dict(os.environ, PYTHONPATH=str(release), RK3588_ROBOT_CONFIG=str(release/'robot_core/robot.json'))
    subprocess.run([sys.executable, '-c', 'from robot_core.config import PROFILE'],
                   cwd=release, env=env, check=True)
    return manifest


def git_export(repo, rev, workdir):
    """把 git 提交 rev 里发布要的目录（TREES 和 README.md）导出到 workdir，只含已提交的内容；返回完整提交号。"""
    def git(*args, **kw):
        return subprocess.run(['git', '-C', str(repo), *args], check=True, **kw)
    commit = git('rev-parse', '--verify', rev + '^{commit}', capture_output=True, text=True).stdout.strip()
    paths = [p for p in (*TREES, 'README.md')
             if subprocess.run(['git', '-C', str(repo), 'cat-file', '-e', commit + ':' + p],
                               capture_output=True).returncode == 0]
    archive = git('archive', '--format=tar', commit, *paths, capture_output=True).stdout
    subprocess.run(['tar', '-x', '-C', str(workdir)], input=archive, check=True)
    return commit


def stage_commit(repo, rev, destination):
    """从 git 提交打发布包：(release_id, 完整提交号)。"""
    with tempfile.TemporaryDirectory() as tmp:
        commit = git_export(repo, rev, Path(tmp))
        return stage(Path(tmp), destination, source_commit=commit), commit


def stage(source, destination, source_commit=None):
    if any(destination.resolve().is_relative_to((source/tree).resolve()) for tree in TREES):
        raise ValueError('destination must be outside source component trees')
    if destination.exists():
        raise ValueError('destination must not exist')
    destination.mkdir(parents=True)
    files = {}
    for tree in TREES:
        for path in sorted((source/tree).rglob('*')):
            archived = any(re.fullmatch(r'.*_[0-9]{8}', part)
                           for part in path.relative_to(source).parts[:-1])
            if (not path.is_file() or archived or '__pycache__' in path.parts
                    or path.suffix == '.pyc' or path.name == '.DS_Store'):
                continue
            if path.is_symlink():
                raise ValueError('release files must not be symlinks: ' + str(path))
            rel = path.relative_to(source)
            out = destination / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, out)
            files[str(rel)] = sha(out)
    if (source/'README.md').is_file():
        shutil.copy2(source/'README.md', destination/'README.md')
        files['README.md'] = sha(destination/'README.md')
    release_id = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()[:16]
    manifest = dict(release_id=release_id, files=files)
    if source_commit:
        manifest['source_commit'] = source_commit      # 不参与 release_id：同样的文件就是同一个发布
    (destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    verify(destination)
    return release_id


def units(base, profile=DEFAULT_PROFILE, user=DEFAULT_USER, group=None, home=None):
    # Fixed installation paths avoid systemd/shell quoting surprises.
    runner = str(base/'current/deployment/run_component.sh')
    group, home = group or user, home or '/home/'+user
    for value in (user, group, home):
        if not re.fullmatch(r'[A-Za-z0-9_./-]+', value):
            raise ValueError('account name or home has unsupported characters: %r' % value)
    common = 'After=network.target rk3588-can.service\n\n[Service]\nType=simple\n'
    # dialout：RTK 接收机的 USB 串口；CAN 原始套接字不需要额外权限
    common += 'User='+user+'\nGroup='+group+'\nSupplementaryGroups=dialout\n'
    common += 'WorkingDirectory='+str(base/'current')+'\nLogsDirectory=rk3588\n'
    common += 'Environment=HOME='+home+'\nEnvironment=USER='+user+'\nEnvironment=ROS_LOG_DIR=/var/log/rk3588\n'
    common += 'KillMode=control-group\nTimeoutStopSec=10\nRestart=on-failure\nRestartSec=2\n'
    # Participating processes arm systemd's watchdog themselves; legacy third-party
    # components (e.g. imu_filter_madgwick) retain their existing restart policy.
    common += 'NotifyAccess=main\nEnvironment=RK3588_WATCHDOG_SEC=20\nTimeoutAbortSec=3\n'
    end = '\n[Install]\nWantedBy=multi-user.target\n'
    # 车上没有雷达：底盘驱动不设这个时按装了雷达算，待命、急停复位会卡在等雷达数据
    lidar = 'Environment=RK3588_LIDAR_INSTALLED=0\n'
    # CAN 口开机起成 500K；restart-ms：总线出错（bus-off）后 0.1 s 自动恢复，不然要人重起接口。
    # 没有 can0 的车（旧车 Wheeltec 走串口）条件不满足，直接跳过，不算失败
    can = ('[Unit]\nDescription=RK3588 CAN bus for Gen5 drivers and IMU\nConditionPathExists=/sys/class/net/can0\n'
           'Before=rk3588-wheeltec.service rk3588-perception.target\n\n[Service]\nType=oneshot\nRemainAfterExit=yes\n'
           'ExecStartPre=-/usr/sbin/ip link set can0 down\n'
           'ExecStart=/usr/sbin/ip link set can0 type can bitrate %d restart-ms 100\n'
           'ExecStart=/usr/sbin/ip link set can0 up\n' % CAN_BITRATE) + end
    return {
        'rk3588-can.service': can,
        'rk3588-wheeltec.service': '[Unit]\nDescription=RK3588 chassis with motion authority\n'+common+lidar+f'ExecStart=/bin/bash {runner} chassis\n'+end,
        'rk3588-perception@.service': '[Unit]\nDescription=RK3588 perception %i\nPartOf=rk3588-perception.target\n'+common+f'ExecStart=/bin/bash {runner} %i\n'+end,
        'rk3588-perception.target': '[Unit]\nDescription=RK3588 perception services\n'+end,
    }


def systemctl(*args):
    subprocess.run(['systemctl', *args], check=True)


def stop_stack():
    names = ['rk3588-perception.target', *('rk3588-perception@'+c+'.service' for c in COMPONENTS),
             'rk3588-wheeltec.service']
    installed = []
    for name in names:
        result = subprocess.run(['systemctl', 'show', '--value', '-p', 'LoadState', name],
                                check=True, capture_output=True, text=True)
        if result.stdout.strip() != 'not-found':
            installed.append(name)
    if installed:
        systemctl('stop', *installed)


def switch_link(base, release):
    pending = base / 'current.pending'
    if pending.is_symlink():
        pending.unlink()
    pending.symlink_to(release)
    pending.replace(base/'current')


def activate(release, base, unit_dir, profile, account=(DEFAULT_USER, None, None)):
    verify(release)
    if (base/'current').exists() and not (base/'current').is_symlink():
        raise ValueError('current must be a managed symlink')
    if ' ' in str(base) or '\n' in str(base):
        raise ValueError('install root must not contain whitespace')
    base.mkdir(parents=True, exist_ok=True)
    previous = str((base/'current').resolve()) if (base/'current').exists() else None
    backup = dict(previous=previous, units={name:(unit_dir/name).read_text() if (unit_dir/name).exists() else None
                                         for name in units(base)})
    # Store restore information before stopping/modifying the installed stack.
    (base/'rollback.json').write_text(json.dumps(backup, indent=2)+'\n')
    stop_stack()
    try:
        switch_link(base, release.resolve())
        unit_dir.mkdir(parents=True, exist_ok=True)
        for name, content in units(base, profile, *account).items():
            (unit_dir/name).write_text(content)
        systemctl('daemon-reload')
        systemctl('start', 'rk3588-can.service', 'rk3588-wheeltec.service', 'rk3588-perception.target',
                  *('rk3588-perception@'+c+'.service' for c in PROFILES[profile]))
        # 开机自启与刚启用的配置一致：只 start 不 enable 时，断电重启后新加的组件（rtk、line）不会起来
        systemctl('enable', 'rk3588-can.service', 'rk3588-wheeltec.service', 'rk3588-perception.target',
                  *('rk3588-perception@'+c+'.service' for c in PROFILES[profile]))
        others = [c for c in COMPONENTS if c not in PROFILES[profile]]
        if others:
            systemctl('disable', *('rk3588-perception@'+c+'.service' for c in others))
    except Exception:
        rollback(base, unit_dir)
        raise


def rollback(base, unit_dir):
    saved = json.loads((base/'rollback.json').read_text())
    stop_stack()
    if saved['previous']:
        switch_link(base, Path(saved['previous']))
    elif (base/'current').is_symlink():
        (base/'current').unlink()
    for name, content in saved['units'].items():
        path = unit_dir/name
        if content is None:
            if path.exists():
                path.unlink()
        else:
            path.write_text(content)
    systemctl('daemon-reload')
    # Leave stopped: an older follower might auto-select motion on startup.


def service_account(user):
    """账号 -> (账号, 主组, 家目录)；板子上没有这个账号就报错，不写出起不来的服务。"""
    import grp
    import pwd
    try:
        pw = pwd.getpwnam(user)
    except KeyError:
        raise SystemExit('板子上没有账号 %s：用 --user 指定服务用哪个账号跑' % user)
    return user, grp.getgrgid(pw.pw_gid).gr_name, pw.pw_dir


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    s = sub.add_parser('stage'); s.add_argument('destination', type=Path); s.add_argument('--source', type=Path, default=ROOT)
    s.add_argument('--git', metavar='REV', help='从这个 git 提交打包（如 HEAD），只含已提交的内容，清单记下提交号')
    v = sub.add_parser('verify'); v.add_argument('release', type=Path)
    i = sub.add_parser('info', help='发布包的 release_id 和来自哪个 git 提交'); i.add_argument('release', type=Path)
    a = sub.add_parser('activate'); a.add_argument('release', type=Path); a.add_argument('--profile', choices=PROFILES, default=DEFAULT_PROFILE)
    a.add_argument('--user', help='服务用哪个账号跑（默认执行 sudo 的账号；没有 sudo 时 %s）' % DEFAULT_USER)
    sub.add_parser('rollback')
    args = p.parse_args()
    if args.action == 'stage':
        if args.git:
            release_id, commit = stage_commit(args.source.resolve(), args.git, args.destination.resolve())
            print(release_id)
            print('source_commit', commit, file=sys.stderr)
        else:
            print('注意：从工作区打包，发布包没有对应的 git 提交（建议先提交，再用 --git HEAD）', file=sys.stderr)
            print(stage(args.source.resolve(), args.destination.resolve()))
    elif args.action == 'verify':
        print(verify(args.release.resolve())['release_id'])
    elif args.action == 'info':
        manifest = verify(args.release.resolve())
        print(manifest['release_id'], manifest.get('source_commit') or '无（从工作区打包，没有对应提交）')
    else:
        if sys.platform != 'linux' or os.geteuid() != 0:
            p.error('activate/rollback require root on the Linux board')
        if args.action == 'activate':
            activate(args.release.resolve(), Path('/opt/rk3588'), Path('/etc/systemd/system'), args.profile,
                     service_account(args.user or os.environ.get('SUDO_USER') or DEFAULT_USER))
        else:
            rollback(Path('/opt/rk3588'), Path('/etc/systemd/system'))


if __name__ == '__main__':
    main()
