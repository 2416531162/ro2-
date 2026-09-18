#!/usr/bin/env python3
"""Versioned full-stack staging, verification and explicit systemd activation.

stage/verify operate without root or hardware. activate/rollback run on the board,
stop the stack before switching the release symlink, and only start passive tasks.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TREES = ('robot_core', 'radar_system', 'wheeltec_protocol', 'deployment', 'docs')
COMPONENTS = ('camera', 'lidar', 'ai', 'web', 'gui', 'follower')
PROFILES = {'perception': ('camera', 'lidar', 'ai', 'web'),
            'follow': COMPONENTS, 'headless': ('camera', 'lidar', 'ai', 'web', 'follower')}


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


def stage(source, destination):
    if any(destination.resolve().is_relative_to((source/tree).resolve()) for tree in TREES):
        raise ValueError('destination must be outside source component trees')
    if destination.exists():
        raise ValueError('destination must not exist')
    destination.mkdir(parents=True)
    files = {}
    for tree in TREES:
        for path in sorted((source/tree).rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts or path.suffix == '.pyc' or path.name == '.DS_Store':
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
    (destination/'manifest.json').write_text(json.dumps(dict(release_id=release_id, files=files), indent=2)+'\n')
    verify(destination)
    return release_id


def units(base):
    # Fixed installation paths avoid systemd/shell quoting surprises.
    runner = str(base/'current/deployment/run_component.sh')
    common = 'After=network.target\n\n[Service]\nType=simple\nEnvironment=ROS_DISTRO=jazzy\nEnvironmentFile=-/etc/rk3588/runtime.env\n'
    common += 'User=root\nWorkingDirectory='+str(base/'current')+'\nLogsDirectory=rk3588\n'
    common += 'Environment=HOME=/root\nEnvironment=ROS_LOG_DIR=/var/log/rk3588\n'
    common += 'KillMode=control-group\nTimeoutStopSec=10\nRestart=on-failure\nRestartSec=2\n'
    end = '\n[Install]\nWantedBy=multi-user.target\n'
    return {
        'rk3588-wheeltec.service': '[Unit]\nDescription=RK3588 chassis with motion authority\n'+common+f'ExecStart=/bin/bash {runner} chassis\n'+end,
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


def activate(release, base, unit_dir, profile):
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
        for name, content in units(base).items():
            (unit_dir/name).write_text(content)
        systemctl('daemon-reload')
        systemctl('start', 'rk3588-wheeltec.service', 'rk3588-perception.target',
                  *('rk3588-perception@'+c+'.service' for c in PROFILES[profile]))
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


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    s = sub.add_parser('stage'); s.add_argument('destination', type=Path); s.add_argument('--source', type=Path, default=ROOT)
    v = sub.add_parser('verify'); v.add_argument('release', type=Path)
    a = sub.add_parser('activate'); a.add_argument('release', type=Path); a.add_argument('--profile', choices=PROFILES, default='headless')
    sub.add_parser('rollback')
    args = p.parse_args()
    if args.action == 'stage':
        print(stage(args.source.resolve(), args.destination.resolve()))
    elif args.action == 'verify':
        print(verify(args.release.resolve())['release_id'])
    else:
        if sys.platform != 'linux' or os.geteuid() != 0:
            p.error('activate/rollback require root on the Linux board')
        if args.action == 'activate':
            activate(args.release.resolve(), Path('/opt/rk3588'), Path('/etc/systemd/system'), args.profile)
        else:
            rollback(Path('/opt/rk3588'), Path('/etc/systemd/system'))


if __name__ == '__main__':
    main()
