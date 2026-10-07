#!/usr/bin/env python3
"""车端升级包（.rkup）：电脑打包签名 → 平板 app 传给车 → 车验签、放进 releases/ → 车空闲时由独立的 systemd 临时单元激活。

.rkup 是一个 zip，正好三个文件：
    info.json        给 app 显示：release_id、source_commit、created、bytes（不参与校验，车以包里的清单为准）
    release.tar.gz   发布目录（顶层目录名 = release_id，内容同 manage.py stage --git 打出来的）
    release.sig      对 release.tar.gz 的 Ed25519 签名（64 字节）
车只认「当前在跑的版本」里 deployment/upgrade_pub.pem 这把公钥签的包。私钥只在打包的电脑上（不进仓库），
连上车热点的人传不进别的代码。换钥匙也走签名包：新版本带新公钥，从下一次升级起生效。

激活要停掉整套服务（包括收包的网页服务），所以由 systemd-run 起一个独立的临时单元 rk3588-upgrade 来做（apply）：
smoke（单独 ROS 域里把节点构造一遍，不过就不动现在的版本）→ manage.activate → 服务全部 active、网页能应答并稳住
STABLE_S 秒才算成功；HEALTH_S 秒内做不到就 manage.rollback 再把原版本启动起来。进度写在 /opt/rk3588/upgrade/status.json，
新旧网页服务都能读给 app。

电脑上：
    python3 deployment/upgrade.py keygen                         # 一次：私钥 ~/.config/rk3588/upgrade_ed25519.pem，公钥写进仓库
    python3 deployment/upgrade.py package --git HEAD 输出.rkup    # 从提交打包、签名
车上（网页服务调用，也可以手动 sudo）：
    upgrade.py unpack <包> | apply <发布目录> | status
"""
import argparse
import datetime
import gzip
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.request
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manage    # noqa: E402

BASE = Path('/opt/rk3588')
UNIT_DIR = Path('/etc/systemd/system')
PUBKEY = 'deployment/upgrade_pub.pem'
DEFAULT_KEY = Path('~/.config/rk3588/upgrade_ed25519.pem').expanduser()
MEMBERS = ('info.json', 'release.sig', 'release.tar.gz')
MAX_PACKAGE = 64 << 20          # 发布包压缩后约 2–5 MB
MAX_UNPACKED = 256 << 20
UNIT = 'rk3588-upgrade'
WEB_URL = 'http://127.0.0.1:8088/app/beacon'
HEALTH_S = 60.0                 # 激活后最多等这么久
STABLE_S = 10.0                 # 服务全部 active、网页能应答要连续这么久（崩溃重启的节点 2 s 就会掉出 active）
SMOKE_TIMEOUT_S = 150.0
RELEASE_ID = re.compile(r'^[0-9a-f]{16}$')


# ---- 签名 ----

def keygen(private=DEFAULT_KEY, public=None):
    """生成 Ed25519 钥匙对；私钥已存在就不动（换钥匙要手动删，免得把车上认的那把弄丢）。"""
    private = Path(private)
    public = Path(public) if public else Path(__file__).resolve().parents[1] / PUBKEY
    if private.exists():
        raise ValueError('私钥已存在：%s（不覆盖）' % private)
    private.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(private.parent, 0o700)
    subprocess.run(['openssl', 'genpkey', '-algorithm', 'ed25519', '-out', str(private)], check=True, capture_output=True)
    os.chmod(private, 0o600)
    subprocess.run(['openssl', 'pkey', '-in', str(private), '-pubout', '-out', str(public)], check=True, capture_output=True)
    return public


def sign(path, key=DEFAULT_KEY):
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / 'sig'
        subprocess.run(['openssl', 'pkeyutl', '-sign', '-inkey', str(key), '-rawin', '-in', str(path), '-out', str(out)],
                       check=True, capture_output=True)
        sig = out.read_bytes()
    if len(sig) != 64:
        raise ValueError('签名长度不对（私钥不是 Ed25519？）')
    return sig


def signature_ok(data, sig, pubkey, use_openssl=False):
    """data 的 Ed25519 签名对不对。优先用 python cryptography，没装就用 openssl 命令（Ubuntu 24.04 自带 3.0）。"""
    if len(sig) != 64:
        return False
    if not use_openssl:
        try:
            from cryptography.exceptions import InvalidSignature
            from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
            from cryptography.hazmat.primitives.serialization import load_pem_public_key
        except ImportError:
            pass
        else:
            key = load_pem_public_key(Path(pubkey).read_bytes())
            if not isinstance(key, Ed25519PublicKey):
                raise ValueError('升级公钥不是 Ed25519')
            try:
                key.verify(sig, data)
                return True
            except InvalidSignature:
                return False
    with tempfile.TemporaryDirectory() as tmp:
        d, s = Path(tmp) / 'data', Path(tmp) / 'sig'
        d.write_bytes(data)
        s.write_bytes(sig)
        r = subprocess.run(['openssl', 'pkeyutl', '-verify', '-pubin', '-inkey', str(pubkey), '-rawin',
                            '-in', str(d), '-sigfile', str(s)], capture_output=True)
        return r.returncode == 0


# ---- 打包（电脑上）----

def _tar_release(release, out):
    """确定性的 tar.gz：顶层目录 = 发布目录名，按名排序，时间、属主清零，权限只留 755 / 644。"""
    def norm(ti):
        ti.uid = ti.gid = 0
        ti.uname = ti.gname = ''
        ti.mtime = 0
        ti.mode = 0o755 if ti.isdir() or ti.mode & 0o111 else 0o644
        return ti
    with open(out, 'wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0) as gz, \
            tarfile.open(fileobj=gz, mode='w', format=tarfile.PAX_FORMAT) as tar:
        tar.add(release, arcname=release.name, recursive=False, filter=norm)
        for p in sorted(release.rglob('*')):
            tar.add(p, arcname=str(Path(release.name) / p.relative_to(release)), recursive=False, filter=norm)


def package_release(release, out, key=DEFAULT_KEY, pubkey=None, source_commit=None, note=''):
    """把已经 stage 好的发布目录（目录名 = release_id）打成签名的 .rkup；返回 info。"""
    release, out = Path(release), Path(out)
    manifest = manage.verify(release)
    if release.name != manifest['release_id']:
        raise ValueError('发布目录名必须是 release_id')
    pubkey = Path(pubkey) if pubkey else release / PUBKEY
    with tempfile.TemporaryDirectory() as tmp:
        tgz = Path(tmp) / 'release.tar.gz'
        _tar_release(release, tgz)
        data = tgz.read_bytes()
        if len(data) > MAX_PACKAGE:
            raise ValueError('发布包太大')
        sig = sign(tgz, key)
        # 自检：私钥和车将来用来验的公钥必须是一对，不然打出来的包车不认
        if not signature_ok(data, sig, pubkey):
            raise ValueError('私钥和公钥 %s 不是一对' % pubkey)
        info = dict(format=1, release_id=manifest['release_id'],
                    source_commit=source_commit or manifest.get('source_commit'),
                    created=datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
                    bytes=len(data), note=note)
        tmp_out = out.with_name(out.name + '.part')
        with zipfile.ZipFile(tmp_out, 'w', zipfile.ZIP_STORED) as z:
            z.writestr('info.json', json.dumps(info, ensure_ascii=False, indent=2) + '\n')
            z.writestr('release.sig', sig)
            z.write(tgz, 'release.tar.gz')
        tmp_out.replace(out)
    return info


def package(repo, rev, out, key=DEFAULT_KEY, note=''):
    """从 git 提交 rev 打签名升级包。发布里必须带公钥（车靠它验下一次升级）。"""
    out = Path(out)
    if out.suffix != '.rkup':
        raise ValueError('升级包文件名要以 .rkup 结尾')
    with tempfile.TemporaryDirectory() as tmp:
        staged = Path(tmp) / 'stage'
        release_id, commit = manage.stage_commit(Path(repo), rev, staged)
        release = staged.rename(Path(tmp) / release_id)
        if not (release / PUBKEY).is_file():
            raise ValueError('提交里没有 %s：先 keygen，把公钥提交了再打包' % PUBKEY)
        return package_release(release, out, key, source_commit=commit, note=note)


# ---- 收包（车上）----

def read_info(package):
    """只读 info.json（显示用，不可信）。"""
    with zipfile.ZipFile(package) as z:
        return json.loads(z.read('info.json').decode('utf-8'))


def _safe_extract(data, dest):
    """只解普通文件和目录，路径不能出 dest、顶层只能一个目录；返回顶层目录名。"""
    tops, total = set(), 0
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as tar:
        members = tar.getmembers()
        for m in members:
            p = Path(m.name)
            if p.is_absolute() or '..' in p.parts or not p.parts or not (m.isfile() or m.isdir()):
                raise ValueError('升级包里有不安全的条目：%s' % m.name)
            tops.add(p.parts[0])
            total += m.size
            if total > MAX_UNPACKED:
                raise ValueError('升级包解开太大')
        if len(tops) != 1:
            raise ValueError('升级包顶层必须正好一个发布目录')
        if hasattr(tarfile, 'data_filter'):
            tar.extractall(dest, members=members, filter='data')
        else:
            tar.extractall(dest, members=members)
    return tops.pop()


def unpack(package, releases, pubkey):
    """验签、解包、核对清单，放进 releases/<release_id>；返回 (发布目录, 清单)。不对就抛 ValueError，不留半成品。"""
    package, releases, pubkey = Path(package), Path(releases), Path(pubkey)
    if package.stat().st_size > MAX_PACKAGE + (1 << 20):
        raise ValueError('升级包太大')
    if not pubkey.is_file():
        raise ValueError('车上当前版本没有升级公钥，不能用 app 升级（先用 USB 部署一次）')
    try:
        with zipfile.ZipFile(package) as z:
            infos = {i.filename: i for i in z.infolist()}
            if sorted(infos) != sorted(MEMBERS):
                raise ValueError('升级包里的文件不对')
            if infos['release.sig'].file_size != 64 or infos['info.json'].file_size > 65536 \
                    or infos['release.tar.gz'].file_size > MAX_PACKAGE:
                raise ValueError('升级包里的文件大小不对')
            sig, data = z.read('release.sig'), z.read('release.tar.gz')
    except zipfile.BadZipFile:
        raise ValueError('不是升级包（zip 格式不对）')
    if not signature_ok(data, sig, pubkey):
        raise ValueError('签名不对：不是配对的私钥打的包，或者包被改过')
    releases.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix='.incoming-', dir=releases))
    try:
        try:
            top = _safe_extract(data, tmp)
        except (tarfile.TarError, OSError, EOFError) as exc:
            raise ValueError('升级包解不开：%s' % exc)
        try:
            manifest = manage.verify(tmp / top)
        except (ValueError, KeyError, OSError, SyntaxError, subprocess.CalledProcessError) as exc:
            raise ValueError('发布包校验不过：%s' % exc)
        if top != manifest['release_id'] or not RELEASE_ID.match(top):
            raise ValueError('发布目录名和清单对不上')
        dest = releases / top
        if dest.exists():
            try:
                manage.verify(dest)
                return dest, manifest        # 车上已经有完整的同一个发布
            except Exception:
                shutil.rmtree(dest)
        (tmp / top).rename(dest)
        return dest, manifest
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---- 状态 ----

def _manifest(release):
    try:
        return json.loads((Path(release) / 'manifest.json').read_text())
    except (OSError, ValueError):
        return {}


def current_release(base=BASE):
    link = Path(base) / 'current'
    return Path(os.path.realpath(link)) if link.exists() else None


def previous_release(base=BASE):
    try:
        prev = json.loads((Path(base) / 'rollback.json').read_text()).get('previous')
    except (OSError, ValueError):
        return None
    return Path(prev) if prev and Path(prev).is_dir() else None


def service_user(unit_dir=UNIT_DIR, default='root'):
    """现在的服务用哪个账号跑（升级后照旧）。"""
    try:
        text = (Path(unit_dir) / 'rk3588-perception@.service').read_text()
    except OSError:
        return default
    m = re.search(r'^User=([A-Za-z0-9_.-]+)$', text, re.M)
    return m.group(1) if m else default


def unit_running(name=UNIT):
    try:
        r = subprocess.run(['systemctl', 'is-active', name + '.service'], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.stdout.strip() in ('active', 'activating', 'reloading')


def read_status(base=BASE):
    try:
        return json.loads((Path(base) / 'upgrade' / 'status.json').read_text())
    except (OSError, ValueError):
        return None


def write_status(base, **fields):
    path = Path(base) / 'upgrade' / 'status.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    data = dict(read_status(base) or {}, **fields, updated=time.time())
    tmp = path.with_name('.status.json.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)
    return data


def describe(release):
    if release is None:
        return None
    m = _manifest(release)
    return dict(release_id=m.get('release_id') or Path(release).name, source_commit=m.get('source_commit'))


def status(base=BASE, running=None):
    base = Path(base)
    cur, prev = current_release(base), previous_release(base)
    rels = []
    for d in sorted((base / 'releases').glob('*'), key=lambda p: p.stat().st_mtime, reverse=True):
        if d.is_dir() and RELEASE_ID.match(d.name):
            rels.append(dict(describe(d), installed=round(d.stat().st_mtime)))
    return dict(current=describe(cur), previous=describe(prev), releases=rels[:12],
                supported=cur is not None and (cur / PUBKEY).is_file(),
                running=unit_running() if running is None else running, last=read_status(base))


# ---- 激活（车上，独立临时单元里跑）----

def launch(release, base=BASE, profile=manage.DEFAULT_PROFILE, user=None, runner=subprocess.run):
    """网页服务调：起临时单元 rk3588-upgrade 去 apply（网页服务自己会被停掉，不能在里面做）。
    用当前在跑、已经验证过的这份 upgrade.py 执行；同名单元还在跑时 systemd-run 会失败，等于互斥。"""
    cur = current_release(base)
    if cur is None:
        raise ValueError('车上没有当前版本')
    script = cur / 'deployment' / 'upgrade.py'
    cmd = ['systemd-run', '--unit', UNIT, '--collect', '--quiet', '--property', 'Type=exec',
           '--setenv', 'HOME=/root', sys.executable, str(script), 'apply', str(release),
           '--base', str(base), '--profile', profile, '--user', user or service_user()]
    r = runner(cmd, capture_output=True, text=True, timeout=15)
    if r.returncode != 0:
        raise RuntimeError((r.stderr or r.stdout or 'systemd-run 失败').strip()[-300:])
    return cmd


def _systemctl(*args, check=True):
    return subprocess.run(['systemctl', *args], check=check, capture_output=True, text=True)


def start_stack(profile=manage.DEFAULT_PROFILE):
    _systemctl('daemon-reload')
    _systemctl('start', 'rk3588-can.service', 'rk3588-wheeltec.service', 'rk3588-perception.target',
               *('rk3588-perception@' + c + '.service' for c in manage.PROFILES[profile]))


def stack_units(profile=manage.DEFAULT_PROFILE):
    return ['rk3588-wheeltec.service', *('rk3588-perception@' + c + '.service' for c in manage.PROFILES[profile])]


def healthy(profile=manage.DEFAULT_PROFILE, url=WEB_URL):
    """(是否健康, 不健康的原因)：每个服务 active，网页 /app/beacon 能应答。"""
    for unit in stack_units(profile):
        state = _systemctl('is-active', unit, check=False).stdout.strip()
        if state != 'active':
            return False, '%s 是 %s' % (unit, state or '未知')
    try:
        with urllib.request.urlopen(url, timeout=2) as r:
            if r.status != 200:
                return False, '网页服务回 %d' % r.status
    except OSError as exc:
        return False, '网页服务没应答（%s）' % exc
    return True, ''


def wait_healthy(profile=manage.DEFAULT_PROFILE, timeout=HEALTH_S, stable=STABLE_S, check=healthy,
                 clock=time.monotonic, sleep=time.sleep):
    deadline, since, why = clock() + timeout, None, '没等到服务起来'
    while clock() < deadline:
        ok, why = check(profile)
        now = clock()
        if ok:
            since = now if since is None else since
            if now - since >= stable:
                return True, ''
        else:
            since = None
        sleep(1.0)
    return False, why


def run_smoke(release, timeout=SMOKE_TIMEOUT_S):
    r = subprocess.run(['bash', str(Path(release) / 'deployment' / 'smoke.sh'), str(release)],
                       capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        tail = (r.stdout + r.stderr).strip().splitlines()[-3:]
        raise RuntimeError('冒烟测试不过：' + ' / '.join(tail))


def apply(release, base=BASE, unit_dir=UNIT_DIR, profile=manage.DEFAULT_PROFILE, user='root',
          smoke=run_smoke, activate=manage.activate, rollback=manage.rollback, start=start_stack,
          wait=wait_healthy, account=manage.service_account):
    """smoke → 激活 → 等健康；激活失败或不健康就回到原版本并启动。返回最后的状态。"""
    release, base = Path(release).resolve(), Path(base)
    before = current_release(base)
    rid = Path(release).name
    common = dict(release=rid, from_release=describe(before), error=None, started=time.time(), finished=None)
    write_status(base, state='checking', message='校验、冒烟测试', **common)
    try:
        manage.verify(release)
        smoke(release)
    except Exception as exc:
        return write_status(base, state='failed', message='没有升级，车上还是原版本', error=str(exc)[:400],
                            finished=time.time())
    write_status(base, state='activating', message='切换版本、重启服务')
    try:
        activate(release, base, unit_dir, profile, account(user))
    except Exception as exc:
        # manage.activate 出错时已经切回原版本，但服务停着
        try:
            start(profile)
        except Exception:
            pass
        return write_status(base, state='rolled_back', message='激活失败，已回到原版本', error=str(exc)[:400],
                            finished=time.time())
    write_status(base, state='health', message='等服务稳定')
    ok, why = wait(profile)
    if ok:
        return write_status(base, state='done', message='升级完成', finished=time.time())
    try:
        rollback(base, unit_dir)
        start(profile)
        state, message = 'rolled_back', '新版本服务起不来，已回到原版本'
    except Exception as exc:
        state, message, why = 'failed', '新版本服务起不来，回退也失败：需要 USB 处理', '%s；回退：%s' % (why, exc)
    return write_status(base, state=state, message=message, error=why[:400], finished=time.time())


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='action', required=True)
    k = sub.add_parser('keygen'); k.add_argument('--key', type=Path, default=DEFAULT_KEY); k.add_argument('--pub', type=Path)
    a = sub.add_parser('package'); a.add_argument('out', type=Path); a.add_argument('--git', default='HEAD')
    a.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1])
    a.add_argument('--key', type=Path, default=DEFAULT_KEY); a.add_argument('--note', default='')
    u = sub.add_parser('unpack'); u.add_argument('package', type=Path); u.add_argument('--base', type=Path, default=BASE)
    y = sub.add_parser('apply'); y.add_argument('release', type=Path); y.add_argument('--base', type=Path, default=BASE)
    y.add_argument('--profile', choices=manage.PROFILES, default=manage.DEFAULT_PROFILE); y.add_argument('--user', default='root')
    s = sub.add_parser('status'); s.add_argument('--base', type=Path, default=BASE)
    args = p.parse_args()
    if args.action == 'keygen':
        print('公钥写到', keygen(args.key, args.pub))
    elif args.action == 'package':
        print(json.dumps(package(args.source, args.git, args.out, args.key, args.note), ensure_ascii=False, indent=2))
    elif args.action == 'unpack':
        cur = current_release(args.base)
        release, manifest = unpack(args.package, args.base / 'releases', (cur or Path('/nonexistent')) / PUBKEY)
        print(release)
    elif args.action == 'apply':
        if sys.platform != 'linux' or os.geteuid() != 0:
            p.error('apply 要在板子上用 root 跑')
        result = apply(args.release, args.base, UNIT_DIR, args.profile, args.user)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        sys.exit(0 if result['state'] == 'done' else 1)
    else:
        print(json.dumps(status(args.base), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
