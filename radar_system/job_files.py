#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业记录（job_recorder 存在 ~/rtk/jobs 的目录）给 app 下载：平板存一份，不连车也能拿去分析。

    GET /app/jobs              列表（新的在前）：name、id、label、created、result、segments、
                               open（还有段没收尾：正在记，或者刚停下 POST_S 秒内）、bytes、updated（job.json 修改时刻）
    GET /app/jobs/<name>.tar   整个作业目录打成 tar（bag 本身已经 zstd 压缩，不再压），HTTP 分块传输边打包边发：
                               最后的 0 长度块到了才算传完，中途断了 app 能发现

    DELETE /app/jobs/<name>?updated=<列表中的 updated>  删除已停止记录且未更新的作业目录。

name 必须是 root 下真实存在的一层目录，不能带路径或指向其它目录的软链接。
"""
import json
import math
import os
import re
import shutil
import tarfile

NAME_RE = re.compile(r'^[^/\\\x00]+$')


def _job_dirs(root):
    try:
        names = os.listdir(root)
    except OSError:
        return []
    return [n for n in names if NAME_RE.match(n) and n not in ('.', '..')
            and job_path(root, n) is not None]


def _size(path):
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(base, f))
            except OSError:
                pass
    return total


def list_jobs(root):
    """作业记录列表，新的在前。读不了的 job.json 跳过。"""
    out = []
    for name in _job_dirs(root):
        path = os.path.join(root, name)
        meta = os.path.join(path, 'job.json')
        try:
            with open(meta, encoding='utf-8') as fh:
                info = json.load(fh)
            updated = os.path.getmtime(meta)
        except (OSError, ValueError):
            continue
        segments = info.get('segments') or []
        out.append(dict(name=name, id=info.get('id'), label=info.get('label'), created=info.get('created'),
                        result=info.get('result'), segments=len(segments),
                        open=any(isinstance(s, dict) and s.get('end') is None for s in segments),
                        bytes=_size(path), updated=round(updated, 3)))
    out.sort(key=lambda j: j['name'], reverse=True)
    return out


def job_path(root, name):
    """name 对应的作业目录；不合法或不存在返回 None（防止 ../ 之类跑出 root）。

    >>> job_path('/tmp', '../etc') is None, job_path('/tmp', 'a/b') is None
    (True, True)
    """
    if not isinstance(name, str) or not NAME_RE.match(name) or name in ('.', '..'):
        return None
    path = os.path.join(root, name)
    if os.path.dirname(os.path.abspath(path)) != os.path.abspath(root):
        return None
    meta = os.path.join(path, 'job.json')
    if os.path.islink(path) or os.path.islink(meta):
        return None
    return path if os.path.isdir(path) and os.path.isfile(meta) else None


class DeleteError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def delete_job(root, name, updated):
    """只删除用户看到的同一份已收尾记录；错误不删除任何文件。"""
    path = job_path(root, name)
    if path is None:
        raise DeleteError(404, '作业记录不存在，请刷新列表')
    if not isinstance(updated, (int, float)) or not math.isfinite(updated):
        raise DeleteError(400, '缺少有效的记录更新时间，请刷新列表')
    meta = os.path.join(path, 'job.json')
    try:
        with open(meta, encoding='utf-8') as fh:
            info = json.load(fh)
        if not isinstance(info, dict) or not isinstance(info.get('segments', []), list):
            raise ValueError('invalid metadata')
        if any(not isinstance(s, dict) or s.get('end') is None for s in info.get('segments', [])):
            raise DeleteError(409, '作业还在记录，收尾后再删除')
        if round(os.path.getmtime(meta), 3) != updated:
            raise DeleteError(409, '作业记录有更新，请刷新列表后重新选择')
        shutil.rmtree(path)
    except (OSError, ValueError) as exc:
        raise DeleteError(500, '作业记录无法删除，请检查存储权限后重试') from exc


def handle_delete(handler, root, can_delete):
    """独立 DELETE 路由；每次请求都重新确认车辆空闲，不接受浏览器来源的删除请求。"""
    import urllib.parse
    url = urllib.parse.urlsplit(handler.path)
    if not url.path.startswith('/app/jobs/'):
        return False
    try:
        if handler.headers.get('Origin'):
            raise DeleteError(403, '不接受网页来源的删除请求')
        query = urllib.parse.parse_qs(url.query)
        if set(query) != {'updated'} or len(query['updated']) != 1:
            raise DeleteError(400, '缺少有效的记录更新时间，请刷新列表')
        try:
            updated = float(query['updated'][0])
        except ValueError as exc:
            raise DeleteError(400, '记录更新时间无效') from exc
        if not can_delete():
            raise DeleteError(409, '请停车并结束当前作业，等待底盘在线后再删除')
        name = urllib.parse.unquote(url.path[len('/app/jobs/'):])
        delete_job(root, name, updated)
        data, status = {'ok': True, 'deleted': name}, 200
    except DeleteError as exc:
        data, status = {'ok': False, 'message': str(exc)}, exc.status
    body = json.dumps(data, ensure_ascii=False).encode('utf-8')
    handler.close_connection = True
    handler.send_response(status)
    handler.send_header('Content-Type', 'application/json; charset=utf-8')
    handler.send_header('Content-Length', str(len(body)))
    handler.send_header('Connection', 'close')
    handler.end_headers()
    handler.wfile.write(body)
    return True


class ChunkedWriter:
    """HTTP/1.1 分块传输：每次 write 一个块，close 写结尾的 0 长度块。"""

    def __init__(self, out):
        self.out = out

    def write(self, data):
        if data:
            self.out.write(b'%x\r\n' % len(data))
            self.out.write(bytes(data))
            self.out.write(b'\r\n')
        return len(data)

    def flush(self):
        flush = getattr(self.out, 'flush', None)
        if flush:
            flush()

    def close(self):
        self.out.write(b'0\r\n\r\n')
        self.flush()


def handle(handler, root):
    """http.server 的 handler 上处理 /app/jobs、/app/jobs/<name>.tar；不是这两个路径返回 False（车端和演示服务器共用）。"""
    import urllib.parse
    path = handler.path.split('?')[0]
    if path == '/app/jobs':
        body = json.dumps({'ok': True, 'jobs': list_jobs(root)}, ensure_ascii=False).encode('utf-8')
        handler.send_response(200)
        handler.send_header('Content-Type', 'application/json; charset=utf-8')
        handler.send_header('Content-Length', str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
        return True
    if not path.startswith('/app/jobs/'):
        return False
    name = urllib.parse.unquote(path[len('/app/jobs/'):])
    job = job_path(root, name[:-4]) if name.endswith('.tar') else None
    if job is None:
        handler.send_error(404)
        return True
    handler.close_connection = True
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/x-tar')
    handler.send_header('Transfer-Encoding', 'chunked')
    handler.send_header('Connection', 'close')
    handler.end_headers()
    out = ChunkedWriter(handler.wfile)
    try:
        write_tar(job, out)
        out.close()
    except (OSError, ValueError):
        pass        # 平板断开、文件正被写：不写结尾块，app 就知道没传完
    return True


def write_tar(path, out):
    """把作业目录写成 tar 流（顶层是目录名）；out 只需要 write()。"""
    with tarfile.open(fileobj=out, mode='w|', format=tarfile.PAX_FORMAT) as tar:
        tar.add(path, arcname=os.path.basename(path))
