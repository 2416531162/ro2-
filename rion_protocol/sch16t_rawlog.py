"""SCH16T 原始 CAN 帧记录：每一帧原样存盘，留着以后离线分析、改进 IMU（标定、滤波、零偏）。

文件：<目录>/sch16t_YYYYmmdd_HHMMSS.raw.gz，默认每小时一个（gzip，约 2 s 刷一次盘，断电最多丢几秒）。
  第 1 行  b'SCH16T-RAW 1\\n'
  第 2 行  一行 JSON（格式说明、开始时间、数据帧 ID），以 \\n 结尾
  其后    每帧 27 字节，struct '<qqHB8s'：墙钟 ns、单调钟 ns、CAN ID、数据长度、8 字节数据（不足补 0）

空间：记录目录总量超过 max_mb，或所在磁盘剩余少于 min_free_mb（作业记录和它在同一块盘上），就从最老的文件删起；
删光了还不够就暂停写，空间够了自动接着写。写盘出错也只暂停、不影响 /imu。

读取：python3 sch16t_rawlog.py 文件...   每帧打印一行：墙钟s 单调钟s ID 数据；--csv 出 CSV。
截断的 gzip（断电没写完的文件）读到哪算哪。
"""
import gzip
import json
import os
import shutil
import struct
import sys
import threading
import time
import zlib

MAGIC = b'SCH16T-RAW 1\n'
RECORD = struct.Struct('<qqHB8s')
PREFIX, SUFFIX = 'sch16t_', '.raw.gz'


class RawLog:
    def __init__(self, directory, ids, max_mb=3000.0, min_free_mb=2048.0, file_s=3600.0, flush_s=2.0,
                 check_s=10.0, wall_ns=time.time_ns, mono_ns=time.monotonic_ns, meta=None):
        self.dir, self.ids = directory, frozenset(ids)
        self.max_bytes, self.min_free = max_mb * 1e6, min_free_mb * 1e6
        self.file_ns, self.flush_ns, self.check_ns = int(file_s * 1e9), int(flush_s * 1e9), int(check_s * 1e9)
        self.wall_ns, self.mono_ns, self.meta = wall_ns, mono_ns, dict(meta or {})
        self._lock = threading.Lock()
        self._f, self._path = None, None
        self._opened = self._flushed = self._checked = None
        self.records = 0          # 本次启动写进去的帧数
        self.deleted = 0          # 为腾空间删掉的文件数
        self.paused = None        # 暂停原因（空间不够 / 写盘出错），None = 正常写
        self.error = None
        self._total = 0           # 目录里已关闭文件的总字节（每次检查时重算）

    # ---- 写 ----
    def write(self, can_id, data):
        """收到一帧就调一次；不是要记的 ID 直接返回。永远不抛异常。"""
        if can_id not in self.ids:
            return
        with self._lock:
            try:
                mono = self.mono_ns()
                if self._checked is None or mono - self._checked >= self.check_ns:
                    self._checked = mono
                    self._housekeep()
                if self.paused:
                    return
                if self._f is None or mono - self._opened >= self.file_ns:
                    self._rotate(mono)
                data = bytes(data)[:8]
                self._f.write(RECORD.pack(self.wall_ns(), mono, can_id, len(data), data.ljust(8, b'\0')))
                self.records += 1
                if mono - self._flushed >= self.flush_ns:
                    self._f.flush()
                    self._flushed = mono
            except OSError as exc:
                self.error = '%s: %s' % (type(exc).__name__, exc)
                self.paused = '写盘出错'
                self._close()

    def _rotate(self, mono):
        self._close()
        os.makedirs(self.dir, exist_ok=True)
        wall = self.wall_ns()
        name = PREFIX + time.strftime('%Y%m%d_%H%M%S', time.localtime(wall / 1e9)) + SUFFIX
        path = os.path.join(self.dir, name)
        n = 1
        while os.path.exists(path):               # 同一秒重启：加序号，别覆盖
            path = os.path.join(self.dir, name.replace(SUFFIX, '_%d%s' % (n, SUFFIX)))
            n += 1
        f = gzip.open(path, 'wb', compresslevel=6)
        head = dict(self.meta, format='<qqHB8s wall_ns mono_ns can_id dlc data8', start_wall_ns=wall,
                    start_mono_ns=mono, ids=sorted(self.ids))
        f.write(MAGIC + json.dumps(head, ensure_ascii=False).encode() + b'\n')
        self._f, self._path, self._opened, self._flushed = f, path, mono, mono

    def _close(self):
        if self._f is not None:
            try:
                self._f.close()
            except OSError:
                pass
        self._f = None

    def close(self):
        with self._lock:
            self._close()

    # ---- 空间 ----
    def _files(self):
        try:
            names = sorted(n for n in os.listdir(self.dir) if n.startswith(PREFIX) and n.endswith(SUFFIX))
        except FileNotFoundError:
            return []
        return [os.path.join(self.dir, n) for n in names]

    def _housekeep(self):
        """超总量或磁盘快满：删最老的已关闭文件；还不够就暂停。"""
        try:
            files = [p for p in self._files() if p != self._path or self._f is None]
            sizes = {p: os.path.getsize(p) for p in files}
            current = os.path.getsize(self._path) if self._f is not None and os.path.exists(self._path) else 0
            free = shutil.disk_usage(self.dir if os.path.isdir(self.dir) else os.path.dirname(self.dir)).free
            total = sum(sizes.values()) + current
            for p in files:                        # 文件名带时间，排序就是从老到新
                if total <= self.max_bytes and free >= self.min_free:
                    break
                os.remove(p)
                self.deleted += 1
                total -= sizes[p]
                free += sizes[p]
            self._total = total
            if free < self.min_free:
                self.paused = '磁盘剩余 %.0f MB < %.0f MB' % (free / 1e6, self.min_free / 1e6)
                self._close()
            elif total > self.max_bytes:
                self.paused = '记录超过 %.0f MB' % (self.max_bytes / 1e6)
                self._close()
            else:                                  # 空间够了 / 上次写盘出错：重新开文件接着写
                self.paused = self.error = None
        except OSError as exc:
            self.error = '%s: %s' % (type(exc).__name__, exc)
            self.paused = '写盘出错'
            self._close()

    def status(self):
        with self._lock:
            return {'dir': self.dir, 'file': os.path.basename(self._path) if self._f is not None else None,
                    'records': self.records, 'total_mb': round(self._total / 1e6, 1), 'deleted': self.deleted,
                    'paused': self.paused, 'error': self.error}


# ---- 读 ----
def read(path):
    """读一个记录文件：返回 (头 dict, [(wall_ns, mono_ns, can_id, data), ...])。截断的文件读到哪算哪。"""
    out, buf = [], b''
    with open(path, 'rb') as raw:
        d = zlib.decompressobj(16 + zlib.MAX_WBITS)
        while True:
            chunk = raw.read(1 << 16)
            if not chunk:
                break
            try:
                buf += d.decompress(chunk)
            except zlib.error:
                break
            while d.unused_data:                   # 多段 gzip（追加写）：接着解下一段
                rest = d.unused_data
                d = zlib.decompressobj(16 + zlib.MAX_WBITS)
                try:
                    buf += d.decompress(rest)
                except zlib.error:
                    break
    if not buf.startswith(MAGIC):
        raise ValueError('%s 不是 SCH16T 原始记录' % path)
    end = buf.index(b'\n', len(MAGIC)) if b'\n' in buf[len(MAGIC):] else None
    if end is None:
        return {}, out
    head = json.loads(buf[len(MAGIC):end])
    body = buf[end + 1:]
    for i in range(0, len(body) - RECORD.size + 1, RECORD.size):
        wall, mono, cid, dlc, data = RECORD.unpack_from(body, i)
        out.append((wall, mono, cid, data[:dlc]))
    return head, out


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    csv = '--csv' in argv
    paths = [a for a in argv if a != '--csv']
    if not paths:
        raise SystemExit(__doc__)
    if csv:
        print('wall_s,mono_s,can_id,data')
    for p in paths:
        head, frames = read(p)
        if not csv:
            print('# %s  %s  %d 帧' % (p, json.dumps(head, ensure_ascii=False), len(frames)))
        for wall, mono, cid, data in frames:
            if csv:
                print('%.6f,%.6f,0x%03X,%s' % (wall / 1e9, mono / 1e9, cid, data.hex()))
            else:
                print('%.6f %.6f %03X %s' % (wall / 1e9, mono / 1e9, cid, data.hex(' ')))


if __name__ == '__main__':
    main()
