"""SCH16T 原始帧记录：只记指定 ID、轮换文件、空间不够删老文件 / 暂停、截断文件能读、写盘出错不抛。"""
import gzip
import os
import shutil
import sys
import tempfile
import unittest
from collections import namedtuple
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'rion_protocol'))

import sch16t_rawlog as rl  # noqa: E402

Usage = namedtuple('Usage', 'total used free')


class Clock:
    def __init__(self):
        self.ns = 1_000_000_000

    def __call__(self):
        return self.ns


class RawLogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.dir = os.path.join(self.tmp, 'imu_raw')
        self.mono, self.wall = Clock(), Clock()
        self.wall.ns = 1_763_500_000 * 10**9

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def log(self, **kw):
        return rl.RawLog(self.dir, (0x400, 0x401, 0x402), wall_ns=self.wall, mono_ns=self.mono, **kw)

    def step(self, s):
        self.mono.ns += int(s * 1e9)
        self.wall.ns += int(s * 1e9)

    def files(self):
        return sorted(os.listdir(self.dir))

    def test_records_only_wanted_ids_and_reads_back(self):
        log = self.log(meta={'data_id': 0x400})
        log.write(0x400, bytes(range(8)))
        log.write(0x116, b'\x0f\x01')          # 驱动器帧不记
        self.step(0.02)
        log.write(0x402, b'\x60\x30\x10\x00')   # 应答不足 8 字节：长度原样存
        log.close()
        (name,) = self.files()
        head, frames = rl.read(os.path.join(self.dir, name))
        self.assertEqual(head['data_id'], 0x400)
        self.assertEqual(head['ids'], [0x400, 0x401, 0x402])
        self.assertEqual([(c, d) for _, _, c, d in frames], [(0x400, bytes(range(8))), (0x402, b'\x60\x30\x10\x00')])
        self.assertEqual(frames[1][1] - frames[0][1], 20_000_000)
        self.assertEqual(log.status()['records'], 2)

    def test_rotates_files_and_same_second_name_not_overwritten(self):
        log = self.log(file_s=60)
        log.write(0x400, b'\0' * 8)
        self.step(61)
        log.write(0x400, b'\1' * 8)
        log.close()
        log2 = self.log(file_s=60)             # 同一秒重启
        log2.write(0x400, b'\2' * 8)
        log2.close()
        self.assertEqual(len(self.files()), 3)
        total = sum(len(rl.read(os.path.join(self.dir, n))[1]) for n in self.files())
        self.assertEqual(total, 3)

    def test_truncated_file_reads_what_was_flushed(self):
        log = self.log(flush_s=0)
        for i in range(100):
            log.write(0x400, bytes([i]) * 8)
        path = os.path.join(self.dir, self.files()[0])
        snapshot = open(path, 'rb').read()      # 没 close：像断电，gzip 尾没写
        log.close()
        cut = os.path.join(self.tmp, 'cut.raw.gz')
        with open(cut, 'wb') as f:
            f.write(snapshot[:-3])
        _, frames = rl.read(cut)
        self.assertGreaterEqual(len(frames), 90)
        self.assertEqual(frames[0][3], b'\0' * 8)

    def test_over_total_deletes_oldest_closed_files(self):
        os.makedirs(self.dir)
        for i in range(3):                      # 三个旧文件各 1 MB
            with open(os.path.join(self.dir, 'sch16t_2026010%d_000000.raw.gz' % (i + 1)), 'wb') as f:
                f.write(b'x' * 1_000_000)
        with open(os.path.join(self.dir, 'notes.txt'), 'w') as f:
            f.write('别人的文件不动')
        log = self.log(max_mb=2.5, min_free_mb=0)
        log.write(0x400, b'\0' * 8)
        log.close()
        names = self.files()
        self.assertNotIn('sch16t_20260101_000000.raw.gz', names)
        self.assertIn('sch16t_20260102_000000.raw.gz', names)
        self.assertIn('notes.txt', names)
        self.assertEqual(log.status()['deleted'], 1)
        self.assertIsNone(log.status()['paused'])

    def test_low_disk_pauses_then_resumes(self):
        free = {'v': 100 * 10**6}
        with mock.patch.object(rl.shutil, 'disk_usage', lambda _: Usage(0, 0, free['v'])):
            log = self.log(min_free_mb=2048, check_s=10)
            log.write(0x400, b'\0' * 8)
            self.assertIn('磁盘剩余', log.status()['paused'])
            self.assertEqual(log.status()['records'], 0)
            free['v'] = 5000 * 10**6
            self.step(5)
            log.write(0x400, b'\0' * 8)          # 还没到下次检查
            self.assertEqual(log.status()['records'], 0)
            self.step(6)
            log.write(0x400, b'\0' * 8)
            self.assertIsNone(log.status()['paused'])
            self.assertEqual(log.status()['records'], 1)
            log.close()

    def test_write_error_pauses_without_raising(self):
        log = self.log()
        log.write(0x400, b'\0' * 8)
        with mock.patch.object(rl.gzip.GzipFile, 'write', side_effect=OSError(28, 'No space left on device')):
            log.write(0x400, b'\0' * 8)
        st = log.status()
        self.assertEqual(st['paused'], '写盘出错')
        self.assertIn('No space', st['error'])
        self.assertIsNone(st['file'])
        self.step(11)
        log.write(0x400, b'\0' * 8)              # 下次检查空间够就接着写
        self.assertIsNone(log.status()['paused'])
        log.close()

    def test_not_a_raw_file(self):
        p = os.path.join(self.tmp, 'x.gz')
        with gzip.open(p, 'wb') as f:
            f.write(b'hello\n')
        with self.assertRaises(ValueError):
            rl.read(p)


if __name__ == '__main__':
    unittest.main()
