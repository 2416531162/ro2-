"""UM982 二进制日志（BESTPOSB / HEADINGB / BESTVELB / BESTNAVHB）解析。

夹具是 2026-10-06 在接收机空闲口 COM1 上同时开 A、B 两种格式各 1 Hz 抓的原始字节（平板 ttyS4，115200），
同一历元两种格式都有，用来核对二进制字段和 ASCII 逐一相同。
"""
import os
import random
import re
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'radar_system'))

import um982  # noqa: E402

SAMPLE = os.path.join(os.path.dirname(__file__), 'fixtures', 'um982_com1_ascii_binary_20261006.bin')


@pytest.fixture(scope='module')
def raw():
    with open(SAMPLE, 'rb') as f:
        return f.read()


def frames(raw):
    """夹具里的完整二进制帧（按同步字找、CRC 校验过）。"""
    out = []
    for m in re.finditer(rb'\xaa\x44[\x12\xb5]', raw):
        kind, end = um982.binary_frame(raw, m.start())
        if kind == 'frame':
            out.append(raw[m.start():end])
    return out


def ascii_logs(raw):
    out = {}
    for m in re.finditer(rb'#(BESTPOSA|BESTNAVHA|HEADINGA|BESTVELA),[^\r\n]*', raw):
        try:
            r = um982.parse_unicore(m.group(0).decode('ascii'))
        except um982.ParseError:
            continue                    # 抓包开头被截断的一行
        out[(type(r).__name__, getattr(r, 'antenna', 0), round(r.tow, 3))] = r
    return out


def test_every_binary_log_in_the_capture_matches_ascii_of_the_same_epoch(raw):
    asc = ascii_logs(raw)
    seen = {}
    for f in frames(raw):
        b = um982.parse_binary(f)
        a = asc.get((type(b).__name__, getattr(b, 'antenna', 0), round(b.tow, 3)))
        if a is None:
            continue
        seen[type(b).__name__, getattr(b, 'antenna', 0)] = seen.get((type(b).__name__, getattr(b, 'antenna', 0)), 0) + 1
        for field in a.__dataclass_fields__:
            va, vb = getattr(a, field), getattr(b, field)
            if isinstance(va, float):
                # ASCII 打印位数有限（高度、标准差 4 位小数；大地水准面差距在二进制里是 float32）
                assert vb == pytest.approx(va, abs=2e-4, rel=1e-6), (field, va, vb)
            else:
                assert va == vb, (field, va, vb)
    assert seen == {('BestPos', 1): 6, ('BestPos', 2): 4, ('Heading', 0): 4, ('BestVel', 0): 5}


def test_capture_is_narrow_float_with_doppler_velocity(raw):
    by_id = {}
    for f in frames(raw):
        msg_id = struct.unpack_from('<H', f, 4)[0]
        by_id.setdefault(msg_id, um982.parse_binary(f))
    assert set(by_id) == {42, 971, 99, 2119}
    assert by_id[42].pos_type == 'NARROW_FLOAT' and by_id[42].sol_status == 'SOL_COMPUTED'
    assert by_id[2119].antenna == 2 and by_id[42].antenna == 1
    assert by_id[99].vel_type == 'DOPPLER_VELOCITY'
    # 整帧长度：BESTPOSB 104、HEADINGB / BESTVELB 76、BESTNAVHB 148 字节（ASCII 约 207 / 169 / 150 / 276）
    assert {len(f) for f in frames(raw)} == {104, 76, 148}


def test_stream_handles_mixed_ascii_and_binary_in_any_chunking(raw):
    whole = um982.Um982Stream().feed(raw, 1.0)
    random.seed(7)
    for _ in range(20):
        s = um982.Um982Stream()
        got, i = [], 0
        while i < len(raw):
            n = random.randint(1, 40)
            got += s.feed(raw[i:i + n], 1.0)
            i += n
        assert [(m['kind'], m['t_rx']) for m in got] == [(m['kind'], m['t_rx']) for m in whole]
    kinds = [m['kind'] for m in whole]
    # ASCII 与二进制各一份：主天线 9+、副天线 8+、测向 8+、速度 8+
    assert kinds.count('fix') >= 9 and kinds.count('fix2') >= 8 and kinds.count('heading') >= 8 and kinds.count('vel') >= 8


def test_binary_only_stream_feeds_fusion_messages(raw):
    s = um982.Um982Stream()
    out = s.feed(b''.join(frames(raw)), 5.0)
    assert {m['kind'] for m in out} == {'fix', 'fix2', 'heading', 'vel'}
    fix = next(m for m in out if m['kind'] == 'fix')
    assert fix['sol'] == 'NARROW_FLOAT' and fix['sats'] == 31 and fix['diff_age'] == pytest.approx(1.0)
    hd = next(m for m in out if m['kind'] == 'heading')
    assert hd['valid'] is False                          # 浮点解、基线 0.83 m：不能用
    assert s.status(5.0)['rates']['fix'] > 0 and s.crc_errors == 0


def test_lost_can_chunk_drops_only_that_frame(raw):
    fs = frames(raw)[:8]
    broken = bytearray(fs[2])
    del broken[30:37]                                    # 经 CAN 丢了 7 字节
    stream = b''.join(fs[:2]) + bytes(broken) + b''.join(fs[3:])
    s = um982.Um982Stream()
    out = []
    for i in range(0, len(stream), 7):
        out += s.feed(stream[i:i + 7], 2.0)
    assert len(out) == 7
    assert s.crc_errors == 1


def test_corrupted_header_does_not_stall_following_frames(raw):
    fs = frames(raw)[:4]
    bad = bytearray(fs[0])
    bad[8:10] = b'\xff\x7f'                              # 长度字段坏了：不等 32 KB 数据
    s = um982.Um982Stream()
    out = s.feed(bytes(bad) + b''.join(fs[1:]), 3.0)
    assert len(out) == 3


def test_half_sync_at_buffer_end_waits_for_the_rest(raw):
    f = frames(raw)[0]
    s = um982.Um982Stream()
    assert s.feed(f[:1], 1.0) == [] and s.feed(f[1:2], 1.0) == []
    assert len(s.feed(f[2:], 1.0)) == 1


def test_gga_and_ascii_still_work_between_binary_frames(raw):
    body = 'GNGGA,024019.00,3000.00000741,N,12000.00004073,E,4,32,0.6,12.3456,M,8.5000,M,1.0,2001'
    gga = ('$%s*%02X\r\n' % (body, um982.nmea_checksum(body))).encode()
    fs = frames(raw)
    s = um982.Um982Stream()
    out = s.feed(fs[0] + gga + fs[1] + b'garbage without newline' + fs[2], 1.0)
    assert len(out) == 3
    assert s.gga is not None and s.gga[0] == 'GNGGA'


def test_unknown_enum_values_are_named_not_crashing(raw):
    f = bytearray(frames(raw)[0])
    head = 28 if f[2] == 0x12 else 24
    struct.pack_into('<I', f, head + 4, 99)
    struct.pack_into('<I', f, len(f) - 4, um982.crc32(bytes(f[:-4])))
    assert um982.parse_binary(bytes(f)).pos_type == 'UNKNOWN_99'
