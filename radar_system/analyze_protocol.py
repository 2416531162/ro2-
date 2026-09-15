#!/usr/bin/env python3
import collections
import math
import statistics
import struct
import serial

ser = serial.Serial("/dev/ttyACM0", 460800, timeout=1)
ser.dtr = True
ser.rts = True
ser.reset_input_buffer()
data = ser.read(30000)
ser.close()
sig = b"\xa5\x5a\x6c\x10"
idx = []
s = 0
while True:
    i = data.find(sig, s)
    if i < 0:
        break
    idx.append(i)
    s = i + 1
print("bytes", len(data), "frames", len(idx))


def packet_stats(label, get_points):
    vars_ = []
    all_d = []
    n_pkt = 0
    for i in idx:
        if i + 108 > len(data):
            break
        pkt = data[i : i + 108]
        pts = get_points(pkt)
        n_pkt += 1
        if len(pts) >= 4:
            vars_.append(statistics.pstdev(pts))
            all_d.extend(pts)
    if not all_d:
        print(label, "NO POINTS")
        return
    buckets = [0] * 13
    for d in all_d:
        buckets[min(12, int(d))] += 1
    print(
        label,
        "n",
        len(all_d),
        "per_rev~",
        round(len(all_d) / max(1, n_pkt / 22.5), 1),
        "min/mean/max",
        round(min(all_d), 2),
        round(sum(all_d) / len(all_d), 2),
        round(max(all_d), 2),
        "stdev_in_pkt_median",
        round(statistics.median(vars_), 2) if vars_ else None,
        "hist",
        buckets,
    )


def le3(pkt, scale=0.001, imin=0, dmax=16):
    out = []
    for k in range(32):
        d, t = struct.unpack_from("<HB", pkt, 8 + k * 3)
        r = d * scale
        if t >= imin and 0.15 < r < dmax:
            out.append(r)
    return out


def be3(pkt, scale=0.001, imin=0, dmax=16):
    out = []
    for k in range(32):
        o = 8 + k * 3
        d = (pkt[o] << 8) | pkt[o + 1]
        t = pkt[o + 2]
        r = d * scale
        if t >= imin and 0.15 < r < dmax:
            out.append(r)
    return out


packet_stats("LE mm i>=0 <16m", lambda p: le3(p, 0.001, 0, 16))
packet_stats("LE mm i>=1 <12m", lambda p: le3(p, 0.001, 1, 12))
packet_stats("LE mm i>=6 <12m", lambda p: le3(p, 0.001, 6, 12))
packet_stats("LE /4 mm i>=0 <12m", lambda p: le3(p, 0.001 / 4, 0, 12))
packet_stats("BE mm i>=0 <12m", lambda p: be3(p, 0.001, 0, 12))
packet_stats("BE mm i>=6 <12m", lambda p: be3(p, 0.001, 6, 12))

# 6-byte from offset 6: dist LE + 4 junk
def le6(pkt):
    out = []
    for k in range(17):
        d = struct.unpack_from("<H", pkt, 6 + k * 6)[0]
        r = d / 1000.0
        if 0.15 < r < 12:
            out.append(r)
    return out


packet_stats("6B LE from off6 <12m", le6)
print("sample frame0", data[idx[5] : idx[5] + 24].hex())
