#!/usr/bin/env python3
import collections
import struct
import serial

ser = serial.Serial("/dev/ttyACM0", 460800, timeout=1)
ser.dtr = True
ser.rts = True
ser.reset_input_buffer()
data = ser.read(24000)
ser.close()
print("raw_bytes", len(data))
sig = b"\xa5\x5a\x6c\x10"
idx = []
s = 0
while True:
    i = data.find(sig, s)
    if i < 0:
        break
    idx.append(i)
    s = i + 1
print("headers", len(idx))
gaps = [b - a for a, b in zip(idx, idx[1:])]
print("gap", collections.Counter(gaps).most_common(6))
angs = []
dists = []
intens = []
for i in idx:
    if i + 108 > len(data):
        break
    pkt = data[i : i + 108]
    raw = struct.unpack_from("<H", pkt, 4)[0]
    angs.append(raw)
    for k in range(32):
        d, t = struct.unpack_from("<HB", pkt, 8 + k * 3)
        dists.append(d)
        intens.append(t)
print("packets", len(angs))
print("raw_ang first24", angs[:24])
if angs:
    dlt = [(angs[i + 1] - angs[i]) & 0xFFFF for i in range(len(angs) - 1)]
    print("delta top", collections.Counter(dlt).most_common(8))
    print("deg/64", [round((a / 64.0) % 360, 1) for a in angs[:18]])
valid = [d for d in dists if 50 < d < 16000]
print("valid", len(valid), "/", len(dists))
if valid:
    print("dist_mm min/max/mean", min(valid), max(valid), round(sum(valid) / len(valid), 1))
    buckets = [0] * 17
    for d in valid:
        buckets[min(16, d // 1000)] += 1
    print("hist_m", list(enumerate(buckets)))
print("inten top", collections.Counter(intens).most_common(12))
# one packet dump
if idx:
    pkt = data[idx[3] : idx[3] + 108]
    print("pkt hex", pkt.hex())
    print("pts", [struct.unpack_from("<HB", pkt, 8 + k * 3) for k in range(32)])
