#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HDA536T 上车前设置（车载电脑上跑，不需要 ROS；设置期间别起 imu 组件）。

    python3 rion_protocol/hda536t_setup.py show            # 听 3 s：收到哪些帧、频率、角度、自检、安装方式
    python3 rion_protocol/hda536t_setup.py gen5            # 一次设好：输出周期 20 ms（50 Hz）、角度+加速度+陀螺三帧
    python3 rion_protocol/hda536t_setup.py period 20       # 单项：输出周期 ms（5–1000）
    python3 rion_protocol/hda536t_setup.py mask 7          # 单项：1 角度、2 加速度、4 陀螺，相加
    python3 rion_protocol/hda536t_setup.py baud 500        # 单项：1000/500/250/125/100 K
    python3 rion_protocol/hda536t_setup.py node 6          # 单项：改节点号（1–127）
    python3 rion_protocol/hda536t_setup.py zero-yaw        # 当前航向清零（相对值，重新上电也会归零）

公共选项：--channel can0、--node-id 5（传感器当前的节点号）、--extended（C3 型号，CAN2.0B 扩展帧）、
--mount-yaw 90（外壳 X 箭头在车上朝哪，只影响 show 显示的车体姿态；默认读同目录 hda536t.yaml 的 mount_yaw_deg）。
每条设置发到 0x600+节点号，传感器原样回一帧才算成功。周期、输出帧、波特率、节点号都要**重新上电**才生效。
整条总线统一 500K（驱动器、传感器、can0）。传感器出厂 125K，波特率由用户自己改成 500K 后再用本脚本。
"""
import argparse
import os
import re
import sys
import time

import hda536t_codec as hc
from hda536t_imu import CanSocket, Hda536t


def request(bus, node_id, payload, timeout_s=1.0):
    """发一条设置，等传感器原样回；返回 True / False。"""
    bus.send(hc.SDO_REQUEST + node_id, payload)
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        got = bus.recv()
        if got and got[0] == hc.SDO_REPLY + node_id and bytes(got[1]) == bytes(payload):
            return True
    return False


def yaml_mount_yaw():
    """同目录 hda536t.yaml 里的 mount_yaw_deg（没有就 0）：show 与驱动按同一个安装方向换算。"""
    try:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hda536t.yaml'), encoding='utf-8') as fh:
            m = re.search(r'^\s*mount_yaw_deg:\s*([-+0-9.]+)', fh.read(), re.M)
        return float(m.group(1)) if m else 0.0
    except OSError:
        return 0.0


def show(bus, node_id, seconds, mount_yaw_deg=0.0):
    imu = Hda536t(node_id, mount_yaw_deg=mount_yaw_deg)
    t0 = time.monotonic()
    others = set()
    while time.monotonic() - t0 < seconds:
        got = bus.recv()
        if got is None:
            continue
        if hc.pdo_node(got[0]) != node_id:
            others.add(got[0])
        imu.feed(time.monotonic(), *got)
    st = imu.status(time.monotonic())
    print('节点 %d：数据帧 ID %s' % (node_id, hex(st['can_id']) if st['can_id'] is not None else '没收到'))
    print('频率：' + '，'.join('%s %.1f Hz' % kv for kv in st['rates_hz'].items()))
    if 'roll_deg' in st:
        side = {0: '车头', 90: '车左', 180: '车尾', 270: '车右'}.get(mount_yaw_deg % 360, '车头逆时针 %g°' % mount_yaw_deg)
        print('安装方向：X 箭头朝%s（mount_yaw_deg %g）' % (side, mount_yaw_deg))
        print('车体姿态（REP-103）：横滚 %.2f°（左侧抬起为正）  俯仰 %.2f°（车头压低为正）  航向 %.2f°（左转为正，相对上电）  温度 %s ℃'
              % (st['roll_deg'], st['pitch_deg'], st['yaw_deg'], st['temp_c']))
    print('自检：%s  安装方式：%s  坏帧：%d' % (st['self_test'], st['mount'], st['bad_frames']))
    if others:
        print('总线上还有这些 ID（驱动器等）：' + ' '.join(hex(i) for i in sorted(others)))
    print('问题：' + (st['problem'] or '没有'))
    return st['problem'] is None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--node-id', type=int, default=5)
    ap.add_argument('--extended', action='store_true')
    ap.add_argument('action', choices=('show', 'gen5', 'period', 'mask', 'baud', 'node', 'zero-yaw'))
    ap.add_argument('value', nargs='?', type=int)
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--mount-yaw', type=float, default=None, help='外壳 X 箭头在车上朝哪，度（默认读 hda536t.yaml）')
    a = ap.parse_args(argv)
    bus = CanSocket(a.channel, a.extended, timeout_s=0.1)
    try:
        if a.action == 'show':
            mount = yaml_mount_yaw() if a.mount_yaw is None else a.mount_yaw
            return 0 if show(bus, a.node_id, a.seconds, mount) else 1
        if a.action == 'gen5':
            steps = [('输出周期 20 ms', hc.set_period(20)), ('角度+加速度+陀螺三帧', hc.set_mask())]
        elif a.action == 'zero-yaw':
            steps = [('航向清零', hc.zero_yaw())]
        else:
            if a.value is None:
                ap.error('%s 要带一个数' % a.action)
            build = {'period': hc.set_period, 'baud': hc.set_baud, 'node': hc.set_node,
                     'mask': lambda m: hc.set_mask(m & 1, m & 2, m & 4)}[a.action]
            steps = [('%s %d' % (a.action, a.value), build(a.value))]
        ok = True
        for label, payload in steps:
            good = request(bus, a.node_id, payload)
            print('%s：%s' % (label, '成功' if good else '没有应答（查波特率、节点号、接线）'))
            ok = ok and good
        if ok and a.action != 'zero-yaw':
            print('重新上电后生效。' + ('波特率改了：can0 也要改成同样的波特率。' if a.action == 'baud' else ''))
        return 0 if ok else 1
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
