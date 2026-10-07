#!/usr/bin/env python3
"""显式将 IMU762 输出设为 50 Hz、115200；默认保存到 Flash。

仅发送输出频率、波特率设置和这两项的查询，不修改算法或校准。
依据用户提供手册第 13–17、19–20 页；波特率设置没有确认回复。
"""
import argparse
import json
from pathlib import Path
import struct
import sys
import time

from imu762_codec import HEADER, checksum, StreamDecoder
from ch340_usb import Ch340Usb


def command(kind, operation, payload=b''):
    body = bytes((kind,))+struct.pack('<H',(len(payload)<<3)|operation)+payload
    return HEADER+body+checksum(body)


def reply(bus, kind, operation, seconds=2.):
    """交互帧和连续输出帧共用帧头，按交互长度、操作符和校验认回复。"""
    buffer = bytearray()
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        buffer.extend(bus.read(256))
        start = buffer.find(HEADER)
        while start >= 0:
            if len(buffer)-start < 5:
                break
            word = struct.unpack_from('<H',buffer,start+3)[0]
            length = word>>3
            if buffer[start+2] == kind and word&7 == operation and length == 1:
                size = length+7
                if len(buffer)-start < size:
                    break
                frame = bytes(buffer[start:start+size])
                if checksum(frame[2:-2]) == frame[-2:]:
                    return frame[5:-2],frame
            start = buffer.find(HEADER,start+1)
        if len(buffer) > 1024:
            del buffer[:-256]
    raise OSError('未收到校验合格的设置／查询回复')


def query(bus, kind):
    bus.write(command(kind,0))
    return reply(bus,kind,0)[0][0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--usb-device',type=Path,required=True)
    parser.add_argument('--current-baud',type=int,default=460800)
    parser.add_argument('--volatile',action='store_true',help='只保存到 RAM，默认保存到 Flash')
    args = parser.parse_args()
    operation = 1 if args.volatile else 2
    bus = Ch340Usb(args.usb_device,args.current_baud)
    result = dict(target_hz=50,target_baud=115200,flash_requested=not args.volatile)
    try:
        # 发送设置前先确认此转换器正在接收完整的 IMU762 输出帧。
        decoder = StreamDecoder()
        deadline = time.monotonic()+2.
        matched = False
        while time.monotonic() < deadline:
            for frame in decoder.feed(bus.read(256)):
                if 'gyro_deg_s' in frame.values:
                    matched = True
                    break
            if matched:
                break
        if not matched:
            raise OSError('当前波特率下没有有效 IMU762 帧，不发送设置')
        rate_command = command(0x03,operation,b'\x07')
        print('发送 50 Hz 设置：'+rate_command.hex(' '),flush=True)
        bus.write(rate_command)
        status,ack = reply(bus,0x03,operation)
        result['frequency_ack'] = ack.hex()
        if status != b'\x00':
            raise OSError('模块拒绝频率设置，不继续改变波特率')
        if query(bus,0x03) != 0x07:
            raise OSError('模块查询未确认 50 Hz，不继续改变波特率')
        baud_command = command(0x02,operation,b'\x03')
        print('发送 115200 设置：'+baud_command.hex(' '),flush=True)
        bus.write(baud_command)
        # 等待旧波特率下的短指令发完，再切换 USB 转换器接收速率。
        time.sleep(.1)
        bus.close()
        bus = Ch340Usb(args.usb_device,115200)
        result['frequency_code'] = query(bus,0x03)
        result['baud_code'] = query(bus,0x02)
        if (result['frequency_code'],result['baud_code']) != (0x07,0x03):
            raise OSError('模块当前参数查询与 50 Hz／115200 不一致')
        print(json.dumps(result,ensure_ascii=False),flush=True)
    finally:
        bus.close()


if __name__ == '__main__':
    try:
        main()
    except OSError as exc:
        print('设置未完成：'+str(exc),file=sys.stderr)
        raise SystemExit(1)
