"""IMU762 串口输出帧解码，依据厂家手册第 9–13、17–18 页。

帧头 59 53；序号 uint16、长度 uint8；多字节数据均为小端。
校验从序号开始，依次累加 CK1、CK2，均取低八位。不生成设置指令。
传感器原始数据采用前、右、下坐标系，单位转换在 to_ros 中完成。
"""
from dataclasses import dataclass
import math
import struct

HEADER = b'\x59\x53'
LENGTHS = {0x01: 2, 0x10: 12, 0x20: 12, 0x30: 12, 0x31: 12,
           0x40: 12, 0x41: 16, 0x51: 4, 0x52: 4}
NAMES = {0x10: 'acceleration_mps2', 0x20: 'gyro_deg_s', 0x30: 'magnetic_normalized',
         0x31: 'magnetic_mgauss', 0x40: 'euler_pitch_roll_yaw_deg', 0x41: 'quaternion_wxyz'}


class FrameError(ValueError):
    """输出帧损坏，不能向下游提供这帧数据。"""


def checksum(data):
    """厂家 CK1、CK2 算法；不包含帧头和校验字节。"""
    first = second = 0
    for value in data:
        first = (first + value) & 0xff
        second = (second + first) & 0xff
    return bytes((first, second))


@dataclass(frozen=True)
class Frame:
    sequence: int
    values: dict
    unknown: dict
    raw: bytes


def decode_frame(data):
    """校验并解析一个完整输出帧；已知数据项的长度不符或重复时整帧拒绝。"""
    data = bytes(data)
    if len(data) < 7 or data[:2] != HEADER or len(data) != data[4] + 7:
        raise FrameError('输出帧头或长度不正确')
    if checksum(data[2:-2]) != data[-2:]:
        raise FrameError('输出帧校验不正确')
    payload = data[5:-2]
    values, unknown, seen = {}, {}, set()
    offset = 0
    while offset < len(payload):
        if len(payload) - offset < 2:
            raise FrameError('数据项头部不完整')
        kind, length = payload[offset:offset + 2]
        offset += 2
        if offset + length > len(payload):
            raise FrameError('数据项超出有效数据长度')
        block = payload[offset:offset + length]
        offset += length
        if kind in seen:
            raise FrameError('同帧数据项重复')
        seen.add(kind)
        if kind not in LENGTHS:
            unknown[f'{kind:02x}'] = block.hex()
            continue
        if length != LENGTHS[kind]:
            raise FrameError('已知数据项长度不正确')
        if kind == 0x01:
            values['temperature_c'] = struct.unpack('<h', block)[0] * .01
        elif kind in (0x51, 0x52):
            name = 'sample_timestamp_us' if kind == 0x51 else 'dataready_timestamp_us'
            values[name] = struct.unpack('<I', block)[0]
        else:
            scale = .001 if kind == 0x31 else .000001
            values[NAMES[kind]] = tuple(value * scale for value in struct.unpack('<' + 'i' * (length // 4), block))
    return Frame(struct.unpack_from('<H', data, 2)[0], values, unknown, data)


class StreamDecoder:
    """支持半帧、粘帧和坏帧重同步，保留的半帧最多 261 字节。"""
    def __init__(self):
        self.buffer = bytearray()
        self.frames = self.bad_checksum = self.bad_payload = self.discarded_bytes = 0

    def _discard(self, length):
        del self.buffer[:length]
        self.discarded_bytes += length

    def _next_complete(self):
        """长度字节损坏时，尝试恢复后面的完整、校验合格帧。"""
        start = self.buffer.find(HEADER, 2)
        while start >= 0:
            if len(self.buffer) - start >= 7:
                size = self.buffer[start + 4] + 7
                if len(self.buffer) - start >= size:
                    try:
                        decode_frame(self.buffer[start:start + size])
                        return start
                    except FrameError:
                        pass
            start = self.buffer.find(HEADER, start + 1)
        return None

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while self.buffer:
            start = self.buffer.find(HEADER)
            if start < 0:
                # 最后一个 59 可能是下一次收到的帧头第一字节。
                self._discard(len(self.buffer) - (self.buffer[-1] == HEADER[0]))
                break
            if start:
                self._discard(start)
            if len(self.buffer) < 7:
                break
            size = self.buffer[4] + 7
            if len(self.buffer) < size:
                following = self._next_complete()
                if following is None:
                    break
                self._discard(following)
                continue
            candidate = bytes(self.buffer[:size])
            if checksum(candidate[2:-2]) != candidate[-2:]:
                self.bad_checksum += 1
                self._discard(1)
                continue
            try:
                frame = decode_frame(candidate)
            except FrameError:
                self.bad_payload += 1
                self._discard(size)
                continue
            del self.buffer[:size]
            self.frames += 1
            out.append(frame)
        return out

    def reset_buffer(self):
        """串口断开时清掉半帧，统计计数保留。"""
        self._discard(len(self.buffer))


def to_ros(frame):
    """FRD → 前、左、上；角速度转 rad/s，四元数返回 ROS 的 x、y、z、w 顺序。

手册欧拉角的顺序是俯仰、横滚、航向；保持横滚，俯仰和航向取反。
磁场强度 mGauss 转 Tesla。原始数值仍保留在 Frame.values 内。
"""
    values, out = frame.values, {}
    for source, target, scale in (
            ('acceleration_mps2', 'linear_acceleration', 1.),
            ('gyro_deg_s', 'angular_velocity', math.pi / 180.),
            ('magnetic_mgauss', 'magnetic_field', 1e-7)):
        if source in values:
            x, y, z = values[source]
            out[target] = (x * scale, -y * scale, -z * scale)
    euler = values.get('euler_pitch_roll_yaw_deg')
    if euler is not None:
        pitch, roll, yaw = (math.radians(v) for v in euler)
        out['rpy'] = (roll, -pitch, math.atan2(math.sin(-yaw), math.cos(-yaw)))
    q = values.get('quaternion_wxyz')
    if q is not None:
        w, x, y, z = q
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if .8 <= norm <= 1.2:
            out['orientation'] = (x / norm, -y / norm, -z / norm, w / norm)
    if 'orientation' not in out and 'rpy' in out:
        roll, pitch, yaw = out['rpy']
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        out['orientation'] = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                              cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    return out
