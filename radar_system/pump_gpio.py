"""GPIO0_C6 的 3.3V 推挽输出；关闭为低电平，开启为高电平占空比。"""
import ctypes as C
import fcntl
import glob
import os


class ChipInfo(C.Structure):
    _fields_ = [('name', C.c_char * 32), ('label', C.c_char * 32), ('lines', C.c_uint32)]


class Attribute(C.Structure):
    _fields_ = [('id', C.c_uint32), ('padding', C.c_uint32), ('values', C.c_uint64)]


class ConfigAttribute(C.Structure):
    _fields_ = [('attr', Attribute), ('mask', C.c_uint64)]


class Config(C.Structure):
    _fields_ = [('flags', C.c_uint64), ('num_attrs', C.c_uint32), ('padding', C.c_uint32 * 5),
                ('attrs', ConfigAttribute * 10)]


class Request(C.Structure):
    _fields_ = [('offsets', C.c_uint32 * 64), ('consumer', C.c_char * 32), ('config', Config),
                ('num_lines', C.c_uint32), ('event_buffer_size', C.c_uint32),
                ('padding', C.c_uint32 * 5), ('fd', C.c_int32)]


class Values(C.Structure):
    _fields_ = [('bits', C.c_uint64), ('mask', C.c_uint64)]


def _ioc(number, struct):
    return (3 << 30) | (C.sizeof(struct) << 16) | (0xB4 << 8) | number


def _ioctl(fd, number, value):
    data = bytearray(bytes(value))
    fcntl.ioctl(fd, _ioc(number, type(value)), data, True)
    C.memmove(C.addressof(value), bytes(data), len(data))


def find_bank(label='gpio0'):
    # gpiochip 编号不等于 bank 编号；必须按内核 label 查找。
    for path in sorted(glob.glob('/dev/gpiochip*')):
        fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC)
        try:
            info = ChipInfo()
            # GPIO_GET_CHIPINFO_IOCTL 的方向是 READ（不是 READ|WRITE）。
            data = bytearray(C.sizeof(info))
            fcntl.ioctl(fd, (2 << 30) | (C.sizeof(info) << 16) | (0xB4 << 8) | 1, data, True)
            info = ChipInfo.from_buffer_copy(data)
            if info.label.decode() == label and info.lines == 32:
                return path
        finally:
            os.close(fd)
    raise OSError('未找到 GPIO bank ' + label)


class PumpOutputLine:
    """GPIO0_C6 推挽输出；申请时原子设为低电平，退出前也设低。"""
    def __init__(self, path=None, offset=22):
        if offset != 22:
            raise ValueError('水泵只允许 GPIO0_C6（bank gpio0 / offset 22）')
        path = path or find_bank()
        chip = os.open(path, os.O_RDONLY | os.O_CLOEXEC)
        self.fd = None
        try:
            info = ChipInfo()
            data = bytearray(C.sizeof(info))
            fcntl.ioctl(chip, (2 << 30) | (C.sizeof(info) << 16) | (0xB4 << 8) | 1, data, True)
            info = ChipInfo.from_buffer_copy(data)
            if info.label.decode() != 'gpio0' or info.lines != 32:
                raise ValueError('指定 gpiochip 不是 gpio0，拒绝操作其他引脚')
            req = Request()
            req.offsets[0], req.num_lines = offset, 1
            req.consumer = b'rk3588-water-pump'
            # OUTPUT | BIAS_DISABLED；不设 ACTIVE_LOW/OPEN_DRAIN，0 为低、1 为 3.3V 高。
            req.config.flags = (1 << 3) | (1 << 10)
            req.config.num_attrs = 1
            req.config.attrs[0].attr.id = 2
            req.config.attrs[0].attr.values = 0
            req.config.attrs[0].mask = 1
            _ioctl(chip, 7, req)
            self.fd = req.fd
            os.set_inheritable(self.fd, False)
        finally:
            os.close(chip)

    def high(self, active):
        _ioctl(self.fd, 15, Values(1 if active else 0, 1))

    def close(self):
        if self.fd is not None:
            try:
                self.high(False)
            finally:
                os.close(self.fd)
                self.fd = None
