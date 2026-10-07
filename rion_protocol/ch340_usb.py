"""内核没有 CH341 串口驱动时，通过 libusb 接收指定 CH340 的串口数据。

寄存器定义与分频约束参考 Linux v6.12 drivers/usb/serial/ch341.c。
USB 控制请求只配置转换器的串口参数；接收器不调用 write。
write 仅供用户显式请求的 IMU 参数设置脚本使用，不发送 break。
只支持已经核对端点的 1a86:7523，不解绑现有内核驱动。
"""
import ctypes as C
from ctypes.util import find_library
from pathlib import Path


def find_ch340():
    """只按 USB 元数据匹配唯一 CH340，不试开其他串口；适配重新插拔后的总线编号。"""
    candidates = []
    for path in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if ((path/'idVendor').read_text().strip(),(path/'idProduct').read_text().strip()) == ('1a86','7523'):
                candidates.append(path)
        except OSError:
            continue
    if not candidates:
        raise OSError('未检测到 CH340，请将 IMU762 转换器接到板子的 USB 主机口')
    if len(candidates) != 1:
        raise OSError('存在多个 CH340，请明确指定 IMU762 专用设备')
    return candidates[0]


class Descriptor(C.Structure):
    _fields_ = [('length',C.c_uint8),('kind',C.c_uint8),('usb',C.c_uint16),
                ('device_class',C.c_uint8),('subclass',C.c_uint8),('protocol',C.c_uint8),
                ('packet_size',C.c_uint8),('vendor',C.c_uint16),('product',C.c_uint16),
                ('version',C.c_uint16),('manufacturer',C.c_uint8),('name',C.c_uint8),
                ('serial',C.c_uint8),('configurations',C.c_uint8)]


class UsbError(OSError):
    def __init__(self, operation, code):
        self.code = code
        super().__init__(f'CH340 USB {operation}失败，libusb 错误 {code}')


def divider(baud, limited=False):
    """搜索硬件支持的时钟分频，取与所需波特率最接近的值。"""
    choices = []
    for prescaler in range(4):
        for factor in (0,1):
            if limited and prescaler < 3 and factor:
                continue
            clock_divisor = 2 ** (12-3*prescaler-factor)
            for divisor in range(9 if factor else 2,257):
                actual = 48000000 / (clock_divisor*divisor)
                register = ((256-divisor)<<8) | (factor<<2) | prescaler
                choices.append((abs(actual-baud),factor,-prescaler,register,actual))
    best = min(choices)
    if best[0] / baud > .03:
        raise ValueError('CH340 无法接近所需波特率')
    return best[3],best[4]


class Ch340Usb:
    """提供 read/in_waiting/close 接口，仅访问用户指定的 USB 设备。"""
    def __init__(self, device_path, baud):
        path = Path(device_path)
        if not path.exists():
            raise OSError('指定 USB 转换器尚未出现：'+str(path))
        if (path/'idVendor').read_text().strip() != '1a86' or (path/'idProduct').read_text().strip() != '7523':
            raise OSError('指定设备不是支持的 CH340（1a86:7523）')
        bus,address = int((path/'busnum').read_text()),int((path/'devnum').read_text())
        library = find_library('usb-1.0')
        if not library:
            raise OSError('系统没有 libusb-1.0')
        self.lib = C.CDLL(library)
        self.ctx,self.handle = C.c_void_p(),C.c_void_p()
        self.initialized = self.claimed = False
        self._bind()
        try:
            self._check(self.lib.libusb_init(C.byref(self.ctx)),'初始化')
            self.initialized = True
            devices = C.POINTER(C.c_void_p)()
            count = self.lib.libusb_get_device_list(self.ctx,C.byref(devices))
            self._check(count,'枚举')
            try:
                for i in range(count):
                    device = devices[i]
                    if self.lib.libusb_get_bus_number(device) != bus or self.lib.libusb_get_device_address(device) != address:
                        continue
                    desc = Descriptor()
                    self._check(self.lib.libusb_get_device_descriptor(device,C.byref(desc)),'检查设备')
                    if (desc.vendor,desc.product) != (0x1a86,0x7523):
                        raise OSError('USB 设备在打开前发生了变化')
                    self._check(self.lib.libusb_open(device,C.byref(self.handle)),'打开')
                    break
            finally:
                self.lib.libusb_free_device_list(devices,1)
            if not self.handle:
                raise OSError('找不到指定 USB 转换器')
            bound = self.lib.libusb_kernel_driver_active(self.handle,0)
            self._check(bound,'检查驱动')
            if bound:
                raise OSError('设备已有内核驱动，改用对应 ttyUSB 串口；不会解绑驱动')
            self._check(self.lib.libusb_claim_interface(self.handle,0),'占用专用接口')
            self.claimed = True
            version = self._input(0x5f,0,0,2)
            if len(version) != 2 or version[0] < 0x30:
                raise OSError('转换器芯片版本不支持此用户态接收方式')
            self.version = version[0]
            limited = False
            try:
                self._input(0x95,0x05,0,2)
            except UsbError as exc:
                if exc.code != -9:
                    raise
                limited = True
            divisor,self.actual_baud = divider(baud,limited)
            if self.version > 0x27:
                divisor |= 0x80
            self._output(0xa1,0,0)
            self._output(0x9a,0x1312,divisor)
            self._output(0x9a,0x2518,0xc3)  # 8N1，启用接收，发送 FIFO 从不写入。
            self._output(0xa4,0xffff,0)    # 不启用 RTS/DTR。
        except Exception:
            self.close()
            raise

    def _bind(self):
        p = C.c_void_p
        definitions = {
            'libusb_init':([C.POINTER(p)],C.c_int),
            'libusb_exit':([p],None),
            'libusb_get_device_list':([p,C.POINTER(C.POINTER(p))],C.c_ssize_t),
            'libusb_free_device_list':([C.POINTER(p),C.c_int],None),
            'libusb_get_bus_number':([p],C.c_uint8),
            'libusb_get_device_address':([p],C.c_uint8),
            'libusb_get_device_descriptor':([p,C.POINTER(Descriptor)],C.c_int),
            'libusb_open':([p,C.POINTER(p)],C.c_int),
            'libusb_close':([p],None),
            'libusb_kernel_driver_active':([p,C.c_int],C.c_int),
            'libusb_claim_interface':([p,C.c_int],C.c_int),
            'libusb_release_interface':([p,C.c_int],C.c_int),
            'libusb_control_transfer':([p,C.c_uint8,C.c_uint8,C.c_uint16,C.c_uint16,p,C.c_uint16,C.c_uint],C.c_int),
            'libusb_bulk_transfer':([p,C.c_ubyte,p,C.c_int,C.POINTER(C.c_int),C.c_uint],C.c_int),
        }
        for name,(arguments,result) in definitions.items():
            function = getattr(self.lib,name)
            function.argtypes,function.restype = arguments,result

    @staticmethod
    def _check(result, operation):
        if result < 0:
            raise UsbError(operation,result)
        return result

    def _input(self, request, value, index, size):
        buffer = C.create_string_buffer(size)
        count = self._check(self.lib.libusb_control_transfer(self.handle,0xc0,request,value,index,buffer,size,1000),'控制读取')
        return buffer.raw[:count]

    def _output(self, request, value, index):
        self._check(self.lib.libusb_control_transfer(self.handle,0x40,request,value,index,None,0,1000),'适配器参数配置')

    @property
    def in_waiting(self):
        # libusb 没有 UART 队列长度接口，每次最多收 256 字节，超时 20 ms。
        return 256

    def read(self, size):
        buffer = C.create_string_buffer(size)
        received = C.c_int()
        result = self.lib.libusb_bulk_transfer(self.handle,0x82,buffer,size,C.byref(received),20)
        if result not in (0,-7):
            self._check(result,'串口接收')
        return buffer.raw[:received.value]

    def write(self, data):
        """发送短设置帧；普通接收程序不会调用这个入口。"""
        data = bytes(data)
        if not 1 <= len(data) <= 64:
            raise ValueError('设置指令长度必须为 1–64 字节')
        buffer = C.create_string_buffer(data)
        sent = C.c_int()
        self._check(self.lib.libusb_bulk_transfer(self.handle,0x02,buffer,len(data),C.byref(sent),1000),'设置帧发送')
        if sent.value != len(data):
            raise OSError('设置帧未完整发送，停止后续设置')
        return sent.value

    def close(self):
        if self.handle:
            if self.claimed:
                self.lib.libusb_release_interface(self.handle,0)
                self.claimed = False
            self.lib.libusb_close(self.handle)
            self.handle = C.c_void_p()
        if self.initialized:
            self.lib.libusb_exit(self.ctx)
            self.initialized = False
