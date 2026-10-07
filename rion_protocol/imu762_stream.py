"""定位节点的 IMU762 采集线程：400 Hz 接收，主线程取最新完整样本。

只配置 USB 转换器的接收速率，不调用模块设置脚本，不发送串口命令。
USB 失联或被其他程序占用时保留错误状态，不影响现有 CAN IMU。
"""
import os
import sys
import threading
import time

from rion_protocol.ch340_usb import Ch340Usb, find_ch340
from rion_protocol.imu762_reader import ReceiverState, open_serial


class Imu762Stream:
    def __init__(self, clock=time.monotonic, enabled=True):
        self.clock = clock
        self.state = ReceiverState()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.version = 0
        self.port = os.environ.get('IMU762_PORT','')
        self.usb_path = os.environ.get('IMU762_USB_DEVICE','')
        self.endpoint = self.port or self.usb_path or '自动匹配唯一 CH340'
        self.baud = 460800
        self.thread = None
        if enabled and sys.platform.startswith('linux'):
            self.thread = threading.Thread(target=self._receive,name='imu762_receive',daemon=True)
            self.thread.start()
        else:
            self.state.error = 'IMU762 采集未在 Linux 板子上启动'

    def _receive(self):
        bus = None
        while not self.stop_event.is_set():
            try:
                if bus is None:
                    if self.port:
                        endpoint = self.port
                        bus = open_serial(self.port,self.baud)
                    else:
                        endpoint = self.usb_path or str(find_ch340())
                        bus = Ch340Usb(endpoint,self.baud)
                    with self.lock:
                        self.endpoint = endpoint
                        self.state.connected,self.state.error = True,None
                raw = bus.read(max(1,min(4096,bus.in_waiting)))
                now = self.clock()
                with self.lock:
                    frames = self.state.feed(raw,now)
                    self.version += len(frames)
            except Exception as exc:
                if bus is not None:
                    try:bus.close()
                    except Exception:pass
                    bus = None
                with self.lock:
                    self.state.disconnected(str(exc))
                    self.version += 1
                self.stop_event.wait(1.)
        if bus is not None:
            try:bus.close()
            except Exception:pass

    def snapshot(self):
        with self.lock:
            status = self.state.status(self.clock(),self.endpoint,self.baud,.2)
            status.update(nominal_hz=400,baud=460800)
            return self.version,self.state.received_at,self.state.latest,status

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.)
