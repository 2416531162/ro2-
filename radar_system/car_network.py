#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车的 WiFi 用哪种方式：连现场 WiFi（client，路由器或手机热点），或车自己发热点（hotspot）。

做法参考 SWOZI 划线机器人：车自己发热点时热点名固定格式（RTKCAR-XX-XX，取网卡 MAC 末两字节）、
密码固定、车的地址固定（192.168.42.1），平板不用到处找车；平板差分自己用 4G 连 CORS，不经过车。

切换由 app 一条指令触发（radar_web_server 的 ``network`` 指令），用 NetworkManager（nmcli）：
- 车热点是一个单独的连接配置 ``rtk-car-hotspot``（没有就建）。切到热点：打开它的自动连接、启用它；
  切回 WiFi：关掉它的自动连接、停用它，NetworkManager 自己连回存着的 WiFi（pattern、手机热点……）。
  自动连接跟着模式走，所以车重启后还是上次选的方式。其它 WiFi 配置一律不动。
- 切换会断网 10–30 s：先回 ack，SWITCH_DELAY_S 后再动网卡，平板来得及收到回复。
- 切过去以后 REVERT_S 内没有平板连上来（热点名或密码不对、驱动不支持、现场没有那个 WiFi……），
  自动切回原来的方式，不会把人锁在车外面。

运行 nmcli 的函数可以换掉（测试用假的）。只在 radar_web_server 里用，不碰运动控制。
"""
import os
import re
import threading
import time

import subprocess

HOTSPOT_CON = 'rtk-car-hotspot'
HOTSPOT_PREFIX = 'RTKCAR'
# 热点密码固定（与 SWOZI 一样写死，平板不用输）；现场要换就在运行环境里设 RTK_HOTSPOT_PSK（至少 8 位）
HOTSPOT_PSK = os.environ.get('RTK_HOTSPOT_PSK') or 'rtkcar2026'
HOTSPOT_IP = '192.168.42.1'
HOTSPOT_PREFIX_LEN = 24
SWITCH_DELAY_S = 1.5
REVERT_S = 120.0
NMCLI_TIMEOUT_S = 40.0
MODES = ('hotspot', 'client')


def run_nmcli(args, timeout=NMCLI_TIMEOUT_S):
    """跑一条 nmcli，返回 (成功, 输出)。没有 nmcli（电脑上跑演示车）时返回失败。"""
    try:
        p = subprocess.run(['nmcli'] + list(args), capture_output=True, text=True, timeout=timeout)
        return p.returncode == 0, (p.stdout or '') + (p.stderr or '')
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)


def _split(line):
    """nmcli -t 的一行按冒号拆开（字段里的冒号写成 \\:）。"""
    return [p.replace('\x00', ':') for p in line.replace('\\:', '\x00').split(':')]


def wifi_device(devices_text):
    """`nmcli -t -f DEVICE,TYPE dev` → 第一块 WiFi 网卡名（没有返回 None）。

    >>> wifi_device('eth0:ethernet\\nwlan0:wifi\\np2p-dev-wlan0:wifi-p2p\\nlo:loopback')
    'wlan0'
    """
    for line in devices_text.splitlines():
        parts = _split(line)
        if len(parts) >= 2 and parts[1] == 'wifi':
            return parts[0]
    return None


def hotspot_ssid(mac):
    """网卡 MAC → 热点名 RTKCAR-XX-XX（末两字节，大写）；读不到 MAC 时 RTKCAR-00-00。

    >>> hotspot_ssid('a4:7e:fa:3f:a2:10')
    'RTKCAR-A2-10'
    """
    parts = re.findall(r'[0-9A-Fa-f]{2}', mac or '')
    tail = parts[-2:] if len(parts) >= 2 else ['00', '00']
    return '%s-%s-%s' % (HOTSPOT_PREFIX, tail[0].upper(), tail[1].upper())


def active_wifi(active_text, device):
    """`nmcli -t -f NAME,TYPE,DEVICE con show --active` → 这块 WiFi 网卡上正启用的连接名（没有返回 None）。

    >>> active_wifi('Wired:802-3-ethernet:eth0\\npattern:802-11-wireless:wlan0', 'wlan0')
    'pattern'
    """
    for line in active_text.splitlines():
        parts = _split(line)
        if len(parts) >= 3 and parts[2] == device and parts[1] in ('802-11-wireless', 'wifi'):
            return parts[0]
    return None


def connection_names(text):
    """`nmcli -t -f NAME con show` → 全部连接名。"""
    return [_split(line)[0] for line in text.splitlines() if line.strip()]


class CarNetwork:
    """车的 WiFi 方式和切换。status() 给遥测 / 车载屏幕，request() 给 app 指令，tick() 每秒调一次。"""

    def __init__(self, *, nmcli=run_nmcli, app_joined=None, clock=time.monotonic,
                 read_mac=None, delay_s=SWITCH_DELAY_S, revert_s=REVERT_S, spawn=None):
        self.nmcli = nmcli
        # 平板一共连上来过几次（只增不减，car_display.Sessions.joined）：切换后比切换时多了才算新连上
        self.app_joined = app_joined or (lambda: 0)
        self.clock = clock
        self.read_mac = read_mac or _read_mac
        self.delay_s, self.revert_s = delay_s, revert_s
        self.spawn = spawn or (lambda fn: threading.Thread(target=fn, daemon=True).start())
        self._lock = threading.Lock()
        self.device = None
        self.mode = None          # 'hotspot' / 'client' / None（没 WiFi 或查不到）
        self.ssid = None          # 现在连着的 WiFi 名（车热点时是热点名）
        self.hotspot = None       # 热点名
        self.busy = False         # 正在切
        self.revert = None        # (原来的方式, 截止时刻, 切好时平板累计连接次数)：等平板连上来
        self.error = None         # 上一次切换失败的原因（给 app 看）
        self.last_switch = None   # 上一次切换的结果说明

    # ---- 查 ----
    def refresh(self):
        ok, devs = self.nmcli(['-t', '-f', 'DEVICE,TYPE', 'dev'], timeout=5)
        device = wifi_device(devs) if ok else None
        mode = ssid = None
        if device:
            ok, active = self.nmcli(['-t', '-f', 'NAME,TYPE,DEVICE', 'con', 'show', '--active'], timeout=5)
            name = active_wifi(active, device) if ok else None
            if name == HOTSPOT_CON:
                mode = 'hotspot'
            elif name:
                mode = 'client'
                ok, out = self.nmcli(['-t', '-g', '802-11-wireless.ssid', 'con', 'show', name], timeout=5)
                ssid = out.strip().splitlines()[0] if ok and out.strip() else name
            else:
                mode = 'client'    # WiFi 网卡在、没连上：按「连 WiFi」算（NetworkManager 会自己去连）
        hotspot = hotspot_ssid(self.read_mac(device)) if device else None
        with self._lock:
            self.device, self.mode, self.hotspot = device, mode, hotspot
            self.ssid = hotspot if mode == 'hotspot' else ssid

    def status(self):
        """给遥测 / 车载屏幕：方式、WiFi 名、车热点名和地址、正在切 / 等平板连上来。"""
        with self._lock:
            out = {'mode': self.mode, 'ssid': self.ssid, 'hotspot_ssid': self.hotspot,
                   'hotspot_ip': HOTSPOT_IP, 'busy': self.busy, 'error': self.error}
            if self.revert is not None:
                out['revert_to'] = self.revert[0]
                out['revert_in_s'] = max(0, int(round(self.revert[1] - self.clock())))
            return out

    def hotspot_info(self):
        """hello 里告诉 app：车热点的名字、密码、车的地址，app 用来帮平板连上热点。"""
        with self._lock:
            return {'hotspot_ssid': self.hotspot, 'hotspot_psk': HOTSPOT_PSK, 'hotspot_ip': HOTSPOT_IP}

    # ---- 切 ----
    def request(self, mode):
        """app 要切到 mode。返回 (接受了没有, 说明)；接受后 SWITCH_DELAY_S 秒再真正动网卡。"""
        if mode not in MODES:
            return False, '方式只能是 hotspot 或 client'
        with self._lock:
            if self.device is None:
                return False, '车上没有找到 WiFi 网卡'
            if self.busy:
                return False, '正在切换，等一会儿'
            if mode == self.mode and self.revert is None:
                return False, '已经是这种方式'
            previous = self.mode or 'client'
            self.busy, self.error = True, None
        self.spawn(lambda: self._switch(mode, previous, delay=True))
        return True, '%d 秒后切换，平板会断开一会儿' % max(1, int(round(self.delay_s)))

    def _switch(self, mode, previous, *, delay, guard=True, note=None):
        """note：切成功后留给 app 看的说明（自动切回时写原因）。"""
        if delay and self.delay_s > 0:
            time.sleep(self.delay_s)
        ok, why = (self._to_hotspot() if mode == 'hotspot' else self._to_client())
        self.refresh()
        with self._lock:
            self.busy = False
            if ok:
                self.error = note
                self.last_switch = '已切到' + ('车热点' if mode == 'hotspot' else 'WiFi')
                # 切过去以后等平板连上来：累计连接次数比切好时多才算（旧连接 3 s 收不到消息就断，不算数）
                self.revert = (previous, self.clock() + self.revert_s, self.app_joined()) if guard else None
            else:
                self.error = why
                self.revert = None
        if not ok and guard:
            # 没切成：照原样回去（热点起不来时网卡可能已经空着）
            self._switch(previous, previous, delay=False, guard=False)
            with self._lock:
                self.error = why

    def _to_hotspot(self):
        dev = self.device
        ok, why = self._ensure_hotspot(dev)
        if not ok:
            return False, why
        self.nmcli(['con', 'modify', HOTSPOT_CON, 'connection.autoconnect', 'yes'], timeout=5)
        ok, out = self.nmcli(['con', 'up', HOTSPOT_CON, 'ifname', dev])
        return (True, '') if ok else (False, '车热点没起来：' + _short(out))

    def _to_client(self):
        self.nmcli(['con', 'modify', HOTSPOT_CON, 'connection.autoconnect', 'no'], timeout=5)
        self.nmcli(['con', 'down', HOTSPOT_CON], timeout=15)
        # 让 NetworkManager 按存着的 WiFi 配置（优先级）自己连
        self.nmcli(['dev', 'wifi', 'rescan', 'ifname', self.device], timeout=10)
        ok, out = self.nmcli(['dev', 'connect', self.device])
        return (True, '') if ok else (False, '没连上存着的 WiFi：' + _short(out))

    def _ensure_hotspot(self, dev):
        """没有车热点配置就建一个；热点名跟着网卡 MAC。"""
        ok, names = self.nmcli(['-t', '-f', 'NAME', 'con', 'show'], timeout=5)
        ssid = hotspot_ssid(self.read_mac(dev))
        if ok and HOTSPOT_CON in connection_names(names):
            self.nmcli(['con', 'modify', HOTSPOT_CON, '802-11-wireless.ssid', ssid,
                        'wifi-sec.psk', HOTSPOT_PSK, 'connection.interface-name', dev], timeout=5)
            return True, ''
        ok, out = self.nmcli([
            'con', 'add', 'type', 'wifi', 'ifname', dev, 'con-name', HOTSPOT_CON, 'autoconnect', 'no',
            'ssid', ssid, '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
            'ipv4.method', 'shared', 'ipv4.addresses', '%s/%d' % (HOTSPOT_IP, HOTSPOT_PREFIX_LEN),
            'ipv6.method', 'disabled', 'wifi-sec.key-mgmt', 'wpa-psk', 'wifi-sec.psk', HOTSPOT_PSK,
            'connection.autoconnect-priority', '100'], timeout=10)
        return (True, '') if ok else (False, '建车热点配置失败：' + _short(out))

    def tick(self):
        """每秒调一次：等平板连上来；过了 REVERT_S 还没有就切回原来的方式。"""
        with self._lock:
            pending = self.revert
            if pending is None or self.busy:
                return
            previous, deadline, joined = pending
            if self.app_joined() > joined:
                self.revert = None          # 平板连上来了，新方式留下
                return
            if self.clock() < deadline:
                return
            self.revert = None
            self.busy = True
        note = '%d 秒内没有平板连上来，已切回原来的方式' % int(self.revert_s)
        self.spawn(lambda: self._switch(previous, previous, delay=False, guard=False, note=note))

    def start(self, period=1.0, refresh_every=5):
        """后台线程：每 period 秒 tick 一次；nmcli 查网络每 refresh_every 次才查一回（切换完会立即查），省板子的 CPU。"""
        def loop():
            n = 0
            while True:
                try:
                    if n % refresh_every == 0:
                        self.refresh()
                    self.tick()
                except Exception:      # 查网络出错不影响网页 / app 服务
                    pass
                n += 1
                time.sleep(period)
        threading.Thread(target=loop, daemon=True).start()
        return self


def _read_mac(device):
    try:
        with open('/sys/class/net/%s/address' % device) as f:
            return f.read().strip()
    except (OSError, TypeError):
        return None


def _short(text):
    text = ' '.join((text or '').split())
    return text[:120] or '未知原因'
