#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平板 USB 直连：把板子的 Type-C 口 J17 配成 USB 设备（CDC-ACM 虚拟串口），平板当 USB 主机插上来，
板子这边出现 /dev/ttyGS<N>，平板的「UM982 转发」把接收机串口原始字节经这根线写过来（gnss_bridge.TabletUsbSerial 读）。

    平板 USB-A ──A 转 C 数据线──> 板子 J17（USB3.1 Type-C 1，usbdrd_dwc3_1，控制器 fc400000.usb）

J17 只能当从机（交接文档 8.1：两端都是 Rd，接不了 U 盘之类的设备），正好当 USB 设备用；J16（fc000000.usb）是
adb / 烧写口，本模块绝不碰它：只用指定的控制器（默认 fc400000.usb，环境变量 RK3588_RTK_USB_UDC 可改），
它被别的功能占着就报错不动。内核要有 USB_CONFIGFS + USB_CONFIGFS_ACM（板子 6.12 内核里都是 =y）。

只建一个名为 um982_relay 的 gadget（序列号 um982-relay-dev 认自己），配置不存闪存、重启就没了：rtk_node 每次启动
调 ensure() 重新建好，已经建好、绑好时什么也不改。要 root（configfs 只有 root 能写；板子上服务就是 root 跑）。
VID/PID 1d6b:0104 是 Linux 复合设备的开发示例号，平板 App 按 CDC-ACM 认设备，不看这个号。

命令行（在板子上）：probe 只读查看；ensure 建好并绑定；disable 只删本工具建的 gadget。
源自 um982-relay 1.5.0 随附的 tools/usb-direct/usb_gadget.py（2026-10-16），改成幂等的 ensure 供 rtk_node 调用。
"""
import argparse
import gzip
import json
import os
import platform
import sys
from pathlib import Path

ROOT = Path('/sys/kernel/config/usb_gadget')
UDCS = Path('/sys/class/udc')
NAME = 'um982_relay'
SERIAL = 'um982-relay-dev'
FUNCTION = 'acm.usb0'
J17_UDC = 'fc400000.usb'        # usbdrd_dwc3_1（rk3588.dtsi usb@fc400000）= 板子 J17
UDC_ENV = 'RK3588_RTK_USB_UDC'


class GadgetError(RuntimeError):
    pass


def _read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def _write(path, value):
    path.write_text(str(value) + '\n')


def preferred_udc():
    return os.environ.get(UDC_ENV) or J17_UDC


def probe(root=ROOT, udcs=UDCS):
    """只读：控制器、已有 gadget、ttyGS 节点、内核配置里相关的几项。"""
    flags = None
    for path in (Path('/proc/config.gz'), Path('/boot/config-' + platform.release())):
        try:
            config = gzip.decompress(path.read_bytes()).decode() if path.suffix == '.gz' else path.read_text()
        except OSError:
            continue
        flags = [line for line in config.splitlines() if any(key in line for key in (
            'CONFIG_USB_CONFIGFS=', 'CONFIG_USB_CONFIGFS_ACM', 'CONFIG_USB_F_ACM', 'CONFIG_USB_DWC3',
            'CONFIG_USB_GADGET='))]
        break
    return dict(kernel=platform.release(), configfs_available=root.is_dir(), kernel_flags=flags,
                preferred_udc=preferred_udc(),
                controllers=[dict(name=p.name, state=_read(p / 'state'), function=_read(p / 'function'))
                             for p in sorted(udcs.glob('*'))],
                gadgets=[dict(name=p.name, udc=_read(p / 'UDC'), functions=sorted(x.name for x in (p / 'functions').glob('*')))
                         for p in sorted(root.glob('*'))],
                serial_devices=sorted(str(p) for p in Path('/dev').glob('ttyGS*')))


def _device(g):
    port = _read(g / 'functions' / FUNCTION / 'port_num')
    return '/dev/ttyGS' + port if port and port.isdigit() else None


def _ours(g):
    return _read(g / 'strings/0x409/serialnumber') == SERIAL


def _check_free(udc, root, udcs):
    if udc not in [p.name for p in udcs.glob('*')]:
        raise GadgetError('没有 USB 设备控制器 %s（J17）：板子内核或设备树不支持，或口子没起来' % udc)
    if any(_read(p / 'UDC') == udc for p in root.glob('*')):
        raise GadgetError('USB 控制器 %s 已被别的 gadget 占用，不动它' % udc)
    if _read(udcs / udc / 'function'):
        raise GadgetError('USB 控制器 %s 已被别的功能占用，不动它' % udc)


def _create(g):
    g.mkdir()
    _write(g / 'idVendor', '0x1d6b')
    _write(g / 'idProduct', '0x0104')
    _write(g / 'bcdUSB', '0x0200')
    _write(g / 'bcdDevice', '0x0100')
    _write(g / 'bDeviceClass', '0x02')
    strings = g / 'strings/0x409'
    strings.mkdir(parents=True)     # 真 configfs 里 strings/、configs/、functions/ 是建 gadget 时自动有的
    _write(strings / 'serialnumber', SERIAL)
    _write(strings / 'manufacturer', 'RK3588 RTK car')
    _write(strings / 'product', 'RK3588 GNSS USB')
    cfg = g / 'configs/c.1'
    cfg.mkdir(parents=True)
    cfgstrings = cfg / 'strings/0x409'
    cfgstrings.mkdir(parents=True)
    _write(cfgstrings / 'configuration', 'GNSS CDC ACM')
    # 板子自己供电，USB 只传数据，不从平板取电
    _write(cfg / 'bmAttributes', '0xc0')
    _write(cfg / 'MaxPower', '2')
    fn = g / 'functions' / FUNCTION
    fn.mkdir(parents=True)
    (cfg / FUNCTION).symlink_to(fn)


def _cleanup(g):
    """只拆本工具建的固定结构，不递归删 configfs。"""
    if _read(g / 'UDC'):
        _write(g / 'UDC', '')
    link = g / 'configs/c.1' / FUNCTION
    if link.is_symlink():
        link.unlink()
    for rel in ('configs/c.1/strings/0x409', 'configs/c.1', 'functions/' + FUNCTION, 'strings/0x409'):
        p = g / rel
        if p.exists():
            p.rmdir()
    g.rmdir()


def ensure(udc=None, root=ROOT, udcs=UDCS):
    """建好并绑定本工具的 gadget，返回 dict(udc, serial_device, created)；已经好了不改任何东西。
    configfs 不可用、控制器不存在或被占、同名 gadget 不是本工具建的 → GadgetError（不动别人的配置）。"""
    udc = udc or preferred_udc()
    if not root.is_dir():
        raise GadgetError('USB configfs 不可用（%s 不存在）：内核要 USB_CONFIGFS，并挂载 configfs' % root)
    g = root / NAME
    created = False
    if g.exists():
        if not _ours(g):
            raise GadgetError('已有同名 gadget %s 但不是本工具建的，不动它' % NAME)
        bound = _read(g / 'UDC')
        if bound:
            return dict(udc=bound, serial_device=_device(g), created=False)
    else:
        _check_free(udc, root, udcs)
        try:
            _create(g)
        except OSError as exc:
            try:
                _cleanup(g)
            except OSError:
                pass
            raise GadgetError('建 USB 虚拟串口失败：%s' % exc) from exc
        created = True
    if not created:
        _check_free(udc, root, udcs)
    try:
        _write(g / 'UDC', udc)
    except OSError as exc:
        raise GadgetError('绑定 USB 控制器 %s 失败：%s' % (udc, exc)) from exc
    return dict(udc=udc, serial_device=_device(g), created=created)


def disable(root=ROOT):
    g = root / NAME
    if not g.exists():
        return dict(removed=False)
    if not _ours(g):
        raise GadgetError('同名 gadget 不是本工具建的，不删')
    if {p.name for p in (g / 'functions').iterdir()} != {FUNCTION}:
        raise GadgetError('gadget 里有别的功能，不删')
    _cleanup(g)
    return dict(removed=True)


def main():
    parser = argparse.ArgumentParser(description='板子 J17 口的 USB 虚拟串口（平板 USB 直连转发定位数据）')
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('probe', help='只读检查，不要 root')
    e = sub.add_parser('ensure', help='建好并绑定（已经好了不改），要 root')
    e.add_argument('--udc', help='USB 设备控制器名（默认 %s = J17，或环境变量 %s）' % (J17_UDC, UDC_ENV))
    sub.add_parser('disable', help='只删本工具建的 gadget，要 root')
    args = parser.parse_args()
    try:
        if args.action != 'probe' and os.geteuid() != 0:
            raise GadgetError('配置 USB gadget 要 root')
        result = probe() if args.action == 'probe' else ensure(args.udc) if args.action == 'ensure' else disable()
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except (OSError, GadgetError) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    sys.exit(main())
