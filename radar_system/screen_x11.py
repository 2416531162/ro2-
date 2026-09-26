#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""使用窗口管理器的显示桌面消息，不关闭其他程序，也不修改桌面快捷键。"""
import ctypes as C
import ctypes.util


class _Data(C.Union):
    _fields_ = [('b', C.c_char * 20), ('s', C.c_short * 10), ('l', C.c_long * 5)]


class _Client(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                ('display', C.c_void_p), ('window', C.c_ulong), ('message_type', C.c_ulong),
                ('format', C.c_int), ('data', _Data)]


class _Event(C.Union):
    _fields_ = [('client', _Client), ('pad', C.c_long * 24)]


class DesktopMode:
    def __init__(self):
        self.lib = C.CDLL(ctypes.util.find_library('X11') or 'libX11.so.6')
        for name, result, args in [
            ('XOpenDisplay', C.c_void_p, [C.c_char_p]),
            ('XDefaultRootWindow', C.c_ulong, [C.c_void_p]),
            ('XInternAtom', C.c_ulong, [C.c_void_p, C.c_char_p, C.c_int]),
            ('XSendEvent', C.c_int, [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.POINTER(_Event)]),
            ('XGetWindowProperty', C.c_int, [C.c_void_p, C.c_ulong, C.c_ulong, C.c_long, C.c_long,
                                            C.c_int, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                                            C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.c_void_p)]),
            ('XFree', C.c_int, [C.c_void_p]), ('XCloseDisplay', C.c_int, [C.c_void_p]),
        ]:
            fn = getattr(self.lib, name)
            fn.restype, fn.argtypes = result, args

    def access(self, desired=None):
        display = self.lib.XOpenDisplay(None)
        if not display:
            return None
        try:
            root = self.lib.XDefaultRootWindow(display)
            atom = self.lib.XInternAtom(display, b'_NET_SHOWING_DESKTOP', False)
            if desired is not None:
                event = _Event()
                event.client.type, event.client.send_event = 33, True
                event.client.display, event.client.window = display, root
                event.client.message_type, event.client.format = atom, 32
                event.client.data.l[0] = int(bool(desired))
                return bool(self.lib.XSendEvent(display, root, False, (1 << 19) | (1 << 20), C.byref(event)))
            actual, form, count, rest, data = C.c_ulong(), C.c_int(), C.c_ulong(), C.c_ulong(), C.c_void_p()
            result = self.lib.XGetWindowProperty(display, root, atom, 0, 1, False, 6,
                C.byref(actual), C.byref(form), C.byref(count), C.byref(rest), C.byref(data))
            try:
                if result == 0 and actual.value == 6 and form.value == 32 and count.value == 1 and data.value:
                    return bool(C.cast(data, C.POINTER(C.c_ulong))[0])
                return None
            finally:
                if data.value:
                    self.lib.XFree(data)
        finally:
            self.lib.XCloseDisplay(display)
