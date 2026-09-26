#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车载屏幕：开机登录桌面后全屏显示 http://127.0.0.1:8088/display（car_display.py 提供的只读页面）。

只做显示：没有地址栏、没有右键菜单、不能缩放，鼠标指针隐藏；窗口开着时告诉桌面不要息屏
（Gtk.Application.inhibit，不改系统的息屏和锁屏设置）。车端网页服务还没起来、重启发布、页面进程崩了
都会自己隔几秒重新打开。用系统自带的 WebKitGTK，不用另装浏览器。

回到 Ubuntu 桌面：点页面右下角「回到桌面」（页面通过 carDesktop 消息通知本窗口）；页面没加载出来时窗口
右下角另有同名按钮。窗口只是隐藏、进程不退出。从桌面再点「车载屏幕」图标（再启动一次本程序，
单实例会转给已在运行的窗口）就切回来。

    python3 /opt/rk3588/current/radar_system/car_display_window.py          # 在车的桌面里
    RK3588_DISPLAY_URL=http://127.0.0.1:8099/display python3 car_display_window.py   # 看演示车
"""
import os
import sys

import gi

gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
gi.require_version('WebKit2', '4.0')
from gi.repository import Gdk, GLib, Gtk, WebKit2  # noqa: E402
from display_control import ScreenControl  # noqa: E402
from screen_x11 import DesktopMode  # noqa: E402

URL = os.environ.get('RK3588_DISPLAY_URL', 'http://127.0.0.1:8088/display')
RETRY_S = 3


class DisplayWindow(Gtk.Application):
    def __init__(self):
        super().__init__(application_id='cn.rk3588.CarDisplay')
        self.view = None
        self.fallback = None
        self.retry = None
        self.window = None
        self.showing = False
        self.inhibit_cookie = 0
        self.control = ScreenControl()
        self.desktop = DesktopMode()
        self.applied_request = object()

    def do_activate(self):
        if self.view is not None:
            # 在桌面上又点了一次「车载屏幕」：切回全屏显示
            self.control.report(self.showing, self.applied_request)
            self.control.request('show')
            self._control_tick()
            return
        window = Gtk.ApplicationWindow(application=self, title='车载屏幕')
        self.window = window
        window.set_decorated(False)
        window.fullscreen()
        settings = WebKit2.Settings()
        settings.set_enable_webgl(True)
        settings.set_hardware_acceleration_policy(WebKit2.HardwareAccelerationPolicy.ALWAYS)
        settings.set_enable_developer_extras(False)
        settings.set_enable_write_console_messages_to_stdout(True)
        # 页面里「回到桌面」按钮发来的消息；只认这一个固定名字，消息内容不用
        content = WebKit2.UserContentManager()
        content.register_script_message_handler('carDesktop')
        content.connect('script-message-received::carDesktop', self._close_to_desktop)
        self.view = WebKit2.WebView(settings=settings, user_content_manager=content)
        self.view.set_background_color(Gdk.RGBA(0.094, 0.129, 0.157, 1.0))
        self.view.connect('context-menu', lambda *_: True)          # 不弹右键菜单
        self.view.connect('load-failed', self._failed)
        self.view.connect('web-process-terminated', self._failed)
        self.view.connect('load-changed', self._loaded)
        window.connect('realize', self._hide_cursor)
        window.connect('delete-event', self._close_to_desktop)
        # 页面没加载出来（网页服务没起来）时页面里没有按钮，窗口自己在右下角补一个
        self.fallback = Gtk.Button(label='回到桌面')
        self.fallback.set_halign(Gtk.Align.END)
        self.fallback.set_valign(Gtk.Align.END)
        self.fallback.set_margin_end(24)
        self.fallback.set_margin_bottom(24)
        self.fallback.set_size_request(180, 64)
        self.fallback.set_no_show_all(True)
        self.fallback.connect('clicked', self._close_to_desktop)
        overlay = Gtk.Overlay()
        overlay.add(self.view)
        overlay.add_overlay(self.fallback)
        window.add(overlay)
        self._control_tick()
        GLib.timeout_add(500, self._control_tick)

    def _close_to_desktop(self, *_):
        self.control.report(self.showing, self.applied_request)
        self.control.request('desktop')
        self._control_tick()
        return True

    def _control_tick(self):
        request = self.control.desired()
        if request['request_id'] != self.applied_request:
            self.showing = request['action'] == 'show'
            if self.showing:
                self.desktop.access(False)
                self.window.show_all()
                self.window.fullscreen()
                self.window.present()
                if not self.inhibit_cookie:
                    self.inhibit_cookie = self.inhibit(self.window, Gtk.ApplicationInhibitFlags.IDLE, '车载屏幕常亮显示')
                self.view.load_uri(URL)
            else:
                # 隐藏自有窗口而不退出进程，避免桌面自启动循环马上将其重新打开。
                if self.retry is not None:
                    GLib.source_remove(self.retry)
                    self.retry = None
                self.window.hide()
                self.desktop.access(True)
                self.view.stop_loading()
                self.view.load_uri('about:blank')
                if self.inhibit_cookie:
                    self.uninhibit(self.inhibit_cookie)
                    self.inhibit_cookie = 0
            self.applied_request = request['request_id']
        self.control.report(self.window.get_visible() and self.window.get_mapped(), self.applied_request,
                            desktop=self.desktop.access())
        return True

    def _hide_cursor(self, window):
        gdk_window = window.get_window()
        if gdk_window is not None:
            gdk_window.set_cursor(Gdk.Cursor.new_for_display(window.get_display(), Gdk.CursorType.BLANK_CURSOR))

    def _failed(self, *_):
        # 车端网页服务没起来或页面进程没了：过几秒重开，不弹错误页；这时页面里没有「回到桌面」，露出窗口自己的
        self.fallback.show()
        if self.showing and self.retry is None:
            self.retry = GLib.timeout_add_seconds(RETRY_S, self._reload)
        return True

    def _loaded(self, _view, event):
        # 页面开始显示内容了，用页面里的「回到桌面」
        if event == WebKit2.LoadEvent.COMMITTED and (self.view.get_uri() or '').startswith('http'):
            self.fallback.hide()

    def _reload(self):
        self.retry = None
        if self.showing:
            self.view.load_uri(URL)
        return False


def main():
    return DisplayWindow().run(sys.argv[:1])


if __name__ == '__main__':
    sys.exit(main())
