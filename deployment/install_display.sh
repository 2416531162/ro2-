#!/bin/bash
# 车载屏幕：登录桌面后自动全屏打开只读显示页面（地图、卫星状态、连接状态）。只需在车上装一次；
# 窗口程序和页面都从 /opt/rk3588/current 读，换发布后自动用新的。
# 车载屏幕右下角「回到桌面」回 Ubuntu 桌面；桌面上点「车载屏幕」图标切回来。
# 卸载：删掉 ~/.config/autostart/rk3588-display.desktop、~/.local/share/applications/rk3588-display.desktop
# 和桌面上的「车载屏幕」图标。
set -euo pipefail
DIR="${HOME}/.config/autostart"
mkdir -p "$DIR"
cat > "$DIR/rk3588-display.desktop" <<'DESKTOP'
[Desktop Entry]
Type=Application
Name=车载屏幕
Comment=全屏只读显示路线地图、卫星状态和连接状态
Exec=/bin/bash -c 'while true; do /usr/bin/python3 /opt/rk3588/current/radar_system/car_display_window.py; sleep 3; done'
X-GNOME-Autostart-enabled=true
NoDisplay=true
DESKTOP
echo "已安装：$DIR/rk3588-display.desktop（下次登录桌面自动打开）"

# 从桌面切回车载屏幕：再启动一次窗口程序，单实例会转给已在运行的窗口，让它重新全屏显示
APPS="${HOME}/.local/share/applications"
mkdir -p "$APPS"
cat > "$APPS/rk3588-display.desktop" <<'DESKTOP'
[Desktop Entry]
Type=Application
Name=车载屏幕
Comment=切回全屏的车载屏幕（路线地图、卫星状态、连接状态）
Exec=/usr/bin/python3 /opt/rk3588/current/radar_system/car_display_window.py
Icon=video-display
Terminal=false
Categories=Utility;
DESKTOP
echo "已安装：$APPS/rk3588-display.desktop（应用列表里的「车载屏幕」，可右键「添加到收藏夹」固定到左侧栏）"

DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "${HOME}/Desktop")"
if [ -d "$DESK" ]; then
    cp "$APPS/rk3588-display.desktop" "$DESK/车载屏幕.desktop"
    chmod +x "$DESK/车载屏幕.desktop"
    # 桌面图标要标记为可信才能直接点开；不在图形会话里执行会失败，就请在桌面上右键图标选「允许启动」
    gio set "$DESK/车载屏幕.desktop" metadata::trusted true 2>/dev/null \
        || echo "提示：请在桌面上右键「车载屏幕」图标，选「允许启动」"
    echo "已放到桌面：$DESK/车载屏幕.desktop"
fi
