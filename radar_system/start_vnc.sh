#!/bin/bash
set -e
mkdir -p /root/.vnc
VNC_PASS="${VNC_PASS:-1234}"
[ -f /root/.vnc/passwd ] || x11vnc -storepasswd "$VNC_PASS" /root/.vnc/passwd >/dev/null
modprobe uinput 2>/dev/null || true
chmod 660 /dev/uinput 2>/dev/null || chmod 666 /dev/uinput 2>/dev/null || true
killall x11vnc 2>/dev/null || true
export DISPLAY=:0
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
[ -r "$XAUTHORITY" ] || export XAUTHORITY=/run/user/1000/gdm/Xauthority
xhost +local:root >/dev/null 2>&1 || true
nohup x11vnc -display :0 -auth "$XAUTHORITY" \
  -localhost -forever -shared -noxdamage -ncache 0 -repeat \
  -cursor most -rfbport 5900 -rfbauth /root/.vnc/passwd \
  -pipeinput UINPUT -noxrecord -scale 2/3 \
  -o /tmp/x11vnc.log >/tmp/x11vnc.out 2>&1 &
echo "x11vnc pid $!  scale=2/3  (TigerVNC: 127.0.0.1:5900)"
