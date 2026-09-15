#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QGuiApplication
app = QApplication([])
screen = QGuiApplication.primaryScreen()
pix = screen.grabWindow(0)
path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/heatmap-live.png'
ok = pix.save(path)
print('saved' if ok else 'failed', path, pix.width(), pix.height())
sys.exit(0 if ok else 1)
