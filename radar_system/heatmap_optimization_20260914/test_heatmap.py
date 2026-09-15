#!/usr/bin/env python3
import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np

root = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1])
sys.path.insert(0, str(root))
spec = importlib.util.spec_from_file_location('board_radar_gui', root / 'board_radar_gui.py')
mod = importlib.util.module_from_spec(spec)
sys.modules['board_radar_gui'] = mod
spec.loader.exec_module(mod)

checks = []


def check(name, fn):
    try:
        fn()
        checks.append((name, True))
    except Exception as e:
        checks.append((name, False))
        print('FAIL ' + name + ': ' + str(e))


def assert_(cond, text='contract failed'):
    if not cond:
        raise AssertionError(text)


def load_pair():
    base = Path('/tmp/camera-baseline')
    depth = np.frombuffer((base / 'depth.bin').read_bytes(), dtype='<u2').reshape(480, 640).copy()
    rgb = np.frombuffer((base / 'rgb.bin').read_bytes(), dtype=np.uint8).reshape(480, 640, 3).copy()
    return depth, rgb


def source_unmutated():
    depth, rgb = load_pair()
    d0, r0 = depth.copy(), rgb.copy()
    img, near, far = mod.render_depth_heatmap(depth, rgb)
    assert_(np.array_equal(depth, d0), 'depth source mutated')
    assert_(np.array_equal(rgb, r0), 'rgb source mutated')
    assert_(img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8)
    assert_(img.shape[1] == 640 and img.shape[0] == 516)
    assert_(far - near >= 300)


def auto_range_spreads_colors():
    depth = np.zeros((80, 80), np.uint16)
    depth[:, :40] = 400
    depth[:, 40:] = 2400
    img, near, far = mod.render_depth_heatmap(depth, scale=1)
    left = img[20, 10].astype(int)
    right = img[20, 70].astype(int)
    assert_(abs(int(near) - 400) < 80 and abs(int(far) - 2400) < 80)
    assert_(np.abs(left - right).sum() > 80, 'near/far mapped to the same color')


def invalid_without_rgb_is_background():
    depth = np.zeros((40, 40), np.uint16)
    depth[10:30, 10:30] = 800
    img, _, _ = mod.render_depth_heatmap(depth, rgb=None, scale=1)
    bg = np.array(mod.HEAT_BG_RGB)
    assert_(np.array_equal(img[2, 2], bg))
    assert_(not np.array_equal(img[20, 20], bg))


def invalid_with_rgb_keeps_scene():
    depth, rgb = load_pair()
    img, _, _ = mod.render_depth_heatmap(depth, rgb)
    # Chair / holes are invalid in this capture; underlay must not be a flat background tile.
    crop = img[200:280, 200:320]
    assert_(np.std(crop.reshape(-1, 3), axis=0).mean() > 4, 'invalid region still a flat mosaic tile')


def center_measurement_untouched():
    depth, _ = load_pair()
    center = int(depth[240, 320])
    assert_(center > 0)
    img, _, _ = mod.render_depth_heatmap(depth)
    # Rendering must not be required to know the center sample; GUI reads the source pixel.
    assert_(int(depth[240, 320]) == center)


def ema_range_is_stable():
    a = np.array([400, 500, 2400, 2500], np.uint16)
    n1, f1 = mod.depth_auto_range(a)
    n2, f2 = mod.depth_auto_range(a, n1, f1)
    assert_(abs(n2 - n1) < 1e-6 and abs(f2 - f1) < 1e-6)
    b = np.array([800, 900, 1000, 1100], np.uint16)
    n3, f3 = mod.depth_auto_range(b, n1, f1, alpha=0.2)
    assert_(n3 > n1 and n3 < 800)


check('source_unmutated', source_unmutated)
check('auto_range_spreads_colors', auto_range_spreads_colors)
check('invalid_without_rgb_is_background', invalid_without_rgb_is_background)
check('invalid_with_rgb_keeps_scene', invalid_with_rgb_keeps_scene)
check('center_measurement_untouched', center_measurement_untouched)
check('ema_range_is_stable', ema_range_is_stable)

passed = sum(ok for _, ok in checks)
print(f'checks={len(checks)} passed={passed} failed={len(checks)-passed}')
sys.exit(0 if passed == len(checks) else 1)
