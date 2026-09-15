#!/usr/bin/env python3
"""Compare the current mosaic colormap with a display-only smooth heatmap."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 5500
HEAT_BG_BGR = (24, 14, 18)


def current_heatmap(depth):
    valid = (depth > 200) & (depth <= 5500)
    clipped = np.clip(depth, 350, 4500).astype(np.float32)
    norm = ((4500.0 - clipped) / (4500.0 - 350.0) * 255.0).astype(np.uint8)
    norm_smooth = cv2.medianBlur(norm, 3)
    colored = cv2.applyColorMap(norm_smooth, cv2.COLORMAP_TURBO)
    colored[~valid] = HEAT_BG_BGR
    return colored


def smooth_heatmap(depth, near_mm=None, far_mm=None, scale=2):
    depth = depth.astype(np.float32, copy=False)
    valid = (depth > DEPTH_MIN_MM) & (depth <= DEPTH_MAX_MM)
    h, w = depth.shape
    if not np.any(valid):
        img = np.full((h * scale, w * scale, 3), HEAT_BG_BGR, np.uint8)
        return img, 0.0, 0.0, 0.0

    vals = depth[valid]
    if near_mm is None:
        near_mm = float(np.percentile(vals, 2))
    if far_mm is None:
        far_mm = float(np.percentile(vals, 98))
    span = max(300.0, far_mm - near_mm)
    far_mm = near_mm + span

    unit = np.zeros((h, w), np.float32)
    unit[valid] = np.clip((far_mm - depth[valid]) / span, 0.0, 1.0)
    u8 = np.clip(unit * 255.0, 0, 255).astype(np.uint8)

    mask = (valid.astype(np.uint8) * 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    holes = (closed > 0) & (~valid)
    if np.any(holes):
        dilated = cv2.dilate(u8, kernel)
        u8 = np.where(holes, dilated, u8).astype(np.uint8)
        display_valid = closed
    else:
        display_valid = mask

    u8 = cv2.bilateralFilter(u8, d=9, sigmaColor=28, sigmaSpace=9)
    out_w, out_h = w * scale, h * scale
    u8_up = cv2.resize(u8, (out_w, out_h), interpolation=cv2.INTER_CUBIC)
    valid_up = cv2.resize(display_valid, (out_w, out_h), interpolation=cv2.INTER_LINEAR)
    colored = cv2.applyColorMap(u8_up, cv2.COLORMAP_TURBO)
    colored[valid_up < 80] = HEAT_BG_BGR
    return colored, near_mm, far_mm, float(np.mean(valid))


def add_legend(bgr, near_mm, far_mm):
    bar_h = 42
    h, w = bgr.shape[:2]
    canvas = np.full((h + bar_h, w, 3), HEAT_BG_BGR, np.uint8)
    canvas[:h] = bgr
    lut = cv2.applyColorMap(np.arange(255, -1, -1, dtype=np.uint8).reshape(1, 256), cv2.COLORMAP_TURBO)
    bar = cv2.resize(lut, (w - 160, 12), interpolation=cv2.INTER_LINEAR)
    canvas[h + 8:h + 20, 16:16 + bar.shape[1]] = bar
    cv2.putText(canvas, f'{near_mm/1000:.2f}m near', (16, h + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 230, 240), 1, cv2.LINE_AA)
    cv2.putText(canvas, f'{far_mm/1000:.2f}m far', (w - 150, h + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 230, 240), 1, cv2.LINE_AA)
    return canvas


def load_depth(path):
    raw = Path(path).read_bytes()
    return np.frombuffer(raw, dtype='<u2').reshape(480, 640)


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/tmp/camera-baseline/depth.bin')
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path('/tmp/heatmap-opt')
    out.mkdir(parents=True, exist_ok=True)
    depth = load_depth(src)
    valid = (depth > 200) & (depth <= 5500)
    print('shape', depth.shape, 'valid', float(valid.mean()), 'unique', int(len(np.unique(depth))),
          'p2', float(np.percentile(depth[valid], 2)) if np.any(valid) else None,
          'p98', float(np.percentile(depth[valid], 98)) if np.any(valid) else None)
    old = current_heatmap(depth)
    new, near, far, frac = smooth_heatmap(depth)
    print('auto_range_mm', near, far, 'valid', frac)
    cv2.imwrite(str(out / 'before.png'), old)
    cv2.imwrite(str(out / 'after.png'), add_legend(new, near, far))
    # side by side at display-ish width
    old_up = cv2.resize(old, (new.shape[1], new.shape[0]), interpolation=cv2.INTER_NEAREST)
    pair = np.concatenate([old_up, new], axis=1)
    cv2.imwrite(str(out / 'compare.png'), pair)
    print('wrote', out)


if __name__ == '__main__':
    main()
