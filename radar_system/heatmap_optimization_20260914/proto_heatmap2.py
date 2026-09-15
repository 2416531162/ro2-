#!/usr/bin/env python3
import sys
from pathlib import Path
import cv2
import numpy as np

DEPTH_MIN_MM, DEPTH_MAX_MM = 200, 5500
HEAT_BG = np.array([18, 14, 24], np.float32)  # RGB


def load_depth(path):
    return np.frombuffer(Path(path).read_bytes(), dtype='<u2').reshape(480, 640)


def load_rgb(path):
    return np.frombuffer(Path(path).read_bytes(), dtype=np.uint8).reshape(480, 640, 3)


def render(depth, rgb=None, scale=2, blend=0.35):
    depth = depth.astype(np.float32, copy=False)
    valid = (depth > DEPTH_MIN_MM) & (depth <= DEPTH_MAX_MM)
    h, w = depth.shape
    vals = depth[valid]
    near = float(np.percentile(vals, 2))
    far = float(np.percentile(vals, 98))
    span = max(300.0, far - near)
    far = near + span

    unit = np.zeros((h, w), np.float32)
    unit[valid] = np.clip((far - depth[valid]) / span, 0.0, 1.0)
    u8 = np.clip(unit * 255.0, 0, 255).astype(np.uint8)
    mask = valid.astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    holes = (closed > 0) & (~valid)
    if np.any(holes):
        u8 = np.where(holes, cv2.dilate(u8, kernel), u8).astype(np.uint8)
    u8 = cv2.bilateralFilter(u8, d=9, sigmaColor=32, sigmaSpace=9)

    out_w, out_h = w * scale, h * scale
    u8_up = cv2.resize(u8, (out_w, out_h), interpolation=cv2.INTER_CUBIC)
    alpha = cv2.resize(closed, (out_w, out_h), interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
    alpha = np.clip((alpha - 0.15) / 0.55, 0.0, 1.0)
    alpha = cv2.GaussianBlur(alpha, (0, 0), 1.2)

    heat_bgr = cv2.applyColorMap(u8_up, cv2.COLORMAP_TURBO)
    heat = cv2.cvtColor(heat_bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
    if rgb is not None:
        rgb_up = cv2.resize(rgb, (out_w, out_h), interpolation=cv2.INTER_LINEAR).astype(np.float32)
        heat = heat * (1.0 - blend) + rgb_up * blend

    bg = np.broadcast_to(HEAT_BG, heat.shape)
    a = alpha[..., None]
    out = heat * a + bg * (1.0 - a)
    return np.clip(out, 0, 255).astype(np.uint8), near, far


def legend(rgb, near, far):
    bar_h = 44
    h, w = rgb.shape[:2]
    canvas = np.full((h + bar_h, w, 3), HEAT_BG, np.uint8)
    canvas[:h] = rgb
    lut = cv2.applyColorMap(np.arange(255, -1, -1, dtype=np.uint8).reshape(1, 256), cv2.COLORMAP_TURBO)
    lut = cv2.cvtColor(lut, cv2.COLOR_BGR2RGB)
    bar = cv2.resize(lut, (max(64, w - 180), 12), interpolation=cv2.INTER_LINEAR)
    canvas[h + 8:h + 20, 16:16 + bar.shape[1]] = bar
    return canvas


def main():
    depth = load_depth(sys.argv[1] if len(sys.argv) > 1 else '/tmp/camera-baseline/depth.bin')
    rgb = load_rgb(sys.argv[2] if len(sys.argv) > 2 else '/tmp/camera-baseline/rgb.bin')
    out = Path(sys.argv[3] if len(sys.argv) > 3 else '/tmp/heatmap-opt2')
    out.mkdir(parents=True, exist_ok=True)
    pure, near, far = render(depth, None, blend=0)
    mixed, _, _ = render(depth, rgb, blend=0.38)
    cv2.imwrite(str(out / 'pure.png'), cv2.cvtColor(legend(pure, near, far), cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(out / 'mixed.png'), cv2.cvtColor(legend(mixed, near, far), cv2.COLOR_RGB2BGR))
    print('range', near, far, 'wrote', out)


if __name__ == '__main__':
    main()
