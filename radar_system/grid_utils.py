#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达扇区与占据栅格的纯计算helpers。

从 radar_web_server.py 里抽出来,不依赖 ROS,因此可以直接跑单元测试。
这两段逻辑都有容易写错的边界情况:扇区跨越 0 度、地图全空、抽样步长上限。
"""

import math

try:
    import numpy as np
except ImportError:      # pragma: no cover - 部署环境一定有 numpy
    np = None

__all__ = ["clean_ranges", "sector_min", "downsample_step", "extract_grid_points"]


def clean_ranges(ranges, range_min, range_max):
    """把一圈距离读数转成 float32 数组,无效值置为 NaN。

    无效包括:非有限值、超出量程上下界。返回 (数组, 有效掩码)。
    """
    if np is None:
        raise RuntimeError("clean_ranges 需要 numpy")
    arr = np.asarray(ranges, dtype=np.float32)
    ok = np.isfinite(arr) & (arr > range_min) & (arr < range_max)
    return np.where(ok, arr, np.nan), ok


def sector_min(clean, start_deg, end_deg, empty=99.0, angle_min_deg=0.0):
    """取某个角度扇区内的最小距离,自动处理跨越 0 度的情况。

    clean 是 clean_ranges() 的输出(无效值已是 NaN)。

    **angle_min_deg 必须传对**,否则整圈数据会整体旋转。LaserScan 的第 0 个
    光束指向 msg.angle_min,而不是 0°。N10P 按惯例发布 angle_min = -π,
    这时光束序号 0 指向**车尾**:

        i = (目标方位 - angle_min) / 角分辨率

    改造前这里写的是 `int(deg / 360 * n)`,等于假设 angle_min = 0。
    实车上这让「正前方测距」读的其实是车尾,于是雷达扫到车自己的车身,
    被当成正前方 0.17m 的障碍物,AEB 一直硬刹停。

    >>> import numpy as np
    >>> a = np.array([5.0, 4.0, 3.0, 2.0], dtype=np.float32)
    >>> round(sector_min(a, 0, 180), 3)      # 前半圈 [5, 4]
    4.0
    >>> round(sector_min(a, 180, 360), 3)    # 后半圈 [3, 2]
    2.0
    >>> round(sector_min(a, 270, 90), 3)     # 跨 0 度: [2, 5]
    2.0
    >>> sector_min(np.array([np.nan, np.nan], dtype=np.float32), 0, 180)
    99.0
    >>> round(sector_min(a, 0, 180, angle_min_deg=-180), 3)   # 光束 0 指向车尾
    2.0
    """
    if np is None:
        raise RuntimeError("sector_min 需要 numpy")
    n = int(clean.shape[0])
    if n == 0:
        return empty
    i0 = int(round((start_deg - angle_min_deg) / 360.0 * n)) % n
    i1 = int(round((end_deg - angle_min_deg) / 360.0 * n)) % n
    if i0 == i1:
        seg = clean                      # 整圈
    elif i0 < i1:
        seg = clean[i0:i1]
    else:
        seg = np.concatenate((clean[i0:], clean[:i1]))   # 跨越 0 度
    if seg.size == 0 or not bool(np.any(np.isfinite(seg))):
        return empty
    return float(np.nanmin(seg))


def downsample_step(width, height, max_points, start=2, limit=32):
    """按地图尺寸选一个抽样步长,使输出点数大致不超过 max_points。

    固定 step=2 在大地图上会吐出几十万个点:序列化与前端渲染都吃不消,
    而且整个提取过程持有 GIL,会把同进程的 HTTP 线程一起卡住。

    >>> downsample_step(200, 200, 12000)
    2
    >>> downsample_step(800, 800, 12000)
    6
    >>> downsample_step(4000, 4000, 12000)
    26
    """
    step = start
    while (width // step) * (height // step) > max_points * 2 and step < limit:
        step += 1
    return step


def extract_grid_points(data, width, height, step):
    """从占据栅格里抽出障碍点与自由点,返回 (obstacles, frees)。

    栅格约定 (nav_msgs/OccupancyGrid): 100 = 障碍, 0 = 自由, -1 = 未知。
    坐标以**原始栅格**为单位(已乘回 step),前端不需要知道抽样倍率。
    """
    if width <= 0 or height <= 0 or step <= 0:
        return [], []
    if np is not None:
        try:
            flat = np.frombuffer(data, dtype=np.int8)       # array.array / bytes:零拷贝
        except (TypeError, BufferError):
            flat = np.asarray(data, dtype=np.int8)          # 退化:普通 list
        if flat.size < width * height:
            return [], []
        grid = flat[:width * height].reshape(height, width)[::step, ::step]
        oy, ox = np.nonzero(grid == 100)
        fy, fx = np.nonzero(grid == 0)
        return (np.stack((ox * step, oy * step), axis=1).tolist(),
                np.stack((fx * step, fy * step), axis=1).tolist())

    obstacles, frees = [], []
    for gy in range(0, height, step):
        row = gy * width
        for gx in range(0, width, step):
            val = data[row + gx]
            if val == 100:
                obstacles.append([gx, gy])
            elif val == 0:
                frees.append([gx, gy])
    return obstacles, frees


if __name__ == "__main__":
    import array
    import doctest
    import random
    import time
    failures, _ = doctest.testmod()

    print("地图尺寸        步长   点数     耗时")
    for w, h in ((200, 200), (400, 400), (800, 800), (1200, 1200)):
        data = array.array('b', [random.choice([-1, 0, 0, 0, 100]) for _ in range(w * h)])
        st = downsample_step(w, h, 12000)
        t0 = time.perf_counter()
        o, f = extract_grid_points(data, w, h, st)
        dt = (time.perf_counter() - t0) * 1000
        print(f"{w}x{h:<10} {st:>3}  {len(o)+len(f):>7}  {dt:7.1f} ms")
    raise SystemExit(1 if failures else 0)
