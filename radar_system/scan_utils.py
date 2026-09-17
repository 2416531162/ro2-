#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""雷达扇区 NumPy 计算（无 ROS 依赖）。"""

import math

try:
    import numpy as np
except ImportError:      # pragma: no cover - 部署环境一定有 numpy
    np = None

__all__ = ["clean_ranges", "sector_min"]


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

