#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPPI 的数组后端。

部署目标是 **Jetson Orin Nano + PyTorch CUDA**;这一层存在的唯一理由是让
MPPI 的数学本身能在没有 CUDA、甚至没有 torch 的机器上跑回归测试。没有它,
控制器就只能在实车上验证 —— 而实车上验证一个采样式控制器,代价是撞东西。

    backend = get_backend()              # 有 CUDA 就用,没有就退 CPU/numpy
    backend = get_backend('cpu')         # 强制 CPU,用于对拍
    backend = get_backend('numpy')       # 强制 numpy,用于无 torch 环境

numpy 路径**不是**第二个部署目标,不要在 RK3588 上指望它跑满帧。它存在是为了
测试可复现:同一份代价函数、同一份运动学,两条后端必须给出同样的结果,
tests/test_mppi.py 里有对拍。
"""

import math

try:                                    # pragma: no cover - 取决于机器
    import torch
    _HAS_TORCH = True
except Exception:                       # pragma: no cover
    torch = None
    _HAS_TORCH = False

import numpy as np


class Backend:
    """MPPI 用到的全部数组操作,只有这些。加新操作前先想想能不能不加。"""

    def __init__(self, kind, device=None, seed=None):
        self.kind = kind                # 'torch' | 'numpy'
        self.device = device
        self.is_gpu = bool(device is not None and 'cuda' in str(device))
        if kind == 'torch':
            self._gen = torch.Generator(device=device)
            if seed is not None:
                self._gen.manual_seed(int(seed))
        else:
            self._gen = np.random.default_rng(seed)

    # ---- 描述 ----

    def describe(self):
        if self.kind == 'torch':
            return f"torch/{self.device}"
        return "numpy/cpu"

    def seed(self, value):
        """按 tick 计数播种。采样式控制器不可复现的话,现场没法复盘。"""
        if self.kind == 'torch':
            self._gen.manual_seed(int(value))
        else:
            self._gen = np.random.default_rng(int(value))

    # ---- 构造 ----

    def array(self, data, dtype='f'):
        if self.kind == 'torch':
            return torch.as_tensor(np.asarray(data, dtype=np.float32),
                                   device=self.device)
        return np.asarray(data, dtype=np.float32)

    def zeros(self, shape):
        if self.kind == 'torch':
            return torch.zeros(shape, device=self.device, dtype=torch.float32)
        return np.zeros(shape, dtype=np.float32)

    def full(self, shape, value):
        if self.kind == 'torch':
            if isinstance(value, torch.Tensor):
                return torch.ones(shape, device=self.device, dtype=torch.float32) * value
            return torch.full(shape, float(value), device=self.device,
                              dtype=torch.float32)
        return np.full(shape, float(value), dtype=np.float32)

    def randn(self, shape):
        if self.kind == 'torch':
            return torch.randn(shape, generator=self._gen, device=self.device,
                               dtype=torch.float32)
        return self._gen.standard_normal(shape).astype(np.float32)

    # ---- 逐元素 ----

    def clip(self, x, lo, hi):
        if self.kind == 'torch':
            return torch.clamp(x, float(lo), float(hi))
        return np.clip(x, lo, hi)

    def where(self, cond, a, b):
        return torch.where(cond, a, b) if self.kind == 'torch' else np.where(cond, a, b)

    def exp(self, x):
        return torch.exp(x) if self.kind == 'torch' else np.exp(x)

    def sqrt(self, x):
        return torch.sqrt(x) if self.kind == 'torch' else np.sqrt(x)

    def abs(self, x):
        return torch.abs(x) if self.kind == 'torch' else np.abs(x)

    def cos(self, x):
        return torch.cos(x) if self.kind == 'torch' else np.cos(x)

    def sin(self, x):
        return torch.sin(x) if self.kind == 'torch' else np.sin(x)

    def tan(self, x):
        return torch.tan(x) if self.kind == 'torch' else np.tan(x)

    def atan2(self, y, x):
        return torch.atan2(y, x) if self.kind == 'torch' else np.arctan2(y, x)

    def hypot(self, x, y):
        return self.sqrt(x * x + y * y)

    def relu(self, x):
        zero = 0.0
        if self.kind == 'torch':
            return torch.clamp(x, min=zero)
        return np.maximum(x, zero)

    def sign(self, x):
        return torch.sign(x) if self.kind == 'torch' else np.sign(x)

    # ---- 规约与索引 ----

    def sum(self, x, axis=None):
        if self.kind == 'torch':
            return torch.sum(x) if axis is None else torch.sum(x, dim=axis)
        return np.sum(x, axis=axis)

    def amin(self, x, axis=None):
        if self.kind == 'torch':
            return torch.amin(x) if axis is None else torch.amin(x, dim=axis)
        return np.min(x, axis=axis)

    def amax(self, x, axis=None):
        if self.kind == 'torch':
            return torch.amax(x) if axis is None else torch.amax(x, dim=axis)
        return np.max(x, axis=axis)

    def argmin(self, x):
        return int(torch.argmin(x)) if self.kind == 'torch' else int(np.argmin(x))

    def take(self, table, index):
        """一维 gather。index 必须已经钳进合法范围。"""
        if self.kind == 'torch':
            return torch.take(table, index)
        return table[index]

    def to_long(self, x):
        return x.long() if self.kind == 'torch' else x.astype(np.int64)

    def stack(self, arrays, axis=0):
        if self.kind == 'torch':
            return torch.stack(arrays, dim=axis)
        return np.stack(arrays, axis=axis)

    def roll_forward(self, seq):
        """把控制序列整体前移一步,末尾复制 —— MPPI 的热启动。"""
        if self.kind == 'torch':
            return torch.cat([seq[1:], seq[-1:]], dim=0)
        return np.concatenate([seq[1:], seq[-1:]], axis=0)

    # ---- 出入口 ----

    def item(self, x):
        return float(x.item()) if self.kind == 'torch' else float(x)

    def to_numpy(self, x):
        if self.kind == 'torch':
            return x.detach().cpu().numpy()
        return np.asarray(x)

    def synchronize(self):
        if self.is_gpu:
            torch.cuda.synchronize(self.device)


def torch_available():
    return _HAS_TORCH


def cuda_available():
    return bool(_HAS_TORCH and torch.cuda.is_available())


def get_backend(prefer='auto', seed=None):
    """prefer: 'auto' | 'cuda' | 'cpu' | 'numpy'

    'auto' 的顺序是 CUDA -> torch CPU -> numpy。实车上拿到的应当是 CUDA;
    如果日志里打出来不是,说明 torch 没装对,**不要**就这么跑跟随 ——
    CPU 上 K=2048 的 rollout 会把控制周期撑爆,而刹车包络是按周期算的。
    """
    if prefer == 'cuda':
        # 明确要 CUDA 却悄悄降级到 CPU/numpy 是最坏的失败方式:操作员以为
        # 自己在 GPU 上跑,实际求解耗时把控制周期撑爆,而刹车包络是按周期
        # 算的 —— 车会以为自己刹得住。宁可起不来。
        if not _HAS_TORCH:
            raise RuntimeError("要求 CUDA 后端但没有安装 torch")
        if not torch.cuda.is_available():
            raise RuntimeError("要求 CUDA 后端但 torch.cuda 不可用")
        return Backend('torch', device=torch.device('cuda'), seed=seed)
    if prefer == 'numpy' or not _HAS_TORCH:
        return Backend('numpy', seed=seed)
    if prefer == 'auto' and torch.cuda.is_available():
        return Backend('torch', device=torch.device('cuda'), seed=seed)
    return Backend('torch', device=torch.device('cpu'), seed=seed)
