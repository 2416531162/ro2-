#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双天线测向的安装偏差 mount_offset_deg（车头 = 测向换算 + 安装偏差，见 docs/GNSS_FUSION.md）。
融合（line_node.load_fusion_config）和 rtk_node（/gnss/heading、状态里的测向车头）按同一条规则取值。"""
import user_rules as rules

# 2026-11-22 用户「那就补偿这1.7度」：这台 Gen5 的双天线测向比车实际直线行驶的方向小约 1.7°。
# 11-19 起这台车所有固定解直线段（2 m、陀螺 < 2°/s）：主天线轨迹走向 − 测向换算的车头 中位 +1.77°（87 段，
# 95% 区间 +1.64～+1.85°），往返两个方向、0.4–0.9 m/s、在喷不在喷都一样；两根天线各自固定解的连线与测向一致到 0.1°
# （接收机没加偏移）；用直线控制学到的 β 反推约 +1.65°。天线没动过、螺丝固定，车上 fusion.json 一直是没标定过的 0.0。
# 没补时每次重启后第一条线开头车头差约 2°（11-22 184902 线头 +2.3 cm、前 12 m 偏到 +4.7 cm）。
GEN5_MOUNT_OFFSET_DEG = rules.GEN5_MOUNT_OFFSET_DEG


def mount_offset(value):
    """标定文件里的 mount_offset_deg（没写为 None）→ 实际用的，度。文件里是 0 或没写时用
    GEN5_MOUNT_OFFSET_DEG；写了别的值照用（以后人工标定过的值优先）。车上的文件不改，回滚版本即恢复。"""
    if value is None or value == 0:
        return GEN5_MOUNT_OFFSET_DEG
    return 0.0 if value is None else float(value)
