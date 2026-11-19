#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘运动服务回给 App 的提示一律中文（2026-11-19 用户：App 上不能出现英文提示）。

故障代码表（robot_core.motion_text）+ 真实 OidDriver 节点（替身 ROS、模拟驱动器，不转动实车轮子）的服务回复。
"""
import re
import unittest
from types import SimpleNamespace as NS

import test_driver_silent_timeout as fixtures  # 先导入：它把仓库各目录加进 sys.path
from robot_core import motion_text  # noqa: E402

LATIN = re.compile('[A-Za-z]')


class TestMotionText(unittest.TestCase):

    def test_health_faults_become_chinese(self):
        self.assertEqual(motion_text.fault_text('feedback_stale'), '底盘反馈超时')
        self.assertEqual(motion_text.fault_text('driver_silent:左轮'), '驱动器没有回应（左轮）')
        self.assertEqual(motion_text.fault_text('driver_hold:feedback_stale'), '底盘暂停（底盘反馈超时）')
        self.assertEqual(motion_text.fault_text('driver_fault:右轮驱动器：MOS 管过温'),
                         '驱动器故障（右轮驱动器：MOS 管过温）')     # 厂家故障名里的器件名照原样
        self.assertEqual(motion_text.fault_text('driver_silent:left'), '驱动器没有回应')
        self.assertEqual(motion_text.fault_text('something_new'), motion_text.UNKNOWN_TEXT)
        self.assertEqual(motion_text.faults_text(['feedback_stale', 'driver_silent:左轮', 'feedback_stale']),
                         '底盘反馈超时、驱动器没有回应（左轮）')
        self.assertEqual(motion_text.faults_text([]), '待确认')

    def test_every_known_code_has_chinese_text(self):
        for table in (motion_text.MODE_TEXT, motion_text.REASON_TEXT, motion_text.READY_TEXT,
                      motion_text.FAULT_TEXT, motion_text.FAULT_PREFIX_TEXT, motion_text.SOURCE_TEXT):
            for code, text in table.items():
                self.assertFalse(LATIN.search(text), (code, text))
        self.assertEqual(motion_text.status_text('ESTOP', 'operator_stop'), '急停锁存：已停车')
        self.assertEqual(motion_text.status_text('???', 'xyz'), '底盘状态异常：底盘状态异常')


class TestDriverRepliesChinese(unittest.TestCase):
    setUp = fixtures.TestGen5DriverSilence.setUp
    step = fixtures.TestGen5DriverSilence.step

    def assertChinese(self, response):
        self.assertTrue(response.message)
        self.assertFalse(LATIN.search(response.message), response.message)

    def test_stop_reset_arm_and_select_replies(self):
        self.faults = []
        self.step(4.0)
        n = self.node
        self.assertChinese(n.select_motion('navigation', NS(data=True), NS()))
        self.assertChinese(n.select_motion('navigation', NS(data=False), NS()))
        self.assertChinese(n.on_stop(None, NS()))
        self.assertChinese(n.on_arm(NS(data=True), NS()))           # 急停锁存中：请先复位
        self.assertChinese(n.select_motion('navigation', NS(data=True), NS()))
        self.step(2.0)
        reset = n.reset_motion(None, NS())
        self.assertTrue(reset.success, reset.message)
        self.assertChinese(reset)
        self.assertChinese(n.on_arm(NS(data=True), NS()))
        self.assertChinese(n.on_arm(NS(data=False), NS()))
        n.lidar_installed = False
        self.assertChinese(n.select_motion('follow', NS(data=True), NS()))  # 跟随要雷达


if __name__ == '__main__':
    unittest.main()
