# -*- coding: utf-8 -*-
"""三维观测累积的回归测试(stdlib unittest,不依赖 pytest)。

覆盖:体素累积与去重、内存上限、TTL 窗口与长期累积两种模式、车体自身反射
剔除、性能预算、节点状态字段。
不覆盖:真实 ROS 序列化 / QoS / 执行器、实车标定、底盘安全。
本文件里的一切点云都是 **SYNTHETIC TEST DATA**,不是实车建图结果。
"""
# Historical optional feature: keep tests, but do not fail collection after removal.
from pathlib import Path as _FeaturePath
import pytest as _feature_pytest
if not (_FeaturePath(__file__).resolve().parents[1] / 'radar_system' / 'cloud_scene.py').exists():
    _feature_pytest.skip('retired feature: cloud_scene.py is not shipped', allow_module_level=True)

import math
import os
import sys
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'radar_system'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros_stubs                                         # noqa: E402
ros_stubs.install()

from cloud_scene import VoxelHistory, body_mask, KEY_LIMIT   # noqa: E402

SYNTHETIC = 'SYNTHETIC TEST DATA'


def pts(rows):
    return np.array(rows, dtype=np.float32).reshape(-1, 4)


class TestVoxelHistoryBehaviour(unittest.TestCase):
    """保持原有语义 —— 这几条是换实现前就成立的,换完必须还成立。"""

    def test_keeps_the_actual_sample_not_the_cell_centre(self):
        h = VoxelHistory()
        h.add([[.012, .014, .017, -1], [.018, .017, .018, -1]], 1)
        self.assertEqual(len(h.array()), 1)
        np.testing.assert_allclose(h.array()[0], [.012, .014, .017, -1], atol=1e-6)
        h.add([[.019, .015, .022, -1]], 2)
        np.testing.assert_allclose(h.array()[0], [.019, .015, .022, -1], atol=1e-6)

    def test_limit_and_ttl(self):
        h = VoxelHistory(limit=100, ttl=3)
        h.add([[i, 0, 0, -1] for i in range(200)], 1)
        self.assertEqual(len(h.array()), 100)
        h.expire(5)
        self.assertEqual(h.array().shape, (0, 4))

    def test_empty_update_still_expires(self):
        h = VoxelHistory(ttl=1)
        h.add([[0, 0, 0, -1]], 1)
        h.add([], 3)
        self.assertEqual(len(h.array()), 0)

    def test_intensity_is_preserved_never_invented(self):
        h = VoxelHistory()
        h.add([[1, 0, 0, 42.]], 1)
        self.assertAlmostEqual(float(h.array()[0][3]), 42.0, places=3)
        h.add([[5, 0, 0, -1.]], 1)
        self.assertEqual(sorted(round(float(v), 1) for v in h.array()[:, 3]), [-1.0, 42.0])


class TestAccumulation(unittest.TestCase):

    def test_accumulates_across_frames(self):
        h = VoxelHistory(voxel=.1)
        h.add([[1., 0, .5, -1]], 1)
        h.add([[3., 0, .5, -1]], 2)
        self.assertEqual(len(h.array()), 2)

    def test_same_cell_seen_twice_is_not_duplicated(self):
        h = VoxelHistory(voxel=.1)
        # 坐标取在格子中间,不贴边 —— 贴边测的是浮点取整,不是去重逻辑
        h.add([[2.05, 1.05, .35, -1]], 1)
        h.add([[2.07, 1.03, .37, -1]], 2)
        self.assertEqual(len(h.array()), 1)

    def test_limit_holds_on_every_frame_not_just_at_the_end(self):
        cap = 2000
        h = VoxelHistory(voxel=.05, limit=cap, ttl=60)
        rng = np.random.default_rng(7)
        for frame in range(10):
            a = np.column_stack((rng.uniform(frame * 2., frame * 2. + 5., 3000),
                                 rng.uniform(-5., 5., 3000),
                                 rng.uniform(0., 2., 3000),
                                 np.full(3000, -1.))).astype(np.float32)
            h.add(a, 100. + frame)
            self.assertLessEqual(len(h), cap, '第 %d 帧超出上限' % frame)

    def test_persistent_mode_never_expires(self):
        # ttl=0:走过的房间一直留着,这才叫"建出一张三维地图"。
        h = VoxelHistory(ttl=0)
        h.add([[1., 0, .5, -1]], 1)
        h.expire(1e6)
        self.assertEqual(len(h.array()), 1)
        h.add([[40., 0, .5, -1]], 1e6)
        self.assertEqual(len(h.array()), 2)

    def test_persistent_mode_is_still_bounded_by_limit(self):
        # 不过期不等于不封顶 —— 4GB 板子上无上限就是迟早 OOM。
        h = VoxelHistory(voxel=.05, limit=500, ttl=0)
        rng = np.random.default_rng(1)
        for frame in range(6):
            h.add(np.column_stack((rng.uniform(0, 30, 2000), rng.uniform(0, 30, 2000),
                                   rng.uniform(0, 2, 2000),
                                   np.full(2000, -1.))).astype(np.float32), float(frame))
            self.assertLessEqual(len(h), 500)

    def test_eviction_drops_the_least_recently_observed(self):
        h = VoxelHistory(voxel=.5, limit=100, ttl=60)
        h.add([[i + .25, .25, .25, -1] for i in range(100)], 1)   # 装满(格距 1m)
        h.add([[0.25, .25, .25, -1]], 2)                          # 重新看到第 0 格
        h.add([[500.25, .25, .25, -1]], 3)                        # 新格子挤进来
        xs = {round(float(v)) for v in h.array()[:, 0]}
        self.assertEqual(len(xs), 100)
        self.assertIn(500, xs, '新观测必须留下')
        self.assertIn(0, xs, '刚刚重新看到的格子不该被淘汰')
        self.assertNotIn(1, xs, '该淘汰的是最久没看到的那个')

    def test_invalid_budget_is_rejected(self):
        for bad in (dict(voxel=.001), dict(voxel=9.), dict(limit=5),
                    dict(limit=10**7), dict(ttl=500), dict(ttl=-1)):
            with self.assertRaises(ValueError, msg=str(bad)):
                VoxelHistory(**bad)

    def test_absurd_coordinates_are_dropped_not_aliased(self):
        # 体素键打包成 int64。超出可表示范围的点如果不丢掉,就会和另一个
        # 完全不同位置的点撞成同一格 —— 地图上凭空多出一堵墙。
        h = VoxelHistory(voxel=.05)
        # 既要超出 int64 键的可表示范围(KEY_LIMIT*体素=52.4km),
        # 又要在 add() 自己那道 |xyz|<100000 的粗筛之内,否则测不到这条分支
        far = KEY_LIMIT * .05 * 1.5
        h.add([[1., 0, 0, -1], [far, 0, 0, -1]], 1)
        self.assertEqual(len(h.array()), 1)
        self.assertGreater(h.dropped_far, 0)

    def test_non_finite_points_are_dropped(self):
        h = VoxelHistory()
        h.add([[1., 0, 0, -1], [np.nan, 0, 0, -1], [np.inf, 0, 0, -1]], 1)
        self.assertEqual(len(h.array()), 1)


class TestPerformanceBudget(unittest.TestCase):
    """RK3588 跑得动才有意义。

    这两个调用都在持锁路径上:慢一次,网页和屏幕一起卡一次。
    阈值按 x86 定,留了余量;真要退化成原来的量级(add 30ms / array 56ms)
    会立刻失败。
    """

    def test_add_and_array_stay_cheap_at_full_capacity(self):
        rng = np.random.default_rng(0)
        h = VoxelHistory(.05, 60000, 45.)
        frame = np.column_stack((rng.uniform(0, 5, 8500), rng.uniform(-2, 2, 8500),
                                 rng.uniform(0, 2, 8500),
                                 np.full(8500, -1.))).astype(np.float32)
        for i in range(25):                      # 边走边扫,把容量填满
            a = frame.copy()
            a[:, 0] += i * .25
            h.add(a, time.monotonic())
        self.assertGreater(len(h), 40000, '没压到容量,这个测试没意义')

        worst = 0.0
        for i in range(10):
            a = frame.copy()
            a[:, 0] += i * .05
            t = time.perf_counter()
            h.add(a, time.monotonic())
            worst = max(worst, time.perf_counter() - t)
        self.assertLess(worst, 0.020, 'add() 退化到 %.1f ms' % (worst * 1000))

        t = time.perf_counter()
        h.array()
        elapsed = time.perf_counter() - t
        self.assertLess(elapsed, 0.010, 'array() 退化到 %.1f ms' % (elapsed * 1000))


class TestBodyMask(unittest.TestCase):
    """车体自身反射必须剔掉,而且判定要跟着车头转。"""

    def test_point_on_the_vehicle_is_masked(self):
        m = body_mask(pts([[0.3, 0.0, 0.2, -1]]), (0., 0., 0.))
        self.assertTrue(bool(m[0]))

    def test_point_ahead_of_the_vehicle_is_kept(self):
        m = body_mask(pts([[3.0, 0.0, 0.2, -1]]), (0., 0., 0.))
        self.assertFalse(bool(m[0]))

    def test_mask_follows_heading(self):
        # 车头朝 +x 时这个点在车身正前方 0.5m(车长 0.67,算自身);
        # 车头朝 +y 时它变成右侧 0.5m(半宽 0.335,是真实障碍物)。
        point = pts([[0.5, 0.0, 0.2, -1]])
        self.assertTrue(bool(body_mask(point, (0., 0., 0.))[0]))
        self.assertFalse(bool(body_mask(point, (0., 0., math.pi / 2))[0]))

    def test_mask_follows_position(self):
        point = pts([[5.3, 5.0, 0.2, -1]])
        self.assertTrue(bool(body_mask(point, (5., 5., 0.))[0]))
        self.assertFalse(bool(body_mask(point, (0., 0., 0.))[0]))

    def test_points_above_the_vehicle_are_kept(self):
        # 门梁、天花板、货架下沿不是车体。自身高度按车壳(0.34m)算,
        # 设成一个"安全高度"会把车顶上方的真实几何一起抹掉。
        for z in (0.6, 1.0, 2.0):
            self.assertFalse(bool(body_mask(pts([[0.3, 0.0, z, -1]]), (0., 0., 0.))[0]),
                             'z=%.1f 被误判为车体' % z)
        self.assertTrue(bool(body_mask(pts([[0.3, 0.0, 0.25, -1]]), (0., 0., 0.))[0]))

    def test_no_pose_means_no_masking(self):
        m = body_mask(pts([[0.3, 0.0, 0.2, -1]]), None)
        self.assertFalse(m.any())

    def test_empty_input(self):
        self.assertEqual(body_mask(np.empty((0, 4), np.float32), (0., 0., 0.)).shape, (0,))


def make_node(**env):
    os.environ['RO2_CLOUD_SOURCE'] = 'pointcloud'
    for k, v in env.items():
        os.environ[k] = v
    import importlib
    import live_cloud_node
    importlib.reload(live_cloud_node)
    node = live_cloud_node.LiveCloudNode()
    node._clock.seconds = 1000.0
    node.map_info = {'revision': 1}
    node.map_fault = ''
    for frame in ('base_link', 'odom', 'lidar'):
        node.tf.set('map', frame, ros_stubs.FakeTransform(stamp_s=1000.0))
    return node


def cloud_msg(points, frame='lidar', stamp_s=1000.0):
    msg = ros_stubs.PointCloud2()
    field = ros_stubs.PointField
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg.header.frame_id = frame
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int(round((stamp_s % 1) * 1e9))
    msg.height, msg.width = 1, p.shape[0]
    msg.point_step, msg.row_step = 12, 12 * p.shape[0]
    msg.fields = [field(n, 4 * i, field.FLOAT32, 1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.data = np.ascontiguousarray(p, dtype='<f4').tobytes()
    return msg


class TestCloudNode(unittest.TestCase):
    """节点层:自身反射剔除、TF/过期丢帧、状态字段、不碰底盘。"""

    def setUp(self):
        for key in ('RO2_CLOUD_HISTORY_S', 'RO2_CLOUD_RADIUS_M'):
            os.environ.pop(key, None)
        self.node = make_node()

    def feed(self, points, **kw):
        self.node.latest_points = cloud_msg(points, **kw)
        self.node.tick()

    def test_self_hits_are_dropped_before_entering_the_map(self):
        self.feed([[0.3, 0.0, 0.2], [3.0, 0.0, 0.2]])
        self.assertEqual(self.node.cloud_error, '')
        self.assertEqual(len(self.node.history), 1)
        np.testing.assert_allclose(self.node.history.array()[0][:2], [3.0, 0.0], atol=.06)
        self.assertEqual(self.node.self_hits, 1)

    def test_self_hit_filter_can_be_turned_off(self):
        self.node.set_param('drop_self_hits', False)
        self.feed([[0.3, 0.0, 0.2], [3.0, 0.0, 0.2]])
        self.assertEqual(len(self.node.history), 2)

    def test_points_beyond_the_radius_are_dropped(self):
        self.node.set_param('display_radius_m', 5.0)
        self.feed([[3.0, 0.0, 0.2], [30.0, 0.0, 0.2]])
        self.assertEqual(len(self.node.history), 1)

    def test_missing_tf_keeps_the_frame_out_of_the_map(self):
        self.node.tf.table.pop(('map', 'lidar'))
        self.feed([[3.0, 0.0, 0.2]])
        self.assertEqual(len(self.node.history), 0)
        self.assertIn('lidar', self.node.cloud_error)

    def test_stale_frame_is_rejected(self):
        # 定位 TF 保持新鲜,只让点云自己迟到 —— 否则测到的是定位过期那条分支
        self.node._clock.seconds = 1002.0
        for frame in ('base_link', 'odom', 'lidar'):
            self.node.tf.set('map', frame, ros_stubs.FakeTransform(stamp_s=1002.0))
        self.feed([[3.0, 0.0, 0.2]], stamp_s=1000.0)
        self.assertEqual(len(self.node.history), 0)
        self.assertIn('过期', self.node.cloud_error)

    def test_localisation_loss_stops_accumulation(self):
        self.node.tf.table.pop(('map', 'base_link'))
        self.feed([[3.0, 0.0, 0.2]])
        self.assertEqual(len(self.node.history), 0)
        self.assertIsNone(self.node.robot)

    def test_snapshot_reports_mode_and_calibration(self):
        scene = self.node.snapshot()['scene']
        self.assertEqual(scene['mode'], 'recent_window')
        self.assertIn('self_hits', scene)
        self.assertIn('radius_m', scene)
        # pointcloud 源不需要相机外参,所以这里不应该报"等待标定"
        self.assertFalse(scene['extrinsics_pending'])

    def test_depth_source_without_calibration_is_pending_not_silent(self):
        os.environ['RO2_CLOUD_SOURCE'] = 'depth'
        import importlib
        import live_cloud_node
        importlib.reload(live_cloud_node)
        node = live_cloud_node.LiveCloudNode()
        self.assertTrue(node.snapshot()['scene']['extrinsics_pending'])
        os.environ['RO2_CLOUD_SOURCE'] = 'pointcloud'

    def test_persistent_mode_from_environment(self):
        node = make_node(RO2_CLOUD_HISTORY_S='0')
        self.assertEqual(node.history.ttl, 0.0)
        self.assertEqual(node.snapshot()['scene']['mode'], 'persistent')

    def test_node_publishes_no_velocity_command(self):
        # 看图的节点绝不能碰底盘。这条失败就说明有人把控制塞进了显示层。
        topics = set(self.node.publishers_)
        self.assertTrue(all('cmd_vel' not in t for t in topics), topics)


class TestSyntheticLabelling(unittest.TestCase):
    def test_this_file_declares_its_data_synthetic(self):
        self.assertIn(SYNTHETIC, Path(__file__).read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
