"""部署配置：只有 rtk 一套服务；ROS 只在本机回环上做发现（换 WiFi 后再启动的节点仍能找到其他节点）。"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'deployment'))

import manage  # noqa: E402


class TestProfiles(unittest.TestCase):

    def test_only_the_rtk_profile(self):
        # 只部署网页/app 链路、RTK 接收机、走线、IMU、作业记录（底盘驱动另起）
        with open(manage.__file__, encoding='utf-8') as f:
            self.assertIn("default=DEFAULT_PROFILE", f.read())   # activate 不写 --profile 时用它
        self.assertEqual(manage.DEFAULT_PROFILE, 'rtk')
        self.assertEqual(set(manage.PROFILES), {'rtk'})
        self.assertEqual(set(manage.PROFILES['rtk']), {'web', 'rtk', 'line', 'imu', 'jobrec'})

    def test_start_all_matches_deploy_profile(self):
        with open(os.path.join(ROOT, 'radar_system', 'start_all.sh'), encoding='utf-8') as f:
            line = next(l for l in f.read().splitlines() if l.startswith('COMPONENTS=('))
        self.assertEqual(set(line.split('(', 1)[1].split(')', 1)[0].split()), set(manage.PROFILES['rtk']))


class TestRosDiscovery(unittest.TestCase):

    def loaded_env(self, distro):
        """source ros_env.sh 之后的 ROS_LOCALHOST_ONLY；假 setup.bash 像 ROS 自带的 hook 一样把它改成 0。"""
        with tempfile.TemporaryDirectory() as ros_root:
            os.makedirs(os.path.join(ros_root, distro))
            with open(os.path.join(ros_root, distro, 'setup.bash'), 'w') as f:
                f.write('export ROS_DISTRO=%s\nexport ROS_LOCALHOST_ONLY=0\n' % distro)
            env = dict(os.environ, RK3588_RUNTIME_RESOLVED='1', RK3588_RUNTIME_ENV_SOURCE='test',
                       ROBOT_RUNTIME_ENV='test', RK3588_ROS_ROOT=ros_root, RK3588_PYTHON=sys.executable,
                       RK3588_POSE_MODEL='x', RK3588_ROBOT_CONFIG='x', RK3588_DEPTH_PATH_CONFIG='x',
                       ROS_DISTRO=distro)
            env.pop('ROS_LOCALHOST_ONLY', None)
            out = subprocess.run(['bash', '-c', 'source radar_system/ros_env.sh && echo "$ROS_LOCALHOST_ONLY"'],
                                 cwd=ROOT, env=env, capture_output=True, text=True, check=True)
            return out.stdout.strip()

    def test_humble_discovers_on_loopback_only(self):
        self.assertEqual(self.loaded_env('humble'), '1')

    def test_other_distros_are_left_to_discovery_range(self):
        # Jazzy 用 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST（各启动脚本已设），不再碰已弃用的变量
        self.assertEqual(self.loaded_env('jazzy'), '0')


class TestUnits(unittest.TestCase):
    """systemd 服务：账号可换（Jetson 是 wheeltec，正点原子 RK3588 用板子的登录账号）；Gen5 的 CAN 开机起成 500K。"""

    def test_default_account_is_the_jetson_one(self):
        u = manage.units(manage.Path('/opt/rk3588'))['rk3588-wheeltec.service']
        self.assertIn('User=wheeltec\nGroup=wheeltec\n', u)
        self.assertIn('Environment=HOME=/home/wheeltec\n', u)
        self.assertIn('SupplementaryGroups=dialout\n', u)       # RTK 接收机的 USB 串口

    def test_rk3588_account(self):
        units = manage.units(manage.Path('/opt/rk3588'), 'rtk', 'atk', 'atk', '/home/atk')
        for name in ('rk3588-wheeltec.service', 'rk3588-perception@.service'):
            self.assertIn('User=atk\nGroup=atk\n', units[name])
            self.assertIn('Environment=HOME=/home/atk\nEnvironment=USER=atk\n', units[name])
            self.assertIn('After=network.target rk3588-can.service', units[name])
            self.assertNotIn('wheeltec\n', units[name].replace('rk3588-wheeltec', ''))
        self.assertRaises(ValueError, manage.units, manage.Path('/opt/rk3588'), 'rtk', 'bad user')
        # 这块板一直用 root（adb shell）：--user root
        root = manage.units(manage.Path('/opt/rk3588'), 'rtk', 'root', 'root', '/root')['rk3588-wheeltec.service']
        self.assertIn('User=root\nGroup=root\n', root)
        self.assertIn('Environment=HOME=/root\n', root)

    def test_can_comes_up_at_500k_and_recovers_from_bus_off(self):
        can = manage.units(manage.Path('/opt/rk3588'))['rk3588-can.service']
        self.assertIn('ConditionPathExists=/sys/class/net/can0', can)   # 没有 CAN 的旧车跳过
        self.assertIn('bitrate 500000 restart-ms 100', can)
        self.assertIn('Before=rk3588-wheeltec.service', can)
        self.assertIn('Type=oneshot', can)
        self.assertIn('WantedBy=multi-user.target', can)
        with open(manage.__file__, encoding='utf-8') as f:
            text = f.read()
        self.assertIn("systemctl('start', 'rk3588-can.service'", text)
        self.assertIn("systemctl('enable', 'rk3588-can.service'", text)


if __name__ == '__main__':
    unittest.main()
