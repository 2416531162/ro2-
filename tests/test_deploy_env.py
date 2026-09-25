"""部署配置：默认不开人体识别；ROS 只在本机回环上做发现（换 WiFi 后再启动的节点仍能找到其他节点）。"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'deployment'))

import manage  # noqa: E402


class TestProfiles(unittest.TestCase):

    def test_default_profile_is_rtk_autonomy_only(self):
        # 默认只部署 RTK 自动驾驶和手动：网页/app 链路、RTK 接收机、走线；不部署相机、雷达、人体识别、跟随、板载 GUI
        with open(manage.__file__, encoding='utf-8') as f:
            self.assertIn("default=DEFAULT_PROFILE", f.read())   # activate 不写 --profile 时用它
        self.assertEqual(manage.DEFAULT_PROFILE, 'rtk')
        self.assertEqual(set(manage.PROFILES['rtk']), {'web', 'rtk', 'line'})
        self.assertNotIn('ai', manage.PROFILES['headless'])

    def test_follow_profile_still_brings_detection(self):
        self.assertIn('ai', manage.PROFILES['follow'])

    def test_start_all_matches_deploy_profiles(self):
        with open(os.path.join(ROOT, 'radar_system', 'start_all.sh'), encoding='utf-8') as f:
            lines = f.read().splitlines()
        for name, components in manage.PROFILES.items():
            line = next(l for l in lines if l.strip().startswith(name + ')'))
            listed = line.split('(', 1)[1].split(')', 1)[0].split()
            self.assertEqual(set(listed), set(components), name)


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

    def test_camera_workspace_does_not_undo_it(self):
        with open(os.path.join(ROOT, 'radar_system', 'run_camera.sh'), encoding='utf-8') as f:
            text = f.read()
        after_source = text.split('source "$CAMERA_WORKSPACE"', 1)[1].split('ros2 launch', 1)[0]
        self.assertIn('export ROS_LOCALHOST_ONLY=1', after_source)


if __name__ == '__main__':
    unittest.main()
