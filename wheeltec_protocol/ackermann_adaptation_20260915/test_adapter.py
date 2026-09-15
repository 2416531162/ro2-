#!/usr/bin/env python3
"""Protocol and fail-stop tests, with no serial device access."""
import ast
import importlib.util
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest


def load(path):
    spec = importlib.util.spec_from_file_location("adapter_under_test", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


SAMPLE = bytes.fromhex("7b 00 00 00 00 00 00 00 ff 7c ff ce 3f d0 ff fd ff ff 00 06 58 00 7a 7d")


def watchdog_regression(path):
    source = Path(path).read_text()
    if "class ControlPolicy:" in source:
        m = load(path)
        p = m.ControlPolicy(m.Config(protocol="twist", protocol_confirmed=True, receive_only=False), 100)
        p.link(True, 100)
        t = m.decode_frame(SAMPLE)
        for _ in range(5):
            p.feedback(t, 104)
        assert p.arm(104)[0]
        p.command("twist", 0.1, 0, 104)
        assert p.tick(104.02) != m.STOP_FRAME
        frames = []
        for i in range(10):
            now = 105 + i * 0.02
            p.feedback(t, now)
            frames.append(p.tick(now))
        n = sum(f == m.STOP_FRAME for f in frames)
    else:
        # Execute the baseline's actual callbacks with only ROS/transport boundaries replaced.
        tree = ast.parse(source)
        node = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == "WheeltecDriver")
        node.bases = []
        node.body = [x for x in node.body if isinstance(x, ast.FunctionDef) and x.name in ("on_cmd_vel", "watchdog")]
        bcc = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "bcc")
        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0, y=0.0)
                self.angular = SimpleNamespace(z=0.0)
        clock = SimpleNamespace(now=100.0)
        env = {"time": SimpleNamespace(time=lambda: clock.now), "Twist": Twist,
               "FRAME_HEADER": 123, "FRAME_TAIL": 125, "SEND_DATA_SIZE": 11}
        exec(compile(ast.Module(body=[bcc, node], type_ignores=[]), str(path), "exec"), env)
        obj = env["WheeltecDriver"]()
        obj.cmd_vel_timeout, obj.connected, obj.last_frame_time = 0.5, True, 0
        obj.get_logger = lambda: SimpleNamespace(warn=lambda *a: None)
        frames = []
        obj.write = lambda data: frames.append(data)
        cmd = Twist()
        cmd.linear.x = 0.1
        obj.on_cmd_vel(cmd)
        frames.clear()
        for i in range(10):
            clock.now = 101 + i * 0.02
            obj.watchdog()
        n = sum(f == bytes.fromhex("7b 00 00 00 00 00 00 00 00 7b 7d") for f in frames)
    passed = n == 10
    print(f"{'PASS' if passed else 'FAIL'} watchdog: zero_frames={n}/10 after command timeout")
    return 0 if passed else 1


class AdapterTests(unittest.TestCase):
    def policy(self, **changes):
        cfg = dict(protocol="steering_angle", protocol_confirmed=True, receive_only=False,
                   wheelbase_m=0.30, steering_scale=0.5, mode_byte=1)
        cfg.update(changes)
        p = M.ControlPolicy(M.Config(**cfg), 0)
        p.link(True, 0)
        for _ in range(5):
            p.feedback(M.decode_frame(SAMPLE), 4)
        self.assertTrue(p.arm(4)[0])
        return p

    def test_known_live_telemetry(self):
        t = M.decode_frame(SAMPLE)
        self.assertEqual(t['velocity'], [0, 0, 0])
        self.assertEqual(t['voltage'], 22.528)
        self.assertAlmostEqual(t['acceleration'][2], 9.771, places=3)

    def test_fragmentation_noise_and_bad_checksum(self):
        bad = bytearray(SAMPLE)
        bad[22] ^= 1
        stream = b'noise' + bad + SAMPLE + SAMPLE
        parser = M.FrameParser()
        out = []
        for i in range(0, len(stream), 7):
            out += parser.feed(stream[i:i+7])
        self.assertEqual(len(out), 2)
        self.assertGreaterEqual(parser.bad, 1)
        self.assertLess(len(parser.buffer), 24)

    def test_default_cannot_arm(self):
        p = M.ControlPolicy(M.Config(), 0)
        self.assertFalse(p.arm(100)[0])
        self.assertEqual(p.tick(100), M.STOP_FRAME)

    def test_unconfirmed_profile_cannot_arm(self):
        p = M.ControlPolicy(M.Config(receive_only=False), 0)
        self.assertEqual(p.arm(100), (False, 'firmware_profile_unconfirmed'))

    def test_startup_and_fresh_stationary_feedback_required(self):
        p = self.policy()
        p.link(True, 5)
        self.assertEqual(p.arm(6)[1], 'startup_stop')
        self.assertEqual(p.arm(9)[1], 'feedback_stale')
        t = M.decode_frame(SAMPLE)
        t['velocity'][0] = 0.1
        for _ in range(10):
            p.feedback(t, 9)
        self.assertFalse(p.arm(9)[0])

    def test_zero_speed_steering_encodes_no_drive_or_lateral(self):
        p = self.policy()
        for i in range(1, 51):
            now = 4 + i * 0.02
            p.feedback(M.decode_frame(SAMPLE), now)
            p.command('ackermann', 0.0, 0.2, now)
            f = p.tick(now)
            self.assertEqual(f[3:7], b'\0\0\0\0')
        self.assertEqual(f[1], 1)
        self.assertEqual(struct.unpack('>h', f[7:9])[0], 100)
        self.assertEqual(M.bcc(f[:9]), f[9])

    def test_twist_stationary_yaw_is_not_steering(self):
        p = self.policy()
        with self.assertRaises(ValueError):
            p.command('twist', 0, 0.2, 4)
        self.assertFalse(p.armed)
        self.assertEqual(p.tick(4.02), M.STOP_FRAME)

    def test_twist_profile_cannot_synthesize_stationary_steering(self):
        p = self.policy(protocol='twist')
        with self.assertRaises(ValueError):
            p.command('ackermann', 0, 0.2, 4)
        self.assertFalse(p.armed)

    def test_reverse_kinematics(self):
        p = self.policy()
        p.command('twist', -0.1, 0.05, 4)
        self.assertAlmostEqual(p.latest[2], math.atan(-0.15))

    def test_unknown_wheelbase_rejects_conversion(self):
        p = self.policy(wheelbase_m=0)
        with self.assertRaises(ValueError):
            p.command('twist', 0.1, 0.1, 4)
        self.assertEqual(p.tick(4.02), M.STOP_FRAME)

    def test_lateral_input_stops(self):
        p = self.policy()
        with self.assertRaises(ValueError):
            p.command('twist', 0.1, 0, 4, lateral=0.1)
        self.assertFalse(p.armed)

    def test_non_finite_input_stops_active_command(self):
        for value in (math.nan, math.inf, -math.inf):
            p = self.policy()
            p.command('ackermann', 0.1, 0, 4)
            with self.assertRaises(ValueError):
                p.command('ackermann', value, 0, 4.02)
            self.assertEqual(p.tick(4.04), M.STOP_FRAME)

    def test_speed_steering_limits_and_no_catchup_burst(self):
        p = self.policy()
        p.command('ackermann', 10, 10, 4)
        self.assertEqual(p.latest[1:], (0.15, 0.35))
        p.tick(4.02)
        self.assertLessEqual(p.output[0], 0.0041)
        p.command('ackermann', 10, 10, 4.2)
        p.feedback(M.decode_frame(SAMPLE), 4.2)
        p.tick(4.2)
        self.assertLessEqual(p.output[0], 0.0081)

    def test_latest_command_replaces_previous(self):
        p = self.policy()
        for _ in range(1000):
            p.command('ackermann', 0.1, 0.2, 4)
        p.command('ackermann', 0, 0, 4.01)
        f = p.tick(4.02)
        self.assertEqual(f[3:9], bytes(6))

    def test_command_timeout_latches_and_keeps_stopping(self):
        p = self.policy()
        p.command('ackermann', 0.1, 0, 4)
        p.tick(4.02)
        for i in range(100):
            now = 4.4 + i * 0.02
            p.feedback(M.decode_frame(SAMPLE), now)
            self.assertEqual(p.tick(now), M.STOP_FRAME)
        self.assertEqual(p.reason, 'command_timeout')
        with self.assertRaises(ValueError):
            p.command('ackermann', 0.1, 0, 6.5)

    def test_feedback_loss_stops_with_recent_command(self):
        p = self.policy()
        p.command('ackermann', 0.1, 0, 4.4)
        self.assertEqual(p.tick(4.4), M.STOP_FRAME)
        self.assertEqual(p.reason, 'feedback_stale')

    def test_reconnect_discards_old_command(self):
        p = self.policy()
        p.command('ackermann', 0.1, 0, 4)
        p.link(False, 4.01)
        p.link(True, 5)
        for _ in range(5):
            p.feedback(M.decode_frame(SAMPLE), 9)
        self.assertTrue(p.arm(9)[0])
        self.assertEqual(p.tick(9.02), M.STOP_FRAME)

    def test_operator_stop_and_rearm_do_not_replay(self):
        p = self.policy()
        p.command('ackermann', 0.1, 0.2, 4)
        p.stop('operator_stop')
        self.assertEqual(p.tick(4.02), M.STOP_FRAME)
        self.assertTrue(p.arm(4.03)[0])
        self.assertEqual(p.tick(4.04), M.STOP_FRAME)

    def test_zero_speed_brakes_without_ramp(self):
        p = self.policy()
        p.command('ackermann', 0.1, 0, 4)
        self.assertNotEqual(p.tick(4.02)[3:5], b'\0\0')
        p.command('ackermann', 0, 0.2, 4.03)
        self.assertEqual(p.tick(4.04)[3:5], b'\0\0')

    def test_bad_configuration_fails_closed(self):
        for c in (dict(wheelbase_m=math.nan), dict(acceleration_m_s2=-1),
                  dict(mode_byte=256), dict(tx_hz=500), dict(startup_stop_s=0)):
            with self.assertRaises(ValueError):
                M.Config(**c)


if __name__ == '__main__':
    path = sys.argv[1]
    if '--watchdog' in sys.argv:
        sys.exit(watchdog_regression(path))
    M = load(path)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(AdapterTests)
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    print(f"CORE_TESTS passed={result.testsRun-len(result.failures)-len(result.errors)}/{result.testsRun}")
    sys.exit(0 if result.wasSuccessful() else 1)
