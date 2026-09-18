"""Hardware-free local planning and real follower-control-loop regressions.

FOLLOWER_SOURCE_DIR can select a preserved baseline for before/after tests.
ROS stubs are message containers only; these are not hardware acceptance tests.
"""
import contextlib
import io
import json
import math
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'radar_system'))
from follower_recovery import ScanEvidence, LocalRecovery, RecoveryConfig
from footprint import VehicleFootprint, SensorMount
from motion_safety import ChassisGeometry, BrakeProfile

import ros_stubs
ros_stubs.install()
sys.modules['std_msgs.msg'].Float32 = ros_stubs.String
srv = types.ModuleType('std_srvs.srv')
srv.SetBool = srv.Trigger = types.SimpleNamespace(Request=lambda: types.SimpleNamespace())
sys.modules['std_srvs'] = types.ModuleType('std_srvs')
sys.modules['std_srvs.srv'] = srv
sys.path.insert(0, os.environ.get('FOLLOWER_SOURCE_DIR', str(ROOT / 'radar_system')))
from person_follower import PersonFollowerNode, FollowerConfig


def scan_message(wall=None, invalid=False, blind=False, points=()):
    ranges = []
    for i in range(360):
        angle = -math.pi + i*math.pi/180
        c = math.cos(angle)
        r = 4.0
        if wall is not None and c > .001:
            r = min(r, (wall-.53)/c)
        if invalid or (blind and (angle >= math.radians(155) or angle <= math.radians(-130))):
            r = float('nan')
        ranges.append(r)
    for x, y in points:
        a = math.atan2(y, x-.53)
        i = round((a+math.pi)/(math.pi/180)) % 360
        ranges[i] = math.hypot(x-.53, y)
    return types.SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                                 angle_increment=math.pi/180, range_min=.02, range_max=8.0)


def evidence(message=None, blind_sectors=()):
    m = message or scan_message()
    fp = VehicleFootprint(margin_m=.035)
    return ScanEvidence(m.ranges, m.angle_min, m.angle_increment, m.range_min,
                        m.range_max, SensorMount(.53, 0, 0), fp, blind_sectors)


def planner(**kw):
    cfg = RecoveryConfig(**kw)
    return LocalRecovery(VehicleFootprint(margin_m=.035), ChassisGeometry(),
                         BrakeProfile(stop_m=.12, hard_stop_m=.06), cfg)


def update(p, now, **kw):
    args = dict(now=now, scan=evidence(), healthy=True, speed=0.0, yaw_rate=0.0,
                target=True, gap=2., bearing=.3, requested_speed=.2,
                requested_steer=.2, current_steer=0., follow_cap=.3, lost_age=0.)
    args.update(kw)
    return p.update(**args)


class FollowerHarness(unittest.TestCase):
    def setUp(self):
        client = lambda *a, **k: types.SimpleNamespace(service_is_ready=lambda: False,
                                                      call_async=lambda _: None)
        self.client_patch = patch.object(ros_stubs.Node, 'create_client', client, create=True)
        self.client_patch.start()
        self.addCleanup(self.client_patch.stop)
        self.time = 100.0
        self.clock_patch = patch('person_follower.time.monotonic', lambda: self.time)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        cfg = FollowerConfig()
        cfg.confirm_frames = 1
        cfg.scan_blind_sectors_deg = ()
        self.node = PersonFollowerNode(cfg, dry_run=True, simulated_odometry=True)
        self.node.people.confirm_hits = 1
        self.node.print_dashboard = lambda _: None

    def tick(self, target=True, wall=None, invalid=False, speed=0., yaw=0., feedback=True):
        self.time += .05
        n = self.node
        n.on_scan(scan_message(wall=wall, invalid=invalid))
        if feedback:
            n.on_driver_status(ros_stubs.String(json.dumps(dict(
                connected=True, armed=True, holding=False, ready='ready', age_ms=0.,
                telemetry=dict(velocity=[speed, 0., yaw])))))
        if target:
            n.last_target_seen = self.time
            item = [{'label': 'person', 'conf': 0.9, 'x': 0.0, 'y': 0.0, 'z': 2.0,
                     'depth_ratio': 0.9, 'range_valid': True}]
            n.on_targets(ros_stubs.String(json.dumps(item)))
        n.control_loop()
        return n.cmd_vx


class TestRegression(FollowerHarness):
    def test_dead_end_can_reverse_with_observed_rear(self):
        for _ in range(25):
            self.tick(speed=0.2)
        speeds = [self.tick(wall=.84) for _ in range(65)]
        self.assertTrue(any(v < 0 for v in speeds), 'dead end never commands a reverse')
        self.assertGreaterEqual(min(speeds), -.12)

    def test_lost_target_search_has_real_motion(self):
        for _ in range(3):
            self.tick()
        speeds = [self.tick(target=False) for _ in range(45)]
        self.assertTrue(any(v > 0 for v in speeds[20:]), 'SEARCHING only stops; never searches')
        self.assertNotEqual(self.node.cmd_wz, 0.)

    def test_all_invalid_scan_stops(self):
        self.tick()
        self.tick(invalid=True)
        self.assertEqual(self.node.cmd_vx, 0., 'invalid laser treated as free space')


class TestNodeGuards(FollowerHarness):
    def test_delayed_scan_receipt_is_not_fresh_evidence(self):
        self.tick()
        m = scan_message()
        m.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=990, nanosec=0))
        self.node.on_scan(m)
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_bad_scan_angles_stop_without_exception(self):
        for value in (0., float('nan'), float('inf')):
            m = scan_message()
            m.angle_increment = value
            self.node.on_scan(m)
            self.node.control_loop()
            self.assertEqual(self.node.cmd_vx, 0.)

    def test_default_blind_mask_is_not_free(self):
        cfg = FollowerConfig()
        e = evidence(blind_sectors=cfg.scan_blind_sectors_deg)
        self.assertFalse(e.free(-.4, 0.))

    def test_stale_feedback_stops(self):
        self.tick()
        for _ in range(10):
            self.tick(feedback=False)
        self.assertEqual(self.node.cmd_vx, 0.)
        self.assertEqual(self.node.limit_reason, 'driver_unavailable')

    def test_old_feedback_republished_is_not_fresh(self):
        self.tick()
        self.node.on_driver_status(ros_stubs.String(json.dumps(dict(
            connected=True, armed=True, holding=False, age_ms=900.,
            telemetry=dict(velocity=[0., 0., 0.])))))
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_stale_scan_stops_immediately(self):
        self.tick()
        self.time += .6
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_hard_fault_interrupts_reverse(self):
        for _ in range(60):
            self.tick(wall=.84)
        self.node.voltage = 20.
        self.tick(wall=.84)
        self.assertEqual(self.node.cmd_vx, 0.)
        self.assertEqual(self.node.state, 'LOW_BATTERY')

    def test_rear_obstacle_interrupts_reverse(self):
        for _ in range(60):
            self.tick(wall=.84)
        self.node.on_scan(scan_message(wall=.84, points=[(-.24, 0)]))
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_driver_hold_interrupts(self):
        self.tick()
        self.node.on_driver_status(ros_stubs.String(json.dumps(dict(
            connected=True, armed=True, holding=True, age_ms=0.,
            telemetry=dict(velocity=[0., 0., 0.])))))
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_no_recovery_flag(self):
        self.node.cfg.recovery.enabled = False
        self.tick()
        for _ in range(50):
            self.tick(target=False)
        self.assertEqual(self.node.cmd_vx, 0.)

    def test_no_target_at_boot_does_not_search(self):
        self.assertTrue(all(self.tick(target=False) == 0 for _ in range(20)))

    def test_sensor_conflict_stops(self):
        self.tick()
        self.node.last_conflict = True
        self.node.control_loop()
        self.assertEqual(self.node.cmd_vx, 0.)
        self.assertEqual(self.node.state, 'SENSOR_CONFLICT')


class TestGeometry(unittest.TestCase):
    def test_recent_scans_allow_observed_rear_quarter_swing(self):
        p = planner()
        e = evidence(blind_sectors=((155., -130.),))
        self.assertGreater(p.clearance(e, -.175), .5)  # masked swing stays within occupied margin
        p.scan_history = [(100., -.45, 0., 0., e)]
        self.assertGreater(p.clearance(e, -.175), .5)
        self.assertGreater(p.clearance(e, .175), .5)

    def test_current_obstacle_overrides_previous_free_scan(self):
        p = planner()
        p.scan_history = [(100., 0., 0., 0., evidence())]
        e = evidence(scan_message(wall=.85))
        self.assertLess(p.clearance(e, 0.), .2)

    def test_padding_does_not_lock_parallel_escape(self):
        p = planner()
        e = evidence(scan_message(points=[(.2, .38)]))
        self.assertGreater(p.clearance(e, 0.), .5)
        # Steering into the same wall must still be rejected.
        self.assertLess(p.clearance(e, .35), .5)

    def test_unknown_is_not_free(self):
        for value in (float('nan'), float('inf'), 0., -1.):
            m = scan_message()
            m.ranges = [value]*360
            e = evidence(m)
            self.assertFalse(e.usable)
            self.assertEqual(planner().clearance(e, 0.), 0.)

    def test_blind_sector_wrap(self):
        e = evidence(blind_sectors=((155., -130.),))
        self.assertFalse(e.free(-.4, 0.))
        self.assertTrue(e.free(1., 0.))

    def test_partial_scan_does_not_certify_rear(self):
        m = scan_message()
        m.angle_min = -math.pi/2
        m.ranges = [4.]*181
        e = evidence(m)
        self.assertTrue(e.free(1., 0.))
        self.assertFalse(e.free(-.4, 0.))

    def test_zero_increment_is_unknown(self):
        m = scan_message()
        m.angle_increment = 0.
        self.assertFalse(evidence(m).usable)

    def test_reverse_checks_rear_hit(self):
        p = planner()
        e = evidence(scan_message(points=[(-.30, 0)]))
        self.assertLess(p.clearance(e, 0., -1), .1)

    def test_turn_checks_rear_swing(self):
        p = planner()
        e = evidence(scan_message(points=[(0., -.39)]))
        self.assertLess(p.clearance(e, .35, 1), .85)

    def test_door_too_narrow_is_not_passable(self):
        p = planner()
        e = evidence(scan_message(points=[(.85, .32), (.85, -.32)]))
        self.assertLess(p.clearance(e, 0.), .20)

    def test_80cm_door_can_pass_straight(self):
        p = planner()
        e = evidence(scan_message(points=[(1., .40), (1., -.40)]))
        self.assertGreater(p.clearance(e, 0.), .50)

    def test_raycast_80cm_door_through_all_approach_positions(self):
        p = planner()
        for position in (0., .2, .4, .5, .6, .7, .8, .9, 1., 1.2, 1.5, 1.7):
            m = scan_message()
            for i in range(360):
                angle = m.angle_min+i*m.angle_increment
                c = math.cos(angle)
                if abs(c) < .001:
                    continue
                distance = (1.4-position-.53)/c
                if distance > .02 and abs(distance*math.sin(angle)) >= .40:
                    m.ranges[i] = min(4., distance)
            self.assertGreater(p.clearance(evidence(m), 0.), .5, position)

    def test_full_body_reverse_arc_differs_from_forward(self):
        p = planner()
        e = evidence(scan_message(points=[(-.4, 0.)]))
        self.assertGreater(p.clearance(e, 0., 1), .5)
        self.assertLess(p.clearance(e, 0., -1), .2)

    def test_blind_reverse_requires_recent_forward_path(self):
        p = planner()
        e = evidence(scan_message(blind=True))
        self.assertEqual(p.clearance(e, 0., -1, allow_history=True), 0.)
        p.history = [(100., -.25, 0., 0., 0.)]
        self.assertGreaterEqual(p.clearance(e, 0., -1, allow_history=True), .15)
        self.assertEqual(p.clearance(e, 0., -1, allow_history=False), 0.)

    def test_history_expires_and_fault_discards_it(self):
        p = planner()
        p.history = [(70., -.25, 0., 0., 0.)]
        update(p, 100.)
        self.assertEqual(p.history, [])
        p.history = [(100., -.25, 0., 0., 0.)]
        update(p, 100.05, healthy=False)
        self.assertEqual(p.history, [])


class TestStateMachine(unittest.TestCase):
    def test_lost_search_acquires_view_then_turns_with_default_blind_sector(self):
        from motion_safety import clamp, yaw_from_steer
        p = planner()
        e = evidence(blind_sectors=((155., -130.),))
        update(p, 100., scan=e)
        speed = yaw_rate = steer = 0.
        moved = turned = False
        for i in range(180):
            r = update(p, 100.05+i*.05, scan=e, target=False, lost_age=1.,
                       speed=speed, yaw_rate=yaw_rate, current_steer=steer)
            steer += clamp(r.steer-steer, -.06, .06)
            clear = p.clearance(e, steer, 1, steer)
            speed = max(0., r.speed) if clear >= .10 else 0.
            yaw_rate = yaw_from_steer(speed, steer, p.geo)
            moved = moved or speed > .01
            turned = turned or abs(yaw_rate) > .01
        self.assertTrue(moved)
        self.assertTrue(turned, 'default blind sector prevents all active search turns')

    def test_adjustment_can_choose_opposite_steering(self):
        p = planner()
        r = update(p, 100., scan=evidence(scan_message(points=[(.95, .34)])),
                   requested_steer=.3)
        self.assertEqual(r.state, 'ALIGNING')
        self.assertLess(r.steer, 0.)
        self.assertGreater(r.speed, 0.)

    def test_target_stops_in_follow_deadband_aborts_recovery(self):
        p = planner()
        update(p, 100.)
        update(p, 100.05, target=False, lost_age=1.)
        r = update(p, 100.10, requested_speed=0., follow_cap=.2)
        self.assertEqual(r.speed, 0.)
        self.assertFalse(p.active)

    def test_reacquisition_does_not_skip_stationary_dwell(self):
        p = planner()
        p.active = True
        p.started = 100.
        p.phase = 'BRAKE'
        p.direction = 1
        p.still_since = None
        r = update(p, 100., speed=-.01)
        self.assertEqual(r.speed, 0.)
        self.assertEqual(r.state, 'RECOVERY_BRAKE')

    def test_turn_budget_stops_search(self):
        p = planner()
        update(p, 100.)
        update(p, 100.05, target=False, lost_age=1.)
        p.total_yaw = math.pi+.11
        r = update(p, 100.10, target=False, lost_age=1.)
        self.assertEqual(r.speed, 0.)
        self.assertTrue(p.exhausted)

    def test_no_blind_budget_replenishment_while_stuck(self):
        p = planner(blocked_s=0.)
        p.blind_distance = p.cfg.blind_reverse_m
        p.history = [(100., -.30, 0., 0., 0.), (100., 0., 0., 0., .30)]
        e = evidence(scan_message(wall=.84, blind=True))
        for i in range(45):
            r = update(p, 100.+i*.05, scan=e, requested_steer=0., bearing=0.)
            self.assertEqual(r.speed, 0.)

    def test_scan_before_turn(self):
        p = planner()
        update(p, 100.)
        r = update(p, 100.05, target=False, lost_age=1.)
        self.assertEqual(r.state, 'SEARCH_SCAN')
        self.assertEqual(r.speed, 0.)

    def test_measured_stop_required_for_reverse(self):
        p = planner(blocked_s=0.)
        r = update(p, 100., scan=evidence(scan_message(wall=.84)), speed=.1)
        self.assertEqual(r.speed, 0.)
        self.assertEqual(r.state, 'RECOVERY_BRAKE')

    def test_timeout_latches_until_normal_progress(self):
        p = planner(timeout_s=1.)
        update(p, 100.)
        for i in range(40):
            r = update(p, 100.05+i*.05, target=False, lost_age=1.)
        self.assertTrue(p.exhausted)
        self.assertEqual(r.speed, 0.)
        for i in range(10):
            r = update(p, 103.+i*.05, target=False, lost_age=2.)
            self.assertEqual(r.speed, 0.)

    def test_person_close_aborts(self):
        p = planner()
        update(p, 100.)
        update(p, 100.05, target=False, lost_age=1.)
        r = update(p, 100.10, follow_cap=0., gap=.5)
        self.assertEqual(r.speed, 0.)
        self.assertFalse(p.active)

    def test_default_enabled_and_reverse_yaw_sign(self):
        self.assertTrue(FollowerConfig().recovery.enabled)
        from motion_safety import yaw_from_steer
        self.assertGreater(yaw_from_steer(-.1, -.3, ChassisGeometry()), 0.)

    def test_blind_reverse_single_budget(self):
        p = planner(blocked_s=0., history_s=10., blind_reverse_m=.15)
        p.history = [(100., -.30, 0., 0., 0.), (100., 0., 0., 0., .30)]
        e = evidence(scan_message(wall=.84, blind=True))
        found = False
        for i in range(90):
            r = update(p, 100.+i*.05, scan=e, requested_steer=0., bearing=0.,
                       speed=-.08 if found else 0.)
            if r.speed < 0:
                found = True
                self.assertLessEqual(abs(r.speed), .08)
                self.assertEqual(r.steer, 0.)
            if found and p.phase == 'BRAKE':
                break
        self.assertTrue(found)
        self.assertTrue(p.blind_used)
        self.assertLessEqual(p.leg_distance, p.cfg.blind_reverse_m)


class TestDoorSimulation(unittest.TestCase):
    def simulate(self, blind):
        from motion_safety import clamp, yaw_from_steer, brake_envelope
        p = planner()
        x, y, heading = 0., .04, math.radians(5)
        speed = yaw_rate = steer = 0.
        for k in range(350):
            m = scan_message()
            lx, ly = x+.53*math.cos(heading), y+.53*math.sin(heading)
            for i in range(360):
                angle = heading+m.angle_min+i*m.angle_increment
                c = math.cos(angle)
                if abs(c) < .001:
                    continue
                distance = (1.4-lx)/c
                if distance > .02 and abs(ly+distance*math.sin(angle)) >= .425:
                    m.ranges[i] = min(4., distance)
            e = evidence(m, blind_sectors=((155., -130.),) if blind else ())
            dx, dy = 3.4-x, -y
            local_x = dx*math.cos(heading)+dy*math.sin(heading)
            local_y = -dx*math.sin(heading)+dy*math.cos(heading)
            bearing = math.atan2(local_y, local_x)
            gap = local_x-.67
            r = update(p, 100.+k*.05, scan=e, speed=speed, yaw_rate=yaw_rate,
                       gap=gap, bearing=bearing, requested_speed=.3,
                       requested_steer=clamp(1.1*bearing, -.35, .35), current_steer=steer,
                       follow_cap=min(.3, brake_envelope(gap, BrakeProfile(stop_m=.8, hard_stop_m=.06))))
            steer += clamp(r.steer-steer, -.06, .06)
            direction = -1 if r.speed < 0 else 1
            clear = p.clearance(e, steer, direction, steer, allow_history=p.active and p.blind_leg)
            profile = BrakeProfile(stop_m=.035, hard_stop_m=.015) if p.active else p.brake
            speed = direction*min(abs(r.speed), brake_envelope(clear, profile)) if clear >= .06 else 0.
            yaw_rate = yaw_from_steer(speed, steer, p.geo)
            x += speed*.05*math.cos(heading+yaw_rate*.025)
            y += speed*.05*math.sin(heading+yaw_rate*.025)
            heading += yaw_rate*.05
            corners = [(x+cx*math.cos(heading)-cy*math.sin(heading),
                        y+cx*math.sin(heading)+cy*math.cos(heading)) for cx, cy in p.fp.corners()]
            for i in range(4):
                ax, ay = corners[i]
                bx, by = corners[(i+1) % 4]
                if min(ax, bx) <= 1.4 <= max(ax, bx) and abs(bx-ax) > 1e-9:
                    at_wall = ay+(by-ay)*(1.4-ax)/(bx-ax)
                    self.assertLess(abs(at_wall), .425, 'physical body touches door frame')
            if x > 1.65:
                return
        self.fail('85 cm door not traversed from 4 cm offset / 5 degree yaw')

    def test_offset_door_with_observed_rear(self):
        self.simulate(False)

    def test_offset_door_with_actual_rear_blind_sector(self):
        self.simulate(True)


if __name__ == '__main__':
    unittest.main(verbosity=2)

class TestMissingObservationWait(unittest.TestCase):
    def test_persistent_front_gap_waits_without_spending_recovery_legs(self):
        p = planner()
        m = scan_message()
        for i in range(360):
            a = math.degrees(m.angle_min + i*m.angle_increment)
            if -32 <= a <= -20:
                m.ranges[i] = float('inf')
        e = evidence(m)
        for i in range(200):
            command = update(p, 100. + .05*i, scan=e, requested_steer=0.)
            self.assertEqual(command.speed, 0.)
            self.assertEqual(command.state, 'OBSERVATION_WAIT')
            self.assertEqual(command.reason, 'insufficient_observation')
        self.assertEqual(p.legs, 0)
        self.assertFalse(p.exhausted)
        self.assertFalse(p.active)
        # Fresh complete observations resume the existing normal path.
        command = update(p, 110., scan=evidence(), requested_steer=0.)
        self.assertGreater(command.speed, 0.)
        self.assertEqual(command.state, 'TRACKING')

    def test_observed_wall_is_not_reported_as_missing_observations(self):
        p = planner(enabled=False)
        command = update(p, 100., scan=evidence(scan_message(wall=.75)), requested_steer=0.)
        self.assertEqual(command.speed, 0.)
        self.assertEqual(command.state, 'PATH_BLOCKED')
        self.assertEqual(command.reason, 'obstacle_path_blocked')
