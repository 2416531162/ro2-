"""Bounded Ackermann local recovery; pure Python, no actuator access.

Unknown laser rays are NOT free. A blind reverse can only retrace a recent
forward footprint, straight, once per recovery episode, for at most 15 cm.
This is a local manoeuvre controller, not a global navigation planner.
"""
import math
from dataclasses import dataclass

from footprint import in_blind_sector, is_self_hit
from motion_safety import BrakeProfile, brake_envelope, yaw_from_steer


@dataclass
class RecoveryConfig:
    enabled: bool = True
    speed_mps: float = 0.12
    blind_speed_mps: float = 0.08
    blind_reverse_m: float = 0.15
    reverse_m: float = 0.35
    forward_m: float = 0.70
    blocked_s: float = 1.0
    scan_s: float = 0.8
    settle_s: float = 0.30
    history_s: float = 4.0
    timeout_s: float = 75.0
    max_legs: int = 12


@dataclass
class Command:
    speed: float = 0.0
    steer: float = 0.0
    state: str = "RECOVERY_WAIT"
    reason: str = "no_observed_path"


class ScanEvidence:
    """Finite laser returns certify only the space before the return.

    Self reflections and masked/NaN/inf/missing rays remain unknown. All
    external finite hits (including masked angles) remain collision obstacles.
    """
    def __init__(self, ranges, angle_min, increment, range_min, range_max,
                 mount, footprint, blind_sectors=()):
        self.ranges = tuple(ranges)
        self.angle_min, self.increment = angle_min, increment
        self.mount = mount
        self.valid = []
        self.points = []
        self.usable = (bool(ranges) and math.isfinite(angle_min)
                       and math.isfinite(increment) and 0 < abs(increment) <= math.radians(2)
                       and math.isfinite(range_min) and math.isfinite(range_max)
                       and 0 <= range_min < range_max)
        for i, r in enumerate(ranges):
            a = angle_min + i * increment
            ok = self.usable and math.isfinite(r) and range_min <= r <= range_max
            if ok:
                x = mount.x_m + r * math.cos(a + mount.yaw_rad)
                y = mount.y_m + r * math.sin(a + mount.yaw_rad)
                external = not is_self_hit(x, y, footprint, skin_m=0.0)
                if external:
                    self.points.append((x, y))
                ok = external and not in_blind_sector(a, blind_sectors)
            self.valid.append(ok)
        self.usable = self.usable and any(self.valid)

    def _indices(self, x, y):
        if not self.usable:
            return ()
        dx, dy = x - self.mount.x_m, y - self.mount.y_m
        a = math.atan2(dy, dx) - self.mount.yaw_rad
        mid = self.angle_min + (len(self.ranges) - 1) * self.increment / 2
        a += round((mid - a) / (2 * math.pi)) * 2 * math.pi
        index = (a - self.angle_min) / self.increment
        n = len(self.ranges)
        full = abs(self.increment) * n >= 2 * math.pi - abs(self.increment) * 1.1
        indices = (math.floor(index), math.ceil(index))
        result = []
        for i in indices:
            if full:
                i %= n
            if not 0 <= i < n or not self.valid[i]:
                return ()
            result.append(i)
        return result

    def covered(self, x, y):
        return bool(self._indices(x, y))

    def free(self, x, y):
        indices = self._indices(x, y)
        if not indices:
            return False
        distance = math.hypot(x-self.mount.x_m, y-self.mount.y_m)
        for i in indices:
            # Account for angular spacing and range noise, rather than extend a
            # thin ray into an arbitrarily wide free wedge.
            if distance + 0.015 + distance * abs(self.increment) / 2 >= self.ranges[i]:
                return False
        return True


class LocalRecovery:
    def __init__(self, footprint, geometry, brake_profile, config=None):
        self.fp, self.geo, self.brake = footprint, geometry, brake_profile
        self.cfg = config or RecoveryConfig()
        self.pose = (0.0, 0.0, 0.0)
        self.history = []
        self.scan_history = []
        self.last_time = None
        self.last_bearing = 0.0
        self.seen = False
        self.active = False
        self.exhausted = False
        self.phase = "IDLE"
        self.blocked_since = None
        self.started = self.phase_time = 0.0
        self.leg_distance = self.total_distance = self.total_yaw = 0.0
        self.legs = 0
        self.blind_used = False
        self.blind_leg = False
        self.direction = 1
        self.turn = 1
        self.steer = 0.0
        self.still_since = None
        self.normal_distance = 0.0
        self.previous_speed = 0.0
        self._edge = self._perimeter()

    def _perimeter(self):
        f = self.fp
        lo, hi, w = -f.rear_m - f.margin_m, f.front_m + f.margin_m, f.effective_half_width
        nx, ny = math.ceil((hi-lo)/0.04), math.ceil(2*w/0.04)
        return ([(lo+(hi-lo)*i/nx, y) for i in range(nx+1) for y in (-w, w)]
                + [(x, -w+2*w*i/ny) for i in range(ny+1) for x in (lo, hi)])

    def _inside(self, x, y, pad=0.0):
        f = self.fp
        return (-f.rear_m-f.margin_m-pad <= x <= f.front_m+f.margin_m+pad
                and abs(y) <= f.effective_half_width+pad)

    def _gap(self, x, y):
        f = self.fp
        return max(-f.rear_m-f.margin_m-x, x-f.front_m-f.margin_m,
                   abs(y)-f.effective_half_width)

    def _history_free(self, x, y):
        px, py, yaw = self.pose
        wx = px + x*math.cos(yaw) - y*math.sin(yaw)
        wy = py + x*math.sin(yaw) + y*math.cos(yaw)
        for _, hx, hy, ha in self.history:
            dx, dy = wx-hx, wy-hy
            if self._inside(dx*math.cos(ha)+dy*math.sin(ha),
                            -dx*math.sin(ha)+dy*math.cos(ha), pad=1e-8):
                return True
        return False

    def _observed_before(self, x, y):
        """Recent actual ray evidence, transformed with measured odometry.

        Needed for rear-quarter swing: an area seen beside the front axle can
        enter the body shadow as the car advances. Do not erase that evidence.
        """
        px, py, yaw = self.pose
        wx = px+x*math.cos(yaw)-y*math.sin(yaw)
        wy = py+x*math.sin(yaw)+y*math.cos(yaw)
        for _, hx, hy, ha, scan in reversed(self.scan_history):
            dx, dy = wx-hx, wy-hy
            qx, qy = dx*math.cos(ha)+dy*math.sin(ha), -dx*math.sin(ha)+dy*math.cos(ha)
            if scan.covered(qx, qy):
                # The newest covering observation wins, including an obstacle.
                return scan.free(qx, qy)
        return False

    def clearance(self, scan, steer, direction=1, current_steer=0.0,
                  allow_history=False, horizon=0.85, allow_memory=True):
        """Sample the FULL rectangular body, including rear swing, both gears.

        Steering is interpolated through its transition; sample spacing is
        covered by a collision pad. Unknown newly swept space stops the path.
        """
        if scan is None or not scan.usable:
            return 0.0
        x = y = yaw = 0.0
        step = 0.02
        count = math.ceil(horizon/step)
        # Sampling padding must not create a permanent virtual collision when
        # a visible wall is already near the safety margin. Such a point may
        # stay equally far away or recede, but must never get closer.
        obstacles = [(ox, oy, min(.015, max(0.0, self._gap(ox, oy))))
                     for ox, oy in scan.points]
        for i in range(count+1):
            distance = i * step
            c, s = math.cos(yaw), math.sin(yaw)
            for ox, oy, clearance_floor in obstacles:
                dx, dy = ox-x, oy-y
                if self._gap(dx*c+dy*s, -dx*s+dy*c) < clearance_floor-1e-9:
                    return max(0.0, distance-step)
            for bx, by in self._edge:
                qx, qy = x+bx*c-by*s, y+bx*s+by*c
                if self._inside(qx, qy, pad=1e-8):
                    continue   # already occupied body, not a free-space claim
                known = scan.free(qx, qy)
                if not known and not scan.covered(qx, qy):
                    known = ((allow_memory and self._observed_before(qx, qy))
                             or (allow_history and self._history_free(qx, qy)))
                if not known:
                    return max(0.0, distance-step)
            # Worst-case steering transition over the first 10 cm.
            fraction = min(1.0, (distance+step)/0.10)
            angle = current_steer + (steer-current_steer)*fraction
            dyaw = yaw_from_steer(float(direction), angle, self.geo)*step
            x += direction*step*math.cos(yaw+dyaw/2)
            y += direction*step*math.sin(yaw+dyaw/2)
            yaw += dyaw
        return horizon

    def _observe(self, now, speed, yaw_rate, healthy):
        dt = 0.0 if self.last_time is None else now-self.last_time
        self.last_time = now
        if not healthy or not 0 <= dt <= 0.25:
            self.history.clear()
            self.scan_history.clear()
            return 0.0
        x, y, a = self.pose
        self.pose = (x+speed*dt*math.cos(a+yaw_rate*dt/2),
                     y+speed*dt*math.sin(a+yaw_rate*dt/2), a+yaw_rate*dt)
        self.history = [h for h in self.history if now-h[0] <= self.cfg.history_s]
        self.scan_history = [h for h in self.scan_history if now-h[0] <= self.cfg.history_s]
        # Only remember forward travel under a validated normal-follow command.
        if not self.active and self.previous_speed > 0.03 and speed > 0.03:
            self.history.append((now, *self.pose))
            self.normal_distance += speed*dt
        if self.active:
            self.leg_distance += abs(speed)*dt
            self.total_distance += abs(speed)*dt
            self.total_yaw += abs(yaw_rate)*dt
        return dt

    def cancel(self):
        # Keep the exhausted latch and seen-target memory: faults must not give
        # repeated fresh reverse budgets or start a new search without a target.
        if self.active:
            self.exhausted = True
        self.active = False
        self.phase = "IDLE"
        self.blocked_since = None
        self.previous_speed = 0.0
        self.history.clear()
        self.scan_history.clear()

    def _brake(self, now, direction):
        self.phase = "BRAKE"
        self.direction = direction
        self.phase_time = now
        self.still_since = None

    def update(self, *, now, scan, healthy, speed, yaw_rate, target, gap,
               bearing, requested_speed, requested_steer, current_steer,
               follow_cap, lost_age):
        dt = self._observe(now, speed, yaw_rate, healthy)
        if not healthy:
            self.cancel()
            return Command(state="RECOVERY_WAIT", reason="sensor_or_driver_unavailable")
        if scan is not None and scan.usable and (not self.scan_history or now-self.scan_history[-1][0] >= .10):
            self.scan_history.append((now, *self.pose, scan))
        if target:
            self.seen = True
            self.last_bearing = bearing
        if self.normal_distance >= 0.30:
            self.exhausted = False
            self.blind_used = False
        # A close person always wins, including during a reverse manoeuvre.
        if target and (follow_cap < 0.04 or (self.active and requested_speed <= 0.001)):
            self.cancel()
            return Command(state="HOLDING", reason="person_close")

        direct = self.clearance(scan, requested_steer, current_steer=current_steer)
        normal = None
        if target and requested_speed > 0:
            # Search BOTH steering directions, not just reduce the target steer.
            angles = [requested_steer, 0.0] + [self.geo.max_steer_rad*f for f in (-1, -.5, .5, 1)]
            choices = []
            for steer in dict.fromkeys(angles):
                clear = direct if steer == requested_steer else self.clearance(
                    scan, steer, current_steer=current_steer)
                cap = min(requested_speed, follow_cap, brake_envelope(clear, self.brake))
                if cap >= 0.08:
                    # Prefer alignment with the person once there is adequate room.
                    score = min(clear, .65) - 0.45*abs(steer-requested_steer)
                    choices.append((score, cap, steer))
            if choices:
                _, cap, steer = max(choices)
                normal = Command(cap, steer, "TRACKING" if steer == requested_steer else "ALIGNING",
                                 "follow" if steer == requested_steer else "local_path")

        if self.active and target and normal and self.phase == "FORWARD" and direct > .45:
            # Never switch reverse -> forward without a measured stationary dwell.
            if speed >= -0.02 and self.previous_speed >= 0:
                self.active = False
                self.phase = "IDLE"
                self.blocked_since = None

        if not self.active:
            stalled = requested_speed >= .08 and abs(speed) < .02 and self.previous_speed >= .08
            blocked = target and requested_speed >= .08 and (normal is None or stalled)
            if blocked:
                if self.blocked_since is None:
                    self.blocked_since = now
            else:
                self.blocked_since = None
            lost = not target and self.seen and lost_age > .4
            trigger = lost or (self.blocked_since is not None and now-self.blocked_since >= self.cfg.blocked_s)
            if trigger and self.cfg.enabled and not self.exhausted:
                self.active = True
                self.normal_distance = 0.0
                self.started = self.phase_time = now
                self.leg_distance = self.total_distance = self.total_yaw = 0.0
                self.legs = 0
                preferred = 1 if self.last_bearing >= 0 else -1
                self.turn = max((preferred, -preferred), key=lambda sign:
                                self.clearance(scan, sign*self.geo.max_steer_rad,
                                               current_steer=current_steer)
                                + (0.10 if sign == preferred else 0.0))
                self.phase = "SCAN" if lost else "BRAKE"
                self.direction = 1 if lost else -1
                self.still_since = None
            else:
                out = normal or Command(state="RECOVERY_EXHAUSTED" if self.exhausted else
                                        ("HOLDING" if target else "SEARCHING_LOST"))
                self.previous_speed = out.speed
                return out

        if (now-self.started >= self.cfg.timeout_s or self.legs >= self.cfg.max_legs
                or self.total_distance >= 6.5 or self.total_yaw >= math.pi + .10):
            self.active = False
            self.exhausted = True
            self.previous_speed = 0.0
            return Command(state="RECOVERY_EXHAUSTED", reason="bounded_search_complete")

        if self.phase == "SCAN":
            self.previous_speed = 0.0
            if now-self.phase_time >= self.cfg.scan_s:
                self._brake(now, 1)
            return Command(state="SEARCH_SCAN", reason="observe_before_turn")

        if self.phase == "BRAKE":
            self.previous_speed = 0.0
            if abs(speed) >= .02:
                self.still_since = None
                return Command(state="RECOVERY_BRAKE", reason="wait_stationary")
            if self.still_since is None:
                self.still_since = now
            if now-self.still_since < self.cfg.settle_s:
                return Command(state="RECOVERY_BRAKE", reason="gear_change_dwell")
            angles = [self.turn*self.geo.max_steer_rad*self.direction*f for f in (1, .5)]
            if target and self.direction > 0:
                angles.insert(0, requested_steer)
            if self.direction < 0:
                angles.append(0.0)
            elif not target:
                # Move into an observed front corridor to obtain a new view of
                # the rear quarter; do not require an impossible blind pivot.
                angles.append(0.0)
            paths = [(self.clearance(scan, a, self.direction, current_steer,
                                     allow_memory=self.direction > 0), a, False) for a in angles]
            # Rear blind: only straight retrace, one tiny budget per episode.
            if self.direction < 0 and not self.blind_used and abs(current_steer) < .03:
                paths.append((min(self.cfg.blind_reverse_m,
                                  self.clearance(scan, 0.0, -1, 0.0, allow_history=True)), 0.0, True))
            clear, steer, blind = max(paths, key=lambda p: p[0] - (0.08 if self.direction > 0 and p[1] == 0 else 0))
            if clear < .10:
                self.legs += 1
                self._brake(now, -self.direction)
                return Command(state="RECOVERY_WAIT", reason="no_observed_path")
            self.steer, self.blind_leg = steer, blind
            if blind:
                self.blind_used = True
            self.phase = "REVERSE" if self.direction < 0 else "FORWARD"
            self.phase_time = now
            self.leg_distance = 0.0
            self.legs += 1

        if self.direction > 0 and not target and abs(self.steer) < .01:
            turn_steer = self.turn*self.geo.max_steer_rad
            if self.clearance(scan, turn_steer, 1, current_steer) >= .30:
                self.steer = turn_steer

        distance_limit = (self.cfg.blind_reverse_m if self.blind_leg else
                          self.cfg.reverse_m if self.direction < 0 else self.cfg.forward_m)
        remaining = distance_limit-self.leg_distance
        clear = self.clearance(scan, self.steer, self.direction, current_steer,
                               allow_history=self.blind_leg,
                               allow_memory=self.direction > 0 or self.blind_leg)
        profile = BrakeProfile(self.brake.decel_mps2, self.brake.latency_s, .035, .015)
        cap = min(self.cfg.blind_speed_mps if self.blind_leg else self.cfg.speed_mps,
                  brake_envelope(min(clear, remaining), profile))
        if self.direction > 0 and target:
            cap = min(cap, follow_cap)
        timed_out = now-self.phase_time > max(4.0, distance_limit/.06+2)
        stalled = now-self.phase_time > 2.5 and self.leg_distance < .015
        if cap < .04 or timed_out or stalled:
            continue_forward = self.direction > 0 and clear > .30 and remaining < .08
            self._brake(now, 1 if continue_forward else -self.direction)
            self.previous_speed = 0.0
            return Command(state="RECOVERY_BRAKE", reason="leg_complete_or_blocked")
        self.previous_speed = self.direction*cap
        return Command(self.previous_speed, self.steer,
                       "RECOVERY_REVERSE" if self.direction < 0 else "SEARCH_TURN",
                       "recent_path_reverse" if self.blind_leg else "observed_path")
