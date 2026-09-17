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
    # Reverse into the unobservable rear is only allowed along the car's own
    # recent trail: every swept body point must lie inside a footprint the
    # car physically occupied (so it was free then). Budget per episode.
    blind_reverse_m: float = 0.30
    reverse_m: float = 0.35
    forward_m: float = 0.70
    blocked_s: float = 1.0
    scan_s: float = 0.8
    settle_s: float = 0.30
    history_s: float = 4.0          # scan memory (observed-before) horizon
    trail_m: float = 1.5            # how much of the recent forward path to keep
    trail_max_age_s: float = 20.0   # older trail is too stale to reverse into
    trail_step_m: float = 0.02
    timeout_s: float = 75.0
    max_legs: int = 12
    exhausted_cooldown_s: float = 20.0


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
                 mount, footprint, blind_sectors=(), self_hit_skin_m=0.0):
        # self_hit_skin_m MUST match the follower's own self-hit filter. With
        # 0 here but 0.05 in the follower, one return 1-5 cm outside the body
        # (wheel bulge, cable, bracket) was "self" for the follower but an
        # obstacle INSIDE the padded footprint here, so clearance() returned 0
        # for every steer forever: person visible, car never moves.
        self.ranges = tuple(ranges)
        self.angle_min, self.increment = angle_min, increment
        self.mount = mount
        self.blind_sectors = tuple(blind_sectors or ())
        self.valid = []
        # Why each ray is (in)valid: ok / none (+inf, NaN: no echo) /
        # near (-inf or < range_min: echo closer than the lidar can range,
        # i.e. car structure) / self (return on the car body) / far / masked.
        self.causes = []
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
                external = not is_self_hit(x, y, footprint, skin_m=self_hit_skin_m)
                if external:
                    self.points.append((x, y))
                masked = in_blind_sector(a, blind_sectors)
                ok = external and not masked
                cause = "ok" if ok else ("self" if not external else "masked")
            elif in_blind_sector(a, blind_sectors):
                cause = "masked"      # configured car structure, whatever the reading
            elif r != r or r == math.inf:
                cause = "none"
            elif r == -math.inf or r < range_min:
                cause = "near"
            else:
                cause = "far"
            self.valid.append(ok)
            self.causes.append(cause)
        self.usable = self.usable and any(self.valid)
        n = len(self.ranges)
        self._full = bool(n) and abs(increment) * n >= 2 * math.pi - abs(increment) * 1.1
        self._reach = (max(1, int(self.NEIGHBOR_WINDOW_RAD / abs(increment) + 1e-9))
                       if self.usable else 1)
        self._reach_struct = (max(self._reach,
                                  int(self.STRUCTURE_WINDOW_RAD / abs(increment) + 1e-9))
                              if self.usable else 1)
        self._mid = angle_min + (n - 1) * increment / 2 if n else 0.0
        # Nearest valid bin at/below and at/above every bin, precomputed once per
        # scan so each free-space query is O(1) (clearance() issues ~10^5/cycle).
        self._lower = [self._nearest_valid(i, -1, self._reach, self._full) for i in range(n)]
        self._upper = [self._nearest_valid(i, +1, self._reach, self._full) for i in range(n)]

    # How far (radians) a query may look for a neighbouring real return.
    #
    # The N10P node bins each sweep into 720 slots (0.5 deg), but the sensor
    # only delivers ~450 samples per revolution at 10 Hz, so roughly a third of
    # the slots are empty (inf) in EVERY sweep. Requiring the two slots right
    # next to a query to both hold returns therefore failed on practically
    # every sweep: clearance() returned 0 ("no_observed_path"), AEB latched at
    # start-up and recovery burned all its legs without moving.
    #
    # A query now uses the nearest real return on each side within this
    # window. 1 deg spans the N10P sample spacing (~0.8 deg) with margin; at
    # 1 m that is ~3.5 cm of interpolated free space. Actual returns are still
    # all collision obstacles, and a wider run of missing rays stays unknown.
    NEIGHBOR_WINDOW_RAD = math.radians(4.5)
    # A narrow run of rays blocked by the car's OWN structure (echo closer
    # than range_min, a return on the body, or a configured blind sector,
    # which by definition marks car structure) says nothing about the space
    # beyond; such a permanent sliver ahead made every path "unknown" at
    # start-up (field report: blocked at lidar bearing 16.7 deg, 0.20 m).
    # Runs of structural rays up to this width are bridged by the real returns
    # on both sides; no-echo (+inf) rays keep the strict 1 deg window, since a
    # dark object also produces them. Wider structural shadows stay unknown.
    STRUCTURE_WINDOW_RAD = math.radians(9.0)

    def _nearest_valid(self, start, step, reach, full):
        n = len(self.ranges)
        loose = 0
        for k in range((getattr(self, "_reach_struct", reach)) + 1):
            i = start + step * k
            if full:
                i %= n
            if not 0 <= i < n:
                return None
            if self.valid[i]:
                return i
            if self.causes[i] not in ("near", "self", "masked"):
                loose += 1
                if loose > reach:
                    return None
        return None

    def explain(self, x, y, half_width_deg=3.0):
        """Rays around the direction of (x, y): [(bearing_deg, range, cause)]."""
        if not self.ranges:
            return []
        a = math.atan2(y - self.mount.y_m, x - self.mount.x_m) - self.mount.yaw_rad
        a += round((self._mid - a) / (2 * math.pi)) * 2 * math.pi
        centre = (a - self.angle_min) / self.increment
        span = int(math.radians(half_width_deg) / abs(self.increment)) + 1
        n = len(self.ranges)
        out = []
        for k in range(-span, span + 1):
            i = int(round(centre)) + k
            if self._full:
                i %= n
            if not 0 <= i < n:
                continue
            r = self.ranges[i]
            deg = math.degrees(self.angle_min + i * self.increment + self.mount.yaw_rad)
            deg = (deg + 180.0) % 360.0 - 180.0
            out.append((round(deg, 1), round(r, 3) if math.isfinite(r) else str(r),
                        self.causes[i]))
        return out

    def _indices(self, x, y):
        if not self.usable:
            return ()
        dx, dy = x - self.mount.x_m, y - self.mount.y_m
        a = math.atan2(dy, dx) - self.mount.yaw_rad
        a += round((self._mid - a) / (2 * math.pi)) * 2 * math.pi
        index = (a - self.angle_min) / self.increment
        n = len(self.ranges)
        lo, hi = math.floor(index), math.ceil(index)
        if self._full:
            lo %= n
            hi %= n
        elif not (0 <= lo < n and 0 <= hi < n):
            return ()
        lower = self._lower[lo]
        if lower is None:
            return ()
        upper = self._upper[hi]
        if upper is None:
            return ()
        return [lower, upper]

    def masked(self, x, y):
        """Direction lies in a configured blind sector (car structure)."""
        if not self.blind_sectors:
            return False
        a = math.atan2(y - self.mount.y_m, x - self.mount.x_m) - self.mount.yaw_rad
        # A direction whose neighbour lookup window reaches into the sector can
        # never be certified either, so the sector edge counts as masked.
        edge = self.NEIGHBOR_WINDOW_RAD + abs(self.increment)
        return (in_blind_sector(a, self.blind_sectors)
                or in_blind_sector(a - edge, self.blind_sectors)
                or in_blind_sector(a + edge, self.blind_sectors))

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
        self.blind_leg = False
        self.blind_distance = 0.0
        self.leg_budget = None
        self.trail_length = 0.0
        # Steer at which the wheels physically stalled (something the lidar
        # cannot see, e.g. a low cart deck). The escape must avoid it.
        self.stall_steer = None
        self.direction = 1
        self.turn = 1
        self.steer = 0.0
        self.still_since = None
        self.normal_distance = 0.0
        self.previous_speed = 0.0
        self._edge = self._perimeter()
        f = footprint
        self._body_r2 = (math.hypot(max(f.front_m, f.rear_m) + f.margin_m,
                                    f.effective_half_width) + 1e-6) ** 2
        self.last_block = None
        self.exhausted_at = None

    @property
    def blind_used(self):
        """Trail-reverse budget for this episode is spent."""
        return self.blind_distance >= self.cfg.blind_reverse_m - 0.05

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

    def _masked_body_shift(self, scan, bx, by, x, y, c, s):
        """Forward gear only: tolerate a sliver the laser can NEVER observe.

        Turning forward, the rear overhang swings out (~1.1 cm at full lock) and
        the inner flank ahead of the rear axle cuts in (~1.7 cm). Where those
        slivers fall inside a configured blind sector (car structure), no scan
        can ever certify them, so the strict rule rejected EVERY turn from
        standstill and the car could only drive dead straight.

        Accepted only when the ray direction is structurally masked AND the
        PHYSICAL body point (margin removed) is still inside the padded
        footprint already occupied, i.e. the sweep stays within margin_m.
        Unmasked unknown rays (dropouts, too-close returns ahead) stay blocking,
        and reverse gear keeps the strict rule.
        """
        f = self.fp
        px = min(max(bx, -f.rear_m), f.front_m)
        py = min(max(by, -f.half_width_m), f.half_width_m)
        if not self._inside(x+px*c-py*s, y+px*s+py*c, pad=1e-8):
            return False
        return scan.masked(x+bx*c-by*s, y+bx*s+by*c)

    def _history_free(self, x, y):
        px, py, yaw = self.pose
        wx = px + x*math.cos(yaw) - y*math.sin(yaw)
        wy = py + x*math.sin(yaw) + y*math.cos(yaw)
        r2 = self._body_r2
        for _, hx, hy, ha, _s in self.history:
            dx, dy = wx-hx, wy-hy
            if dx*dx + dy*dy > r2:
                continue        # cannot lie inside that footprint
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
        self.last_block = None
        if scan is None or not scan.usable:
            self.last_block = ("no_scan", 0.0, 0.0, 0.0)
            return 0.0
        x = y = yaw = 0.0
        step = 0.02
        count = math.ceil(horizon/step)
        # Sampling padding must not create a permanent virtual collision when
        # a visible wall is already near the safety margin. Such a point may
        # stay equally far away or recede, but must never get closer.
        # Only returns the body can reach within the horizon matter. A point
        # farther than (body circumradius + pad) from the current rear-axle
        # origin cannot touch the body at this sample, so skip the exact test.
        # Same result, far fewer _gap() calls (this runs up to ~7x per cycle).
        f = self.fp
        body_r = math.hypot(max(f.front_m, f.rear_m) + f.margin_m,
                            f.effective_half_width) + .016
        reach2 = (horizon + step + body_r) ** 2
        # A return already inside the safety margin (negative gap) is allowed
        # to stay equally far or recede, never to get closer; clamping its
        # floor at 0 made it block even straight motion away from it.
        obstacles = [(ox, oy, min(.015, self._gap(ox, oy)))
                     for ox, oy in scan.points if ox*ox + oy*oy <= reach2]
        body_r2 = body_r * body_r
        g_rear, g_front = -f.rear_m-f.margin_m, f.front_m+f.margin_m
        g_half = f.effective_half_width
        for i in range(count+1):
            distance = i * step
            c, s = math.cos(yaw), math.sin(yaw)
            for ox, oy, clearance_floor in obstacles:
                dx, dy = ox-x, oy-y
                if dx*dx + dy*dy > body_r2:
                    continue
                lx, ly = dx*c+dy*s, -dx*s+dy*c
                # inline self._gap(lx, ly)
                gap = g_rear-lx
                if lx-g_front > gap:
                    gap = lx-g_front
                if abs(ly)-g_half > gap:
                    gap = abs(ly)-g_half
                if gap < clearance_floor-1e-9:
                    self.last_block = ("obstacle", ox, oy, distance)
                    return max(0.0, distance-step)
            for bx, by in self._edge:
                qx, qy = x+bx*c-by*s, y+bx*s+by*c
                # inline self._inside(qx, qy, pad=1e-8)
                if g_rear-1e-8 <= qx <= g_front+1e-8 and abs(qy) <= g_half+1e-8:
                    continue   # already occupied body, not a free-space claim
                if direction > 0 and self._masked_body_shift(scan, bx, by, x, y, c, s):
                    continue
                known = scan.free(qx, qy)
                if not known and not scan.covered(qx, qy):
                    known = ((allow_memory and self._observed_before(qx, qy))
                             or (allow_history and self._history_free(qx, qy)))
                if not known:
                    self.last_block = ("unknown", qx, qy, distance)
                    return max(0.0, distance-step)
            # Worst-case steering transition over the first 10 cm.
            fraction = min(1.0, (distance+step)/0.10)
            angle = current_steer + (steer-current_steer)*fraction
            dyaw = yaw_from_steer(float(direction), angle, self.geo)*step
            x += direction*step*math.cos(yaw+dyaw/2)
            y += direction*step*math.sin(yaw+dyaw/2)
            yaw += dyaw
        return horizon

    def _observe(self, now, speed, yaw_rate, healthy, odom_ok=None):
        dt = 0.0 if self.last_time is None else now-self.last_time
        self.last_time = now
        odom_ok = healthy if odom_ok is None else odom_ok
        if not odom_ok or not 0 <= dt <= 0.5:
            # Dead-reckoning is broken: trail and scan memory can no longer be
            # placed relative to the car. Anything else (a late control tick,
            # a camera dropout) keeps them.
            self.history.clear()
            self.scan_history.clear()
            self.trail_length = 0.0
            return 0.0
        x, y, a = self.pose
        self.pose = (x+speed*dt*math.cos(a+yaw_rate*dt/2),
                     y+speed*dt*math.sin(a+yaw_rate*dt/2), a+yaw_rate*dt)
        self.scan_history = [h for h in self.scan_history if now-h[0] <= self.cfg.history_s]
        # Trail of footprints the car physically occupied while driving forward
        # (normal follow AND forward recovery legs), kept by distance and age.
        if speed > 0.03:
            last = self.history[-1] if self.history else None
            moved = math.hypot(self.pose[0]-last[1], self.pose[1]-last[2]) if last else 1e9
            if moved >= self.cfg.trail_step_m:
                s_cum = (last[4] + moved) if last else 0.0
                self.history.append((now, *self.pose, s_cum))
        if self.history:
            newest = self.history[-1][4]
            self.history = [h for h in self.history
                            if now-h[0] <= self.cfg.trail_max_age_s
                            and newest-h[4] <= self.cfg.trail_m]
        self.trail_length = (self.history[-1][4]-self.history[0][4]) if self.history else 0.0
        if not self.active and self.previous_speed > 0.03 and speed > 0.03:
            self.normal_distance += speed*dt
        if self.active:
            self.leg_distance += abs(speed)*dt
            self.total_distance += abs(speed)*dt
            self.total_yaw += abs(yaw_rate)*dt
            if self.blind_leg and speed < 0:
                self.blind_distance += abs(speed)*dt
        return dt

    def cancel(self):
        # Keep seen-target memory and the blind-reverse flag: faults must not
        # give repeated fresh BLIND reverse budgets (blind_used only resets
        # after real forward travel). An interrupted episode is NOT exhausted:
        # latching it here meant a 0.4 s camera dropout followed by the person
        # reappearing (or one late control tick) left the car "exhausted"
        # before it had moved at all, and it could never clear while blocked.
        self.active = False
        self.phase = "IDLE"
        self.blocked_since = None
        self.previous_speed = 0.0
        # Trail and scan memory stay: the car's own recent path is exactly
        # what a later reverse needs, and cancelling does not break odometry.

    def _brake(self, now, direction):
        self.phase = "BRAKE"
        self.direction = direction
        self.phase_time = now
        self.still_since = None

    def update(self, *, now, scan, healthy, speed, yaw_rate, target, gap,
               bearing, requested_speed, requested_steer, current_steer,
               follow_cap, lost_age, odom_ok=None):
        dt = self._observe(now, speed, yaw_rate, healthy, odom_ok)
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
            self.blind_distance = 0.0
        elif (self.exhausted and self.exhausted_at is not None
              and now - self.exhausted_at >= self.cfg.exhausted_cooldown_s):
            # Bounded retry: a fresh (non-blind) search budget after a pause.
            self.exhausted = False
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
            best = -math.inf
            # Cheapest-penalty first; an angle whose best possible score
            # (.65 - penalty) is already below the best found cannot win, so
            # its full sweep is skipped. The chosen command is unchanged.
            for steer in sorted(dict.fromkeys(angles), key=lambda a: abs(a-requested_steer)):
                if .65 - 0.45*abs(steer-requested_steer) < best:
                    continue
                clear = direct if steer == requested_steer else self.clearance(
                    scan, steer, current_steer=current_steer)
                cap = min(requested_speed, follow_cap, brake_envelope(clear, self.brake))
                if cap >= 0.08:
                    # Prefer alignment with the person once there is adequate room.
                    score = min(clear, .65) - 0.45*abs(steer-requested_steer)
                    choices.append((score, cap, steer))
                    best = max(best, score)
                    if steer == requested_steer and score >= .65:
                        # Maximum possible score; every other angle is penalised
                        # for deviating, so it cannot win. Skip ~5 full sweeps.
                        break
            if choices:
                _, cap, steer = max(choices)
                normal = Command(cap, steer, "TRACKING" if steer == requested_steer else "ALIGNING",
                                 "follow" if steer == requested_steer else "local_path")

        if self.active and target and normal and direct > .45:
            # Person found again with a certified path: hand back to following.
            # Never switch reverse -> forward without a measured stationary dwell.
            # After a physical stall the lidar-certified path is not trusted:
            # finish enough of the escape leg before handing back.
            resume = False
            if self.stall_steer is not None:
                resume = self.phase == "FORWARD" and self.leg_distance >= .30
            elif self.phase == "FORWARD":
                resume = speed >= -0.02 and self.previous_speed >= 0
            elif self.phase == "SCAN":
                resume = True          # still standing, only observing
            elif self.phase == "BRAKE" and self.direction > 0:
                resume = abs(speed) < .02 and self.previous_speed >= 0
            if resume:
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
                self.stall_steer = requested_steer if (stalled and not lost) else None
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
                if normal:
                    out = normal
                elif target and requested_speed > 0:
                    # Person visible but no steer has a certified path. Say so,
                    # whatever the recovery budget: that is the actionable fact.
                    out = Command(state="PATH_BLOCKED", reason="no_observed_path")
                elif target:
                    out = Command(state="HOLDING", reason="hold")
                elif self.exhausted:
                    out = Command(state="RECOVERY_EXHAUSTED", reason="recovery_budget_used")
                else:
                    out = Command(state="SEARCHING_LOST",
                                  reason="no_target" if not self.seen else "recovery_disabled")
                if normal and self.stall_steer is not None and not self.active:
                    self.stall_steer = None
                self.previous_speed = out.speed
                return out

        if (now-self.started >= self.cfg.timeout_s or self.legs >= self.cfg.max_legs
                or self.total_distance >= 6.5 or self.total_yaw >= math.pi + .10):
            self.active = False
            self.exhausted = True
            self.exhausted_at = now
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
            # Rear blind: retrace the car's own recent trail, any steer whose
            # whole swept body stays inside it, within the episode budget.
            budget = self.cfg.blind_reverse_m - self.blind_distance
            if self.direction < 0 and budget >= .05 and self.history:
                m = self.geo.max_steer_rad
                for a in dict.fromkeys([0.0, -self.turn*m, -self.turn*m*.5,
                                        self.turn*m*.5, self.turn*m]):
                    paths.append((min(budget, self.clearance(
                        scan, a, -1, current_steer, allow_history=True,
                        allow_memory=False, horizon=min(.85, budget + .10))), a, True))
            def score(p):
                bonus = -0.08 if self.direction > 0 and p[1] == 0 else 0.0
                if self.direction > 0 and self.stall_steer is not None:
                    # Do not drive back into what stopped the wheels.
                    bonus -= 0.6 * max(0.0, 1.0 - abs(p[1]-self.stall_steer)/self.geo.max_steer_rad)
                elif self.direction < 0 and self.stall_steer is not None:
                    # Reversing with the same-side lock swings the nose away.
                    bonus += 0.05 if p[1]*self.stall_steer > 0 else 0.0
                return p[0] + bonus
            if self.direction > 0 and self.stall_steer is not None:
                m = self.geo.max_steer_rad
                away = -math.copysign(1.0, self.stall_steer) if abs(self.stall_steer) > .02 else -self.turn
                for a in (away*m, away*m*.5):
                    if all(abs(a-p[1]) > 1e-6 for p in paths):
                        paths.append((self.clearance(scan, a, 1, current_steer), a, False))
            clear, steer, blind = max(paths, key=score)
            if clear < (.05 if blind else .10):
                self.legs += 1
                self._brake(now, -self.direction)
                return Command(state="RECOVERY_WAIT", reason="no_observed_path")
            self.steer, self.blind_leg = steer, blind
            self.leg_budget = min(budget, clear) if blind else None
            self.phase = "REVERSE" if self.direction < 0 else "FORWARD"
            self.phase_time = now
            self.leg_distance = 0.0
            self.legs += 1

        if self.direction > 0 and not target and abs(self.steer) < .01:
            turn_steer = self.turn*self.geo.max_steer_rad
            if self.clearance(scan, turn_steer, 1, current_steer) >= .30:
                self.steer = turn_steer

        distance_limit = (self.leg_budget or self.cfg.blind_reverse_m
                          if self.blind_leg else
                          self.cfg.reverse_m if self.direction < 0 else self.cfg.forward_m)
        remaining = distance_limit-self.leg_distance
        clear = self.clearance(scan, self.steer, self.direction, current_steer,
                               allow_history=self.blind_leg,
                               allow_memory=self.direction > 0 or self.blind_leg,
                               horizon=(min(.85, max(remaining, 0.0) + .10)
                                        if self.blind_leg else .85))
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
