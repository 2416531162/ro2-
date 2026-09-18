#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一多人跟踪器:相机 + 激光雷达,在里程计坐标系里跟踪每一个人。

为什么要重写
------------
旧结构是「相机锁一个人(TargetLock)+ 雷达另起一条轨迹接力(LidarPersonTrack)」,
两套状态各管各的,靠一个时间阈值切换:
  - 人出相机画面的瞬间要「交接」,交接失败就傻眼;
  - 相机回来时要重新「认领」,认错就跟错人;
  - 旁边的人只要进了锁定半径,就可能被当成目标。

参考 SPENCER / sobits_follower / leg_tracker 的通行做法,改为:
  1. 每个人一条轨迹,匀速模型卡尔曼滤波,状态在 **odom 系**(车动不影响人的速度估计);
  2. 相机检测、雷达腿部点簇都是这些轨迹的**观测**,按各自采集时刻换算到 odom
     (相机推理有几十~上百毫秒延迟,按收到时刻关联会错位);
  3. 马氏距离门控 + 匈牙利算法全局最优关联,不是「谁近选谁」;
  4. ByteTrack 两级关联:高分框可新建轨迹,低分框(被遮挡、侧身、光线差)
     只用来延续已确认的轨迹 —— 直接减少「时有时无」;
  5. 雷达点簇只更新已确认的轨迹,绝不单独「创造」一个人(箱子、椅子腿太多);
  6. 跟随目标锁的是**轨迹编号**。相机看不到时雷达照常更新同一条轨迹,
     不存在「交接」;雷达单独维持的时间有上限(身份会漂)。
  7. 目标轨迹记录走过的路径点(breadcrumbs),供跟随控制「沿人走过的路走」。

纯 Python,不依赖 ROS / numpy,便于单元测试。
坐标约定:车体系原点在后轴中心,x 前 y 左；实车位姿由共享 /odom 输入。
"""

import math
from collections import deque

__all__ = ["OdomBuffer", "PersonTracker", "hungarian"]

CHI2_2DOF_99 = 9.21


# =============================================================================
# 里程计:积分底盘实测速度,保存最近一段位姿,供「按采集时刻换算」
# =============================================================================

from runtime_config import PROFILE
from robot_core.odometry import PoseHistory as OdomBuffer


# =============================================================================
# 匈牙利算法(最小代价指派,矩形矩阵),n 很小,O(n^3) 足够
# =============================================================================

def hungarian(cost):
    """cost: rows x cols。返回 [(row, col), ...],行列各至多匹配一次。

    >>> sorted(hungarian([[4, 1, 3], [2, 0, 5], [3, 2, 2]]))
    [(0, 1), (1, 0), (2, 2)]
    >>> hungarian([])
    []
    """
    rows = len(cost)
    if rows == 0:
        return []
    cols = len(cost[0])
    if cols == 0:
        return []
    transposed = rows > cols
    if transposed:
        cost = [list(r) for r in zip(*cost)]
        rows, cols = cols, rows
    INF = float("inf")
    u = [0.0] * (rows + 1)
    v = [0.0] * (cols + 1)
    p = [0] * (cols + 1)
    way = [0] * (cols + 1)
    for i in range(1, rows + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (cols + 1)
        used = [False] * (cols + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = INF, 0
            for j in range(1, cols + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(cols + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    pairs = [(p[j] - 1, j - 1) for j in range(1, cols + 1) if p[j] != 0]
    if transposed:
        pairs = [(c, r) for r, c in pairs]
    return pairs


# =============================================================================
# 单条轨迹:匀速模型卡尔曼滤波(odom 系)
# =============================================================================

class Track:
    def __init__(self, tid, t, x, y, r_var, source):
        self.id = tid
        self.t = t
        self.state = [x, y, 0.0, 0.0]
        self.P = [[r_var, 0, 0, 0], [0, r_var, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
        self.created = t
        self.last_update = t
        self.last_camera = t if source == "camera" else None
        # 最近一次「身份可信」的时刻:相机命中,或周围没有其他候选的雷达命中
        self.last_confident = t
        # 在相机视野内却连续没被相机看到的起始时刻(反向证据)
        self.unseen_in_view_since = None
        self.last_source = source
        self.camera_hits = 1 if source == "camera" else 0
        self.lidar_hits = 0 if source == "camera" else 1
        self.confirmed = False
        self.misses = 0
        self.conf = 0.0
        self.label = None
        self.meta = {}
        self.crumbs = deque()          # 走过的路径点 (x, y),odom 系

    # ---- 线性代数小工具(4x4 / 2x2,手写以免依赖 numpy) ----
    def predict(self, t, accel_sigma):
        dt = t - self.t
        if dt <= 0:
            return
        x, y, vx, vy = self.state
        self.state = [x + vx * dt, y + vy * dt, vx, vy]
        P = self.P
        # F P F^T,F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
        FP = [[P[0][j] + dt * P[2][j] for j in range(4)],
              [P[1][j] + dt * P[3][j] for j in range(4)],
              list(P[2]), list(P[3])]
        FPF = [[FP[i][0] + dt * FP[i][2], FP[i][1] + dt * FP[i][3], FP[i][2], FP[i][3]]
               for i in range(4)]
        q = accel_sigma ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        FPF[0][0] += q * dt4 / 4
        FPF[1][1] += q * dt4 / 4
        FPF[0][2] += q * dt3 / 2
        FPF[2][0] += q * dt3 / 2
        FPF[1][3] += q * dt3 / 2
        FPF[3][1] += q * dt3 / 2
        FPF[2][2] += q * dt2
        FPF[3][3] += q * dt2
        self.P = FPF
        self.t = t

    def innovation(self, zx, zy, R):
        """返回 (残差, S, S 的逆, 马氏距离平方)。R = (var_x, var_y, cov_xy)。"""
        rx = zx - self.state[0]
        ry = zy - self.state[1]
        s00 = self.P[0][0] + R[0]
        s11 = self.P[1][1] + R[1]
        s01 = self.P[0][1] + R[2]
        det = s00 * s11 - s01 * s01
        if det <= 1e-12:
            return (rx, ry), None, None, float("inf")
        inv = (s11 / det, -s01 / det, s00 / det)
        d2 = rx * rx * inv[0] + 2 * rx * ry * inv[1] + ry * ry * inv[2]
        return (rx, ry), (s00, s01, s11), inv, d2

    def update(self, zx, zy, R):
        (rx, ry), _S, inv, _d2 = self.innovation(zx, zy, R)
        if inv is None:
            return
        P = self.P
        # K = P H^T S^-1,H 取前两维
        K = [[P[i][0] * inv[0] + P[i][1] * inv[1], P[i][0] * inv[1] + P[i][1] * inv[2]]
             for i in range(4)]
        self.state = [self.state[i] + K[i][0] * rx + K[i][1] * ry for i in range(4)]
        # P = (I - K H) P
        newP = [[P[i][j] - K[i][0] * P[0][j] - K[i][1] * P[1][j] for j in range(4)]
                for i in range(4)]
        # 保持对称
        self.P = [[(newP[i][j] + newP[j][i]) / 2 for j in range(4)] for i in range(4)]

    @property
    def pos(self):
        return self.state[0], self.state[1]

    @property
    def vel(self):
        return self.state[2], self.state[3]

    def position_sigma(self):
        return math.sqrt(max(0.0, (self.P[0][0] + self.P[1][1]) / 2))

    def add_crumb(self, spacing, max_len):
        x, y = self.pos
        if not self.crumbs or math.hypot(x - self.crumbs[-1][0], y - self.crumbs[-1][1]) >= spacing:
            self.crumbs.append((x, y))
            while len(self.crumbs) > max_len:
                self.crumbs.popleft()


# =============================================================================
# 跟踪器
# =============================================================================

class PersonTracker:
    """观测接口:add_camera() / add_lidar();查询:target_view()。"""

    def __init__(self, high_conf=0.45, low_conf=0.15, confirm_hits=3,
                 accel_sigma=1.5, gate_d2=CHI2_2DOF_99, gate_max_m=1.2,
                 tentative_timeout_s=0.5, confirmed_timeout_s=1.5,
                 lidar_only_max_s=8.0, reacquire_after_s=1.0,
                 reacquire_radius_m=1.5, max_tracks=12,
                 crumb_spacing_m=0.10, crumb_max=80, prefer_distance_m=1.0,
                 lidar_ambiguity_m=0.8, unseen_in_view_max_s=1.5):
        self.high_conf = high_conf
        self.low_conf = low_conf
        self.confirm_hits = confirm_hits
        self.accel_sigma = accel_sigma
        self.gate_d2 = gate_d2
        self.gate_max_m = gate_max_m
        self.tentative_timeout_s = tentative_timeout_s
        self.confirmed_timeout_s = confirmed_timeout_s
        self.lidar_only_max_s = lidar_only_max_s
        self.reacquire_after_s = reacquire_after_s
        self.reacquire_radius_m = reacquire_radius_m
        self.max_tracks = max_tracks
        self.crumb_spacing_m = crumb_spacing_m
        self.crumb_max = crumb_max
        self.prefer_distance_m = prefer_distance_m
        self.lidar_ambiguity_m = lidar_ambiguity_m
        self.unseen_in_view_max_s = unseen_in_view_max_s
        self.dropped_unseen = 0
        self.odom = OdomBuffer()
        self.tracks = []
        self.next_id = 1
        self.target_id = None
        self.target_lost_at = None
        self.last_target_pos = None
        self.rejected = 0
        self.switches = 0
        self.reacquires = 0
        self.lidar_enabled = True
        self.camera_in_view = None

    # ---------------------------------------------------------------- 里程计
    def step_odom(self, t, speed, yaw_rate, ok=True):
        if not ok:
            # 里程计断了:旧观测无法再和现在对齐,轨迹位置也就失去意义
            if self.odom.t is not None:
                self.reset()
            return
        self.odom.step(t, speed, yaw_rate)

    def reset(self, keep_odom=False):
        if not keep_odom:
            self.odom.reset()
        self.tracks = []
        self.target_id = None
        self.target_lost_at = None
        self.last_target_pos = None

    # ---------------------------------------------------------------- 观测噪声
    @staticmethod
    def camera_R(px, py, pose, depth_sigma=None):
        """相机:沿视线方向的深度误差随距离增大,横向误差小。转成 odom 系协方差。"""
        rng = math.hypot(px, py)
        s_long = depth_sigma if depth_sigma is not None else 0.06 + 0.03 * rng
        s_lat = 0.05 + 0.01 * rng
        ang = math.atan2(py, px) + pose[2]
        c, s = math.cos(ang), math.sin(ang)
        a, b = s_long ** 2, s_lat ** 2
        return (a * c * c + b * s * s, a * s * s + b * c * c, (a - b) * c * s)

    LIDAR_R = (0.08 ** 2, 0.08 ** 2, 0.0)

    # ---------------------------------------------------------------- 关联
    def _predict_all(self, t):
        for tr in self.tracks:
            if t > tr.t:
                tr.predict(t, self.accel_sigma)

    def _associate(self, tracks, meas, t):
        """meas: [(ox, oy, R)]。返回 [(track, meas_index)]。"""
        if not tracks or not meas:
            return []
        big = 1e6
        cost = []
        for tr in tracks:
            row = []
            for ox, oy, R in meas:
                # 观测比轨迹时刻早(相机延迟):按轨迹速度把观测推到轨迹时刻
                ox, oy = self._shift(ox, oy, t, tr)
                _, _, _, d2 = tr.innovation(ox, oy, R)
                dist = math.hypot(ox - tr.state[0], oy - tr.state[1])
                ok = d2 <= self.gate_d2 and dist <= self.gate_max_m
                row.append(d2 if ok else big)
            cost.append(row)
        pairs = []
        for r, c in hungarian(cost):
            if cost[r][c] < big:
                pairs.append((tracks[r], c))
            else:
                self.rejected += 1
        return pairs

    def _shift(self, ox, oy, t_meas, tr):
        """把 t_meas 时刻的观测平移到轨迹当前时刻(轨迹已预测到更晚)。"""
        lag = tr.t - t_meas
        if lag <= 0:
            return ox, oy
        vx, vy = tr.vel
        return ox + vx * lag, oy + vy * lag

    def add_camera(self, detections, t_meas, t_now, in_view=None):
        """detections: [{'x','y' (车体系, t_meas 时刻), 'conf', 'label', 'depth_sigma', ...}]

        in_view(x, y) -> bool:车体系位置是否在相机可靠视野内。给出时启用反向证据:
        已确认的轨迹明明在视野里,却持续没被相机看到,说明它不是人
        (雷达把柱子、椅子腿当成了人),删掉。
        """
        pose = self.odom.pose_at(t_meas)
        if pose is None:
            return
        self.camera_in_view = in_view
        self._predict_all(t_now)
        high, low = [], []
        for d in detections:
            conf = float(d.get("conf", 0.0))
            if conf < self.low_conf:
                continue
            ox, oy = OdomBuffer.vehicle_to_odom(pose, d["x"], d["y"])
            R = self.camera_R(d["x"], d["y"], pose, d.get("depth_sigma"))
            (high if conf >= self.high_conf else low).append((ox, oy, R, d))

        # 第一级:所有轨迹 x 高分框
        matched = set()
        used_high = set()
        meas_h = [(ox, oy, R) for ox, oy, R, _ in high]
        for tr, j in self._associate(self.tracks, meas_h, t_meas):
            ox, oy, R, d = high[j]
            ox, oy = self._shift(ox, oy, t_meas, tr)
            self._apply(tr, ox, oy, R, "camera", t_now, d)
            matched.add(tr.id)
            used_high.add(j)
        # 第二级:剩下的**已确认**轨迹 x 低分框(ByteTrack)
        rest = [tr for tr in self.tracks if tr.confirmed and tr.id not in matched]
        meas_l = [(ox, oy, R) for ox, oy, R, _ in low]
        for tr, j in self._associate(rest, meas_l, t_meas):
            ox, oy, R, d = low[j]
            ox, oy = self._shift(ox, oy, t_meas, tr)
            self._apply(tr, ox, oy, R, "camera", t_now, d)
            matched.add(tr.id)
        # 反向证据
        if in_view is not None:
            pose_now = self.odom.current()
            for tr in self.tracks:
                if not tr.confirmed:
                    continue
                if tr.id in matched:
                    tr.unseen_in_view_since = None
                    continue
                vx, vy = OdomBuffer.odom_to_vehicle(pose_now, *tr.pos)
                if in_view(vx, vy):
                    if tr.unseen_in_view_since is None:
                        tr.unseen_in_view_since = t_now
                else:
                    tr.unseen_in_view_since = None
        # 没匹配上的高分框 -> 新的待确认轨迹;低分框不建轨迹
        for j, (ox, oy, R, d) in enumerate(high):
            if j in used_high or len(self.tracks) >= self.max_tracks:
                continue
            tr = Track(self.next_id, t_now, ox, oy, max(R[0], R[1]), "camera")
            self.next_id += 1
            tr.conf = float(d.get("conf", 0.0))
            tr.label = d.get("label")
            tr.meta = {k: d[k] for k in ("range_source", "depth_ratio", "raw_z") if k in d}
            if tr.camera_hits >= self.confirm_hits:
                tr.confirmed = True
            self.tracks.append(tr)
        self._housekeeping(t_now)

    def add_lidar(self, clusters, t_meas, t_now):
        """clusters: [(x, y)] 车体系(t_meas 时刻)。只更新已确认轨迹,不建新轨迹。"""
        if not self.lidar_enabled:
            return
        pose = self.odom.pose_at(t_meas)
        if pose is None:
            return
        self._predict_all(t_now)
        meas = []
        for cx, cy in clusters:
            ox, oy = OdomBuffer.vehicle_to_odom(pose, cx, cy)
            meas.append((ox, oy, self.LIDAR_R))
        tracks = [tr for tr in self.tracks if tr.confirmed]
        amb2 = self.lidar_ambiguity_m ** 2
        pairs = self._associate(tracks, meas, t_meas)
        target = self._get(self.target_id) if self.target_id is not None else None

        # A normal gate is deliberately tight so a chair leg cannot move a
        # track by a metre in one frame.  A confirmed target crossing the rear
        # of the car is the one exception: once the camera has gone quiet, a
        # unique rear cluster inside the short handoff window is stronger
        # evidence than a stale front association.  Work this out before
        # applying the normal pairs so a front clutter hit cannot suppress it.
        forced = None
        target_pair = next(((tr, j) for tr, j in pairs if tr is target), None)
        occupied_by_other = {j for tr, j in pairs if tr is not target}
        if (target is not None and target.confirmed and target.last_camera is not None
                and t_now - target.last_camera >= self.reacquire_after_s):
            candidates = []
            for j, (mx, my, R) in enumerate(meas):
                sx, sy = self._shift(mx, my, t_meas, target)
                d = math.hypot(sx - target.state[0], sy - target.state[1])
                if d <= self.reacquire_radius_m and j not in occupied_by_other:
                    candidates.append((d, j, sx, sy, R))

            # If the old track is still in front, prefer a unique cluster that
            # is already behind the axle.  This is the fast rear crossing case;
            # front candidates are commonly the point which caused the miss.
            pose_now = self.odom.current()
            rear = []
            for candidate in candidates:
                bx, by = OdomBuffer.odom_to_vehicle(pose_now, candidate[2], candidate[3])
                if bx < -0.05 or abs(math.atan2(by, bx)) > math.radians(110.0):
                    rear.append(candidate)
            old_x, _ = OdomBuffer.odom_to_vehicle(pose_now, *target.pos)
            if old_x >= 0.0 and rear:
                rear.sort(key=lambda c: c[0])
                best = rear[0]
                ambiguous = len(rear) > 1 and rear[1][0] - best[0] < self.lidar_ambiguity_m
                if not ambiguous:
                    forced = best
            elif old_x >= 0.0 and target_pair is None and candidates:
                candidates.sort(key=lambda c: c[0])
                best = candidates[0]
                ambiguous = len(candidates) > 1 and candidates[1][0] - best[0] < self.lidar_ambiguity_m
                if not ambiguous:
                    forced = best

        for tr, j in pairs:
            if forced is not None and tr is target:
                continue
            ox, oy, R = meas[j]
            ox, oy = self._shift(ox, oy, t_meas, tr)
            # Check alternatives against the SAME pre-update prediction and
            # timestamp used for association. A nearby return outside the gate
            # cannot be this track; it must not expire a stationary rear target.
            ambiguous = False
            for k, (mx, my, other_R) in enumerate(meas):
                if k == j:
                    continue
                mx, my = self._shift(mx, my, t_meas, tr)
                if (mx - ox) ** 2 + (my - oy) ** 2 > amb2:
                    continue
                _, _, _, d2 = tr.innovation(mx, my, other_R)
                if (d2 <= self.gate_d2
                        and math.hypot(mx - tr.pos[0], my - tr.pos[1]) <= self.gate_max_m):
                    ambiguous = True
                    break
            self._apply(tr, ox, oy, R, "lidar", t_now, None)
            if not ambiguous:
                tr.last_confident = t_now
        if forced is not None:
            _, j, ox, oy, R = forced
            self._relocate(target, ox, oy, R, t_now)
        self._housekeeping(t_now)

    def _relocate(self, tr, ox, oy, R, t_now):
        """Snap a confirmed track to a unique rear radar return.

        A Kalman update is intentionally gradual.  For a front-to-rear
        crossing that would make the estimated person pass through the car
        for several control cycles, exactly when the controller must turn.
        The radar association has already been gated and ambiguity checked;
        relocate the position immediately while bounding the derived velocity.
        """
        old_x, old_y = tr.pos
        dt = max(t_now - tr.last_update, 0.05)
        vx, vy = (ox - old_x) / dt, (oy - old_y) / dt
        speed = math.hypot(vx, vy)
        if speed > 2.5:
            scale = 2.5 / speed
            vx, vy = vx * scale, vy * scale
        tr.state = [ox, oy, vx, vy]
        tr.P[0][0] = max(R[0], 0.08 ** 2)
        tr.P[1][1] = max(R[1], 0.08 ** 2)
        tr.P[0][1] = tr.P[1][0] = 0.0
        tr.P[0][2] = tr.P[0][3] = tr.P[1][2] = tr.P[1][3] = 0.0
        tr.P[2][0] = tr.P[2][1] = tr.P[3][0] = tr.P[3][1] = 0.0
        tr.t = tr.last_update = t_now
        tr.last_source = "lidar"
        tr.last_confident = t_now
        tr.misses = 0
        tr.lidar_hits += 1
        self.reacquires += 1
        if tr.id == self.target_id:
            tr.add_crumb(self.crumb_spacing_m, self.crumb_max)

    def _apply(self, tr, ox, oy, R, source, t_now, det):
        tr.update(ox, oy, R)
        tr.last_update = t_now
        tr.last_source = source
        tr.misses = 0
        if source == "camera":
            tr.last_camera = t_now
            tr.last_confident = t_now
            tr.camera_hits += 1
            if det is not None:
                tr.conf = float(det.get("conf", tr.conf))
                tr.label = det.get("label", tr.label)
                tr.meta = {k: det[k] for k in ("range_source", "depth_ratio", "raw_z") if k in det}
            if tr.camera_hits >= self.confirm_hits:
                tr.confirmed = True
        else:
            tr.lidar_hits += 1
        if tr.id == self.target_id:
            tr.add_crumb(self.crumb_spacing_m, self.crumb_max)

    # ---------------------------------------------------------------- 维护
    def _housekeeping(self, t_now):
        keep = []
        for tr in self.tracks:
            # Radar may have moved the track outside the camera view since
            # the last image. Old negative evidence no longer applies there.
            if tr.unseen_in_view_since is not None and self.camera_in_view is not None:
                bx, by = OdomBuffer.odom_to_vehicle(self.odom.current(), *tr.pos)
                if not self.camera_in_view(bx, by):
                    tr.unseen_in_view_since = None
            age = t_now - tr.last_update
            limit = self.confirmed_timeout_s if tr.confirmed else self.tentative_timeout_s
            if age > limit:
                continue
            if tr.confirmed and t_now - tr.last_confident > self.lidar_only_max_s:
                continue            # 纯靠雷达且一直有干扰,身份不可信
            if (tr.unseen_in_view_since is not None
                    and t_now - tr.unseen_in_view_since > self.unseen_in_view_max_s):
                self.dropped_unseen += 1
                continue            # 在相机视野里却一直看不到:不是人
            if tr.position_sigma() > 1.5:
                continue
            keep.append(tr)
        self.tracks = keep
        self._select_target(t_now)

    def _get(self, tid):
        for tr in self.tracks:
            if tr.id == tid:
                return tr
        return None

    def _select_target(self, t_now):
        target = self._get(self.target_id) if self.target_id is not None else None
        if target is not None:
            self.last_target_pos = target.pos
            self.target_lost_at = None
            cam_age = t_now - target.last_camera if target.last_camera is not None else 1e9
            if cam_age <= self.reacquire_after_s:
                return
            # 目标只靠雷达维持时,若附近出现一条相机正看着的已确认轨迹,
            # 说明雷达轨迹可能已经漂移/挂在别的东西上:换到相机那条(就近原则)
            alt = self._best_near(t_now, target.pos, self.reacquire_radius_m, exclude=target.id)
            if alt is not None:
                self._switch(alt, target)
            return
        if self.target_id is not None and self.target_lost_at is None:
            self.target_lost_at = t_now
        # 没有目标:刚丢失时优先在丢失位置附近找;否则按「最正前方、最接近期望距离」挑
        cand = None
        if self.last_target_pos is not None and self.target_lost_at is not None \
                and t_now - self.target_lost_at < 3.0:
            cand = self._best_near(t_now, self.last_target_pos, self.reacquire_radius_m + 1.0)
        if cand is None:
            cand = self._best_front(t_now)
        if cand is not None:
            self._switch(cand, None)

    def _camera_fresh(self, tr, t_now, max_age=0.3):
        return tr.confirmed and tr.last_camera is not None and t_now - tr.last_camera <= max_age

    def _best_near(self, t_now, pos, radius, exclude=None):
        best, best_d = None, radius
        for tr in self.tracks:
            if tr.id == exclude or not self._camera_fresh(tr, t_now):
                continue
            d = math.hypot(tr.pos[0] - pos[0], tr.pos[1] - pos[1])
            if d <= best_d:
                best, best_d = tr, d
        return best

    def _best_front(self, t_now):
        pose = self.odom.current()
        best, best_score = None, float("inf")
        for tr in self.tracks:
            if not self._camera_fresh(tr, t_now):
                continue
            vx, vy = OdomBuffer.odom_to_vehicle(pose, *tr.pos)
            if vx <= 0:
                continue
            score = abs(vy) * 1.5 + abs(vx - self.prefer_distance_m)
            if score < best_score:
                best, best_score = tr, score
        return best

    def _switch(self, new, old):
        if old is not None:
            self.switches += 1
            # 继承路径点:人没变,只是轨迹编号换了
            new.crumbs = deque(old.crumbs)
        self.target_id = new.id
        self.target_lost_at = None
        new.add_crumb(self.crumb_spacing_m, self.crumb_max)

    # ---------------------------------------------------------------- 查询
    def target_view(self, t_now, lidar_after_s=0.25):
        """目标在「当前车体系」里的视图,没有目标返回 None。"""
        tr = self._get(self.target_id) if self.target_id is not None else None
        if tr is None:
            return None
        if t_now > tr.t:
            tr.predict(t_now, self.accel_sigma)
        pose = self.odom.current()
        px, py = OdomBuffer.odom_to_vehicle(pose, *tr.pos)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        vx, vy = tr.vel
        v_fwd = c * vx + s * vy          # 人的对地速度在车头方向上的分量
        v_lat = -s * vx + c * vy
        cam_age = t_now - tr.last_camera if tr.last_camera is not None else 1e9
        upd_age = t_now - tr.last_update
        if cam_age <= lidar_after_s:
            source = "camera"
        elif tr.last_source == "lidar" and upd_age <= 0.3:
            source = "lidar"
        else:
            source = "predicted"
        crumbs = [OdomBuffer.odom_to_vehicle(pose, cx, cy) for cx, cy in tr.crumbs]
        return {
            "id": tr.id, "x": px, "y": py, "v_fwd": v_fwd, "v_lat": v_lat,
            "source": source, "camera_age": cam_age, "update_age": upd_age,
            "sigma": tr.position_sigma(), "conf": tr.conf, "label": tr.label,
            "confident_age": t_now - tr.last_confident,
            "meta": dict(tr.meta), "lidar_hits": tr.lidar_hits,
            "camera_hits": tr.camera_hits, "crumbs": crumbs,
        }

    def prune_crumbs(self, min_ahead_m=0.25):
        """丢掉车已经走过的路径点(在车体系里落到车后或贴近后轴)。"""
        tr = self._get(self.target_id) if self.target_id is not None else None
        if tr is None:
            return
        pose = self.odom.current()
        while tr.crumbs:
            vx, vy = OdomBuffer.odom_to_vehicle(pose, *tr.crumbs[0])
            if vx < min_ahead_m or math.hypot(vx, vy) < min_ahead_m:
                tr.crumbs.popleft()
            else:
                break

    def summary(self, t_now, limit=6):
        pose = self.odom.current()
        out = []
        for tr in sorted(self.tracks, key=lambda t: t.id)[:limit]:
            vx, vy = OdomBuffer.odom_to_vehicle(pose, *tr.pos)
            out.append({
                "id": tr.id, "x": round(vx, 2), "y": round(vy, 2),
                "confirmed": tr.confirmed, "target": tr.id == self.target_id,
                "src": tr.last_source,
                "cam_age": round(t_now - tr.last_camera, 2) if tr.last_camera is not None else None,
            })
        return out


if __name__ == "__main__":
    import doctest
    doctest.testmod()
