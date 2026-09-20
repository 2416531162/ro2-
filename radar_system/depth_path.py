"""Conservative depth corroboration at the calibrated lidar scan plane.

Pure NumPy; no ROS or actuator access. Camera free space can supplement missing
lidar coverage, but never override a finite lidar obstacle. Invalid pixels,
out-of-view points, the minimum-range blind zone and stale frames stay unknown.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class DepthCalibration:
    optical_frame: str
    x_m: float
    y_m: float
    yaw_rad: float
    pitch_rad: float
    lidar_minus_camera_height_m: float
    min_depth_m: float = .35
    max_depth_m: float = 2.5

    def __post_init__(self):
        values = (self.x_m, self.y_m, self.yaw_rad, self.pitch_rad,
                  self.lidar_minus_camera_height_m, self.min_depth_m, self.max_depth_m)
        if not self.optical_frame or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError('invalid depth calibration')
        if (abs(self.x_m) > 2 or abs(self.y_m) > 2 or abs(self.pitch_rad) > .6
                or abs(self.yaw_rad) > math.pi or abs(self.lidar_minus_camera_height_m) > 2
                or not .15 <= self.min_depth_m < self.max_depth_m <= 6):
            raise ValueError('depth calibration outside supported bounds')

    @classmethod
    def load(cls, path, profile_hash):
        data = json.loads(Path(path).read_text())
        if data.pop('schema_version', None) != 1 or data.pop('profile_hash', None) != profile_hash:
            raise ValueError('depth calibration profile mismatch')
        return cls(**data)


@dataclass(frozen=True)
class DepthIntrinsics:
    width: int
    height: int
    frame: str
    stamp: float
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_info(cls, *, width, height, frame, stamp, k, d, r, p, binning, roi):
        # This path consumes the raw depth image, so K maps its pixels. P is for
        # rectified images and must not veto a valid undistorted raw stream.
        # Do not silently apply K to distorted, cropped or binned pixels.
        if (not frame or not 3 <= width <= 1920 or not 3 <= height <= 1080
                or len(k) != 9 or len(r) != 9 or len(p) != 12
                or not all(math.isfinite(v) for v in [stamp, *k, *d, *r, *p])
                or stamp <= 0 or k[0] <= 0 or k[4] <= 0
                or not 0 <= k[2] < width or not 0 <= k[5] < height
                or any(abs(v) > 1e-8 for v in d)
                or any(v not in (0, 1) for v in binning)
                or any(roi)):
            raise ValueError('unsupported depth intrinsics')
        if (not np.allclose(r, np.eye(3).ravel(), atol=1e-8)
                or not np.allclose(k, [k[0], 0, k[2], 0, k[4], k[5], 0, 0, 1], atol=1e-8)):
            raise ValueError('depth raw intrinsics/rectification mismatch')
        return cls(width, height, frame.lstrip('/'), stamp, k[0], k[4], k[2], k[5])


class DepthEvidence:
    """One immutable metric-depth frame, located at its capture pose."""
    MARGIN_M = .05
    PLANE_HALF_BAND_M = .03

    def __init__(self, depth_m, intrinsics, calibration, stamp, pose):
        self.info, self.cal, self.stamp, self.pose = intrinsics, calibration, stamp, tuple(pose)
        d = np.asarray(depth_m, dtype=np.float32)
        if d.shape != (intrinsics.height, intrinsics.width):
            raise ValueError('depth shape/intrinsics mismatch')
        valid = np.isfinite(d) & (d >= calibration.min_depth_m) & (d <= calibration.max_depth_m)
        # Entire 3x3 patch must have valid depth to certify free space. A closer
        # valid pixel still vetoes clearance even if another pixel has a hole.
        padded = np.pad(np.where(valid, d, np.inf), 1, constant_values=np.inf)
        masks = np.pad(valid, 1, constant_values=False)
        self.minimum = np.minimum.reduce([padded[y:y+d.shape[0], x:x+d.shape[1]]
                                          for y in range(3) for x in range(3)])
        self.all_valid = np.logical_and.reduce([masks[y:y+d.shape[0], x:x+d.shape[1]]
                                               for y in range(3) for x in range(3)])
        self.valid_ratio = float(valid.mean())
        v, u = np.nonzero(valid)
        z = d[v, u]
        xo = (u-intrinsics.cx)*z/intrinsics.fx
        yo = (v-intrinsics.cy)*z/intrinsics.fy
        c, s = math.cos(calibration.pitch_rad), math.sin(calibration.pitch_rad)
        forward, left, up = z*c-yo*s, -xo, -z*s-yo*c
        band = np.abs(up-calibration.lidar_minus_camera_height_m) <= self.PLANE_HALF_BAND_M
        cy, sy = math.cos(calibration.yaw_rad), math.sin(calibration.yaw_rad)
        pts = np.column_stack((calibration.x_m+cy*forward[band]-sy*left[band],
                               calibration.y_m+sy*forward[band]+cy*left[band]))
        # Keep all actual samples in the scan-plane band; no artificial returns.
        self.points_np = pts

    def at_pose(self, pose):
        return DepthView(self, pose)

    def query_many(self, x, y):
        x, y = np.broadcast_arrays(x, y)
        cal, info = self.cal, self.info
        cy, sy = math.cos(cal.yaw_rad), math.sin(cal.yaw_rad)
        dx, dy = x-cal.x_m, y-cal.y_m
        forward, left = cy*dx+sy*dy, -sy*dx+cy*dy
        c, s = math.cos(cal.pitch_rad), math.sin(cal.pitch_rad)
        free = np.ones(x.shape, dtype=bool)
        blocked = np.zeros(x.shape, dtype=bool)
        # Check a thin vertical band around the actual lidar plane, not the
        # person's bounding box or a fixed image row of unrelated height.
        for delta in (-self.PLANE_HALF_BAND_M, 0., self.PLANE_HALF_BAND_M):
            up = cal.lidar_minus_camera_height_m+delta
            zo, yo = c*forward-s*up, -s*forward-c*up
            legal = (np.isfinite(zo) & np.isfinite(left) & (zo >= cal.min_depth_m)
                     & (zo <= cal.max_depth_m-self.MARGIN_M))
            safe_z = np.where(legal, zo, 1.)
            uf = info.cx-info.fx*left/safe_z
            vf = info.cy+info.fy*yo/safe_z
            finite = np.isfinite(uf) & np.isfinite(vf)
            u = np.rint(np.where(finite, uf, 0)).astype(np.int64)
            v = np.rint(np.where(finite, vf, 0)).astype(np.int64)
            legal &= finite & (u >= 1) & (u < info.width-1) & (v >= 1) & (v < info.height-1)
            u, v = np.clip(u, 0, info.width-1), np.clip(v, 0, info.height-1)
            minimum = self.minimum[v, u]
            clear = legal & self.all_valid[v, u] & (minimum > zo+self.MARGIN_M)
            blocked |= legal & np.isfinite(minimum) & (minimum <= zo+self.MARGIN_M)
            free &= clear
        return free, blocked


class DepthView:
    """Current base-frame view of a depth image using measured odometry."""
    def __init__(self, evidence, pose):
        self.evidence, self.pose = evidence, tuple(pose)
        ex, ey, ea = evidence.pose
        px, py, pa = self.pose
        pts = evidence.points_np
        wx = ex+math.cos(ea)*pts[:, 0]-math.sin(ea)*pts[:, 1]
        wy = ey+math.sin(ea)*pts[:, 0]+math.cos(ea)*pts[:, 1]
        self.points_np = np.column_stack((math.cos(pa)*(wx-px)+math.sin(pa)*(wy-py),
                                         -math.sin(pa)*(wx-px)+math.cos(pa)*(wy-py)))

    def query_many(self, x, y):
        px, py, pa = self.pose
        ex, ey, ea = self.evidence.pose
        wx = px+math.cos(pa)*x-math.sin(pa)*y-ex
        wy = py+math.sin(pa)*x+math.cos(pa)*y-ey
        return self.evidence.query_many(math.cos(ea)*wx+math.sin(ea)*wy,
                                       -math.sin(ea)*wx+math.cos(ea)*wy)


class DepthPathSensor:
    MAX_AGE_S = .20
    MAX_PAIR_S = .08

    def __init__(self):
        self.calibration = None
        self.intrinsics = None
        self.evidence = None
        self.reason = 'calibration_missing'
        self.calibration_reason = 'calibration_missing'
        self.received = 0
        self.accepted = 0
        self.last_received = None
        self.calibration_error = None
        self.intrinsics_error = None

    def load_calibration(self, path, profile_hash):
        self.evidence = None
        try:
            self.calibration = DepthCalibration.load(path, profile_hash)
            self.calibration_error = None
            self.calibration_reason = None
            self.reason = 'waiting_depth'
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.calibration = None
            self.calibration_error = str(exc)
            self.calibration_reason = ('calibration_missing' if isinstance(exc, FileNotFoundError)
                                       else 'calibration_invalid')
            self.reason = self.calibration_reason

    def invalidate(self, reason):
        self.evidence = None
        self.reason = reason

    def observe(self, depth_m, stamp, pose, frame, ros_stamp, now):
        self.received += 1
        self.last_received = now
        self.evidence = None
        if self.calibration is None:
            return
        if not 0 <= now-stamp <= self.MAX_AGE_S:
            self.reason = 'depth_stale'
            return
        if pose is None:
            self.reason = 'pose_missing'
            return
        info = self.intrinsics
        if (info is None or info.frame != frame.lstrip('/')
                or frame.lstrip('/') != self.calibration.optical_frame.lstrip('/')
                or abs(info.stamp-ros_stamp) > self.MAX_PAIR_S):
            self.reason = 'intrinsics_mismatch'
            return
        try:
            self.evidence = DepthEvidence(depth_m, info, self.calibration, stamp, pose)
        except (ValueError, TypeError):
            self.reason = 'invalid_depth'
            return
        self.accepted += 1
        self.reason = 'ready' if self.evidence.valid_ratio > 0 else 'depth_invalid'

    def view(self, now, pose):
        if self.evidence is None or pose is None:
            return None
        if not 0 <= now-self.evidence.stamp <= self.MAX_AGE_S:
            self.reason = 'depth_stale'
            return None
        return self.evidence.at_pose(pose)

    def status(self, now):
        age = None if self.evidence is None else now-self.evidence.stamp
        reason = (self.calibration_reason if self.calibration is None else
                  'depth_stale' if age is not None and not 0 <= age <= self.MAX_AGE_S else self.reason)
        return dict(reason=reason, calibration_error=self.calibration_error,
                    intrinsics_error=self.intrinsics_error,
                    calibrated=self.calibration is not None, received=self.received, accepted=self.accepted,
                    age_ms=round(age*1000) if age is not None else None,
                    valid_ratio=round(self.evidence.valid_ratio, 3) if self.evidence is not None else None)
