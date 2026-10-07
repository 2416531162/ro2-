"""共同轮程比例的保守在线标定；不可变状态可随融合历史回放。

用互不重叠的直行段和原始固定解拟合，不把融合位置当成独立真值。
不估左右轮各自比例：直行不足以可靠区分轮径差、滑移与 IMU 偏差。
"""
import math
import statistics
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class WheelScale:
    applied: float
    error: float
    rows: tuple = ()
    estimates: tuple = ()
    blocks: int = 0
    rejected: int = 0

    def clear(self):
        return replace(self, rows=()) if self.rows else self

    def observe(self, t, odo, x, y, yaw, sigma, *, configured, distance_m):
        rows = self.rows
        if rows:
            last, first = rows[-1], rows[0]
            turn = abs(math.atan2(math.sin(yaw-first[4]), math.cos(yaw-first[4])))
            if t-last[0] > .5 or (odo-last[1])*(last[1]-first[1]) < 0 or turn > math.radians(3):
                rows = ()
            elif t-last[0] < .25 - 1e-9:
                return self
        rows += ((t, odo, x, y, yaw, sigma),)
        span = abs(odo-rows[0][1])
        if span < distance_m:
            return replace(self, rows=rows[-512:])
        # 下一段从下一帧开始，不重复使用本段观测。
        base = replace(self, rows=())
        if len(rows) < 20 or t-rows[0][0] < 5:
            return replace(base, rejected=self.rejected+1)
        n = len(rows)
        mo, mx, my = (sum(r[k] for r in rows)/n for k in (1, 2, 3))
        voo = sum((r[1]-mo)**2 for r in rows)
        bx = sum((r[1]-mo)*(r[2]-mx) for r in rows)/voo
        by = sum((r[1]-mo)*(r[3]-my) for r in rows)/voo
        scale = math.hypot(bx, by)
        residual = math.sqrt(sum((r[2]-mx-bx*(r[1]-mo))**2
                                 + (r[3]-my-by*(r[1]-mo))**2 for r in rows)/n)
        noise = max(.01, max(r[5] for r in rows), residual)
        course = math.atan2(by, bx)
        mismatch = abs(math.atan2(math.sin(course-yaw), math.cos(course-yaw)))
        if not .9*configured < scale < 1.1*configured or residual > .025 or mismatch > math.radians(3):
            return replace(base, rejected=self.rejected+1)
        # GNSS 连续帧相关，端点误差不能按 20 Hz 样本数除小。
        bound = 2*math.sqrt(2)*noise/span
        estimates = (self.estimates + ((scale, bound),))[-6:]
        target = statistics.median(v[0] for v in estimates)
        scatter = max(abs(v[0]-target) for v in estimates)
        applied, error = self.applied, self.error
        if len(estimates) >= 3:
            error = max(.005, max(v[1] for v in estimates)+scatter+abs(target-applied)) / applied
            if scatter <= .0075*configured:
                applied += max(-.005*configured, min(.005*configured, target-applied))
                error = max(.005, max(v[1] for v in estimates)+scatter+abs(target-applied)) / applied
        return replace(base, estimates=estimates, blocks=self.blocks+1, applied=applied, error=error)
