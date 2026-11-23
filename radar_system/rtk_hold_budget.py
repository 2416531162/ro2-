"""定位降级限速的平滑（DegradedSpeedLimiter，line_mission 用）。

2026-11-22：原来这里还有一套「喷头推算误差预算」（evaluate_hold：固定解断了以后按协方差 + 轮速比例 / 滑移余量估喷头
横向 3 cm、沿线 10 cm，超了停车），一直没接进任务层、没人调用，连同只给它用的 FusionConfig.hold_* 参数一起删了。
固定解断了怎么办现在只看 rtk_hold_s（Gen5 9 s：满 8 s 停喷、再走 1 s 停车，见 paint_quality.RTK_LOST_PAINT_STOP_S）。
"""


class DegradedSpeedLimiter:
    """短时预算波动只收紧；持续允许的限速可缓慢恢复，避免起步被锁在零速。"""
    RECOVERY_S = 1.0
    RECOVERY_ACCEL_MPS2 = .2

    def __init__(self):
        self.reset()

    def reset(self):
        self.limit = None
        self.healthy_since = None
        self.last_time = None
        self.allowed_since = self.allowed_ceiling = None

    def update(self, now, requested, *, healthy, cruise, previous_speed, dt):
        if self.last_time is not None and not 0 <= now-self.last_time <= .2+1e-9:
            self.healthy_since = None  # 没有连续控制观测的时间不能算作定位已稳定。
            self.allowed_since = self.allowed_ceiling = None
        self.last_time = now
        if requested is not None:
            requested = max(0., min(cruise, requested))
            if self.limit is None:
                # 降级发生在低速行驶/起步中时也不能因限速值较大而反向加速。
                self.limit = min(requested, abs(previous_speed))
            else:
                self.limit = min(self.limit, requested)
        if self.limit is None:
            return None
        if not healthy:
            self.healthy_since = None
            if requested is not None and requested > self.limit+1e-9:
                # 仍在降级但每拍都明确允许行驶：连续一秒确认后恢复到窗口内
                # 最保守的限速。更严预算/零速/观测缺口立即打断，不借偶尔好帧加速。
                if (self.allowed_since is None or now < self.allowed_since
                        or requested < self.allowed_ceiling-1e-9):
                    self.allowed_since, self.allowed_ceiling = now, requested
                if now-self.allowed_since >= self.RECOVERY_S-1e-9:
                    self.limit = min(self.allowed_ceiling, requested,
                                     self.limit+self.RECOVERY_ACCEL_MPS2*max(0., min(.2, dt)))
            else:
                self.allowed_since = self.allowed_ceiling = None
            return self.limit
        self.allowed_since = self.allowed_ceiling = None
        if self.healthy_since is None or now < self.healthy_since:
            self.healthy_since = now
        if now-self.healthy_since < self.RECOVERY_S-1e-9:
            return self.limit
        ceiling = cruise if requested is None else requested
        self.limit = min(ceiling, self.limit+self.RECOVERY_ACCEL_MPS2*max(0., min(.2, dt)))
        if self.limit >= cruise-1e-9:
            self.reset()
            return None
        return self.limit
