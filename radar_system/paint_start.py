"""开喷前的连续稳定证据；仅用于有助跑的前进直线，不控制已开始的喷线。"""
import math

class PaintStartGuard:
    OFFSET_M = .02
    HEADING_RAD = math.radians(1.)
    AGREEMENT_M = .015
    # 收稳后留少量回差，避免厘米级定位波动反复撤销放行；仍严于正常喷线的 3 cm 边界。
    HOLD_OFFSET_M = .025
    HOLD_HEADING_RAD = math.radians(1.5)
    HOLD_AGREEMENT_M = .02
    CALIBRATION_WINDOW_M = .75
    SETTLE_S = .2
    MIN_FRAMES = 2
    STOP_MARGIN_M = .12

    def __init__(self):
        self.stable_s = 0.
        self.frames = 0
        self.last_frame = None
        self.reason = '等待起步稳定'
        self.started = False
        self.rtk_error = self.heading_error = None

    @property
    def ready(self):
        return self.stable_s >= self.SETTLE_S-1e-9 and self.frames >= self.MIN_FRAMES

    def update(self, dt, *, trusted, frame, weight, rtk_error, guide_error, heading_error):
        finite=lambda v:isinstance(v,(int,float)) and math.isfinite(v)
        self.rtk_error=rtk_error if finite(rtk_error) else None
        self.heading_error=heading_error if finite(heading_error) else None
        was_ready=self.ready
        offset=self.HOLD_OFFSET_M if was_ready else self.OFFSET_M
        agreement=self.HOLD_AGREEMENT_M if was_ready else self.AGREEMENT_M
        heading=self.HOLD_HEADING_RAD if was_ready else self.HEADING_RAD
        if not trusted or not finite(frame) or not all(finite(v) for v in (weight,rtk_error)):
            reason='等待可信定位与 IMU'
        elif weight < .99:reason='等待 IMU 完成接管'
        elif not all(finite(v) for v in (guide_error,heading_error)):reason='等待 IMU 完成接管'
        elif abs(rtk_error)>offset:reason='喷头尚未对准线头'
        elif abs(rtk_error-guide_error)>agreement:reason='定位与推算尚未对齐'
        elif abs(heading_error)>heading:reason='方向尚未收稳'
        else:reason=None
        if reason:
            self.stable_s=0.;self.frames=0;self.last_frame=None;self.reason=reason
        else:
            self.stable_s+=max(0.,min(.1,dt))
            if frame != self.last_frame:self.frames+=1;self.last_frame=frame
            self.reason=None if self.ready else '等待连续稳定'
        return self.ready

    def status(self):
        return dict(start_guard=True,start_ready=self.ready,start_started=self.started,
                    start_stable_s=round(self.stable_s,3),start_frames=self.frames,start_blocked=self.reason,
                    start_rtk_error_m=self.rtk_error,start_heading_error_deg=None if self.heading_error is None else math.degrees(self.heading_error))
