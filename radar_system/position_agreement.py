"""定位观测先与惯性推算比对，再决定是否学习；固定解标签本身不构成恢复证据。"""
import math

# 沿用定位维持的 8 s 停喷、再过 1 s 停车规则，不能因仍报固定解而无限延长。
HOLD_PAINT_STOP_S = 8.
HOLD_MOTION_STOP_S = 9.


class PositionAgreement:
    def __init__(self, *, enter_m=.02, recover_m=.01, recover_s=1., blend_s=1.,
                 enter_heading=math.radians(1.), recover_heading=math.radians(.5)):
        self.enter_m,self.recover_m,self.recover_s,self.blend_s=enter_m,recover_m,recover_s,blend_s
        self.enter_heading,self.recover_heading=enter_heading,recover_heading
        self.holding=False
        self.age_s=self.agree_s=0.
        self.weight=1.
        self.reason=None
        self.disagreements=0
        self._counted=False
        self.position_error=self.heading_error=None

    def update(self, dt, *, quality_ok, position_error, heading_error):
        dt=max(0.,dt)
        finite=all(math.isfinite(v) for v in (position_error,heading_error))
        self.position_error=position_error if finite else None
        self.heading_error=heading_error if finite else None
        mismatch=finite and (position_error>self.enter_m or abs(heading_error)>self.enter_heading)
        if not quality_ok or not finite or mismatch:
            if not self.holding:self.age_s=0.;self._counted=False
            self.holding=True
            self.agree_s=0.
            self.weight=0.
            self.reason='position_disagreement' if mismatch else 'position_untrusted'
            if mismatch and not self._counted:
                self.disagreements+=1;self._counted=True
        if self.holding:
            self.age_s+=dt
            if (quality_ok and finite and position_error<=self.recover_m
                    and abs(heading_error)<=self.recover_heading):
                self.agree_s+=dt
            else:self.agree_s=0.
            if self.agree_s>=self.recover_s-1e-9:
                self.holding=False;self.age_s=0.;self.reason=None;self.weight=0.
            return 0.
        self.weight=min(1.,self.weight+dt/self.blend_s)
        return self.weight
