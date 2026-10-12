"""Conservative, passive steering-response adaptation; no excitation is injected.

Closed-loop commands correlate with disturbances, so a least-squares ratio alone
is not evidence of plant gain. Require varied commands, held-out prediction,
consistent split-window gains and two disjoint windows. Delay/tau candidates are
nuisance models for validation, not online PID/delay tuning or a slip measurement.
"""
import math
from bisect import bisect_right
from collections import deque


class TurnResponse:
    SCALE_MIN = .85
    SCALE_MAX = 1.15
    SCALE_RATE_S = .01
    ROLLBACK_RATE_S = .03
    SAMPLE_S = .05
    WINDOW_S = 12.
    CONFIDENCE_S = 36.
    DELAYS = (.10,.20,.30,.40,.50,.60,.70)
    TAUS = (.06,.12,.24)

    def __init__(self):
        self.sent=deque(maxlen=512)
        self.scale=self.target_scale=1.
        self.last_observation=None
        self.last_accepted=None
        self.last_fit=None
        self.rate_lp=None
        self.rows=[]
        self.models=[(delay,tau) for delay in self.DELAYS for tau in self.TAUS]
        self.predictions=[0.]*len(self.models)
        self.filtered=[0.]*len(self.models)
        self.confirmation=None
        self.accepted_windows=self.rejected_windows=0
        self.reason='waiting_for_motion'
        self.fit_gain=self.fit_delay=self.fit_tau=self.fit_score=None
        self.verified_model=None
        self.mismatch_energy=self.prediction_energy=self.mismatch_s=0.
        self.fast_rollback=False
        self.response_changes=0

    def record_sent(self,t,command):
        if not math.isfinite(t) or not math.isfinite(command):
            raise ValueError('转向响应历史必须是有限值')
        if self.sent and t<self.sent[-1][0]:
            raise ValueError('转向响应历史时间不能倒退')
        if self.sent and t==self.sent[-1][0]:self.sent[-1]=(t,command)
        else:self.sent.append((t,command))
        while len(self.sent)>1 and self.sent[1][0]<t-2.:self.sent.popleft()

    def _clear_window(self,reason):
        self.rows.clear()
        self.confirmation=None
        self.last_observation=None
        self.rate_lp=None
        self.predictions=[0.]*len(self.models)
        self.filtered=[0.]*len(self.models)
        self.reason=reason

    def observe(self,now,sample_t,rate,*,eligible):
        if not all(math.isfinite(v) for v in (now,sample_t,rate)):
            raise ValueError('转向响应样本必须是有限值')
        if sample_t>now:raise ValueError('转向响应样本不能来自未来')
        if not eligible or now-sample_t>.10 or abs(rate)>.25:
            self._clear_window('quality_or_motion_gate')
            return
        previous=self.last_observation
        if previous is not None:
            if sample_t<=previous:
                # Repeated IMU frames must not accumulate evidence.
                self.reason='waiting_for_fresh_imu'
                return
            if sample_t-previous<self.SAMPLE_S-1e-8:return
            if sample_t-previous>.15:
                self._clear_window('imu_gap')
                previous=None
        if previous is None:
            self.last_observation=sample_t
            self.rate_lp=rate
            self.reason='collecting'
            return
        dt=sample_t-previous
        self.last_observation=sample_t
        if not self.sent or self.sent[0][0]>previous-max(self.DELAYS):
            self.reason='command_history_warmup'
            return
        history=list(self.sent)
        if any(abs(u)>.30 for t,u in history if t>=previous-max(self.DELAYS)):
            self._clear_window('large_or_saturated_command')
            return
        # Integrate each delayed command's actual zero-order-held intervals.
        times=[p[0] for p in history]
        for k,(delay,tau) in enumerate(self.models):
            start,end=previous-delay,sample_t-delay
            value=self.predictions[k]
            first=max(0,bisect_right(times,start)-1)
            for j in range(first,len(history)):
                t,u=history[j]
                a=max(start,t)
                b=min(end,history[j+1][0] if j+1<len(history) else end)
                if b>a:value+=(1.-math.exp(-(b-a)/tau))*(u-value)
                if t>end:break
            self.predictions[k]=value
        alpha=1.-math.exp(-dt/.10)
        self.rate_lp+=alpha*(rate-self.rate_lp)
        for k,value in enumerate(self.predictions):self.filtered[k]+=alpha*(value-self.filtered[k])
        if self.verified_model is not None:
            k,gain,offset=self.verified_model
            predicted=gain*self.filtered[k]+offset
            alpha=1.-math.exp(-dt/.25)
            self.mismatch_energy+=alpha*((self.rate_lp-predicted)**2-self.mismatch_energy)
            self.prediction_energy+=alpha*(predicted**2-self.prediction_energy)
            bad=math.sqrt(self.mismatch_energy)>max(.003,.35*math.sqrt(self.prediction_energy))
            self.mismatch_s=max(0.,self.mismatch_s+(dt if bad else -dt))
            if self.mismatch_s>=.4:
                # Withdraw a stale correction before waiting for two new fit
                # windows. Never jump directly to a different learned gain.
                self.target_scale=1.;self.last_accepted=None
                self.fast_rollback=True;self.verified_model=None;self.confirmation=None
                self.rows.clear();self.mismatch_s=0.;self.response_changes+=1
                self.reason='response_changed'
        self.rows.append((sample_t,self.rate_lp,tuple(self.filtered)))
        if len(self.rows)<2 or sample_t-self.rows[0][0]<self.WINDOW_S:return
        rows=self.rows
        self.rows=[]
        self.last_fit=sample_t
        self._fit(rows,sample_t)

    @staticmethod
    def _regress(rows,k):
        n=len(rows)
        x=[r[2][k] for r in rows];y=[r[1] for r in rows]
        mx=sum(x)/n;my=sum(y)/n
        xx=sum((v-mx)**2 for v in x);yy=sum((v-my)**2 for v in y)
        if xx/n<.0015**2 or yy/n<.001**2:return None
        if max(x)-min(x)<.006:return None
        # Both directions relative to the mean must carry evidence.
        if min(sum(v>mx+.001 for v in x),sum(v<mx-.001 for v in x))<n*.15:return None
        gain=sum((a-mx)*(b-my) for a,b in zip(x,y))/xx
        if not .55<=gain<=1.65:return None
        offset=my-gain*mx
        if abs(offset)>.008:return None
        return gain,offset,yy/n

    def _fit(self,rows,now):
        if len(rows)<70:
            self.reason='insufficient_samples';self.confirmation=None;return
        middle=len(rows)//2
        train,check=rows[:middle],rows[middle:]
        candidates=[]
        for k,model in enumerate(self.models):
            a=self._regress(train,k);b=self._regress(check,k)
            if a is None or b is None:continue
            if abs(a[0]-b[0])>.15*min(a[0],b[0]):continue
            mse=sum((r[1]-(a[0]*r[2][k]+a[1]))**2 for r in check)/len(check)
            reverse=sum((r[1]-(b[0]*r[2][k]+b[1]))**2 for r in train)/len(train)
            score=max(mse/b[2],reverse/a[2])
            if score>.20 or math.sqrt(max(mse,reverse))>.004:continue
            fit=self._regress(rows,k)
            if fit is not None:candidates.append((score,fit[0],model))
        if not candidates:
            self.rejected_windows+=1;self.confirmation=None
            varied=any(max(r[2][k] for r in rows)-min(r[2][k] for r in rows)>=.006
                       for k in range(len(self.models)))
            self.reason='unexplained_response' if varied else 'insufficient_excitation'
            if varied:self.target_scale=1.
            return
        candidates.sort()
        best=candidates[0]
        plausible=[c for c in candidates if c[0]<=best[0]+.02]
        if max(c[1] for c in plausible)-min(c[1] for c in plausible)>.15:
            self.rejected_windows+=1;self.confirmation=None
            self.target_scale=1.
            self.reason='ambiguous_response';return
        score,gain,(delay,tau)=best
        self.fit_gain,self.fit_delay,self.fit_tau,self.fit_score=gain,delay,tau,1.-score
        previous=self.confirmation
        self.confirmation=(now,gain)
        if previous is None or now-previous[0]>self.WINDOW_S+1. or abs(gain-previous[1])>.12*min(gain,previous[1]):
            self.reason='confirming_response';return
        self.target_scale=max(self.SCALE_MIN,min(self.SCALE_MAX,1./((gain+previous[1])/2.)))
        k=self.models.index((delay,tau))
        fitted=self._regress(rows,k)
        self.verified_model=(k,gain,fitted[1])
        self.mismatch_energy=self.prediction_energy=self.mismatch_s=0.
        self.fast_rollback=False
        self.accepted_windows+=1
        self.last_accepted=now
        self.reason='tracking_response'

    def advance(self,now,dt):
        if not math.isfinite(now) or not math.isfinite(dt) or dt<0:
            raise ValueError('转向响应更新时间无效')
        if self.last_accepted is None or now-self.last_accepted>self.CONFIDENCE_S:
            self.target_scale=1.
            target=1.
        else:target=self.target_scale
        rate=self.ROLLBACK_RATE_S if self.fast_rollback and target==1. else self.SCALE_RATE_S
        step=rate*min(dt,.1)
        self.scale+=max(-step,min(step,target-self.scale))
        self.scale=max(self.SCALE_MIN,min(self.SCALE_MAX,self.scale))
        if self.scale==1.:self.fast_rollback=False
        return self.scale

    def status(self):
        return dict(response_scale=self.scale,response_target_scale=self.target_scale,
                    response_reason=self.reason,response_samples=len(self.rows),
                    response_accepted_windows=self.accepted_windows,response_rejected_windows=self.rejected_windows,
                    response_fit_gain=self.fit_gain,response_fit_delay_s=self.fit_delay,
                    response_fit_tau_s=self.fit_tau,response_fit_score=self.fit_score,
                    response_changes=self.response_changes,response_fast_rollback=self.fast_rollback)
