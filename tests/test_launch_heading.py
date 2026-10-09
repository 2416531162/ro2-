"""静止直线起步：两轮响应不同也应及时稳向，交接、失效和停车不能留下旧目标。"""
from dataclasses import replace
import math
from unittest import TestCase, mock

from test_smooth_runup import make, pose, DT


def tick(tr,t,*,yaw=0.,rate=0.,x=0.,v=0.,fused_yaw=None,imu=True):
    p=pose(x,v);p.yaw=yaw if fused_yaw is None else fused_yaw;p.yaw_rate=rate
    tr.sync(t,(t,yaw,rate) if imu else (t-.3,yaw,rate))
    tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
    return tr.update(p,DT)


class LaunchHeading(TestCase):
    def test_first_frame_controls_yaw_before_absolute_alignment_is_ready(self):
        tr=make();tr.start(pose())
        v,w=tick(tr,.02)
        self.assertFalse(tr._straight_guide.locked)
        self.assertEqual(tr._straight_status['control_law'],'imu_launch')
        self.assertGreater(v,0.);self.assertLess(v,.01)
        # 向左开始甩头时，必须在绝对航向学习完成之前给向右的制动纠正。
        for i in range(2,8):v,w=tick(tr,i*DT,yaw=.01,rate=.05)
        self.assertLess(w,0.)
        self.assertFalse(tr._straight_guide.locked)
        self.assertFalse(tr.paint_start_allowed())
        self.assertEqual(tr._straight_core.integral,0.)

    def test_rtk_heading_noise_does_not_move_the_launch_reference(self):
        a,b=make(),make()
        for tr in (a,b):tr.start(pose());tick(tr,.02)
        for i in range(2,30):
            expected=tick(a,i*DT,yaw=.005,rate=.01)
            actual=tick(b,i*DT,yaw=.005,rate=.01,fused_yaw=.06 if i%2 else -.06)
            for x,y in zip(actual,expected):self.assertAlmostEqual(x,y,places=12)
            self.assertEqual(b._straight_target,a._straight_target)

    def test_handoff_keeps_reference_and_sent_history_across_angle_wrap(self):
        tr=make();tr.start(pose())
        off=math.pi-.001
        for i in range(1,70):
            t=i*DT
            yaw=math.atan2(math.sin(off+.002*math.sin(t)),math.cos(off+.002*math.sin(t)))
            p=pose(.1*t,.15);p.yaw=.002*math.sin(t)
            tr.sync(t,(t,yaw,.002*math.cos(t)))
            tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
            was=tr._straight_weight
            old_target=tr._straight_target
            tr.update(p,DT)
            if was==0. and tr._straight_weight>0.:
                self.assertLess(abs(math.remainder(tr._straight_target-old_target,2*math.pi)),math.radians(.1))
                self.assertLess(tr._straight_core.sent[0][0],t-.1)
        self.assertGreater(tr._straight_weight,0.)
        self.assertEqual(tr._straight_status['control_law'],'imu_first')

    def test_stale_imu_drops_launch_and_recovery_cannot_reuse_its_target(self):
        tr=make();tr.start(pose());tick(tr,.02)
        self.assertGreater(tr._launch_weight,0.)
        tick(tr,.04,imu=False)
        self.assertEqual(tr._launch_weight,0.)
        self.assertIsNone(tr._straight_target)
        tick(tr,.06,yaw=.04)
        self.assertEqual(tr._launch_weight,0.)
        self.assertNotEqual(tr._straight_status.get('control_law'),'imu_launch')

    def test_protective_stop_pause_and_stop_clear_launch_output(self):
        for how in ('cap','pause','stop','pose'):
            with self.subTest(how=how):
                tr=make();tr.start(pose());tick(tr,.02)
                if how=='cap':
                    tr.speed_limit_mps=0.;out=tick(tr,.04,yaw=.04,rate=.1)
                elif how=='pose':
                    p=pose();p.usable_for_control=False;out=tr.update(p,DT)
                else:
                    out=getattr(tr,how)()
                self.assertEqual(out,(0.,0.))
                self.assertEqual(tr._launch_weight,0.)

    def test_missing_alignment_has_bounded_hold_and_no_false_paint_permission(self):
        tr=make();tr.start(pose())
        with mock.patch.object(tr._straight_guide,'learn_offset',return_value=False):
            for i in range(1,210):tick(tr,i*DT)
        self.assertEqual(tr._launch_weight,0.)
        self.assertFalse(tr.paint_start_allowed())
        self.assertEqual(tr.st.state,'RUNNING')

    def test_rolling_entries_and_disabled_imu_keep_the_existing_controller(self):
        for moving,enabled in ((.2,True),(0.,False)):
            with self.subTest(moving=moving,enabled=enabled):
                tr=make();tr.cfg=replace(tr.cfg,straight_imu_enabled=enabled)
                tr.start(pose(0.,moving));tick(tr,.02,v=moving)
                self.assertEqual(tr._launch_weight,0.)

    def test_restart_uses_the_new_stationary_heading(self):
        tr=make();tr.start(pose());tick(tr,.02,yaw=.01)
        tr.pause();tr.start(pose())
        tick(tr,1.04,yaw=1.,fused_yaw=0.)
        self.assertAlmostEqual(tr._straight_target,1.)

    def test_launch_ramp_obeys_a_new_lower_speed_cap(self):
        tr=make();tr.start(pose())
        for i in range(1,30):tick(tr,i*DT,v=tr._v_prev)
        self.assertGreater(tr._v_prev,.1)
        tr.speed_limit_mps=.05
        self.assertLessEqual(tick(tr,.6,v=tr._v_prev)[0],.05)
        tr.speed_limit_mps=0.
        self.assertEqual(tick(tr,.62,v=.05),(0.,0.))

    def test_zero_cap_before_motion_preserves_the_fresh_launch(self):
        tr=make();tr.start(pose());tr.speed_limit_mps=0.
        self.assertEqual(tick(tr,.02),(0.,0.))
        tr.speed_limit_mps=None
        v,_=tick(tr,.04)
        self.assertLess(v,.005)
        self.assertEqual(tr._straight_status['control_law'],'imu_launch')

    def test_reference_handoff_has_bounded_target_rate_and_no_output_reset(self):
        tr=make();tr.start(pose())
        previous=None;changes=[]
        for i in range(1,105):
            t=i*DT
            # 车头基本没动、可信定位显示在路线旁 2 cm；换算就绪后渐入横向修正。
            p=pose(.15*t,.2);p.y=.02
            tr.sync(t,(t,.2,0.));tr.set_heading_baseline(t,.412);tr.set_rtk_quality(t,.008,32,.005)
            _,w=tr.update(p,DT)
            target=tr._straight_target
            if previous is not None and target is not None:
                rate=abs(math.remainder(target-previous,2*math.pi))/DT
                self.assertLessEqual(rate,math.radians(3.)+1e-8)
            previous=target
            changes.append(w)
        self.assertEqual(tr._straight_weight,1.)
        self.assertEqual(tr._launch_weight,0.)
        self.assertLess(max(abs(b-a) for a,b in zip(changes,changes[1:])),.025)
