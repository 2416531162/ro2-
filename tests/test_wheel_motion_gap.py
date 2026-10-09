"""反馈间隔 0.2→1 s 只放宽累计轮程重建，不扩张插值和融合输入时效。"""
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'oid_protocol'),str(ROOT/'radar_system')]
from oid_link import WheelConfig,MotorState
from robot_core.wheel_motion import WheelMotionAligner
from wheel_input import WheelInput


class WheelGap(unittest.TestCase):
    def make(self,**kw):
        wheels=WheelConfig(pole_pairs=20)
        return wheels,WheelMotionAligner(wheels,**kw),{1:MotorState(),2:MotorState()}

    def feed(self,wheels,aligner,states,t,motors=(1,2),speed=.5):
        for motor in motors:
            sign=wheels.left_sign if motor==1 else wheels.right_sign
            states[motor].t_position=states[motor].t_speed=t
            states[motor].position_cdeg=round(sign*speed*t/wheels.centideg_to_m(1))
            states[motor].speed_erpm=sign*wheels.to_erpm(speed)
        return dict(aligner.update(t,states),epoch='wheel-gap-test')

    def ready(self,**kw):
        wheels,a,states=self.make(**kw)
        for t in (0.,.04,.08):self.feed(wheels,a,states,t)
        return wheels,a,states

    def test_delayed_either_wheel_retains_counter_through_one_second(self):
        for motor in (1,2):
            for gap in (.21,.35,.7,1.):
                with self.subTest(motor=motor,gap=gap):
                    wheels,a,states=self.ready()
                    other=3-motor;t=.08+gap
                    for i in range(1,math.ceil(gap/.04)):
                        self.feed(wheels,a,states,min(.08+.04*i,t),motors=(other,))
                    frame=self.feed(wheels,a,states,t)
                    self.assertEqual(a.resets,0)
                    self.assertGreater(abs(a.position[motor][-1][1]),.04)
                    WheelInput(0.).decode(frame,t)  # 恢复包必须通过原有格式及不确定度校验。

    def test_gap_over_one_second_still_reinitializes(self):
        wheels,a,states=self.ready()
        for i in range(1,28):self.feed(wheels,a,states,.08+.04*i,motors=(2,))
        frame=self.feed(wheels,a,states,1.18)
        self.assertEqual(a.resets,1)
        self.assertEqual(a.position[1][-1][1],0.)
        self.assertEqual(frame['reason'],'encoder_reset')

    def test_impossible_angle_jump_still_reinitializes_before_timeout(self):
        wheels,a,states=self.ready()
        states[1].t_position=.1
        states[1].position_cdeg+=round(5./wheels.centideg_to_m(1))
        a.update(.1,states)
        self.assertEqual(a.resets,1)

    def test_long_gap_does_not_create_long_encoder_interval(self):
        wheels,a,states=self.ready();decoder=WheelInput(0.)
        frame=self.feed(wheels,a,states,.68)
        self.assertEqual(a.resets,0)
        self.assertNotEqual(frame['quality'],'encoder')
        self.assertNotIn('start',frame)
        self.assertEqual(frame['reason'],'encoder_gap')
        self.assertEqual(frame['speed_source'],'readback_fallback')
        self.assertIsNotNone(decoder.decode(frame,.68))
        # 新数据重新到齐后接短区间；融合无需放宽 0.2 s 的测量区间校验。
        for t in (.72,.76,.80):
            frame=self.feed(wheels,a,states,t)
            decoded=decoder.decode(frame,t)
            self.assertIsNotNone(decoded)
            self.assertEqual(frame['quality'],'encoder')
            self.assertLessEqual(frame['t']-frame['start'],.2)

    def test_both_wheels_resume_without_invalid_uncertainty(self):
        for gap in (.21,.35,.6,.8,1.):
            with self.subTest(gap=gap):
                wheels,a,states=self.ready();t=.08+gap
                frame=self.feed(wheels,a,states,t)
                self.assertEqual(a.resets,0)
                self.assertEqual(frame['reason'],'encoder_gap')
                self.assertIsNotNone(WheelInput(0.).decode(frame,t))

    def test_current_feedback_and_interpolation_still_expire_at_point_two(self):
        wheels,a,states=self.ready()
        self.assertEqual(a.max_age,.2)
        self.assertEqual(a.max_gap,.2)
        frame=a.update(.31,states)
        self.assertEqual(frame['quality'],'invalid')
        self.assertIsNone(a.stationary(.31,states))
        self.assertIsNone(a._at([(0.,0.),(.5,.25)],.25))

    def test_legacy_override_and_invalid_reset_threshold(self):
        wheels,a,states=self.ready(reset_gap_s=.2)
        self.feed(wheels,a,states,.33)
        self.assertEqual(a.resets,2)
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                WheelMotionAligner(wheels,reset_gap_s=value)


if __name__=='__main__':unittest.main()
