"""停稳设置、持久化开关、四向停稳原地旋转标定（含坡度抵消、坑包识别、中途停止）和真实节点协议链路；不连任何硬件。"""
import dataclasses
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'radar_system'), str(ROOT), str(ROOT/'radar_system/tools')]
import ros_stubs
ros_stubs.install()
sys.modules['sensor_msgs.msg'].Imu = ros_stubs._simple('Imu')
import gnss_fusion as gf
import app_link
import line_node
from attitude_controls import AttitudeControls, solve_four_way_zero


class Controls(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'fusion.json'
        self.original = dict(antenna_height_m=.53, antenna_xy=[.24,.2075], mount_offset_deg=1.2, unknown={'keep':True})
        self.path.write_text(json.dumps(self.original))
        self.f = gf.GnssFusion(gf.FusionConfig(antenna_height_m=.53, yaw_input='angle'))
        self.f.set_origin(30.,120.)
        self.m = SimpleNamespace(fusion=self.f, phase='IDLE', operations=SimpleNamespace(held=False), braking=False,
                                 differential=True)
        self.c = AttitudeControls(self.m, str(self.path))
        self.motion = dict(mode='IDLE', healthy=True, waiting_stationary=False, command=dict(vx=0.,wz=0.))
        self.observe(1.)
        self.c.status(1., self.motion, self.att)
        self.observe(1.4)

    def observe(self,t,roll=2.,pitch=-1.,speed=0.,turn=0.,yaw=40.):
        self.f.add_wheel(gf.WheelSample(t,speed,turn))
        self.f.add_imu(gf.ImuSample(t,turn,math.radians(yaw),math.radians(roll),math.radians(pitch)))
        ax,ay=self.f.antenna_lever()
        try:
            lat,lon=self.f.to_geodetic(ax,ay)
            self.f.add_fix(gf.GnssFix(t,lat,lon,0.,'NARROW_INT',.01,.01,.02,1.,30))
            self.f.add_heading(gf.DualHeading(t,180.,0.,.415,'NARROW_INT',.1,30))
        except ValueError:
            pass
        self.c.observe_wheel(t,speed,turn,abs(speed)<=.02 and abs(turn)<=.05)
        self.att=(t,roll,pitch,yaw)

    def drive_rotation(self, ground_roll=0.0, ground_pitch=0.0, bump_at_90=0.0):
        """驱动四向原地旋转闭环，模拟真车在坡地（含可选 90° 处坑包）的 IMU 与轮速反馈。"""
        t = self.att[0]
        r0, p0, yaw0 = self.att[1], self.att[2], self.att[3]
        yaw = yaw0
        dt = 0.05
        for _ in range(1000):
            if not self.c.active:
                break
            t = round(t + dt, 4)
            # 先读上一拍的指令角速度推进车身朝向
            cal = self.c._cal
            wz = cal['wz'] if cal is not None else 0.0
            yaw = ((yaw + math.degrees(wz * dt) + 180.0) % 360.0) - 180.0
            turned = cal['turned_deg'] + math.degrees(wz * dt) if cal is not None else 0.0
            rad = math.radians(turned)
            bump = bump_at_90 if abs(turned - 90.0) < 25.0 else 0.0
            raw_roll = r0 + ground_roll * math.cos(rad) + ground_pitch * math.sin(rad) + bump
            raw_pitch = p0 - ground_roll * math.sin(rad) + ground_pitch * math.cos(rad)
            self.c.observe_wheel(t, 0.0, wz, abs(wz) <= 0.05)
            self.att = (t, raw_roll, raw_pitch, yaw)
            vx, cmd_wz, select = self.c.tick(t, 'NAVIGATION', self.att)
        # 标定完停在 360°（同起始朝向），恢复静止观测供后续断言
        self.observe(round(t + 0.4, 4), roll=r0 + ground_roll, pitch=p0 + ground_pitch, yaw=yaw0)
        return self.c.pop_result()

    def command(self,kind,id='x', complete_rotation=True, ground_roll=0.0, ground_pitch=0.0, **args):
        ok, message = self.c.command(dict(cmd=kind,id=id,**args),self.att[0],self.motion,self.att)
        if kind == 'imu_zero' and ok and self.c.calibrating and complete_rotation:
            done = self.drive_rotation(ground_roll=ground_roll, ground_pitch=ground_pitch)
            if done is not None:
                return done['ok'], done['message']
        return ok, message

    def test_off_keeps_height_and_unknown_config_and_restarts_off(self):
        self.assertTrue(self.command('tilt',enabled=False)[0])
        saved=json.loads(self.path.read_text());self.assertEqual(saved,{**self.original,'tilt_compensation_enabled':False})
        self.assertEqual(self.f.tilt_status()['reason'],'disabled')
        self.assertEqual(self.f.antenna_lever(), self.f.config.antenna_xy)
        self.assertNotIn('tilt_uncompensated',self.f.pose(1.4).reasons)
        restarted=gf.GnssFusion(dataclasses.replace(self.f.config))
        self.assertFalse(restarted.tilt_status()['enabled'])
        self.assertTrue(self.command('tilt','on',enabled=True)[0])
        self.assertTrue(self.f.tilt_status()['active'])

    def test_zero_is_saved_restored_used_by_lever_and_does_not_change_yaw(self):
        before=self.path.read_bytes()
        self.assertTrue(self.command('imu_zero')[0])
        self.assertEqual(self.path.read_bytes(),before)
        saved = json.loads(Path(self.c.zero_path).read_text())
        self.assertAlmostEqual(saved['roll_deg'], 2., places=5)
        self.assertAlmostEqual(saved['pitch_deg'], -1., places=5)
        self.assertEqual(self.f.antenna_lever(),self.f.config.antenna_xy)
        st=self.f.tilt_status();self.assertTrue(st['zeroed']);self.assertAlmostEqual(st['roll_deg'],0.,places=5)
        self.observe(self.att[0] + 0.5,roll=5.,pitch=1.)
        st=self.f.tilt_status();self.assertAlmostEqual(st['roll_deg'],3.,places=5);self.assertAlmostEqual(st['pitch_deg'],2.,places=5)
        # 重复网络帧不能重新选取零点；再次主动标定使用新的 id。
        self.assertTrue(self.command('imu_zero')[0]);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],3.,places=5)
        self.observe(self.att[0] + 0.4, roll=5., pitch=1.)
        self.assertTrue(self.command('imu_zero','new')[0]);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],0.,places=5)
        restarted=gf.GnssFusion(self.f.config)
        restored=AttitudeControls(SimpleNamespace(fusion=restarted), str(self.path))
        self.assertTrue(restarted.tilt_status()['zeroed'])
        self.assertIsNone(restored.zero_save_error)
        self.assertAlmostEqual(restarted.tilt_angles(math.radians(5.), math.radians(1.))[0], 0., places=5)
        self.assertAlmostEqual(restarted.tilt_angles(math.radians(5.), math.radians(1.))[1], 0., places=5)

    def test_four_way_rotation_cancels_sloped_ground_and_handles_imperfect_stop_angles(self):
        # 车停在有坡度的草地上（地面横坡 +1.2°、纵坡 -0.8°），真实 IMU 安装偏差为 (-0.57°, -0.55°)。
        # 单点清零会把地面坡度当成零点；四向旋转标定应完全抵消地面坡度，并在标定后如实显示地面坡度。
        t0 = self.att[0]
        self.observe(t0 + 0.1, roll=-0.57, pitch=-0.55)
        self.c.status(t0 + 0.1, self.motion, self.att)
        self.observe(t0 + 0.5, roll=-0.57, pitch=-0.55)
        ok, msg = self.command('imu_zero', 'slope1', ground_roll=1.2, ground_pitch=-0.8)
        self.assertTrue(ok, msg)
        self.assertIn('地面坡度', msg)
        saved = json.loads(Path(self.c.zero_path).read_text())
        self.assertAlmostEqual(saved['roll_deg'], -0.57, places=4)
        self.assertAlmostEqual(saved['pitch_deg'], -0.55, places=4)
        # 标定完转回原朝向，融合姿态应准确读出地面本身的 (+1.2°, -0.8°) 坡度，而非 0°
        st = self.f.tilt_status()
        self.assertAlmostEqual(st['roll_deg'], 1.2, places=3)
        self.assertAlmostEqual(st['pitch_deg'], -0.8, places=3)

        # 即便四向停稳角度偏离 90° 整数倍（如 0.5°、93.2°、177.8°、272.4°），最小二乘谐波解依然精确消除坡度
        stops = []
        for deg in (0.5, 93.2, 177.8, 272.4):
            rad = math.radians(deg)
            stops.append(dict(
                turned_deg=deg,
                roll_deg=-0.57 + 1.2 * math.cos(rad) - 0.8 * math.sin(rad),
                pitch_deg=-0.55 - 1.2 * math.sin(rad) - 0.8 * math.cos(rad),
            ))
        r0, p0, slope, diff = solve_four_way_zero(stops)
        self.assertAlmostEqual(r0, -0.57, places=6)
        self.assertAlmostEqual(p0, -0.55, places=6)
        self.assertAlmostEqual(slope, math.hypot(1.2, -0.8), places=6)
        self.assertAlmostEqual(diff, 0.0, places=6)

    def test_uneven_pit_at_one_direction_is_rejected_without_overwriting_zero(self):
        self.assertTrue(self.command('imu_zero', 'good')[0])
        before = Path(self.c.zero_path).read_bytes()
        t0 = self.att[0]
        self.observe(t0 + 0.1, roll=2., pitch=-1.)
        self.c.status(t0 + 0.1, self.motion, self.att)
        self.observe(t0 + 0.5, roll=2., pitch=-1.)
        self.assertTrue(self.command('imu_zero', 'pit', complete_rotation=False)[0])
        done = self.drive_rotation(ground_roll=0.4, ground_pitch=-0.3, bump_at_90=2.2)
        self.assertFalse(done['ok'])
        self.assertIn('四向倾角不一致', done['message'])
        self.assertEqual(Path(self.c.zero_path).read_bytes(), before)

    def test_operator_stop_and_control_loss_abort_rotation_and_release_navigation(self):
        self.assertTrue(self.command('imu_zero', 'run1', complete_rotation=False)[0])
        self.assertTrue(self.c.calibrating)
        self.assertEqual(self.c.tick(self.att[0] + 0.02, 'IDLE', self.att), (0.0, 0.0, True))
        self.c.tick(self.att[0] + 0.04, 'NAVIGATION', self.att)
        # 操作员中途点「停止标定」
        ok, msg = self.c.command(dict(cmd='imu_zero', action='stop', id='stop1'), self.att[0] + 0.1, self.motion, self.att)
        self.assertTrue(ok)
        self.assertFalse(self.c.calibrating)
        self.assertEqual(self.c.tick(self.att[0] + 0.12, 'NAVIGATION', self.att), (0.0, 0.0, False))
        self.assertFalse(Path(self.c.zero_path).exists())

        # 控制权被急停或手动切走时立即中止并交还控制权
        t0 = self.att[0] + 1.0
        self.observe(t0, roll=2., pitch=-1.)
        self.c.status(t0, self.motion, self.att)
        self.observe(t0 + 0.4, roll=2., pitch=-1.)
        self.assertTrue(self.command('imu_zero', 'run2', complete_rotation=False)[0])
        self.c.tick(t0 + 0.42, 'NAVIGATION', self.att)
        self.assertEqual(self.c.tick(t0 + 0.44, 'ESTOP', self.att), (0.0, 0.0, False))
        done = self.c.pop_result()
        self.assertFalse(done['ok'])
        self.assertIn('控制权被切走', done['message'])

    def test_save_failure_keeps_previous_disk_zero_and_current_angles(self):
        from unittest.mock import patch
        self.assertTrue(self.command('imu_zero')[0])
        before=Path(self.c.zero_path).read_bytes();offset=self.f._tilt_zero
        t0 = self.att[0]
        self.observe(t0 + 0.1,roll=6.,pitch=3.)
        self.c.status(t0 + 0.1, self.motion, self.att)
        self.observe(t0 + 0.5,roll=6.,pitch=3.)
        with patch('attitude_controls.os.replace', side_effect=OSError('disk full')):
            self.assertFalse(self.command('imu_zero','failed-save')[0])
        self.assertEqual(Path(self.c.zero_path).read_bytes(),before)
        self.assertEqual(self.f._tilt_zero,offset)
        self.assertEqual(list(Path(self.tmp.name).glob('*.tmp')),[])

    def test_corrupt_zero_is_reported_and_explicit_clear_can_repair_it(self):
        for data in ('{bad', '{"version":1,"roll_deg":true,"pitch_deg":0}',
                     '{"version":1,"roll_deg":181,"pitch_deg":0}'):
            Path(self.c.zero_path).write_text(data)
            f=gf.GnssFusion(self.f.config)
            c=AttitudeControls(SimpleNamespace(fusion=f),str(self.path))
            self.assertFalse(f.tilt_status()['zeroed']);self.assertIsNotNone(c.zero_save_error)
        self.c=AttitudeControls(self.m,str(self.path))
        self.c.observe_wheel(1.,0.,0.,True);self.c.status(1.,self.motion,(1.,2.,-1.,40.))
        self.c.observe_wheel(1.4,0.,0.,True);self.att=(1.4,2.,-1.,40.)
        self.assertTrue(self.command('imu_zero','repair')[0]);self.assertIsNone(self.c.zero_save_error)
        self.assertAlmostEqual(json.loads(Path(self.c.zero_path).read_text())['roll_deg'],2.,places=5)

    def test_old_uneven_zero_uses_measured_level_until_the_user_zeroes_again(self):
        def restart():
            f=gf.GnssFusion(self.f.config);c=AttitudeControls(SimpleNamespace(fusion=f),str(self.path))
            return f,c
        zero=Path(self.c.zero_path)
        zero.write_text(json.dumps(dict(version=1,roll_deg=-0.2500064008288567,pitch_deg=0.40999609702888323)))
        f,c=restart()
        st=f.tilt_status();self.assertTrue(st['zeroed'])
        self.assertAlmostEqual(st['zero_roll_deg'],-.57);self.assertAlmostEqual(st['zero_pitch_deg'],-.55)
        self.assertIsNone(c.zero_save_error)
        self.assertEqual(json.loads(zero.read_text())['roll_deg'],-0.2500064008288567)
        self.c=AttitudeControls(self.m,str(self.path))
        self.c.observe_wheel(1.,0.,0.,True);self.c.status(1.,self.motion,(1.,2.,-1.,40.))
        self.c.observe_wheel(1.4,0.,0.,True);self.att=(1.4,2.,-1.,40.)
        self.assertTrue(self.command('imu_zero','user')[0])
        self.assertAlmostEqual(self.f.tilt_status()['zero_roll_deg'],2.,places=5)
        f,c=restart();self.assertAlmostEqual(f.tilt_status()['zero_roll_deg'],2.,places=5);self.assertAlmostEqual(f.tilt_status()['zero_pitch_deg'],-1.,places=5)
        zero.write_text(json.dumps(dict(version=1,roll_deg=-.30,pitch_deg=.41)))
        f,c=restart();self.assertAlmostEqual(f.tilt_status()['zero_roll_deg'],-.30);self.assertAlmostEqual(f.tilt_status()['zero_pitch_deg'],.41)
        other=Path(self.tmp.name)/'imu_zero-imu762.json'
        other.write_text(json.dumps(dict(version=1,roll_deg=-.25,pitch_deg=.41)))
        f,c=restart();c.select_imu_source('imu762')
        self.assertAlmostEqual(f.tilt_status()['zero_roll_deg'],-.25);self.assertAlmostEqual(f.tilt_status()['zero_pitch_deg'],.41)

    def test_reference_barrier_discards_old_observations(self):
        self.assertTrue(self.command('imu_zero')[0]);before=self.f._s.t
        self.f.add_imu(gf.ImuSample(1.2,0.,.7,.4,.3))
        self.assertEqual(self.f._s.t,before);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],0.,places=5)

    def test_running_paused_braking_and_missing_motion_rejected(self):
        for field,value in [('phase','RUNNING'),('phase','PAUSED'),('braking',True)]:
            previous=getattr(self.m,field);setattr(self.m,field,value)
            self.assertFalse(self.command('imu_zero',field+str(value))[0]);setattr(self.m,field,previous)
        self.m.operations.held=True
        self.assertFalse(self.command('tilt',enabled=False)[0])
        self.m.operations.held=False
        self.assertIsNotNone(self.c.status(1.4,None,self.att)['blocked'])

    def test_moving_and_stale_wheel_rejected(self):
        self.observe(1.5,speed=.2)
        self.assertFalse(self.command('imu_zero')[0])
        self.motion['command']['wz']=.1
        self.assertIsNotNone(self.c.status(1.5,self.motion,self.att)['blocked'])
        self.motion['command']['wz']=0.
        self.assertIsNotNone(self.c.status(4.,self.motion,self.att)['blocked'])

    def test_stale_imu_missing_height_invalid_boolean_and_bad_file_rejected(self):
        self.att=(.1,2.,1.,40.)
        self.assertFalse(self.c.command(dict(cmd='imu_zero',id='stale'),1.4,self.motion,self.att)[0])
        self.assertFalse(self.command('tilt','bad',enabled='false')[0])
        self.f.config=dataclasses.replace(self.f.config,antenna_height_m=0.)
        self.assertFalse(self.command('tilt','height',enabled=True)[0])
        self.path.write_text('{bad');before=self.f.config
        self.assertFalse(self.command('tilt','broken',enabled=False)[0])
        self.assertEqual(self.path.read_text(),'{bad');self.assertIs(self.f.config,before)
        for v in ['true',1,None]:
            self.assertRaises(ValueError,gf.FusionConfig,tilt_compensation_enabled=v)

    def test_node_command_to_encoded_telemetry_with_same_id(self):
        now=[1.4]
        self.m.tick=lambda t,mode:(0.0,0.0,None)
        self.m.on_wheel_motion=lambda sample:None
        node=line_node.LineNode(self.m,clock=lambda:now[0],fusion_path=str(self.path))
        node.motion.fresh=lambda:True;node.motion.state=self.motion
        node.motion.select=lambda active:(self.motion.__setitem__('mode','NAVIGATION' if active else 'IDLE') or True)
        node.motion.active=lambda:self.motion['mode']=='NAVIGATION'
        node.motion.publish=lambda vx,wz:True
        node.attitude_controls=self.c;node._attitude=self.att
        self.m.status=lambda:dict(phase='IDLE',tilt=self.f.tilt_status())
        cmd=app_link.parse_command(json.dumps(dict(type='imu_zero',id='zero1')))
        node.on_command(SimpleNamespace(data=json.dumps(dict(cmd=cmd['type'],id=cmd['id']))))
        self.assertTrue(node.attitude_controls.calibrating)
        # 推进节点 50 Hz 控制循环直至四向旋转标定完成
        yaw = 40.0
        for _ in range(1000):
            if not node.attitude_controls.active:
                break
            now[0] = round(now[0] + 0.05, 4)
            cal = node.attitude_controls._cal
            wz = cal['wz'] if cal is not None else 0.0
            yaw = ((yaw + math.degrees(wz * 0.05) + 180.0) % 360.0) - 180.0
            self.c.observe_wheel(now[0], 0.0, wz, abs(wz) <= 0.05)
            node._attitude = (now[0], 2.0, -1.0, yaw)
            node.control()
        self.assertTrue(node.last_result['ok']);self.assertEqual(node.last_result['id'],'zero1')
        a=node.attitude_status();self.assertAlmostEqual(a['roll_deg'],0.,places=2);self.assertAlmostEqual(a['yaw_deg'],40.,delta=1.5)
        payload=app_link._line_tilt(self.f.tilt_status())
        self.assertTrue(payload['zeroed']);self.assertTrue(payload['enabled'])
        now[0] = round(now[0] + 0.1, 4)
        self.observe(now[0],roll=3.);node._attitude=self.att
        self.assertAlmostEqual(node.attitude_status()['roll_deg'],1.,places=2)

    def test_zero_without_rtk_or_initialized_position_uses_chassis_feedback(self):
        self.f = self.m.fusion = gf.GnssFusion(gf.FusionConfig(antenna_height_m=.53))
        self.f.add_imu(gf.ImuSample(1.4,0.,None,math.radians(2.),math.radians(-1.)))
        self.assertIsNone(self.f.pose(1.4))
        self.c.observe_wheel(1.4,0.,0.,True)
        self.assertTrue(self.command('imu_zero')[0])
        self.assertEqual(self.f.tilt_status()['zeroed'],True)
        self.assertTrue(self.command('tilt','off',enabled=False)[0])

    def test_missing_stale_future_or_moving_wheel_feedback_is_rejected(self):
        self.c.wheel=None
        self.assertFalse(self.command('imu_zero','missing')[0])
        self.c.observe_wheel(.1,0.,0.,True)
        self.assertFalse(self.command('imu_zero','stale')[0])
        self.c.observe_wheel(1.5,0.,0.,True)
        self.assertFalse(self.command('imu_zero','future')[0])
        self.c.wheel=None;self.c.observe_wheel(1.4,0.,0.,False)
        self.assertFalse(self.command('imu_zero','hall-moving')[0])
        self.c.wheel=None;self.c.observe_wheel(1.4,0.,.2,True)
        self.assertFalse(self.command('imu_zero','turning')[0])

    def test_encoder_interval_uses_current_speed_and_bad_feedback_does_not_refresh(self):
        self.c.wheel=None
        self.c.observe_sample(gf.WheelSample(1.2,.1,.2,end_t=1.4,end_speed=0.,end_yaw_rate=0.,hall_still=True))
        self.assertEqual(self.c.wheel,(1.4,0.,0.,True))
        self.c.observe_wheel(1.5,float('nan'),0.)
        self.c.observe_wheel(1.3,.1,.1)
        self.assertEqual(self.c.wheel,(1.4,0.,0.,True))

    def test_parser_strict_and_legacy_telemetry(self):
        self.assertEqual(app_link.parse_command('{"type":"tilt","enabled":false,"id":"t1"}')['enabled'],False)
        self.assertEqual(app_link.parse_command('{"type":"imu_zero","id":"z1","action":"stop"}')['action'],'stop')
        for msg in ['{"type":"tilt","enabled":1}','{"type":"imu_zero"}','{"type":"imu_zero","id":"z1","action":"bad"}']:
            self.assertRaises(app_link.CommandError,app_link.parse_command,msg)
        old=app_link._line_tilt(dict(active=False,reason='no_height',height_m=0.))
        self.assertNotIn('enabled',old);self.assertNotIn('imu_zero_supported',old)

if __name__=='__main__':unittest.main()
