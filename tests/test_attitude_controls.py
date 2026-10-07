"""停稳设置、持久化开关、永久清零和真实节点协议链路；不连任何硬件。"""
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
from attitude_controls import AttitudeControls


class Controls(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'fusion.json'
        self.original = dict(antenna_height_m=.53, antenna_xy=[.24,.2075], mount_offset_deg=1.2, unknown={'keep':True})
        self.path.write_text(json.dumps(self.original))
        self.f = gf.GnssFusion(gf.FusionConfig(antenna_height_m=.53, yaw_input='angle'))
        self.f.set_origin(30.,120.)
        self.m = SimpleNamespace(fusion=self.f, phase='IDLE', operations=SimpleNamespace(held=False), braking=False)
        self.c = AttitudeControls(self.m, str(self.path))
        self.motion = dict(mode='IDLE', healthy=True, waiting_stationary=False, command=dict(vx=0.,wz=0.))
        self.observe(1.)
        self.c.status(1., self.motion, self.att)
        self.observe(1.4)

    def observe(self,t,roll=2.,pitch=-1.,speed=0.,turn=0.):
        self.f.add_wheel(gf.WheelSample(t,speed,turn))
        self.f.add_imu(gf.ImuSample(t,0.,.7,math.radians(roll),math.radians(pitch)))
        ax,ay=self.f.antenna_lever()
        lat,lon=self.f.to_geodetic(ax,ay)
        self.f.add_fix(gf.GnssFix(t,lat,lon,0.,'NARROW_INT',.01,.01,.02,1.,30))
        self.f.add_heading(gf.DualHeading(t,180.,0.,.415,'NARROW_INT',.1,30))
        self.c.observe_wheel(t,speed,turn)
        self.att=(t,roll,pitch,40.)

    def command(self,kind,id='x',**args):
        return self.c.command(dict(cmd=kind,id=id,**args),self.att[0],self.motion,self.att)

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
        before=self.path.read_bytes();yaw=self.f._s.imu_yaw
        self.assertTrue(self.command('imu_zero')[0])
        self.assertEqual(self.path.read_bytes(),before)
        saved = json.loads(Path(self.c.zero_path).read_text())
        self.assertEqual(saved, dict(version=1, roll_deg=2., pitch_deg=-1.))
        self.assertEqual(self.f._s.imu_yaw,yaw)
        self.assertEqual(self.f.antenna_lever(),self.f.config.antenna_xy)
        st=self.f.tilt_status();self.assertTrue(st['zeroed']);self.assertAlmostEqual(st['roll_deg'],0.)
        self.observe(1.5,roll=5.,pitch=1.)
        st=self.f.tilt_status();self.assertAlmostEqual(st['roll_deg'],3.);self.assertAlmostEqual(st['pitch_deg'],2.)
        # 重复网络帧不能重新选取零点；再次主动清零使用新的 id。
        self.assertTrue(self.command('imu_zero')[0]);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],3.)
        self.assertTrue(self.command('imu_zero','new')[0]);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],0.)
        restarted=gf.GnssFusion(self.f.config)
        restored=AttitudeControls(SimpleNamespace(fusion=restarted), str(self.path))
        self.assertTrue(restarted.tilt_status()['zeroed'])
        self.assertIsNone(restored.zero_save_error)
        self.assertAlmostEqual(restarted.tilt_angles(math.radians(5.), math.radians(1.))[0], 0.)
        self.assertAlmostEqual(restarted.tilt_angles(math.radians(5.), math.radians(1.))[1], 0.)
        self.assertEqual(json.loads(Path(self.c.zero_path).read_text()), dict(version=1, roll_deg=5., pitch_deg=1.))

    def test_save_failure_keeps_previous_disk_zero_and_current_angles(self):
        from unittest.mock import patch
        self.assertTrue(self.command('imu_zero')[0])
        before=Path(self.c.zero_path).read_bytes();offset=self.f._tilt_zero
        self.observe(1.5,roll=6.,pitch=3.)
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
        self.c.observe_wheel(1.4,0.,0.,True)
        self.assertTrue(self.command('imu_zero','repair')[0]);self.assertIsNone(self.c.zero_save_error)
        self.assertEqual(json.loads(Path(self.c.zero_path).read_text())['roll_deg'],2.)

    def test_reference_barrier_discards_old_observations(self):
        self.assertTrue(self.command('imu_zero')[0]);before=self.f._s.t
        self.f.add_imu(gf.ImuSample(1.2,0.,.7,.4,.3))
        self.assertEqual(self.f._s.t,before);self.assertAlmostEqual(self.f.tilt_status()['roll_deg'],0.)

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
        node=line_node.LineNode(self.m,clock=lambda:now[0],fusion_path=str(self.path))
        node.motion.fresh=lambda:True;node.motion.state=self.motion
        node.attitude_controls=self.c;node._attitude=self.att
        self.m.status=lambda:dict(phase='IDLE',tilt=self.f.tilt_status())
        cmd=app_link.parse_command(json.dumps(dict(type='imu_zero',id='zero1')))
        node.on_command(SimpleNamespace(data=json.dumps(dict(cmd=cmd['type'],id=cmd['id']))))
        self.assertTrue(node.last_result['ok']);self.assertEqual(node.last_result['id'],'zero1')
        a=node.attitude_status();self.assertAlmostEqual(a['roll_deg'],0.);self.assertEqual(a['yaw_deg'],40.)
        payload=app_link._line_tilt(self.f.tilt_status())
        self.assertTrue(payload['zeroed']);self.assertTrue(payload['enabled'])
        self.observe(1.5,roll=3.);node._attitude=self.att;now[0]=1.5
        self.assertAlmostEqual(node.attitude_status()['roll_deg'],1.)

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
        for msg in ['{"type":"tilt","enabled":1}','{"type":"imu_zero"}']:
            self.assertRaises(app_link.CommandError,app_link.parse_command,msg)
        old=app_link._line_tilt(dict(active=False,reason='no_height',height_m=0.))
        self.assertNotIn('enabled',old);self.assertNotIn('imu_zero_supported',old)

if __name__=='__main__':unittest.main()
