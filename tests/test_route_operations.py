"""Exercise real mission/planner/controller transitions, including persisted recovery."""
import copy
import json
import math
import os
import unittest
from unittest.mock import patch

from test_line_mission import Base, FakeFusion, SAMPLES
from line_mission import LineMission
from route_operations import entry_to, rework_sections, tail
from path_tracker import Path, Section
import route_plan
import app_link

ROUTE = dict(kind='path', points=[[0, 0], [20, 0], [20, 10]], closed=False, cruise=.5)


class TestOperations(Base):
    def setUp(self):
        super().setUp()
        self.route_file = os.path.join(self.tmp.name, 'route.json')
        self.m = LineMission(self.fusion, SAMPLES, route_path=self.route_file)
        self.fusion.current = FakeFusion.make(-1, 0, 0)
        self.m.tick(0, 'IDLE')

    def start(self, route=ROUTE):
        ok, message = self.m.command(dict(cmd='route_start', route=route), 0)
        self.assertTrue(ok, message)
        self.drive(.05, 8)
        self.assertEqual('RUNNING', self.m.phase)

    def command(self, action, now=9, **kw):
        r = self.m.operations.record
        payload = dict(cmd='route_' + action, id=action, route_rev=self.m.route_rev,
                       job_id=r['id'] if r else None, **kw)
        return self.m.command(payload, now)

    def pause(self, now=9):
        self.assertTrue(self.command('pause', now)[0])
        p = self.fusion.current
        self.fusion.current = FakeFusion.make(p.x, p.y, math.degrees(p.yaw), speed=.1)
        self.assertEqual((0, 0, None), self.m.tick(now, 'NAVIGATION'))
        self.assertEqual('PAUSING', self.m.phase)
        self.fusion.current.speed = 0
        self.m.tick(now + .1, 'NAVIGATION')
        self.assertEqual((0, 0, False), self.m.tick(now + .45, 'NAVIGATION'))
        self.assertEqual('PAUSED', self.m.phase)
        self.m.tick(now + .5, 'IDLE')

    def preview(self, operation='resume', now=10, **kw):
        ok, msg = self.command('preview', now, operation=operation, **kw)
        self.assertTrue(ok, msg)
        return self.m.operations.preview['token']

    def resume(self, now=10):
        token = self.preview(now=now)
        ok, msg = self.command('resume', now + .1, preview_token=token)
        self.assertTrue(ok, msg)
        return token

    def test_operator_pause_never_auto_resumes_even_when_position_recovers(self):
        self.start(); self.pause()
        checkpoint = self.m.operations.record['cursor_m']
        for usable in (False, True):
            self.fusion.current.usable_for_control = usable
            for t in range(20, 30):
                self.assertEqual((0, 0, None), self.m.tick(t, 'IDLE'))
        self.assertEqual('PAUSED', self.m.phase)
        self.assertEqual(checkpoint, self.m.operations.record['cursor_m'])
        self.assertFalse(self.m.command({'cmd': 'route_start'}, 31)[0])

    def test_continue_in_place_finishes_remaining_route(self):
        self.start(); self.pause(); checkpoint=self.m.operations.record['cursor_m']
        self.resume()
        self.drive(10.2, 150)
        self.assertEqual('DONE', self.m.operations.record['state'])
        self.assertGreater(self.m.operations.record['total_m'], checkpoint)
        x, y = self.centre()
        self.assertAlmostEqual(20, x, delta=.1)
        self.assertAlmostEqual(10, y, delta=.1)

    def centre(self):
        p=self.fusion.current;c=(self.m.body[0]-self.m.body[1])/2
        return p.x+c*math.cos(p.yaw),p.y+c*math.sin(p.yaw)

    def test_moved_car_gets_entry_without_losing_checkpoint(self):
        self.start();self.pause()
        before=self.m.operations.record['cursor_m']
        self.fusion.current=FakeFusion.make(2,-3,90)
        self.m.tick(10,'MANUAL');self.m.tick(11,'IDLE')
        token=self.preview(now=12)
        self.assertGreater(self.m.operations.preview['entry_m'],1)
        self.assertTrue(self.command('resume',12.1,preview_token=token)[0])
        self.drive(12.2,14)
        self.assertEqual(before,self.m.operations.record['cursor_m'])  # driving to the join isn't task progress
        self.drive(14.1,180)
        self.assertEqual('DONE',self.m.operations.record['state'])

    def test_restart_preserves_branch_and_never_drives_on_load(self):
        circle=dict(kind='circle',center=[0,4],radius=4,laps=3,clockwise=False,cruise=.5)
        self.start(circle);self.drive(8.1,75);self.pause(76)
        previous=copy.deepcopy(self.m.operations.record)
        self.m=LineMission(self.fusion,SAMPLES,route_path=self.route_file)
        self.assertEqual('PAUSED',self.m.phase)
        self.assertEqual(previous['id'],self.m.operations.record['id'])
        self.assertGreater(previous['cursor_m'],2*math.pi*4)
        self.assertEqual((0,0,None),self.m.tick(80,'IDLE'))
        self.resume(81)
        self.drive(81.2,300)
        self.assertEqual('DONE',self.m.operations.record['state'])

    def test_progress_recovers_after_unplanned_process_restart(self):
        self.start();before=self.m.operations.record['cursor_m']
        self.m=LineMission(self.fusion,SAMPLES,route_path=self.route_file)
        self.assertEqual('PAUSED',self.m.phase)
        self.assertAlmostEqual(before,self.m.operations.record['cursor_m'],delta=.6)
        self.assertEqual('restart',self.m.operations.record['reason'])

    def test_rework_range_then_restore_original_progress(self):
        self.start();self.pause()
        original=copy.deepcopy(self.m.operations.record)
        token=self.preview('rework',element=0,from_m=5,to_m=8)
        self.assertTrue(self.command('rework',10.1,preview_token=token)[0])
        self.assertEqual('rework',self.m.operations.record['kind'])
        self.drive(10.2,100)
        self.assertEqual('PAUSED',self.m.phase)
        self.assertEqual(original,self.m.operations.record)
        self.assertEqual('DONE',self.m.operations.history[-1]['state'])
        self.assertEqual(3,self.m.operations.history[-1]['total_m'])
        x,y=self.centre();self.assertAlmostEqual(x,8,delta=.1);self.assertAlmostEqual(y,0,delta=.05)

    def test_cancel_rework_preserves_parent_and_restart(self):
        self.start();self.pause();parent=self.m.operations.record['id']
        token=self.preview('rework',element=0,from_m=5,to_m=8)
        self.command('rework',10.1,preview_token=token)
        self.drive(10.2,12)
        self.m=LineMission(self.fusion,SAMPLES,route_path=self.route_file)
        self.assertEqual(parent,self.m.operations.parent['id'])
        self.assertTrue(self.m.command({'cmd':'stop'},13)[0])
        self.assertEqual(parent,self.m.operations.record['id'])
        self.assertEqual('PAUSED',self.m.phase)

    def test_preview_is_read_only_and_duplicate_commit_does_not_restart(self):
        self.start();self.pause();before=copy.deepcopy(self.m.operations.record)
        token=self.preview()
        self.assertEqual(before,self.m.operations.record)
        self.assertEqual((0,0,None),self.m.tick(10.05,'IDLE'))
        self.command('resume',10.1,preview_token=token)
        self.drive(10.2,12)
        tracker=self.m.tracker
        self.assertTrue(self.command('resume',12.1,preview_token=token)[0])
        self.assertIs(tracker,self.m.tracker)

    def test_expired_moved_stale_revision_or_estop_cannot_commit(self):
        self.start();self.pause()
        token=self.preview()
        self.assertFalse(self.command('resume',100,preview_token=token)[0])
        token=self.preview(now=101);self.fusion.current.x+=.5
        self.assertFalse(self.command('resume',101.1,preview_token=token)[0])
        token=self.preview(now=102);self.m.tick(102.1,'ESTOP')
        self.assertFalse(self.command('resume',102.2,preview_token=token)[0])
        self.m.tick(103,'IDLE');token=self.preview(now=103.1)
        self.m.route_rev+=1
        self.assertFalse(self.command('resume',103.2,preview_token=token)[0])

    def test_reverse_checkpoint_keeps_direction(self):
        self.start()
        tracker=self.m.tracker
        index=next(i for i,s in enumerate(tracker.sections) if s.direction<0)
        tracker.index=index
        tr=tracker.trackers[index];sec=tracker.sections[index]
        s=sec.path.length/2
        x,y,yaw=sec.car_pose(s)
        self.fusion.current=FakeFusion.make(x,y,math.degrees(yaw))
        tr._hint=s;tr.st.s=s;tracker._sync()
        self.pause()
        self.preview()
        self.assertEqual(-1,self.m.operations.preview['rest'][0].direction)
        self.resume();self.drive(10.2,180)
        self.assertEqual('DONE',self.m.operations.record['state'])

    def test_clear_or_edit_cannot_destroy_paused_progress(self):
        self.start();self.pause();identity=self.m.operations.record['id']
        for cmd in ({'cmd':'route_clear'},{'cmd':'route_set','route':ROUTE}):
            self.assertFalse(self.m.command(cmd,10)[0])
        self.assertEqual(identity,self.m.operations.record['id'])
        self.m.command({'cmd':'stop'},11)
        self.assertEqual('CANCELLED',self.m.operations.record['state'])
        self.assertTrue(self.m.command({'cmd':'route_set','route':ROUTE},12)[0])

    def test_storage_failure_stops_and_is_reported(self):
        self.start()
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.assertEqual((0,0,None),self.m.tick(20,'NAVIGATION'))
        self.assertEqual('PAUSING',self.m.phase)
        self.assertIn('保存失败',self.m.message)

    def test_pause_still_stops_when_checkpoint_cannot_be_written(self):
        self.start()
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.pause()
        self.assertEqual('PAUSED',self.m.phase)
        self.assertIn('保存失败',self.m.message)

    def test_failed_rework_persistence_keeps_parent_and_does_not_arm(self):
        self.start();self.pause();before=copy.deepcopy(self.m.operations.record)
        token=self.preview('rework',element=0,from_m=5,to_m=8)
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.assertFalse(self.command('rework',10.1,preview_token=token)[0])
        self.assertEqual(before,self.m.operations.record)
        self.assertEqual('PAUSED',self.m.phase)
        self.assertIsNone(self.m.operations.parent)

    def test_unsafe_ranges_and_wire_inputs_rejected(self):
        self.start();self.pause()
        for element,a,b in [(0,-1,3),(0,3,99),(0,3,3.01),(999,0,3),(True,0,3),(0,float('nan'),3)]:
            ok,_=self.command('preview',operation='rework',element=element,from_m=a,to_m=b)
            self.assertFalse(ok)
        for payload in [dict(action='pause',route_rev=True,job_id='x'),dict(action='resume',route_rev=1),
                        dict(action='preview',route_rev=1,operation='rework',element=False,from_m=0,to_m=3),
                        dict(action='preview',route_rev=1,operation='rework',element=0,from_m=-1,to_m=3),
                        dict(action='preview',route_rev=1,operation='resume',job_id='x',overlap_m=1)]:
            with self.assertRaises(app_link.CommandError):
                app_link.parse_command(json.dumps(dict(type='route',**payload)))

    def test_resume_near_finish_and_short_rework(self):
        self.start()
        t=self.m.tracker;t.index=len(t.sections)-1
        tr=t.trackers[t.index];sec=t.sections[t.index];u=tr.path.length-.2
        x,y,yaw=sec.car_pose(u)
        self.fusion.current=FakeFusion.make(x,y,math.degrees(yaw))
        tr._hint=u;tr.st.s=u;t._sync()
        self.pause();self.resume();self.drive(10.2,20)
        self.assertEqual('DONE',self.m.operations.record['state'])
        self.fusion.current.speed=0;self.m.tick(21,'IDLE')
        token=self.preview('rework',now=22,element=0,from_m=5,to_m=5.25)
        self.assertTrue(self.command('rework',22.1,preview_token=token)[0])
        self.drive(22.2,200)
        self.assertEqual('DONE',self.m.operations.history[-1]['state'])
        self.assertAlmostEqual(.25,self.m.operations.history[-1]['total_m'])

    def test_real_telemetry_includes_confirmed_checkpoint_and_preview(self):
        self.start();self.pause();self.preview()
        raw=self.m.status();raw['_rx_monotonic']=10
        wire=app_link.line_telemetry(raw,10)
        payload=json.loads(json.dumps(wire,allow_nan=False))
        self.assertEqual('PAUSED',payload['operations']['job']['state'])
        self.assertEqual('PAUSED',payload['track']['state'])
        self.assertEqual('resume',payload['operations']['preview']['operation'])
        self.assertFalse(payload['operations']['spray_supported'])
        self.assertEqual({'stale':True},app_link.line_telemetry(raw,100))

    def test_origin_change_reprojects_saved_checkpoint(self):
        self.start();self.pause()
        plan=self.m.operations.decode(self.m.operations.record)
        original=plan[0].car_pose(0)
        class ShiftFusion(FakeFusion):
            def to_local(self,lat,lon):
                x,y=super().to_local(lat,lon);return x-50,y+20
            def to_geodetic(self,x,y):return super().to_geodetic(x+50,y-20)
        self.fusion=ShiftFusion()
        self.m=LineMission(self.fusion,SAMPLES,route_path=self.route_file)
        changed=self.m.operations.decode(self.m.operations.record)[0].car_pose(0)
        self.assertAlmostEqual(original[0]-50,changed[0],places=6)
        self.assertAlmostEqual(original[1]+20,changed[1],places=6)


class TestReworkGeometry(unittest.TestCase):
    def test_circle_interval_remains_an_arc(self):
        spec=route_plan.parse_spec(dict(kind='circle',center=[0,5],radius=5,laps=3,clockwise=False))
        sections,_=rework_sections(spec,0,5,10,2,(.67,.08,.335))
        self.assertEqual(5,sections[0].path.length)
        self.assertAlmostEqual(.2,sections[0].path.pieces[0].kappa)
        x,y,_=sections[0].path.pose(0)
        self.assertAlmostEqual(5,math.hypot(x,y-5))

    def test_tail_at_cusp_does_not_restart_completed_segment(self):
        sections=[Section(1,Path((0,0,0),[(5,0)])),Section(-1,Path((5,0,math.pi),[(2,0)]))]
        rest=tail(sections,5)
        self.assertEqual(1,len(rest));self.assertEqual(-1,rest[0].direction)


if __name__=='__main__':unittest.main()
