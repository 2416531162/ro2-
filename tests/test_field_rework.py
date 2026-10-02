"""用真实任务状态机和理想运动替身验证整场局部补画，不连接车辆硬件。"""
import copy
import json
import math
import unittest
from unittest.mock import patch

import app_link
from field_rework import crop, decode_source, drawing_points, length
from field_planner import parse_strokes, polygon, stroke_options, Settings
from line_mission import LineMission
from line_tracker import LineTracker, LineConfig
from test_line_mission import FakeFusion, SAMPLES
import test_field_planner as field_tests
from test_field_planner import BOUNDARY, LINE


class TestFieldRework(unittest.TestCase):
    setUp = field_tests.TestFieldExecution.setUp
    tearDown = field_tests.TestFieldExecution.tearDown
    prepare = field_tests.TestFieldExecution.prepare

    def start_paused(self, elements=None, selected=None):
        p = self.prepare(elements, selected)
        value = self.cache.get(p['token'], .1)
        ok, msg = self.m.command(dict(cmd='route_field_start', field_plan=value, allow_outside=True), .1)
        self.assertTrue(ok, msg)
        self.m.tick(.2, 'NAVIGATION')
        t = self.m.tracker
        t.index = next(i for i, s in enumerate(t.sections) if s.field_role == 'paint')
        sec, tr = t.sections[t.index], t.trackers[t.index]
        mid = sec.path.length / 3
        x, y, h = sec.car_pose(mid)
        self.f.current = FakeFusion.make(x, y, math.degrees(h))
        tr._hint = mid
        tr.st.s = mid
        t._sync()
        self.assertTrue(self.command('pause', .3)[0])
        self.m.tick(.4, 'NAVIGATION')
        self.m.tick(.8, 'NAVIGATION')
        self.m.tick(.9, 'IDLE')
        self.assertEqual('PAUSED', self.m.phase)
        return copy.deepcopy(self.m.operations.record)

    def command(self, action, now=1, **values):
        return self.m.command(dict(cmd='route_' + action, id=action, route_rev=self.m.route_rev,
                                   job_id=self.m.operations.record['id'], **values), now)

    def preview_rework(self, ident='line', edge=0, start=1, end=3, now=1):
        ok, msg = self.command('preview', now, operation='rework', element_id=ident, edge_index=edge,
                               from_m=start, to_m=end)
        self.assertTrue(ok, msg)
        return self.m.operations.preview['token']

    def accept(self, token, now=1.1):
        ok, msg = self.command('rework', now, preview_token=token, allow_outside=True)
        self.assertTrue(ok, msg)

    def drive_until_parent(self, parent, now=1.2, limit=200):
        dt = .02
        while now < limit:
            v, w, _ = self.m.tick(now, 'NAVIGATION')
            p = self.f.current
            h = p.yaw + w * dt
            if abs(w) > 1e-9:
                x = p.x + v/w * (math.sin(h)-math.sin(p.yaw))
                y = p.y - v/w * (math.cos(h)-math.cos(p.yaw))
            else:
                x, y = p.x + v*dt*math.cos(h), p.y + v*dt*math.sin(h)
            self.f.current = FakeFusion.make(x, y, math.degrees(h), speed=v)
            self.f.current.yaw_rate = w
            if self.m.phase == 'PAUSED' and self.m.operations.record['id'] == parent['id']:
                return now
            now += dt
        self.fail('补画未能完成并恢复原断点：' + self.m.message)

    def restart(self, fusion=None):
        if fusion is not None:
            self.f = fusion
        self.m = LineMission(self.f, SAMPLES, LineTracker(LineConfig(), self.geo), body=(.9,.6,.39),
                             nozzle_x=.3, route_path=self.m.route_path)

    def test_complete_local_rework_restores_exact_parent_then_resumes_original_sections(self):
        parent = self.start_paused([LINE, dict(LINE, id='next', points=[[20,10],[25,10]])])
        token = self.preview_rework()
        before = copy.deepcopy(self.m.operations.preview)
        self.assertEqual(parent, self.m.operations.record)
        self.assertTrue(all(s.field_role == 'transit' for s in before['entry']))
        self.assertEqual('paint', before['rest'][0].field_role)
        self.accept(token)
        child = self.m.operations.record
        self.assertEqual(parent, self.m.operations.parent)
        self.assertTrue(child['field_plan'])
        self.assertEqual(parent['id'], self.m.operations.status()['job']['parent_job_id'])
        self.assertEqual(2, child['target_length_m'])
        end = self.drive_until_parent(parent)
        self.assertEqual(parent, self.m.operations.record)
        self.assertIsNone(self.m.operations.parent)
        self.assertEqual('DONE', self.m.operations.history[-1]['state'])
        self.f.current.speed = self.f.current.yaw_rate = 0
        self.m.tick(end+.1, 'IDLE')
        ok, msg = self.command('preview', end+.2, operation='resume')
        self.assertTrue(ok, msg)
        self.assertEqual(parent['cursor_m'], self.m.operations.preview['base'])
        token = self.m.operations.preview['token']
        self.assertTrue(self.command('resume', end+.3, preview_token=token, allow_outside=True)[0])
        self.assertEqual(parent['plan'], self.m.operations.record['plan'])

    def test_closed_last_edge_and_outside_original_scope_are_explicit_choices(self):
        frame = dict(id='frame', kind='polyline', points=BOUNDARY, closed=True)
        parent = self.start_paused([frame], [dict(element_id='frame',edge_index=0)])
        wire = self.m.operations.status()
        self.assertEqual(4, len(wire['elements']))
        self.assertEqual(4, len(wire['context']))
        self.assertIn('field_rework', wire['actions'])
        token = self.preview_rework('frame', 3, 2, 4)
        preview = self.m.operations.status()['preview']
        self.assertTrue(preview['field_plan'])
        self.assertEqual(4, len(preview['context']))
        self.assertIn('start', preview)
        points = preview['target'][0]['points']
        self.assertAlmostEqual(0, points[0][0], places=7)
        self.assertEqual(parent, self.m.operations.record)
        self.accept(token)
        self.assertEqual(parent['selected_strokes'], self.m.operations.parent['selected_strokes'])
        self.assertTrue(self.m.command(dict(cmd='stop'),1.2)[0])
        self.assertEqual(parent, self.m.operations.record)

    def test_original_81_edges_with_one_selected_edge_remains_available_for_local_rework(self):
        long_line=dict(id='long',kind='polyline',points=[[5+i*.25,5] for i in range(82)])
        parent=self.start_paused([long_line],[dict(element_id='long',edge_index=0)])
        status=self.m.operations.status()
        self.assertIn('field_rework',status['actions'])
        self.assertEqual(81,len(status['elements']))
        self.assertEqual(81,len(status['context']))
        self.assertEqual(1,sum(s.field_role=='paint' for s in self.m.operations.decode(parent)))
        token=self.preview_rework('long',80,.05,.25)
        points=self.m.operations.status()['preview']['target'][0]['points']
        self.assertEqual({25.05,25.25},{round(p[0],2) for p in points})
        self.accept(token)
        self.assertEqual(parent,self.m.operations.parent)
        self.assertEqual(parent['selected_strokes'],self.m.operations.parent['selected_strokes'])

    def test_rework_interval_wire_fields_are_strict_and_legacy_element_still_works(self):
        base=dict(type='route',action='preview',operation='rework',route_rev=1,job_id='parent',
                  element_id='arc',edge_index=None,from_m=1,to_m=3)
        parsed=app_link.parse_command(json.dumps(base))
        self.assertEqual('arc',parsed['element_id']);self.assertIsNone(parsed['edge_index'])
        for bad in (dict(edge_index=True),dict(element_id=''),dict(element=0),dict(job_id=None),
                    dict(from_m=-1),dict(to_m=float('nan'))):
            with self.assertRaises(app_link.CommandError):app_link.parse_command(json.dumps(dict(base,**bad)))
        legacy=dict(base);legacy.pop('element_id');legacy.pop('edge_index');legacy['element']=0
        self.assertEqual(0,app_link.parse_command(json.dumps(legacy))['element'])

    def test_restart_and_changed_origin_preserve_circle_source_and_parent(self):
        arc=dict(id='arc',kind='arc',center=[15,10],radius=3,start_deg=40,sweep_deg=-120)
        parent=self.start_paused([LINE,arc])
        self.accept(self.preview_rework('arc',None,.5,2))
        child=copy.deepcopy(self.m.operations.record)
        parent=json.loads(json.dumps(parent))
        child=json.loads(json.dumps(child))
        class ShiftFusion(FakeFusion):
            def to_local(self,lat,lon):
                x,y=super().to_local(lat,lon);return x-2,y+1
            def to_geodetic(self,x,y):return super().to_geodetic(x+2,y-1)
        shifted=ShiftFusion();p=self.f.current
        shifted.current=FakeFusion.make(p.x-2,p.y+1,math.degrees(p.yaw))
        self.restart(shifted)
        self.assertEqual('PAUSED',self.m.phase)
        self.assertEqual(parent,self.m.operations.parent)
        self.assertEqual(child['plan'],self.m.operations.record['plan'])
        source=decode_source(self.m.operations.record['field_source'],shifted)
        self.assertAlmostEqual(13,source['elements'][1]['center'][0],places=6)
        self.assertAlmostEqual(11,source['elements'][1]['center'][1],places=6)
        self.assertAlmostEqual(40,source['elements'][1]['start_deg'],places=6)
        self.assertEqual((0,0,None),self.m.tick(2,'IDLE'))
        self.assertTrue(self.m.command(dict(cmd='stop'),2.1)[0])
        self.assertEqual(parent,self.m.operations.record)
        self.m.tick(2.2,'IDLE')
        self.preview_rework('arc',None,.5,2,now=2.3)

    def test_estop_or_failure_keeps_child_and_parent_until_explicit_cancel(self):
        for failure in ('ESTOP','FAULT'):
            with self.subTest(failure=failure):
                parent=self.start_paused()
                self.accept(self.preview_rework())
                self.m.tick(1.2,'NAVIGATION')
                self.m.tick(1.3,failure)
                self.assertEqual('INTERRUPTED',self.m.operations.record['state'])
                self.assertEqual(parent,self.m.operations.parent)
                self.assertFalse(self.command('preview',1.4,operation='resume')[0])
                self.m.tick(1.5,'IDLE')
                self.assertTrue(self.command('preview',1.6,operation='resume')[0])
                self.assertTrue(self.m.command(dict(cmd='stop'),1.7)[0])
                self.assertEqual(parent,self.m.operations.record)
                self.assertEqual('PAUSED',self.m.phase)
                self.m.command(dict(cmd='stop'),1.8)
                self.f.current=FakeFusion.make(3,5,0);self.m.tick(0,'IDLE')

    def test_acceptance_write_failure_keeps_parent_and_preview_unarmed(self):
        parent=self.start_paused();token=self.preview_rework();tracker=self.m.tracker
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.assertFalse(self.command('rework',1.1,preview_token=token,allow_outside=True)[0])
        self.assertEqual(parent,self.m.operations.record);self.assertIsNone(self.m.operations.parent)
        self.assertIs(tracker,self.m.tracker);self.assertEqual(token,self.m.operations.preview['token'])

    def test_completion_write_failure_keeps_parent_until_single_successful_retry(self):
        parent=self.start_paused();self.accept(self.preview_rework())
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.m.command(dict(cmd='stop'),1.2)
        self.assertEqual(parent,self.m.operations.parent)
        self.assertEqual('PAUSED',self.m.operations.record['state'])
        self.assertEqual('rework',self.m.operations.record['kind'])
        self.assertEqual([],self.m.operations.history)
        self.assertTrue(self.m.command(dict(cmd='stop'),1.3)[0])
        self.assertEqual(parent,self.m.operations.record)
        self.assertEqual(1,len(self.m.operations.history))
        self.restart();self.assertEqual(parent['id'],self.m.operations.record['id'])
        self.assertIsNone(self.m.operations.parent);self.assertEqual(1,len(self.m.operations.history))

    def test_done_save_failure_and_retry_do_not_duplicate_history_or_lose_parent(self):
        parent=self.start_paused();self.accept(self.preview_rework())
        child_id=self.m.operations.record['id']
        with patch('route_operations.os.replace',side_effect=OSError('disk full')):
            self.m._finish('补画完成',state='DONE')
        self.assertEqual(parent,self.m.operations.parent)
        self.assertEqual(child_id,self.m.operations.record['id'])
        self.assertEqual('PAUSED',self.m.phase);self.assertEqual([],self.m.operations.history)
        self.m._finish('补画完成',state='DONE')
        self.assertEqual(parent,self.m.operations.record);self.assertIsNone(self.m.operations.parent)
        self.assertEqual(1,len(self.m.operations.history));self.assertEqual('DONE',self.m.operations.history[0]['state'])

    def test_partial_circle_runs_to_exact_nozzle_endpoint_then_restores_parent(self):
        circle=dict(id='circle',kind='circle',center=[15,10],radius=3,start_deg=40)
        parent=self.start_paused([LINE,circle])
        token=self.preview_rework('circle',None,.5,2)
        target=self.m.operations.preview['target'][0]['points']
        self.accept(token);self.drive_until_parent(parent)
        p=self.f.current
        tip=(p.x+.3*math.cos(p.yaw)+.5*math.sin(p.yaw),p.y+.3*math.sin(p.yaw)-.5*math.cos(p.yaw))
        self.assertLess(math.dist(tip,target[-1]),.08)
        self.assertEqual(parent,self.m.operations.record)

    def test_delayed_bound_end_of_completed_child_never_stops_restored_parent(self):
        parent=self.start_paused();self.accept(self.preview_rework());child_id=self.m.operations.record['id']
        end=self.drive_until_parent(parent);tracker=self.m.tracker
        command=dict(cmd='route_end',route_rev=self.m.route_rev,job_id=child_id)
        self.assertTrue(self.m.command(command,end+.1)[0])
        self.assertEqual(parent,self.m.operations.record);self.assertIs(tracker,self.m.tracker)
        self.f.current.speed=self.f.current.yaw_rate=0;self.m.tick(end+.2,'IDLE')
        self.assertTrue(self.command('preview',end+.3,operation='resume')[0])
        self.assertTrue(self.command('resume',end+.4,preview_token=self.m.operations.preview['token'],allow_outside=True)[0])
        self.m.tick(end+.5,'NAVIGATION');tracker=self.m.tracker
        self.assertTrue(self.m.command(command,end+.6)[0])
        self.assertEqual('RUNNING',self.m.phase);self.assertIs(tracker,self.m.tracker)
        self.assertEqual('RUNNING',tracker.st.state)
        self.assertFalse(self.m.command(dict(command,job_id='unknown'),end+.7)[0])
        self.assertEqual('RUNNING',tracker.st.state)

    def test_corrupt_source_does_not_crash_telemetry_or_replace_saved_parent_geometry(self):
        parent=self.start_paused()
        self.m.operations.record['field_source']['elements'][0]['invalid']=float('nan')
        self.assertNotIn('field_rework',self.m.operations.status()['actions'])
        self.assertEqual(parent['plan'],self.m.operations.record['plan'])

    def test_preview_stale_source_profile_position_parent_and_nested_rework_rejected(self):
        parent=self.start_paused();token=self.preview_rework()
        self.assertFalse(self.command('rework',62,preview_token=token,allow_outside=True)[0])
        token=self.preview_rework(now=63);self.f.current.x+=.11
        self.assertFalse(self.command('rework',63.1,preview_token=token,allow_outside=True)[0])
        token=self.preview_rework(now=64);self.m.operations.record['field_context']['right']+=.01
        self.assertFalse(self.command('rework',64.1,preview_token=token,allow_outside=True)[0])
        self.m.operations.record=copy.deepcopy(parent)
        token=self.preview_rework(now=65);self.m.operations.record['field_source']['elements'][0]['name']='变更'
        self.assertFalse(self.command('rework',65.1,preview_token=token,allow_outside=True)[0])
        self.m.operations.record=copy.deepcopy(parent)
        token=self.preview_rework(now=66);self.accept(token,66.1);tracker=self.m.tracker
        self.assertTrue(self.command('rework',66.2,preview_token=token,allow_outside=True)[0])
        self.assertIs(tracker,self.m.tracker)
        self.m.operations.record['state']='PAUSED';self.m.phase='PAUSED';self.m.tick(66.3,'IDLE')
        self.assertFalse(self.command('preview',66.4,operation='rework',element_id='line',edge_index=0,from_m=1,to_m=3)[0])

    def test_old_field_checkpoint_still_resumes_but_does_not_offer_rework(self):
        self.start_paused()
        for key in ('field_source','field_source_hash','field_profile'):self.m.operations.record.pop(key)
        self.m.operations.save();self.restart();self.m.tick(1,'IDLE')
        self.assertNotIn('field_rework',self.m.operations.status()['actions'])
        self.assertTrue(self.command('preview',1.1,operation='resume')[0])
        self.assertFalse(self.command('preview',1.2,operation='rework',element_id='line',edge_index=0,from_m=1,to_m=3)[0])


class TestFieldReworkGeometry(unittest.TestCase):
    def test_circle_and_negative_arc_crop_use_target_length_and_hit_exact_nozzle_endpoints(self):
        cfg=Settings(.3,.5,(.9,.6,.39),.557);_,planes=polygon(BOUNDARY)
        for kind,sweep in (('circle',360),('arc',-120)):
            item=dict(id='s',kind=kind,center=[15,10],radius=3,start_deg=40,sweep_deg=sweep)
            stroke=parse_strokes([item])[0][0]
            cropped=crop(stroke,.5,2)
            ends=drawing_points(cropped)[::len(drawing_points(cropped))-1]
            for option in stroke_options(cropped,cfg,planes):
                sec=option['section'];actual=[]
                self.assertAlmostEqual(1.5,option['nozzle_length'])
                self.assertNotAlmostEqual(1.5,sec.path.length,places=4)
                for s in (0,sec.path.length):
                    x,y,h=sec.car_pose(s)
                    actual.append((x+cfg.forward*math.cos(h)+cfg.right*math.sin(h),
                                   y+cfg.forward*math.sin(h)-cfg.right*math.cos(h)))
                wanted=ends[::-1] if option['reverse'] else ends
                for a,b in zip(actual,wanted):self.assertLess(math.dist(a,b),1e-8)
            self.assertAlmostEqual(length(stroke),3*abs(math.radians(sweep)))


if __name__=='__main__':unittest.main()
