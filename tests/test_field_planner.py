import json
import math
import os
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import patch

from test_line_mission import FakeFusion,SAMPLES
from field_planner import Settings,plan,path_violation,polygon,corner_positions,violation,stroke_options,target_points
from field_plan_service import FieldPlanService
from path_tracker import Path,Section,SectionTracker,PathConfig
from line_mission import LineMission
from line_tracker import LineTracker,LineConfig
from robot_core.kinematics import ChassisGeometry
import app_link

BOUNDARY=[[0,0],[30,0],[30,20],[0,20]]
LINE=dict(id='line',name='测试线',kind='polyline',points=[[5,5],[15,5]])


class TestFieldPlanner(unittest.TestCase):
    def setUp(self):self.cfg=Settings(.3,.5,(.9,.6,.39),.557)

    def test_right_spray_outer_boundary_uses_counterclockwise_travel(self):
        job=dict(boundary=BOUNDARY,elements=[dict(id='edge',kind='polyline',points=BOUNDARY,closed=True)])
        out=plan(job,(2,18,0),self.cfg)
        self.assertEqual(4,len(out['order']))
        self.assertTrue(all(not x['reversed'] for x in out['order']))
        self.assertFalse(out['wet_paint_considered']);self.assertFalse(out['global_optimum'])

    def test_starts_from_current_vehicle_pose_and_changes_first_job(self):
        a=dict(LINE,id='a',points=[[5,5],[10,5]])
        b=dict(LINE,id='b',points=[[20,15],[25,15]])
        job=dict(boundary=BOUNDARY,elements=[a,b])
        near_a=plan(job,(4.7,5.5,0),self.cfg)
        near_b=plan(job,(19.7,15.5,0),self.cfg)
        self.assertEqual('a/0',near_a['order'][0]['id']);self.assertEqual('b/0',near_b['order'][0]['id'])
        for out in (near_a,near_b):self.assertEqual(out['start'][:2],out['motions'][0]['start'][:2])

    def test_offset_transformation_hits_the_original_spray_geometry(self):
        _,planes=polygon(BOUNDARY)
        cfg=Settings(.3,.24,(.911,.604,.39),.557)
        for kind in ('line','circle','arc'):
            s=dict(id='s',name='s',kind=kind,a=(5,5),b=(15,5),c=(15,10),r=4,start=.2,sweep=1.7)
            for v in stroke_options(s,cfg,planes):
                sec=v['section']
                for i in range(21):
                    x,y,h=sec.car_pose(sec.path.length*i/20)
                    q=(x+cfg.forward*math.cos(h)+cfg.right*math.sin(h),y+cfg.forward*math.sin(h)-cfg.right*math.cos(h))
                    if kind=='line':self.assertAlmostEqual(5,q[1],places=8)
                    else:self.assertAlmostEqual(4,math.dist(q,s['c']),places=8)

    def test_complete_body_is_checked_not_only_the_axle(self):
        _,planes=polygon(BOUNDARY)
        sec=Section(1,Path((.1,5,0),[(10,0)]))
        self.assertGreater(path_violation(sec,planes,self.cfg),.4)
        safe=Section(1,Path((2,5,0),[(10,0)]))
        self.assertEqual(0,path_violation(safe,planes,self.cfg))

    def test_continuous_arc_check_catches_violation_between_endpoints(self):
        _,planes=polygon([[0,0],[10,0],[10,10],[0,10]])
        cfg=Settings(.3,.5,(.3,.3,.2),.2)
        sec=Section(1,Path((5,9.5,math.pi/2),[(math.pi,1)]))
        exact=path_violation(sec,planes,cfg)
        self.assertGreater(exact,.5)
        sampled=max(violation(sec.car_pose(sec.path.length*i/10000),planes,cfg.body) for i in range(10001))
        self.assertAlmostEqual(exact,sampled,places=5)

    def test_current_small_side_offset_reports_unavoidable_outside_space(self):
        out=plan(dict(boundary=BOUNDARY,elements=[dict(LINE,points=[[1,0],[29,0]])]),(2,2,0),Settings(.3,.24,(.911,.604,.39),.557))
        self.assertFalse(out['inside']);self.assertGreaterEqual(out['max_outside_m'],.15-1e-6)
        self.assertTrue(out['outside'])

    def test_all_requested_strokes_preserved_and_wet_settings_ignored(self):
        job=dict(boundary=BOUNDARY,elements=[LINE,dict(id='circle',kind='circle',center=[20,10],radius=3)])
        a=plan(job,(2,3,0),self.cfg)
        b=plan(dict(job,wet_minutes=999,avoid_wet=True),(2,3,0),self.cfg)
        self.assertEqual(a,b);self.assertEqual({'line/0','circle'},{v['id'] for v in a['order']})
        self.assertAlmostEqual(10+6*math.pi,a['paint_length_m'])

    def test_one_edge_of_a_closed_frame_keeps_the_whole_boundary(self):
        job=dict(boundary=BOUNDARY,elements=[dict(id='edge',name='边框',kind='polyline',points=BOUNDARY,closed=True)],
                 selected_strokes=[dict(element_id='edge',edge_index=2)])
        before=json.dumps(job,sort_keys=True)
        out=plan(job,(2,18,0),self.cfg)
        self.assertEqual('selected',out['selection_mode']);self.assertEqual(1,len(out['order']))
        self.assertEqual(dict(element_id='edge',edge_index=2),out['order'][0]['selection'])
        self.assertEqual({(30,20),(0,20)},{tuple(q) for q in out['target'][0]})
        self.assertEqual([tuple(q) for q in BOUNDARY],out['boundary'])
        self.assertAlmostEqual(30,out['paint_length_m']);self.assertEqual(before,json.dumps(job,sort_keys=True))

    def test_selected_line_ignores_unselected_infeasible_curve_and_manual_points(self):
        job=dict(boundary=BOUNDARY,elements=[LINE,dict(id='tiny',kind='circle',center=[10,10],radius=.1),
                                           dict(id='spot',kind='point',center=[15,10])])
        with self.assertRaises(ValueError):plan(job,(3,5,0),self.cfg)
        out=plan(dict(job,selected_strokes=[dict(element_id='line',edge_index=0)]),(3,5,0),self.cfg)
        self.assertEqual(['line/0'],[v['id'] for v in out['order']]);self.assertEqual([],out['manual'])

    def test_selecting_disconnected_edges_and_curves_does_not_add_paint_connectors(self):
        elements=[dict(id='frame',kind='polyline',points=BOUNDARY,closed=True),dict(id='circle',kind='circle',center=[15,10],radius=3),
                  dict(id='arc',kind='arc',center=[10,10],radius=2,start_deg=0,sweep_deg=90)]
        chosen=[dict(element_id='frame',edge_index=0),dict(element_id='frame',edge_index=2),dict(element_id='circle',edge_index=None),dict(element_id='arc',edge_index=None)]
        out=plan(dict(boundary=BOUNDARY,elements=elements,selected_strokes=chosen),(2,3,0),self.cfg)
        self.assertEqual({(x['element_id'],x['edge_index']) for x in chosen},
                         {(x['selection']['element_id'],x['selection']['edge_index']) for x in out['order']})
        self.assertAlmostEqual(60+7*math.pi,out['paint_length_m'])
        self.assertEqual(4,sum(s['role']=='paint' for s in out['motions']))
        self.assertTrue(any(s['role']=='transit' for s in out['motions']))

    def test_invalid_empty_duplicate_or_manual_selections_are_not_full_field_requests(self):
        elements=[LINE,dict(id='dot',kind='point',center=[10,10])]
        valid=dict(element_id='line',edge_index=0)
        invalid=[None,[],[valid,valid],[dict(element_id='missing',edge_index=0)],
                 [dict(element_id='line',edge_index=True)],[dict(element_id='line',edge_index=0.0)],
                 [dict(element_id='line',edge_index=1)],[dict(element_id='line',edge_index=None)],
                 [dict(element_id='dot',edge_index=None)],[dict(element_id='line')]]
        for selection in invalid:
            with self.subTest(selection=selection),self.assertRaises(ValueError):
                plan(dict(boundary=BOUNDARY,elements=elements,selected_strokes=selection),(2,3,0),self.cfg)

    def test_original_element_ids_with_slashes_do_not_collide_with_edge_numbers(self):
        elements=[LINE,dict(id='line/0',kind='circle',center=[20,10],radius=3)]
        selected=[dict(element_id='line',edge_index=0),dict(element_id='line/0',edge_index=None)]
        out=plan(dict(boundary=BOUNDARY,elements=elements,selected_strokes=selected),(3,5,0),self.cfg)
        self.assertEqual(2,len(out['order']));self.assertAlmostEqual(10+6*math.pi,out['paint_length_m'])
        self.assertEqual({'line','line/0'},{v['selection']['element_id'] for v in out['order']})

    def test_partial_selection_can_use_a_drawing_with_more_than_80_edges(self):
        job=dict(boundary=[[0,0],[120,0],[120,20],[0,20]],elements=[dict(id='long',kind='polyline',points=[[i,5] for i in range(101)])])
        with self.assertRaises(ValueError):plan(job,(2,3,0),self.cfg)
        out=plan(dict(job,selected_strokes=[dict(element_id='long',edge_index=99)]),(95,5,0),self.cfg)
        self.assertEqual(1,len(out['order']));self.assertAlmostEqual(1,out['paint_length_m'])

    def test_invalid_and_nonconvex_boundaries_fail_without_guessing(self):
        for boundary in ([],[[0,0],[1,0],[2,0]],[[0,0],[10,0],[5,5],[10,10],[0,10]]):
            with self.assertRaises(ValueError):plan(dict(boundary=boundary,elements=[LINE]),(0,0,0),self.cfg)

    def test_far_away_car_is_rejected_and_large_curves_have_bounded_preview(self):
        with self.assertRaises(ValueError):plan(dict(boundary=BOUNDARY,elements=[LINE]),(-100,0,0),self.cfg)
        _,planes=polygon([[-300,-300],[300,-300],[300,300],[-300,300]])
        s=dict(id='circle',name='circle',kind='circle',c=(0,0),r=200,start=0,sweep=2*math.pi)
        self.assertTrue(all(len(target_points(v))<=2049 for v in stroke_options(s,self.cfg,planes)))

    def test_reverse_arc_continuous_body_envelope_matches_dense_independent_sampling(self):
        _,planes=polygon(BOUNDARY)
        for direction in (1,-1):
            for kappa in (-.7,.7):
                sec=Section(direction,Path((1,1,.4),[(8,kappa)]))
                sampled=max(violation(sec.car_pose(sec.path.length*i/10000),planes,self.cfg.body) for i in range(10001))
                self.assertAlmostEqual(path_violation(sec,planes,self.cfg),sampled,places=5)

    def test_whole_football_job_completes_all_24_strokes_with_real_section_controller(self):
        with open(os.path.join(os.path.dirname(__file__),'fixtures','field_job.json')) as f:source=json.load(f)
        elements=[dict(s['field'],id=s['id'],name=s['name'],kind=s['kind']) for key in ('sequence','manual') for s in source[key]]
        p=plan(dict(boundary=[[0,0],[105,0],[105,68],[0,68]],elements=elements),(1,66,0),
               Settings(.3,.24,(.911,.604,.39),.557,accel=.5))
        self.assertEqual(24,len(p['order']));self.assertEqual(3,len(p['manual']))
        sections=[]
        for s in p['motions']:
            sec=Section(s['direction'],Path(s['start'],s['pieces']));sec.cruise_mps=s['speed'];sections.append(sec)
        tracker=SectionTracker(sections,replace(PathConfig(),track_x_m=0,start_min_left_m=0),
                               ChassisGeometry(drive='differential',diff_min_radius_m=.5),
                               pivot_rate=.8,pivot_accel=1,reverse_gain=1)
        pose=FakeFusion.make(1,66,0);self.assertTrue(tracker.start(pose))
        hits=set();peak=0;dt=.05
        for i in range(200000):
            v,w=tracker.update(pose,dt)
            if p['motions'][tracker.index]['role']=='paint':hits.add(tracker.index);peak=max(peak,abs(tracker.st.e))
            h=pose.yaw+w*dt
            if abs(w)>1e-9:
                pose.x+=v/w*(math.sin(h)-math.sin(pose.yaw));pose.y+=v/w*(math.cos(pose.yaw)-math.cos(h))
            else:pose.x+=v*math.cos(h)*dt;pose.y+=v*math.sin(h)*dt
            pose.yaw=h;pose.speed=v;pose.yaw_rate=w
            if tracker.st.state not in ('RUNNING','PAUSED'):break
        self.assertEqual('DONE',tracker.st.state,tracker.st.reason)
        self.assertEqual(24,len(hits));self.assertLess(peak,.05)
        self.assertTrue(any(s['outside_spans'] for s in p['preview']))


class TestFieldExecution(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.f=FakeFusion();self.f.current=FakeFusion.make(3,5,0)
        self.geo=ChassisGeometry(drive='differential',diff_min_radius_m=.5)
        self.m=LineMission(self.f,SAMPLES,LineTracker(LineConfig(),self.geo),body=(.9,.6,.39),nozzle_x=.3,
                           route_path=os.path.join(self.tmp.name,'route.json'))
        self.m.tick(0,'IDLE')
        self.m.command(dict(cmd='route_set',route=dict(kind='path',points=[[5,5],[10,5]],closed=False)),0)
        self.cache=FieldPlanService()
    def tearDown(self):self.m._close_log();self.tmp.cleanup()
    def prepare(self,elements=None,selected=None):
        line=self.m.status();line.update(_rx_monotonic=0,motion_mode='IDLE')
        job=dict(boundary=BOUNDARY,elements=elements or [dict(LINE,points=[[5,5],[10,5]])])
        if selected is not None:job['selected_strokes']=selected
        return self.cache.prepare(dict(origin=self.m._origin(),nozzle=[.3,.5],job=job),line,0)

    def test_selected_edge_stops_at_its_end_and_checkpoint_contains_no_other_edges(self):
        chosen=[dict(element_id='frame',edge_index=0)]
        p=self.prepare([dict(id='frame',kind='polyline',points=BOUNDARY,closed=True)],chosen)
        value=self.cache.get(p['token'],.1)
        self.assertEqual(4,len(value['source_job']['elements'][0]['points']))
        ok,msg=self.m.command(dict(cmd='route_field_start',field_plan=value,allow_outside=True),.1)
        self.assertTrue(ok,msg)
        self.assertEqual(chosen,self.m.operations.record['selected_strokes'])
        self.assertEqual(1,sum(s.field_role=='paint' for s in self.m.operations.decode(self.m.operations.record)))
        self.assertEqual('选边作业 · 1 条',self.m.operations.record['label'])
        for i in range(1,9000):
            v,w,_=self.m.tick(.1+i*.02,'NAVIGATION')
            p=self.f.current;h=p.yaw+w*.02
            self.f.current=FakeFusion.make(p.x+v*math.cos(h)*.02,p.y+v*math.sin(h)*.02,math.degrees(h),speed=v)
            if self.m.phase=='IDLE':break
        self.assertEqual('DONE',self.m.operations.record['state'])
        p=self.f.current;qx=p.x+.3*math.cos(p.yaw)+.5*math.sin(p.yaw);qy=p.y+.3*math.sin(p.yaw)-.5*math.cos(p.yaw)
        self.assertLess(min(math.dist((qx,qy),(0,0)),math.dist((qx,qy),(30,0))),.08)
    def test_preview_does_not_arm_and_start_uses_the_cached_path(self):
        preview=self.prepare();self.assertEqual('IDLE',self.m.phase)
        value=self.cache.get(preview['token'],.1)
        ok,msg=self.m.command(dict(cmd='route_field_start',field_plan=value,allow_outside=not value['inside']),.1)
        self.assertTrue(ok,msg);self.assertEqual('SELECTING',self.m.phase)
        for i in range(1,3000):
            v,w,_=self.m.tick(.1+i*.02,'NAVIGATION')
            p=self.f.current;h=p.yaw+w*.02
            self.f.current=FakeFusion.make(p.x+v*math.cos(h)*.02,p.y+v*math.sin(h)*.02,math.degrees(h),speed=v)
            if self.m.phase=='IDLE':break
        self.assertEqual('DONE',self.m.operations.record['state'])
        x,y,h=self.f.current.x,self.f.current.y,self.f.current.yaw
        nozzle=(x+.3*math.cos(h)+.5*math.sin(h),y+.3*math.sin(h)-.5*math.cos(h))
        self.assertLess(min(math.dist(nozzle,(5,5)),math.dist(nozzle,(10,5))),.08)
    def test_full_field_cached_plan_passes_mission_checks_without_losing_a_motion(self):
        with open(os.path.join(os.path.dirname(__file__),'fixtures','field_job.json')) as f:source=json.load(f)
        elements=[dict(s['field'],id=s['id'],name=s['name'],kind=s['kind']) for key in ('sequence','manual') for s in source[key]]
        line=self.m.status();line.update(_rx_monotonic=0,motion_mode='IDLE')
        p=self.cache.prepare(dict(origin=self.m._origin(),nozzle=[.3,.24],job=dict(boundary=[[0,0],[105,0],[105,68],[0,68]],elements=elements)),line,0)
        value=self.cache.get(p['token'],.1)
        ok,msg=self.m.command(dict(cmd='route_field_start',field_plan=value,allow_outside=True),.1)
        self.assertTrue(ok,msg)
        self.assertEqual(len(value['motions']),len(self.m.tracker.sections))
        self.assertEqual(24,sum(s.field_role=='paint' for s in self.m.tracker.sections))
    def test_moved_vehicle_and_expired_preview_rejected(self):
        p=self.prepare();value=self.cache.get(p['token'],.1)
        self.f.current.x+=.3
        self.assertFalse(self.m.command(dict(cmd='route_field_start',field_plan=value),.1)[0])
        with self.assertRaises(ValueError):self.cache.get(p['token'],61)
    def test_outside_path_needs_visible_acknowledgement(self):
        p=self.prepare([dict(LINE,points=[[0,0],[30,0]])]);value=self.cache.get(p['token'],.1)
        self.assertFalse(value['inside'])
        self.assertFalse(self.m.command(dict(cmd='route_field_start',field_plan=value),.1)[0])
        self.assertTrue(self.m.command(dict(cmd='route_field_start',field_plan=value,allow_outside=True),.2)[0])
    def test_speed_and_field_mode_survive_pause_and_restart(self):
        p=self.prepare();value=self.cache.get(p['token'],.1)
        self.m.command(dict(cmd='route_field_start',field_plan=value,allow_outside=True),.1)
        self.m.tick(.2,'NAVIGATION')
        job=self.m.operations.record
        self.m.command(dict(cmd='route_pause',route_rev=self.m.route_rev,job_id=job['id']),.3)
        self.m.tick(.4,'NAVIGATION');self.m.tick(.8,'NAVIGATION')
        restored=LineMission(self.f,SAMPLES,LineTracker(LineConfig(),self.geo),body=(.9,.6,.39),nozzle_x=.3,route_path=self.m.route_path)
        self.assertTrue(restored.operations.record['field_plan']);self.assertEqual('PAUSED',restored.phase)
        self.assertTrue(all(s.cruise_mps is not None for s in restored.operations.decode(restored.operations.record)))
        restored.tick(1.0,'IDLE')
        self.f.current=FakeFusion.make(4,7,90)
        identity=dict(route_rev=restored.route_rev,job_id=job['id'])
        ok,msg=restored.command(dict(cmd='route_preview',operation='resume',id='resume',**identity),1.1)
        self.assertTrue(ok,msg)
        preview=restored.operations.status()['preview']
        self.assertIsNotNone(preview['max_outside_m']);self.assertLess(preview['max_outside_m'],1e-6)
        ok,msg=restored.command(dict(cmd='route_resume',preview_token=preview['token'],**identity),1.2)
        self.assertTrue(ok,msg)
        for i in range(1,8000):
            v,w,_=restored.tick(1.2+i*.02,'NAVIGATION')
            p=self.f.current;h=p.yaw+w*.02
            self.f.current=FakeFusion.make(p.x+v*math.cos(h)*.02,p.y+v*math.sin(h)*.02,math.degrees(h),speed=v)
            if restored.phase=='IDLE':break
        self.assertEqual('DONE',restored.operations.record['state'])
        restored._close_log()


if __name__=='__main__':unittest.main()
