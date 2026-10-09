"""Short paint arcs warm the observer before spraying and preserve it across plan pieces."""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT)]
from path_tracker import Path as Route,PathTracker,PathConfig,differential_config
from robot_core.kinematics import ChassisGeometry

GEO=ChassisGeometry(drive='differential',track_m=.675,diff_min_radius_m=.5)
DT=.02

def tracker(split=True,k=-.5,full=False):
    paint_length=(2*math.pi if full else math.pi/2)/abs(k)
    pieces=[(2.,0.)]+([(1.5,k),(paint_length,k),(.5,k)] if split else [(2.+paint_length,k)])
    tr=PathTracker(Route((0.,0.,0.),pieces),differential_config(PathConfig()),GEO)
    tr.set_nozzle(.24,-.4575)
    tr.paint_from_m=3.5;tr.paint_to_m=3.5+paint_length
    tr.paint_spans=[(tr.paint_from_m,tr.paint_to_m)]
    return tr

def feed(tr,i,shift=0.,fixed=True):
    now=i*DT;s=.5*now;x,y,h=tr.path.pose(s);k=tr.path.kappa(s)
    tr.st.s=s;tr.sync(now,(now,h+1.7,.5*k))
    tr.set_heading_baseline(now,.415)
    tr.set_rtk_quality(now,.01,24,.005,fixed)
    for lever,setter in (((.235,.197),tr.set_primary_fix),((.235,-.218),tr.set_secondary_fix)):
        ax,ay=lever
        setter(now,x-math.sin(h)*shift+ax*math.cos(h)-ay*math.sin(h),
               y+math.cos(h)*shift+ax*math.sin(h)+ay*math.cos(h),fixed,lever)
    p=NS(x=x,y=y,yaw=h,speed=.5,yaw_rate=.5*k,usable_for_control=True,
         quality='RTK_FIXED' if fixed else 'RTK_FLOAT',reasons=(),rtk_trusted=fixed,correction_xy=(0.,0.))
    out=tr._arc_first(p,DT)
    return out,dict(tr._arc_status)

class PartialArcObserver(unittest.TestCase):
    def test_piece_boundaries_do_not_reset_a_warmed_observer_in_either_direction(self):
        for k in (-.5,.5):
            split,merged=tracker(k=k),tracker(False,k=k)
            seen=set()
            for i in range(1,int(split.path.length/.5/DT)):
                a,sa=feed(split,i);b,sb=feed(merged,i)
                self.assertEqual(sa['active'],sb['active'])
                if a is not None:
                    for key in ('x','y','yaw'):self.assertAlmostEqual(getattr(a,key),getattr(b,key),places=10)
                if split.st.s>=split.paint_from_m:
                    self.assertTrue(sa['active'])
                    self.assertEqual(sa['weight'],1.)
                    seen.add(split.path.pieces.index(split.path._piece(split.st.s)))
            self.assertEqual(seen,{2,3})

    def test_unpainted_transit_and_straight_painting_do_not_enable_the_observer(self):
        for mode in ('transit','straight','disabled'):
            tr=tracker()
            if mode=='transit':tr._arc_work_role=False
            if mode=='straight':tr.paint_spans=[(0.,1.)]
            if mode=='disabled':tr.cfg=replace(tr.cfg,arc_partial_imu_first=False)
            for i in range(1,600):
                result,state=feed(tr,i)
                self.assertIsNone(result)
                self.assertFalse(state['active'])

    def test_full_circle_keeps_the_original_eligibility_and_state(self):
        a,b=tracker(full=True),tracker(full=True)
        b.cfg=replace(b.cfg,arc_partial_imu_first=False)
        for i in range(1,1000):
            pa,sa=feed(a,i);pb,sb=feed(b,i)
            self.assertEqual(sa,sb)
            self.assertEqual(pa,pb)

    def test_small_corner_keeps_its_existing_controller(self):
        a,b=tracker(k=-.7001463154759235),tracker(k=-.7001463154759235)
        b.cfg=replace(b.cfg,arc_partial_imu_first=False)
        for i in range(1,int(a.path.length/.5/DT)):
            pa,sa=feed(a,i);pb,sb=feed(b,i)
            self.assertIsNone(pa)
            self.assertEqual(sa,sb)
            self.assertEqual(pa,pb)

    def test_missing_raw_antenna_positions_do_not_enable_the_short_arc_observer(self):
        tr=tracker()
        for i in range(1,400):feed(tr,i)
        piece=tr.path._piece(tr.st.s)
        tr._primary=None
        self.assertIsNone(tr._partial_work_arc_piece(piece))

    def test_common_jump_is_held_across_the_spray_start_boundary(self):
        tr=tracker()
        for i in range(1,345):feed(tr,i)
        self.assertTrue(tr._arc_status['active'])
        for i in range(345,390):
            _,state=feed(tr,i,shift=.04)
            self.assertTrue(state['holding'])
            self.assertEqual(state['radial_rtk_weight'],0.)
            self.assertEqual(state['radial_correction_m'],0.)
        _,state=feed(tr,390)
        self.assertFalse(state['holding'])

    def test_float_positions_cannot_pull_the_short_arc_observer(self):
        a,b=tracker(),tracker()
        for i in range(1,350):feed(a,i);feed(b,i)
        for i in range(350,400):
            _,sa=feed(a,i,fixed=False);_,sb=feed(b,i,shift=.10,fixed=False)
            self.assertEqual(sb['radial_rtk_weight'],0.)
            self.assertEqual(sa['e_est'],sb['e_est'])

if __name__=='__main__':unittest.main()
