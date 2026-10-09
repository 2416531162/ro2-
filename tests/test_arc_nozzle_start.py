"""Start at the actual nozzle crossing; preserve tail geometry and read-only status."""
from pathlib import Path
import sys,math,unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_continuous_field_arcs as fa
from route_operations import RouteOperations


def setup(k=.4,length=2.):
    sec=fa.Section(1,fa.Route((0.,0.,0.),[(1.,0.),(1.,k),(length,k),(.5,k)]))
    sec.field_role='paint';sec.paint_from_m=2.;sec.paint_to_m=2.+length
    tr=fa.tracker([sec]);tr.st.state='RUNNING';tr.trackers[0].st.state='RUNNING'
    m=NS(route_path=None,tracker=tr,route=None,differential=True,phase='RUNNING',nozzle=(.24,-.4575))
    ops=RouteOperations(m);ops.record=dict(id='test',field_plan=True,field_context=dict(forward=.24,right=.4575))
    def at(s,yaw=0.,update=True):
        child=tr.trackers[0];child.st.s=s;child.st.kappa_ref=sec.path.kappa(s)
        x,y,h=sec.car_pose(s);m.pose=fa.pose(x,y,h+yaw,.5,.5*sec.path.kappa(s))
        if update:ops.update_paint_tail(s)
        return ops.paint_status()
    return sec,m,ops,at


class ArcNozzleStart(unittest.TestCase):
    def test_side_nozzle_can_reach_the_head_before_the_axle_in_both_directions(self):
        for k in (-.7,.4):
            sec,m,ops,at=setup(k)
            self.assertTrue(at(sec.paint_from_m-.005,math.radians(2))['painting'])

    def test_axle_crossing_does_not_open_before_a_lagging_nozzle(self):
        for k in (-.7,.4):
            sec,m,ops,at=setup(k)
            self.assertFalse(at(sec.paint_from_m+.005,-math.radians(2))['painting'])
            self.assertTrue(at(sec.paint_from_m+.05,-math.radians(2))['painting'])

    def test_crossing_latches_on_control_updates_but_status_reads_do_not_latch(self):
        sec,m,ops,at=setup()
        self.assertTrue(at(sec.paint_from_m+.001,update=False)['painting'])
        self.assertFalse(at(sec.paint_from_m-.001,update=False)['painting'])
        self.assertTrue(at(sec.paint_from_m+.001)['painting'])
        self.assertTrue(at(sec.paint_from_m-.001)['painting'])
        self.assertFalse(at(sec.paint_to_m-.04)['painting'])

    def test_pause_rearms_the_crossing_and_nearby_transit_does_not_open(self):
        sec,m,ops,at=setup()
        self.assertTrue(at(sec.paint_from_m+.001)['painting'])
        m.phase='PAUSED';self.assertFalse(at(sec.paint_from_m-.01)['painting'])
        m.phase='RUNNING';self.assertFalse(at(sec.paint_from_m-.01)['painting'])
        at(sec.paint_from_m-.3,update=False)
        m.pose=fa.pose(*sec.car_pose(sec.paint_from_m+.01),.5,.2)
        self.assertFalse(ops.paint_status()['painting'])

    def test_empty_early_stop_interval_remains_dry(self):
        sec,m,ops,at=setup(length=.02)
        self.assertFalse(at(sec.paint_from_m-.005,math.radians(2))['painting'])
        self.assertFalse(at(sec.paint_from_m+.001)['painting'])

    def test_straight_entry_before_an_arc_keeps_the_original_start_rule(self):
        sec,m,ops,at=setup()
        sec.paint_from_m=1.
        self.assertFalse(at(.995,math.radians(2))['painting'])

    def test_straight_and_full_circle_keep_the_axle_start_rule(self):
        for k,length in ((0.,2.),(.4,2*math.pi/.4)):
            sec,m,ops,at=setup(k,length)
            self.assertFalse(at(sec.paint_from_m-.005,math.radians(2))['painting'])
            self.assertTrue(at(sec.paint_from_m+.005,-math.radians(2))['painting'])

if __name__=='__main__':unittest.main()
