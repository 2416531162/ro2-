"""Explicitly synthetic room samples, solely for renderer regression/demo. NO ROS input."""
import numpy as np


def room_scene():
    rng=np.random.default_rng(42)
    groups=[]
    def plane(axis,value,r1,r2,step=.055):
        axes=[i for i in range(3) if i!=axis]
        a,b=np.meshgrid(np.arange(*r1,step),np.arange(*r2,step))
        p=np.zeros((a.size,4),np.float32);p[:,axis]=value;p[:,axes[0]]=a.ravel();p[:,axes[1]]=b.ravel();p[:,3]=-1
        groups.append(p)
    # Open ceiling; measured-sample look, no faces/meshes or fabricated infill in viewer.
    plane(0,-4,(-3,6),(0,2.4));plane(0,4,(-3,6),(0,2.4));plane(1,6,(-4,4),(0,2.4))
    plane(1,-3,(-4,-1.3),(0,2.4));plane(1,-3,(1.3,4),(0,2.4))
    plane(0,0,(-3,.6),(0,2.4));plane(0,0,(2.,6),(0,2.4));plane(0,0,(.6,2.),(2.,2.4))
    # Two tables, chair seats/backs and thin vertical legs.
    for cx,cy in [(-2.1,1.1),(2.1,3.7)]:
        plane(2,.8,(cx-.75,cx+.75),(cy-.48,cy+.48),.035)
        for x in [cx-.66,cx+.66]:
            for y in [cy-.40,cy+.40]:
                plane(0,x,(y-.025,y+.025),(0,.8),.025)
        plane(2,.44,(cx-.3,cx+.3),(cy-1.1,cy-.6),.03)
        plane(1,cy-1.1,(cx-.3,cx+.3),(.44,1.0),.035)
    # Sparse ground returns rather than a solid shaded floor.
    n=3500;floor=np.column_stack((rng.uniform(-4,4,n),rng.uniform(-3,6,n),rng.normal(0,.008,n),np.full(n,-1)))
    groups.append(floor)
    p=np.concatenate(groups).astype(np.float32)
    p[:,:3]+=rng.normal(0,.008,p[:,:3].shape).astype(np.float32)
    return p[::max(1,len(p)//55000)]


def fixture_state(meta):
    scene=dict(meta,source='depth',age_s=.12,live=True,voxel_m=.05,limit=60000,history_s=45,
               kind='recent_observations',error='',reset_reason='测试数据',calibrated=True)
    return dict(scene=scene,epoch=meta['epoch'],localized=True,robot=[-1.8,-.5,1.2],
                map=dict(known_area_m2=65.4),scan_live=True,scan=[[-3.95,float(y)] for y in np.linspace(-2.5,3,50)],
                trajectory=[[-2.9,-2.3],[-2.7,-1.8],[-2.35,-1.3],[-2.1,-.9],[-1.8,-.5]],
                plan=[[-1.8,-.5],[-1.6,0],[-1.2,.6]],people=[dict(x=-1.0,y=1.5,locked=True)],
                goal=dict(x=-1.3,y=.2,validated=False),error='',session=dict(mode='mapping',maps=['office_01']),
                test_fixture='界面验收 · 合成测试数据，非实车扫描')
