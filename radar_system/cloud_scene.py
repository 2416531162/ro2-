"""Bounded recent-observation 3D display, not a navigation/collision map.

The binary wire format is N x [x,y,z,intensity] little-endian float32.
Missing intensity = -1 (never invented from height). Actual sample locations
are retained; voxel cells only select samples, not fabricated surfaces.
"""
import gzip
import math
from collections import OrderedDict, deque
import uuid
import numpy as np

DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
RAMP = np.array([[.64,.18,.85],[.14,.43,1.],[.04,.82,.86],[.23,.86,.40],[1.,.86,.20],[1.,.31,.17]])


def read_xyzi(msg, limit=8000):
    """Read arbitrary PointCloud2 offsets, row padding, byte order and scalar types."""
    w,h,ps,rs = int(msg.width),int(msg.height),int(msg.point_step),int(msg.row_step)
    if w == 0 or h == 0:
        return np.empty((0,4),np.float32)
    if min(w,h,ps,rs,limit) <= 0 or w*h > 2_000_000 or rs < w*ps or len(msg.data) < rs*h:
        raise ValueError('invalid/oversized PointCloud2 dimensions, stride or buffer')
    fields={}
    for f in msg.fields:
        if f.name in fields:
            raise ValueError('duplicate point field')
        fields[f.name]=f
    indices=np.arange(0,w*h,max(1,math.ceil(w*h/limit)))
    rows,cols=indices//w,indices%w
    out=np.full((len(indices),4),-1.,dtype=np.float32)
    for j,name in enumerate(('x','y','z','intensity')):
        if name not in fields:
            if j<3:
                raise ValueError('PointCloud2 missing '+name)
            continue
        f=fields[name]
        if f.datatype not in DTYPES or f.count != 1:
            raise ValueError('non-scalar/unsupported '+name)
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+DTYPES[f.datatype])
        if f.offset<0 or f.offset+dtype.itemsize>ps:
            raise ValueError('point field outside point_step')
        values=np.ndarray((h,w),dtype=dtype,buffer=msg.data,offset=f.offset,strides=(rs,ps))
        out[:,j]=values[rows,cols]
    valid=np.isfinite(out[:,:3]).all(axis=1)&(np.abs(out[:,:3])<100000).all(axis=1)
    out[~np.isfinite(out[:,3])|(out[:,3]<0),3]=-1
    return out[valid]


def depth_xyzi(msg, info, step=6, max_range=5.5):
    """Metric optical XYZ; requires registered depth and matching intrinsics/frame."""
    if (info is None or not msg.header.frame_id or info.header.frame_id!=msg.header.frame_id
            or (info.width,info.height)!=(msg.width,msg.height)):
        raise ValueError('深度图与 CameraInfo 必须同 frame、同尺寸')
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    if not all(math.isfinite(v) for v in (fx,fy,cx,cy)) or min(fx,fy)<=0:
        raise ValueError('invalid camera intrinsics')
    if msg.width*msg.height>2_000_000 or min(msg.width,msg.height)<=0:
        raise ValueError('oversized/empty depth image')
    enc=msg.encoding.lower()
    if enc in ('16uc1','mono16'):
        dtype=np.dtype('>u2' if msg.is_bigendian else '<u2'); scale=.001
    elif enc=='32fc1':
        dtype=np.dtype('>f4' if msg.is_bigendian else '<f4'); scale=1.
    else:
        raise ValueError('unsupported depth encoding')
    if msg.step < msg.width*dtype.itemsize or len(msg.data)<msg.step*msg.height:
        raise ValueError('invalid depth stride/buffer')
    step=max(2,int(step))
    depth=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.step,dtype.itemsize))
    v,u=np.mgrid[0:msg.height:step,0:msg.width:step]
    z=depth[::step,::step].astype(np.float32)*scale
    mask=np.isfinite(z)&(z>.2)&(z<max_range)
    z=z[mask]
    return np.column_stack(((u[mask]-cx)*z/fx,(v[mask]-cy)*z/fy,z,np.full(len(z),-1))).astype(np.float32)


def apply_matrix(points, translation, quaternion):
    """Rigid transform preserving measured intensity; normalize the quaternion."""
    out=np.array(points,dtype=np.float32,copy=True).reshape(-1,4)
    q=np.asarray(quaternion,dtype=float)
    norm=np.linalg.norm(q)
    if not math.isfinite(norm) or norm<1e-8 or not np.isfinite(translation).all():
        raise ValueError('invalid transform')
    x,y,z,w=q/norm
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    out[:,:3]=out[:,:3]@r.T+translation
    return out


def pose_jump(anchor, pose, distance=.25, angle=.10):
    if anchor is None:
        return False
    return (math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])>distance or
            abs(math.atan2(math.sin(pose[2]-anchor[2]),math.cos(pose[2]-anchor[2])))>angle)


class VoxelHistory:
    """LRU + TTL bound. Recent observations may contain temporary moving-object trails."""
    def __init__(self, voxel=.05, limit=60000, ttl=45.):
        if not .02<=voxel<=.5 or not 100<=limit<=120000 or not 1<=ttl<=120:
            raise ValueError('invalid display budget')
        self.voxel,self.limit,self.ttl=voxel,int(limit),ttl
        self.cells=OrderedDict()
        self.dirty=False

    def clear(self):
        self.cells.clear(); self.dirty=True

    def expire(self, now):
        while self.cells and next(iter(self.cells.values()))[0]<now-self.ttl:
            self.cells.popitem(last=False); self.dirty=True

    def add(self, points, now):
        self.expire(now)
        a=np.asarray(points,dtype=np.float32).reshape(-1,4)
        a=a[np.isfinite(a[:,:3]).all(axis=1)&(np.abs(a[:,:3])<100000).all(axis=1)]
        if not len(a):
            return
        keys=np.floor(a[:,:3]/self.voxel).astype(np.int64)
        # One sample/cell/frame: deterministic, bounded input handling.
        _,ids=np.unique(keys,axis=0,return_index=True)
        for i in ids:
            key=tuple(keys[i])
            self.cells[key]=(now,a[i].copy())
            self.cells.move_to_end(key)
        while len(self.cells)>self.limit:
            self.cells.popitem(last=False)
        self.dirty=True

    def array(self):
        return np.array([v[1] for v in self.cells.values()],dtype=np.float32).reshape(-1,4)


class ScenePackets:
    """Epoch/revision-bound immutable packets. Reconnects cannot reuse another map's bytes."""
    def __init__(self):
        self.epoch=uuid.uuid4().hex[:16]
        self.revision=0
        self.cache=deque(maxlen=3)
        self.meta={}
        self.update(np.empty((0,4)))

    def reset(self):
        self.epoch=uuid.uuid4().hex[:16]
        self.revision=0; self.cache.clear()
        self.update(np.empty((0,4)))

    def update(self, points):
        a=np.ascontiguousarray(points,dtype='<f4').reshape(-1,4)
        if len(a)>120000 or not np.isfinite(a).all():
            raise ValueError('invalid scene packet')
        raw=a.tobytes()
        if self.cache and self.cache[-1][1]==raw:
            return
        self.revision+=1
        self.cache.append((self.revision,raw,gzip.compress(raw,compresslevel=2,mtime=0)))
        valid=a[:,3][a[:,3]>=0]
        self.meta=dict(epoch=self.epoch,revision=self.revision,count=len(a),frame='map',
                       stride=16,format='xyzi-f32le',intensity_available=bool(len(valid)),
                       intensity_range=[float(valid.min()),float(valid.max())] if len(valid) else None,
                       bounds=[a[:,:3].min(axis=0).tolist(),a[:,:3].max(axis=0).tolist()] if len(a) else None)

    def get(self, epoch, revision, compressed=False):
        if epoch!=self.epoch:
            return None
        return next((p[2 if compressed else 1] for p in self.cache if p[0]==revision),None)


def camera_basis(target, distance, azimuth, elevation):
    e=max(.06,min(math.pi/2,elevation))
    direction=np.array([math.cos(e)*math.cos(azimuth),math.cos(e)*math.sin(azimuth),math.sin(e)])
    right=np.array([-math.sin(azimuth),math.cos(azimuth),0.])
    up=np.cross(direction,right)
    return np.asarray(target)+max(.5,distance)*direction,right,up,-direction


def project(points, target, distance, azimuth, elevation, width, height):
    eye,r,u,f=camera_basis(target,distance,azimuth,elevation)
    rel=np.asarray(points).reshape(-1,3)-eye
    depth=rel@f
    focal=height/(2*math.tan(math.pi/8))
    safe=np.maximum(depth,.001)
    return np.column_stack((width/2+(rel@r)*focal/safe,height/2-(rel@u)*focal/safe,depth))


def colors(points, mode='height', low=-.2, high=3., robot=(0,0,0), intensity_range=None):
    a=np.asarray(points).reshape(-1,4)
    if mode=='intensity' and intensity_range:
        v=a[:,3]; low,high=intensity_range
    elif mode=='distance':
        v=np.linalg.norm(a[:,:3]-np.asarray(robot),axis=1); low,high=0.,10.
    else:
        v=a[:,2]
    t=np.clip((v-low)/max(high-low,1e-6),0,1)*5
    i=np.minimum(t.astype(int),4)
    result=RAMP[i]*(1-(t-i))[:,None]+RAMP[i+1]*(t-i)[:,None]
    if mode=='intensity':
        result[a[:,3]<0]=[.55,.58,.62]
    return (result*255).round().astype(np.uint8)


def raster(points, width=960, height=640, target=(0,0,0), distance=14., azimuth=-2.1,
           elevation=.9, color_mode='height', z_low=-.2, z_high=3., point_size=2,
           light=False, robot=(0,0,0), intensity_range=None):
    """Native Qt fallback: real perspective + nearest-depth pixels, vectorized NumPy.

    No per-point QPainter calls or fake extruded surfaces. Called off the GUI thread.
    """
    width=max(1,min(1600,int(width))); height=max(1,min(1200,int(height)))
    a=np.asarray(points,dtype=np.float32).reshape(-1,4)
    a=a[np.isfinite(a[:,:3]).all(axis=1)&(a[:,2]>=z_low)&(a[:,2]<=z_high)]
    image=np.empty((height,width,4),np.uint8)
    image[:]=[235,241,246,255] if light else [15,22,32,255]
    if not len(a):
        return image
    p=project(a[:,:3],target,distance,azimuth,elevation,width,height)
    valid=(p[:,2]>.1)&(p[:,0]>=0)&(p[:,0]<width)&(p[:,1]>=0)&(p[:,1]<height)
    p,a=p[valid],a[valid]
    if not len(a):
        return image
    x,y=p[:,0].astype(int),p[:,1].astype(int)
    indices=y*width+x
    order=np.lexsort((p[:,2],indices))
    ordered=indices[order]
    keep=np.r_[True,ordered[1:]!=ordered[:-1]]
    chosen=order[keep]
    x,y,depth=x[chosen],y[chosen],p[chosen,2]
    rgb=colors(a[chosen],color_mode,z_low,z_high,robot,intensity_range)
    zbuf=np.full((height,width),np.inf,np.float32)
    size=max(1,min(4,int(point_size)))
    for dy in range(-(size//2),size-size//2):
        for dx in range(-(size//2),size-size//2):
            xx,yy=x+dx,y+dy
            inside=(xx>=0)&(xx<width)&(yy>=0)&(yy<height)
            ids=np.flatnonzero(inside)
            ids=ids[depth[ids]<zbuf[yy[ids],xx[ids]]]
            zbuf[yy[ids],xx[ids]]=depth[ids]
            image[yy[ids],xx[ids],:3]=rgb[ids]
    return image
