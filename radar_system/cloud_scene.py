"""Bounded recent-observation 3D display, not a navigation/collision map.

The binary wire format is N x [x,y,z,intensity] little-endian float32.
Missing intensity = -1 (never invented from height). Actual sample locations
are retained; voxel cells only select samples, not fabricated surfaces.
"""
import gzip
import math
from collections import deque
import uuid
import numpy as np

DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
RAMP = np.array([[.64,.18,.85],[.14,.43,1.],[.04,.82,.86],[.23,.86,.40],[1.,.86,.20],[1.,.31,.17]])


def read_xyzi(msg, limit=25000):
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
    for j,name in enumerate(('x','y','z')):
        if name not in fields:
            raise ValueError('PointCloud2 missing '+name)
        f=fields[name]
        if f.datatype not in DTYPES or f.count != 1:
            raise ValueError('non-scalar/unsupported '+name)
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+DTYPES[f.datatype])
        if f.offset<0 or f.offset+dtype.itemsize>ps:
            raise ValueError('point field outside point_step')
        values=np.ndarray((h,w),dtype=dtype,buffer=msg.data,offset=f.offset,strides=(rs,ps))
        out[:,j]=values[rows,cols]
    color_field = 'intensity' if 'intensity' in fields else ('rgb' if 'rgb' in fields else None)
    if color_field:
        f=fields[color_field]
        if f.datatype not in DTYPES or f.count != 1:
            raise ValueError('non-scalar/unsupported '+color_field)
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+DTYPES[f.datatype])
        if f.offset<0 or f.offset+dtype.itemsize>ps:
            raise ValueError('point field outside point_step')
        values=np.ndarray((h,w),dtype=dtype,buffer=msg.data,offset=f.offset,strides=(rs,ps))
        out[:,3]=values[rows,cols].view(np.float32) if color_field=='rgb' else values[rows,cols]
    valid=np.isfinite(out[:,:3]).all(axis=1)&(np.abs(out[:,:3])<100000).all(axis=1)
    if color_field!='rgb':
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


KEY_LIMIT = 1 << 20          # 单轴体素索引上限,打包成 int64 用(±52km @5cm)


def body_mask(points, robot, front=.67, rear=.18, half_width=.335, height=.45):
    """标出落在车体自身范围内的点(世界坐标 → 车体坐标判断)。

    俯视 15° 的相机看得见自己的车头,N10P 也会打到自己的车壳。这些点如果
    进了地图,车一走就在身后拖出一条不存在的墙,而且它**正好跟着车动**,
    看起来特别像真的障碍物。

    判断必须在车体坐标系里做、跟着车头转:用一个固定朝向的世界方框,
    车一转弯就会开始吃掉真实的障碍物。

    height 是车壳顶面高度(实测车体 0.34m,默认留到 0.45m),不是一个"安全
    高度"。设得太高会把车体正上方的门梁、货架下沿一起抹掉 —— 那些是真实
    几何,应该出现在地图里。
    """
    a = np.asarray(points, dtype=np.float32).reshape(-1, 4)
    if not len(a) or robot is None:
        return np.zeros(len(a), dtype=bool)
    x, y, yaw = float(robot[0]), float(robot[1]), float(robot[2])
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = a[:, 0] - x, a[:, 1] - y
    bx = dx * c - dy * s
    by = dx * s + dy * c
    return (bx >= -rear) & (bx <= front) & (np.abs(by) <= half_width) & (a[:, 2] <= height)


class VoxelHistory:
    """有界的三维观测累积。

    两种模式,由 ttl 决定:
        ttl > 0   最近 ttl 秒的观测窗口。走过的房间会过期消失,但移动的人
                  留下的拖影也会消失。
        ttl == 0  长期累积。走过的地方一直留着 —— 这才是"建出一张三维地图",
                  内存只由 limit 兜底(按最近观测时间淘汰)。

    实现用三个**按体素键排序**的并行数组,而不是 OrderedDict:
        _keys[i]  体素键(int64 打包),升序唯一
        _data[i]  该格最近一次的真实采样 xyzi(不是格中心,不编造表面)
        _seen[i]  最近观测的墙钟时间,用于 TTL
        _rank[i]  单调递增的观测序号,用于 LRU 淘汰

    为什么换掉 OrderedDict:原来的实现每帧对每个格子做一次 Python 字典操作,
    array() 还要用 Python 列表把六万行 numpy 逐行拼回去。实测(x86)
    add 29ms、array 56ms,RK3588 上是这个的三倍左右,而且全程占着同一把锁,
    网页和屏幕都得排队等 —— 这正是之前"点一下网页按钮要等半天"的来源。
    改成向量化之后同样的数据在 2ms 以内。

    对外行为保持不变:同一帧同一格保留**第一个**采样,跨帧后来的覆盖先前的,
    超出 limit 时淘汰最久没被观测到的。
    """

    def __init__(self, voxel=.05, limit=60000, ttl=45.):
        if not .02 <= voxel <= .5 or not 100 <= limit <= 120000:
            raise ValueError('invalid display budget')
        if ttl != 0 and not 1 <= ttl <= 120:
            raise ValueError('invalid display budget')
        self.voxel, self.limit, self.ttl = voxel, int(limit), float(ttl)
        self._keys = np.empty(0, dtype=np.int64)
        self._data = np.empty((0, 4), dtype=np.float32)
        self._seen = np.empty(0, dtype=np.float64)
        self._rank = np.empty(0, dtype=np.int64)
        self._tick = 0
        self.dropped_far = 0
        self.dirty = False

    # -- 兼容旧接口:仍然可以问"现在有多少格" --
    def __len__(self):
        return int(self._keys.shape[0])

    @property
    def cells(self):
        """只读视图,给诊断脚本用。不要拿它当可写容器。"""
        return {int(k): (float(t), d) for k, t, d in
                zip(self._keys, self._seen, self._data)}

    def clear(self):
        self._keys = np.empty(0, dtype=np.int64)
        self._data = np.empty((0, 4), dtype=np.float32)
        self._seen = np.empty(0, dtype=np.float64)
        self._rank = np.empty(0, dtype=np.int64)
        self.dirty = True

    def _pack(self, xyz):
        idx = np.floor(np.asarray(xyz, dtype=np.float64) / self.voxel).astype(np.int64)
        ok = np.all(np.abs(idx) < KEY_LIMIT, axis=1)
        packed = (((idx[:, 0] + KEY_LIMIT) << 42) |
                  ((idx[:, 1] + KEY_LIMIT) << 21) |
                  (idx[:, 2] + KEY_LIMIT))
        return packed, ok

    def expire(self, now):
        if self.ttl <= 0 or not len(self._keys):
            return
        keep = self._seen >= now - self.ttl
        if keep.all():
            return
        self._keys, self._data = self._keys[keep], self._data[keep]
        self._seen, self._rank = self._seen[keep], self._rank[keep]
        self.dirty = True

    def add(self, points, now):
        self.expire(now)
        a = np.asarray(points, dtype=np.float32).reshape(-1, 4)
        a = a[np.isfinite(a[:, :3]).all(axis=1) & (np.abs(a[:, :3]) < 100000).all(axis=1)]
        if not len(a):
            return
        keys, ok = self._pack(a[:, :3])
        self.dropped_far += int((~ok).sum())
        keys, a = keys[ok], a[ok]
        if not len(keys):
            return

        # 同一帧同一格只留第一个采样(np.unique 返回首次出现的下标,按键升序)
        keys, first = np.unique(keys, return_index=True)
        a = a[first]

        slots = np.searchsorted(self._keys, keys)
        hit = np.zeros(keys.shape[0], dtype=bool)
        if self._keys.shape[0]:
            inside = slots < self._keys.shape[0]
            hit[inside] = self._keys[slots[inside]] == keys[inside]

        ranks = self._tick + np.arange(1, keys.shape[0] + 1, dtype=np.int64)
        self._tick += keys.shape[0]

        if hit.any():                      # 已有格子:更新采样、时间与 LRU 次序
            where = slots[hit]
            self._data[where] = a[hit]
            self._seen[where] = now
            self._rank[where] = ranks[hit]

        fresh = ~hit
        if fresh.any():
            self._keys = np.concatenate((self._keys, keys[fresh]))
            self._data = np.concatenate((self._data, a[fresh]))
            self._seen = np.concatenate((self._seen, np.full(int(fresh.sum()), now)))
            self._rank = np.concatenate((self._rank, ranks[fresh]))
            order = np.argsort(self._keys, kind='stable')
            self._keys, self._data = self._keys[order], self._data[order]
            self._seen, self._rank = self._seen[order], self._rank[order]

        self._evict()
        self.dirty = True

    def _evict(self):
        n = self._keys.shape[0]
        if n <= self.limit:
            return
        keep = np.argpartition(self._rank, n - self.limit)[n - self.limit:]
        keep.sort()
        self._keys, self._data = self._keys[keep], self._data[keep]
        self._seen, self._rank = self._seen[keep], self._rank[keep]

    def array(self):
        return self._data.copy()


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
        has_rgb=bool(len(a) and (a[:,3]!=-1).any())
        self.meta=dict(epoch=self.epoch,revision=self.revision,count=len(a),frame='map',
                       stride=16,format='xyzi-f32le',intensity_available=bool(len(valid)),
                       rgb_available=has_rgb,
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


def colors(points, mode='rgb', low=-.2, high=3., robot=(0,0,0), intensity_range=None):
    a=np.asarray(points,dtype=np.float32).reshape(-1,4)
    if mode=='rgb':
        u32=np.ascontiguousarray(a[:,3]).view(np.uint32)
        r=((u32>>16)&0xFF).astype(np.uint8)
        g=((u32>>8)&0xFF).astype(np.uint8)
        b=(u32&0xFF).astype(np.uint8)
        is_missing=np.isnan(a[:,3])|(u32==0)|(a[:,3]==-1.0)|(a[:,3]<0)
        if is_missing.all():
            v=a[:,2]
            t=np.clip((v-low)/max(high-low,1e-6),0,1)*5
            i=np.minimum(t.astype(int),4)
            return ((RAMP[i]*(1-(t-i))[:,None]+RAMP[i+1]*(t-i)[:,None])*255).round().astype(np.uint8)
        out=np.column_stack((r,g,b))
        if np.any(is_missing):
            out[is_missing]=[180,180,180]
        return out
    elif mode=='intensity' and intensity_range:
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
           elevation=.9, color_mode='rgb', z_low=-.2, z_high=3., point_size=2,
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
