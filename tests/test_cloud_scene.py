import gzip
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_system'))
from cloud_scene import (read_xyzi,depth_xyzi,apply_matrix,VoxelHistory,ScenePackets,
                         pose_jump,project,colors,raster)


def make_cloud(endian=False):
    dtype=np.dtype('>f4' if endian else '<f4')
    rows=[]
    for i in range(2):
        rows.append(np.array([[1+i,2,3,11],[4+i,5,6,22]],dtype=dtype).tobytes()+b'padding!')
    return NS(width=2,height=2,point_step=16,row_step=40,data=b''.join(rows),is_bigendian=endian,
              fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))])


@pytest.mark.parametrize('endian',[False,True])
def test_read_stride_byte_order(endian):
    p=read_xyzi(make_cloud(endian))
    assert p.tolist()==[[1,2,3,11],[4,5,6,22],[2,2,3,11],[5,5,6,22]]


def test_missing_intensity_not_invented():
    c=make_cloud();c.fields=c.fields[:3]
    assert np.all(read_xyzi(c)[:,3]==-1)


def test_intensity_integer_and_nonstandard_offset():
    c=make_cloud();c.fields[-1].datatype=6
    a=read_xyzi(c)
    assert a[0,3]>1e8  # uint bit pattern, not interpreted as the original float


def test_nan_xyz_rejected():
    c=make_cloud();data=bytearray(c.data);data[:4]=np.float32(np.nan).tobytes();c.data=bytes(data)
    assert len(read_xyzi(c))==3


@pytest.mark.parametrize('change',[{'row_step':8},{'data':b''},{'point_step':2},{'width':2000001}])
def test_invalid_cloud_shape(change):
    c=make_cloud();vars(c).update(change)
    with pytest.raises(ValueError):read_xyzi(c)


def test_invalid_field():
    c=make_cloud();c.fields[0].count=3
    with pytest.raises(ValueError):read_xyzi(c)
    c=make_cloud();c.fields[0].offset=19
    with pytest.raises(ValueError):read_xyzi(c)


def test_empty_cloud():
    c=make_cloud();c.width=0
    assert read_xyzi(c).shape==(0,4)


def test_cloud_limit():
    assert len(read_xyzi(make_cloud(),limit=2))==2


def make_depth():
    header=NS(frame_id='camera_optical')
    depth=NS(width=4,height=4,step=12,encoding='16UC1',is_bigendian=True,header=header,
             data=b''.join(np.full(4,1000,dtype='>u2').tobytes()+b'pad!' for _ in range(4)))
    info=NS(width=4,height=4,header=header,k=[100.,0.,0.,0.,100.,0.,0.,0.,1.])
    return depth,info


def test_registered_depth_metric():
    d,i=make_depth();p=depth_xyzi(d,i,step=2)
    assert p[:,2]==pytest.approx([1,1,1,1])
    assert p[:,0]==pytest.approx([0,.02,0,.02])
    assert p[:,1]==pytest.approx([0,0,.02,.02])
    assert np.all(p[:,3]==-1)


def test_float_depth_metres():
    d,i=make_depth();d.encoding='32FC1';d.is_bigendian=False;d.step=16;d.data=np.full((4,4),1.5,dtype='<f4').tobytes()
    assert depth_xyzi(d,i)[:,2]==pytest.approx([1.5])


@pytest.mark.parametrize('kind',['frame','intrinsics','size','missing','stride','encoding'])
def test_bad_depth(kind):
    d,i=make_depth()
    if kind=='frame':i.header=NS(frame_id='another')
    if kind=='intrinsics':i.k[0]=0
    if kind=='size':i.width=8
    if kind=='missing':i=None
    if kind=='stride':d.step=2
    if kind=='encoding':d.encoding='rgb8'
    with pytest.raises(ValueError):depth_xyzi(d,i)


def test_transform_retains_intensity_and_true_height():
    p=apply_matrix([[1,0,.7,123]],(10,20,1),(0,0,2**.5,2**.5))
    assert p[0]==pytest.approx([10,21,1.7,123])


def test_invalid_transform():
    with pytest.raises(ValueError):apply_matrix([[0,0,0,-1]],(0,0,0),(0,0,0,0))


def test_voxel_last_actual_sample_not_cell_center():
    h=VoxelHistory();h.add([[.012,.014,.017,-1],[.018,.017,.018,-1]],1)
    assert len(h.array())==1
    assert h.array()[0]==pytest.approx([.012,.014,.017,-1])
    h.add([[.019,.015,.022,-1]],2)
    assert h.array()[0]==pytest.approx([.019,.015,.022,-1])


def test_bound_ttl_and_eviction():
    h=VoxelHistory(limit=100,ttl=3)
    h.add([[i,0,0,-1] for i in range(200)],1)
    assert len(h.array())==100
    h.expire(5);assert h.array().shape==(0,4)


def test_empty_update_still_expires():
    h=VoxelHistory(ttl=1);h.add([[0,0,0,-1]],1);h.add([],3)
    assert not len(h.array())


def test_binary_epoch_gzip_revision():
    s=ScenePackets();a=np.array([[1,2,3,-1],[4,5,6,20]],'<f4');s.update(a)
    meta=dict(s.meta);p=s.get(meta['epoch'],meta['revision'])
    assert p==a.tobytes()
    assert gzip.decompress(s.get(meta['epoch'],meta['revision'],True))==p
    assert s.meta['intensity_available'] is True
    s.update(a);assert s.revision==meta['revision']
    s.reset();assert s.get(meta['epoch'],meta['revision']) is None
    assert s.meta['count']==0


def test_binary_reject_nan():
    with pytest.raises(ValueError):ScenePackets().update([[float('nan'),0,0,-1]])


def test_packet_eviction():
    s=ScenePackets();epoch,rev=s.epoch,s.revision
    for i in range(4):s.update([[i,0,0,-1]])
    assert s.get(epoch,rev) is None


def test_no_intensity_flag_from_xyz():
    s=ScenePackets();s.update([[0,0,0,-1],[1,1,2,-1]])
    assert not s.meta['intensity_available']


@pytest.mark.parametrize('p,expected',[((.01,0,0),False),((.3,0,0),True),((0,0,.2),True),((0,0,2*math.pi),False)])
def test_localization_jump(p,expected):
    assert pose_jump((0,0,0),p)==expected


def test_perspective_depth_and_axes():
    p=project([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],(0,0,0),10,-math.pi/2,math.pi/2,800,600)
    assert p[0,:2]==pytest.approx([400,300])
    assert p[1,0]>400 and p[2,1]<300 and p[3,2]<10


def test_shared_palette_endpoints():
    c=colors([[0,0,-.2,-1],[0,0,3,-1]])
    assert c.tolist()==[[163,46,217],[255,79,43]]


def test_raster_nonempty_and_z_filter():
    a=np.array([[0,0,0,-1]],np.float32)
    image=raster(a,width=200,height=120,target=(0,0,0),point_size=3)
    assert ((image[:,:,:3]!=[15,22,32]).any(axis=2)).sum()==9
    empty=raster(a,width=200,height=120,z_low=1,z_high=2)
    assert (empty[:,:,:3]==[15,22,32]).all()


def test_nearest_depth_wins_not_array_order():
    a=np.array([[0,0,0,-1],[0,0,1,-1]],np.float32)
    kw=dict(width=200,height=100,target=(0,0,0),azimuth=0,elevation=math.pi/2,point_size=1)
    assert np.array_equal(raster(a,**kw),raster(a[::-1],**kw))


def test_raster_size_bound():
    im=raster([],width=4000,height=4000)
    assert im.shape==(1200,1600,4)
