"""Pure regression tests; these do not certify ROS integration or vehicle safety."""
# Historical optional feature: keep tests, but do not fail collection after removal.
from pathlib import Path as _FeaturePath
import pytest as _feature_pytest
if not (_FeaturePath(__file__).resolve().parents[1] / 'radar_system' / 'live_map_core.py').exists():
    _feature_pytest.skip('retired feature: live_map_core.py is not shipped', allow_module_level=True)

import math
import struct
import sys
import zlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_system'))
from live_map_core import (encode_grid,transform_points,grid_to_world,following_point,
                           optical_to_base,decode_depth,yaw)
from mapping_session import MappingSession


def decode_png_gray(png):
    assert png[:8]==b'\x89PNG\r\n\x1a\n'
    pos=8
    payload=b''
    while pos<len(png):
        n=struct.unpack('!I',png[pos:pos+4])[0]
        kind=png[pos+4:pos+8]
        data=png[pos+8:pos+8+n]
        if kind==b'IHDR':
            w,h=struct.unpack('!II',data[:8])
        elif kind==b'IDAT':
            payload+=data
        pos+=n+12
    arr=np.frombuffer(zlib.decompress(payload),np.uint8).reshape(h,w+1)
    assert np.all(arr[:,0]==0)
    return arr[:,1:]


def test_unknown_free_occupied_and_row_order():
    png,meta=encode_grid([-1,0,100,50],2,2,.05)
    assert decode_png_gray(png).tolist()==[[196,248],[44,196]]
    assert meta['occupied_cells']==1


def test_pooling_never_drops_thin_wall():
    grid=np.zeros((16,16),dtype=np.int8)
    grid[:,7]=100
    png,meta=encode_grid(grid.ravel(),16,16,.05,max_side=4)
    a=decode_png_gray(png)
    assert (a[:,1]==44).all()
    assert meta['step']==4


def test_partial_unknown_block_not_whitened():
    png,_=encode_grid([0,0,0,-1],2,2,.1,max_side=1)
    assert decode_png_gray(png)[0,0]==196


@pytest.mark.parametrize('args', [([],0,0,.1),([0],2,2,.1),([101],1,1,.1),([-2],1,1,.1),([0],1,1,0),([0],1,1,float('nan'))])
def test_reject_bad_grids(args):
    with pytest.raises(ValueError):
        encode_grid(*args)


def test_rotated_map_origin():
    x,y=grid_to_world(2,1,.5,10.,20.,math.pi/2)
    assert (x,y)==pytest.approx((9.5,21.))


def test_transform_rotation_translation_and_normalization():
    result=transform_points([[1,0,0]],(10,20,1),(0,0,2**.5,2**.5))
    assert result[0]==pytest.approx((10,21,1))


def test_invalid_quaternion():
    with pytest.raises(ValueError):
        transform_points([[0,0,0]],(0,0,0),(0,0,0,0))


def test_camera_axes_and_pitch():
    assert optical_to_base(1,0,2,camera_x=.54,pitch=0)==pytest.approx((2.54,-1))
    assert optical_to_base(0,1,2,pitch=math.pi/6)[0]==pytest.approx(.54+math.sqrt(3)-.5)


def test_goal_accounts_for_front_overhang():
    g=following_point((0,0,0),(4,0),clearance=1.2,front=.67)
    assert g==pytest.approx((2.13,0,0))
    assert 4-(g[0]+.67)==pytest.approx(1.2)


def test_near_person_holds_no_reverse():
    assert following_point((0,0,0),(1,0))==pytest.approx((0,0,0))


def test_depth_stride_and_endianness():
    msg=SimpleNamespace(encoding='16UC1',is_bigendian=True,width=2,height=2,step=6,
                        data=bytes.fromhex('03e8 07d0 ffff 0bb8 0fa0 ffff'))
    assert decode_depth(msg).ravel()==pytest.approx([1,2,3,4])


def test_depth_float_metres():
    msg=SimpleNamespace(encoding='32FC1',is_bigendian=False,width=2,height=1,step=8,
                        data=np.array([1.25,2.5],dtype='<f4').tobytes())
    assert decode_depth(msg).ravel()==pytest.approx([1.25,2.5])


def test_depth_reject_bad_stride():
    with pytest.raises(ValueError):
        decode_depth(SimpleNamespace(encoding='16UC1',is_bigendian=False,width=2,height=1,step=2,data=b'00'))


@pytest.mark.parametrize('name',['../x','/tmp/map','map;touch_pwn','',None,'a'*65,'hello.yaml'])
def test_map_name_validation(tmp_path,monkeypatch,name):
    monkeypatch.setenv('RO2_MAP_DIR',str(tmp_path))
    session=MappingSession(tmp_path)
    with pytest.raises(ValueError):
        session.name(name)


def test_map_save_uses_full_map_cli_and_no_shell(tmp_path,monkeypatch):
    monkeypatch.setenv('RO2_MAP_DIR',str(tmp_path))
    session=MappingSession(tmp_path)
    def run(command,**kw):
        assert command[:3]==['ros2','run','nav2_map_server']
        assert 'shell' not in kw
        assert '-f' in command
        (tmp_path/'office_01.yaml').write_text('image: office_01.pgm')
        return SimpleNamespace(returncode=0,stdout='',stderr='')
    with patch('mapping_session.subprocess.run',side_effect=run):
        assert session.save('office_01')['ok']
    with pytest.raises(ValueError):
        session.save('office_01')


def test_symlink_not_read_or_overwritten(tmp_path,monkeypatch):
    monkeypatch.setenv('RO2_MAP_DIR',str(tmp_path))
    (tmp_path/'bad.yaml').symlink_to(tmp_path/'other.yaml')
    session=MappingSession(tmp_path)
    assert 'bad' not in session.list_maps()
    with pytest.raises(ValueError):
        session.map_file('bad')


def test_duplicate_slam_not_started(tmp_path,monkeypatch):
    monkeypatch.setenv('RO2_MAP_DIR',str(tmp_path))
    session=MappingSession(tmp_path)
    with patch('mapping_session.subprocess.run',return_value=SimpleNamespace(stdout='/slam_toolbox\n')):
        with pytest.raises(ValueError,match='已有外部'):
            session.start('mapping')


def test_invalid_session_mode(tmp_path,monkeypatch):
    monkeypatch.setenv('RO2_MAP_DIR',str(tmp_path))
    with pytest.raises(ValueError):
        MappingSession(tmp_path).start('drive')


def test_self_hit_filter_does_not_remove_near_outside_obstacle():
    from live_map_core import body_self_hit_mask
    a=body_self_hit_mask([[.6,0,0],[.68,0,0],[0,.34,0],[-.19,0,0]])
    assert a.tolist()==[True,False,False,False]


@pytest.mark.parametrize('stamp,expected',[(0,False),(9.9,True),(9.,False),(10.5,False),(float('nan'),False)])
def test_reject_stale_or_future_sensor_stamp(stamp,expected):
    from live_map_core import recent_stamp
    assert recent_stamp(10.,stamp)==expected
