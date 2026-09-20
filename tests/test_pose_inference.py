"""Pose export decoding contract and geometry (no ROS/NPU required)."""
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'radar_system'))
from pose_inference import postprocess, nms, PoseRKNN
from depth_measurement import decode_depth, decode_rgb


def outputs():
    return [np.full((1,65,n,n), -20, dtype=np.float32) for n in (80,40,20)] + [
        np.zeros((1,17,3,8400), dtype=np.float32)]


def person(out, branch=0, row=30, col=30, score=5., point=(240.,240.,.9)):
    h = out[branch]
    h[0,:64,row,col] = -20
    for side in range(4):
        h[0,side*16+2,row,col] = 20
    h[0,64,row,col] = score
    index = sum(n*n for n in (80,40,20)[:branch])+row*h.shape[-1]+col
    out[3][0,:,:,index] = point


@pytest.mark.parametrize('branch,row,col', [(0,30,30),(1,15,15),(2,7,7)])
def test_each_scale_keeps_keypoint_anchor_alignment(branch,row,col):
    out = outputs()
    person(out,branch,row,col)
    result = postprocess(out,(1.,0,80,640,480))
    assert len(result)==1
    d = result[0]
    assert d['label']=='person'
    stride = 640/out[branch].shape[-1]
    assert d['box'] == list(np.rint([(col-1.5)*stride,(row-1.5)*stride-80,
                                    (col+2.5)*stride,(row+2.5)*stride-80]).astype(int))
    assert d['keypoints'][0] == pytest.approx([240,160,.9])
    assert len(d['keypoints'])==17
    assert d['conf']>.99


def test_head_reordering_and_flat_keypoints():
    out=outputs();person(out,1,10,15)
    expected=postprocess(out,(1,0,0,640,640))
    out=[out[2],out[0],out[1],out[3].reshape(1,51,8400)]
    assert postprocess(out,(1,0,0,640,640))==expected


def test_nms_keeps_winning_persons_joints_without_mutating_outputs():
    out=outputs();person(out,score=5.,point=(100,200,.8));person(out,col=31,score=3.,point=(400,500,.9))
    before=out[3].copy()
    d=postprocess(out,(.5,0,80,1280,960))
    assert len(d)==1
    assert d[0]['keypoints'][0]==pytest.approx([200,240,.8])
    np.testing.assert_array_equal(out[3],before)


def test_invalid_and_out_of_image_joints_are_hidden():
    out=outputs();person(out)
    index=30*80+30
    out[3][0,0,:,index]=[np.nan,100,.9]
    out[3][0,1,:,index]=[100,700,.9]
    points=postprocess(out,(1,0,0,640,640))[0]['keypoints']
    assert points[0]==points[1]==[0,0,0]


def test_no_people():
    assert postprocess(outputs(),(1,0,0,640,640))==[]


@pytest.mark.parametrize('out',[None,[],[np.zeros((1,84,8400))],[np.zeros((1,64,80,80))]*6])
def test_detection_only_or_other_export_is_rejected(out):
    with pytest.raises(ValueError):postprocess(out,(1,0,0,640,640))


def test_wrong_keypoint_shape_rejected():
    out=outputs();out[3]=np.zeros((1,17,2,8400))
    with pytest.raises(ValueError):postprocess(out,(1,0,0,640,640))


def test_missing_pose_weights_do_not_fall_back_to_old_detector(tmp_path):
    with pytest.raises(FileNotFoundError, match='Pose RKNN missing'):
        PoseRKNN(str(tmp_path/'missing.rknn'))


@pytest.mark.parametrize('encoding,dtype,values,expected',[
    ('16UC1','>u2',[1000,2000],[1000,2000]),
    ('32FC1','>f4',[1.,2.],[1000,2000]),
])
def test_padded_big_endian_depth(encoding,dtype,values,expected):
    data=np.array(values,dtype=dtype).tobytes()+b'\x00'*4
    msg=SimpleNamespace(encoding=encoding,is_bigendian=1,width=2,height=1,step=len(data),data=data)
    np.testing.assert_array_equal(decode_depth(msg),[expected])


def test_padded_bgr_to_rgb():
    msg=SimpleNamespace(encoding='bgr8',width=1,height=2,step=4,data=bytes([1,2,3,0,4,5,6,0]))
    assert decode_rgb(msg).tolist()==[[[3,2,1]],[[6,5,4]]]


def test_short_depth_buffer_rejected():
    msg=SimpleNamespace(encoding='16UC1',is_bigendian=0,width=2,height=1,step=4,data=b'\x00')
    with pytest.raises(ValueError):decode_depth(msg)
