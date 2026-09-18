"""End-to-end camera message publication with ROS/NPU stand-ins."""
from pathlib import Path
from unittest.mock import patch
import json
import sys
import numpy as np
import pytest
pytest.importorskip('cv2')
sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1]/'radar_system')]
import ros_stubs
ros_stubs.install()
import person_pose_node as camera


class FakePose:
    last_inference_ms=12.
    last_total_ms=14.
    def __init__(self,*args):pass


@pytest.mark.parametrize('depth_age,shape,expected',[(0.,(480,640),True),(.2,(480,640),False),
                                                    (0.,(240,320),False)])
def test_pose_message_stays_compatible_with_follower(depth_age,shape,expected):
    with patch.object(camera,'PoseRKNN',FakePose), patch.object(camera.threading.Thread,'start'):
        node=camera.PersonPoseNode()
    def inference(bgr):
        node.running=False
        return [dict(label='person',conf=.9,box=[200,100,400,400],keypoints=[[300,200,.9]]*17)]
    node.yolo.infer=inference
    rgb=ros_stubs.Image();rgb.height=480;rgb.width=640;rgb.step=1920;rgb.encoding='rgb8'
    rgb.header.stamp.sec=1000;rgb.data=np.zeros((480,640,3),np.uint8).tobytes()
    depth=ros_stubs.Image();depth.height,depth.width=shape;depth.step=shape[1]*2;depth.encoding='16UC1'
    depth.header.stamp.sec=1000;depth.header.stamp.nanosec=int(depth_age*1e9)
    depth.data=np.full(shape,1500,np.uint16).tobytes()
    node.rgb_cb(rgb);node.depth_cb(depth);node.inference_loop()
    targets=json.loads(node.pub_json.sent[-1].data)
    assert len(targets)==1 and targets[0]['label']=='person'
    assert len(targets[0]['keypoints'])==17
    assert targets[0]['range_valid']==expected
    assert targets[0]['stamp']==1000.
    assert 'bearing_rad' in targets[0]
    if expected:assert targets[0]['z']==1.5
    else:assert 'z' not in targets[0]
    status=json.loads(node.pub_status.sent[-1].data)
    assert status['depth_ok']==expected
    assert status['pair_skew_ms']==pytest.approx(depth_age*1000)
    assert status['model']=='YOLOv8n-pose' and status['classes']==1
    assert len(status['keypoint_names'])==17
    assert node.pub_annotated.sent[-1].header.stamp.sec==1000
