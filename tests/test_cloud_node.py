"""Logic tests with explicit ROS/TF stubs; not ROS executor or hardware integration."""
# Historical optional feature: keep tests, but do not fail collection after removal.
from pathlib import Path as _FeaturePath
import pytest as _feature_pytest
if not (_FeaturePath(__file__).resolve().parents[1] / 'radar_system' / 'live_cloud_node.py').exists():
    _feature_pytest.skip('retired feature: live_cloud_node.py is not shipped', allow_module_level=True)

import importlib.util
import sys
import threading
import time
from pathlib import Path
from types import ModuleType,SimpleNamespace as NS
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'radar_system'))
ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def node(monkeypatch):
    monkeypatch.delenv('RO2_CLOUD_SOURCE',raising=False)
    monkeypatch.delenv('SENSOR_TF_CALIBRATED',raising=False)
    class TransformError(Exception):pass
    class Time:
        def __init__(self,seconds=0):self.seconds=seconds
        @classmethod
        def from_msg(cls,stamp):return cls(stamp.sec+stamp.nanosec*1e-9)
    class TF:
        def __init__(self):self.calls=[];self.x=0;self.fail=False
        def lookup_transform(self,to,source,stamp):
            self.calls.append((to,source,stamp.seconds))
            if self.fail:raise TransformError('missing TF')
            return NS(transform=NS(translation=NS(x=self.x,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    class Base:
        def __init__(self):
            self.lock=threading.RLock();self.params={};self.ros_now=100.;self.tf=TF()
            self.map_info={'frame':'map'};self.map_fault='';self.robot=[0,0,0];self.pose_at=time.monotonic()
            self.cloud=[];self.cloud_revision=0;self.pending_cloud=None
        def declare_parameter(self,k,v):self.params[k]=v
        def get_parameter(self,k):return NS(value=self.params[k])
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=int(self.ros_now*1e9)))
        def create_subscription(self,*args):return None
        def create_timer(self,*args):return None
        def tick(self):self.pose_at=time.monotonic()
        def snapshot(self):return dict(localized=self.robot is not None)
        def reset_display(self):self.map_info=None;self.robot=None
        def initial_pose(self,*args):pass
    modules={
        'rclpy':{},'rclpy.time':{'Time':Time},'rclpy.qos':{'qos_profile_sensor_data':object()},
        'tf2_ros':{'TransformException':TransformError},'sensor_msgs':{},
        'sensor_msgs.msg':{'Image':type('Image',(),{}),'CameraInfo':type('CameraInfo',(),{}),'PointCloud2':type('PointCloud2',(),{})},
        'live_map_node':{'LiveMapNode':Base,'stamp_s':lambda s:s.sec+s.nanosec*1e-9},
    }
    for name,attrs in modules.items():
        m=ModuleType(name);vars(m).update(attrs);monkeypatch.setitem(sys.modules,name,m)
    spec=importlib.util.spec_from_file_location('_test_live_cloud_node',ROOT/'radar_system/live_cloud_node.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    n=mod.LiveCloudNode()
    header=NS(stamp=NS(sec=99,nanosec=900000000),frame_id='camera_optical')
    n.latest_depth=NS(header=header,width=12,height=12,step=24,encoding='16UC1',is_bigendian=False,
                      data=np.full((12,12),1000,dtype='<u2').tobytes())
    n.latest_info=NS(header=header,width=12,height=12,k=[100,0,0,0,100,0,0,0,1])
    return n


def test_no_calibration_no_points(node):
    node.tick();node.publish_scene()
    assert node.snapshot()['scene']['count']==0
    assert '外参未确认' in node.cloud_error


def test_depth_transformed_at_acquisition_stamp(node):
    node.params['sensor_tf_calibrated']=True
    node.tick();node.publish_scene()
    assert node.snapshot()['scene']['count']==4
    assert ('map','camera_optical',99.9) in node.tf.calls
    assert node.snapshot()['scene']['live']


def test_repeated_source_stamp_not_marked_new(node):
    node.params['sensor_tf_calibrated']=True;node.tick();node.publish_scene()
    at,rev=node.observed_at,node.packets.revision
    node.tick();node.publish_scene()
    assert node.observed_at==at and node.packets.revision==rev


def test_stale_source_not_accumulated(node):
    node.params['sensor_tf_calibrated']=True;node.ros_now=102;node.tick();node.publish_scene()
    assert node.packets.meta['count']==0 and '过期' in node.cloud_error


def test_tf_failure_does_not_fabricate_pose(node):
    node.params['sensor_tf_calibrated']=True;node.tf.fail=True;node.tick()
    assert node.observed_at is None


def test_loop_correction_clears_history(node):
    node.params['sensor_tf_calibrated']=True;node.tick();node.publish_scene();epoch=node.packets.epoch
    node.tf.x=.4;node.tick();node.publish_scene()
    assert node.packets.epoch!=epoch and node.packets.meta['count']==0
    assert '回环' in node.reset_reason


def test_reset_between_transform_and_insert_rejects_old_frame(node):
    node.params['sensor_tf_calibrated']=True
    transform=node._transform
    def race(*a):
        p=transform(*a);node.reset_display();return p
    node._transform=race;node.tick();node.publish_scene()
    assert node.packets.meta['count']==0


def test_bad_map_disables_live_scene(node):
    node.params['sensor_tf_calibrated']=True;node.tick();node.map_fault='duplicate map';node.tick()
    assert not node.snapshot()['scene']['live']
    assert node.robot is None


def test_octomap_empty_snapshot_clears(node):
    node.source='octomap';node.packets.update([[1,2,3,-1]])
    node.pending_cloud=NS(markers=[]);node.encode_cloud()
    assert node.packets.meta['count']==0


def test_clock_rollback_resets_epoch(node):
    node.params['sensor_tf_calibrated']=True;node.tick();epoch=node.packets.epoch
    node.ros_now=10;node.tick()
    assert node.packets.epoch!=epoch


def test_base_link_fallback_when_no_map(node):
    node.params['sensor_tf_calibrated']=True
    node.map_info=None
    node.tick();node.publish_scene()
    snap=node.snapshot()
    assert snap['scene']['count']==4
    assert snap['scene']['frame']=='base_link'
    assert snap['scene']['live']
    assert ('base_link','camera_optical',99.9) in node.tf.calls
