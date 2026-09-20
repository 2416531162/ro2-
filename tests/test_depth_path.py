"""Depth can certify missing lidar coverage only from calibrated fresh pixels."""
from pathlib import Path
import json
import math
import sys
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'radar_system'))
from depth_path import DepthCalibration, DepthIntrinsics, DepthEvidence, DepthPathSensor
from follower_recovery import LocalRecovery, ScanEvidence
from footprint import SensorMount, VehicleFootprint
from motion_safety import ChassisGeometry, BrakeProfile


def calibration(**kw):
    return DepthCalibration(**dict(dict(optical_frame='camera_rgb_optical_frame', x_m=0., y_m=0.,
        yaw_rad=0., pitch_rad=0., lidar_minus_camera_height_m=0., min_depth_m=.35, max_depth_m=2.5), **kw))


def intrinsics(stamp=10.):
    return DepthIntrinsics.from_info(width=160, height=120, frame='camera_rgb_optical_frame',
        stamp=stamp, k=[100.,0,79.5,0,100.,59.5,0,0,1], d=[0.]*5,
        r=np.eye(3).ravel(), p=[100.,0,79.5,0,0,100.,59.5,0,0,0,1,0],binning=(0,0),roi=(0,0,0,0,False))


def frame(value=2., cal=None, pose=(0.,0.,0.)):
    data = np.full((120,160), value, dtype=np.float32) if np.isscalar(value) else value
    return DepthEvidence(data, intrinsics(), cal or calibration(), 10., pose)


def scan(gap=True, obstacle=False):
    ranges=[5.]*720
    if gap:
        for i in range(720):
            a=(i*.5+180)%360-180
            if abs(a)<=9:ranges[i]=math.inf
    if obstacle:ranges[0]=.85
    return ScanEvidence(ranges, 0.,math.pi/360,.15,12.,SensorMount(),VehicleFootprint())


def planner():
    return LocalRecovery(VehicleFootprint(),ChassisGeometry(),BrakeProfile(stop_m=.12,hard_stop_m=.06))


def test_camera_confirms_lidar_hole_in_view_and_ahead_of_min_range():
    p=planner();e=scan()
    assert p.clearance(e,0.,allow_memory=False)<.1
    p.depth_evidence=frame().at_pose((0.,0.,0.))
    assert p.clearance(e,0.,allow_memory=False)>.8
    assert p.last_depth_used


@pytest.mark.parametrize('value',[0.,math.nan,math.inf,-1.,.1,8.])
def test_invalid_depth_is_not_free(value):
    p=planner();p.depth_evidence=frame(value).at_pose((0.,0.,0.))
    assert p.clearance(scan(),0.,allow_memory=False)<.1


def test_camera_does_not_override_lidar_obstacle():
    p=planner();p.depth_evidence=frame().at_pose((0.,0.,0.))
    assert p.clearance(scan(obstacle=True),0.,allow_memory=False)<.25
    assert p.last_block[0]=='obstacle'


def test_depth_obstacle_overrides_lidar_and_historical_free_space():
    p=planner();p.scan_history=[(10.,0.,0.,0.,scan(False))]
    p.depth_evidence=frame(.85).at_pose((0.,0.,0.))
    assert p.clearance(scan(False),0.)<.25
    assert p.last_block[0]=='obstacle'


def test_hole_in_depth_patch_stays_unknown():
    data=np.full((120,160),2.,dtype=np.float32)
    data[59,79]=0.
    free,blocked=frame(data).query_many(np.array([1.]),np.array([0.]))
    assert not free[0] and not blocked[0]


def test_closer_pixel_vetoes_even_with_hole_in_same_patch():
    data=np.full((120,160),2.,dtype=np.float32)
    data[59,79]=0.;data[60,80]=.7
    free,blocked=frame(data).query_many(np.array([1.]),np.array([0.]))
    assert not free[0] and blocked[0]


def test_camera_near_blind_zone_and_outside_fov_not_certified():
    free,_=frame().query_many(np.array([.18,1.,-1.]),np.array([0.,2.,0.]))
    assert not free.any()


def test_calibrated_height_and_pitch_change_visible_plane():
    assert frame().query_many(np.array([1.]),np.array([0.]))[0][0]
    assert not frame(cal=calibration(lidar_minus_camera_height_m=1.)).query_many(
        np.array([1.]),np.array([0.]))[0][0]
    # Place the test point at the optical axis for the pitched camera.
    cal=calibration(pitch_rad=.3,lidar_minus_camera_height_m=-math.tan(.3))
    assert frame(cal=cal).query_many(np.array([1.]),np.array([0.]))[0][0]


def test_capture_pose_compensates_vehicle_translation_and_rotation():
    ev=frame(pose=(3.,4.,math.pi/2))
    original=ev.query_many(np.array([1.]),np.array([0.]))
    # Current vehicle advanced 0.2 m along its +X at yaw pi/2.
    transformed=ev.at_pose((3.,4.2,math.pi/2)).query_many(np.array([.8]),np.array([0.]))
    np.testing.assert_array_equal(original,transformed)
    # Same world point after current vehicle rotates another 90 degrees.
    transformed=ev.at_pose((3.,4.,math.pi)).query_many(np.array([0.]),np.array([-1.]))
    np.testing.assert_array_equal(original,transformed)


def test_unknown_rear_is_not_certified_by_forward_camera():
    p=planner();p.depth_evidence=frame().at_pose((0.,0.,0.))
    ranges=[5.]*720
    for i in range(300,421):ranges[i]=math.inf
    ev=ScanEvidence(ranges,0.,math.pi/360,.15,12.,SensorMount(),VehicleFootprint())
    assert p.clearance(ev,0.,direction=-1,allow_memory=False)<.1
    assert not p.last_depth_used


def test_stale_depth_cannot_be_reused():
    s=DepthPathSensor();s.calibration=calibration();s.intrinsics=intrinsics()
    s.observe(np.full((120,160),2.),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    assert s.view(10.1,(0.,0.,0.)) is not None
    assert s.view(10.21,(0.,0.,0.)) is None
    assert s.status(10.21)['reason']=='depth_stale'
    assert s.view(9.9,(0.,0.,0.)) is None


@pytest.mark.parametrize('frame_id,stamp,pose', [('wrong',10.,(0.,0.,0.)),
    ('camera_rgb_optical_frame',10.15,(0.,0.,0.)),('camera_rgb_optical_frame',10.,None)])
def test_intrinsics_frame_timestamp_or_missing_pose_invalidates(frame_id,stamp,pose):
    s=DepthPathSensor();s.calibration=calibration();s.intrinsics=intrinsics()
    s.observe(np.full((120,160),2.),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    s.observe(np.full((120,160),2.),10.1,pose,frame_id,stamp,10.1)
    assert s.view(10.1,(0.,0.,0.)) is None


def test_invalid_frame_replaces_previous_free_frame():
    s=DepthPathSensor();s.calibration=calibration();s.intrinsics=intrinsics()
    s.observe(np.full((120,160),2.),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    s.observe(np.zeros((120,160)),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    assert not s.view(10.,(0.,0.,0.)).query_many(np.array([1.]),np.array([0.]))[0][0]


def test_no_calibration_never_confirms_free_space(tmp_path):
    s=DepthPathSensor();s.load_calibration(tmp_path/'missing.json','abc')
    s.intrinsics=intrinsics()
    s.observe(np.full((120,160),2.),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    assert s.view(10.,(0.,0.,0.)) is None
    assert s.reason=='calibration_missing'


def test_calibration_requires_matching_profile_and_height(tmp_path):
    path=tmp_path/'cal.json';data=dict(calibration().__dict__,schema_version=1,profile_hash='abc')
    path.write_text(json.dumps(data));assert DepthCalibration.load(path,'abc').x_m==0
    with pytest.raises(ValueError):DepthCalibration.load(path,'other')
    data['lidar_minus_camera_height_m']=None;path.write_text(json.dumps(data))
    with pytest.raises(ValueError):DepthCalibration.load(path,'abc')


def test_intrinsics_reject_distortion_and_cropped_projection():
    args=dict(width=160,height=120,frame='camera',stamp=10.,k=[100.,0,79.5,0,100.,59.5,0,0,1],
        d=[.1,0,0,0,0],r=np.eye(3).ravel(),p=[100.,0,79.5,0,0,100.,59.5,0,0,0,1,0],
        binning=(0,0),roi=(0,0,0,0,False))
    with pytest.raises(ValueError):DepthIntrinsics.from_info(**args)
    args['d']=[0.]*5;args['roi']=(10,0,0,0,False)
    with pytest.raises(ValueError):DepthIntrinsics.from_info(**args)


def test_control_caps_camera_fallback_and_stops_after_depth_dropout():
    from follower_engine import FollowerEngine
    from follower_config import FollowerConfig
    cfg=FollowerConfig();cfg.recovery.enabled=False;cfg.follow_breadcrumbs=False
    clock=[10.]
    e=FollowerEngine(cfg,now=lambda:clock[0],ros_time=lambda:clock[0],dry_run=True,
                     simulated_odometry=True)
    e.print_dashboard=lambda _:None
    e.people.add_camera([dict(x=2.2,y=0.,conf=.9)],10.,10.)
    e.people.confirm_hits=1
    e.people.add_camera([dict(x=2.2,y=0.,conf=.9)],10.,10.)
    e.scan_evidence=scan();e.scan_points=list(e.scan_evidence.points)
    e.min_front_scan=5.
    e.depth_path.calibration=calibration()
    commands=[]
    for i in range(20):
        clock[0]=10.+i*.05
        now=clock[0]
        e.depth_path.intrinsics=intrinsics(now)
        e.depth_path.observe(np.full((120,160),2.),now,(0.,0.,0.),
                             'camera_rgb_optical_frame',now,now)
        e.feedback_stamp=e.scan_stamp=now;e.feedback_healthy=True
        e.people.add_camera([dict(x=2.2,y=0.,conf=.9)],now,now)
        e.step();commands.append(e.cmd_vx)
        assert e.recovery.last_depth_used
        assert 0 <= e.cmd_vx <= .12
    assert max(commands)>.05
    for i in range(1,8):
        clock[0]=10.95+i*.05;now=clock[0]
        e.feedback_stamp=e.scan_stamp=now
        e.people.add_camera([dict(x=2.2,y=0.,conf=.9)],now,now)
        e.step()
    assert e.cmd_vx==0.
    assert e.state=='OBSERVATION_WAIT'
    assert e.depth_path.status(clock[0])['reason']=='depth_stale'


def test_ros_adapter_depth_and_camera_info_contract():
    import ros_stubs
    ros_stubs.install()
    from person_follower import PersonFollowerNode, FollowerConfig
    node=PersonFollowerNode(FollowerConfig(),dry_run=True,simulated_odometry=True)
    node.engine.now=lambda:10.;node.engine.ros_time=lambda:10.
    node.depth_path.calibration=calibration()
    header=SimpleNamespace(frame_id='camera_rgb_optical_frame',stamp=SimpleNamespace(sec=10,nanosec=0))
    msg=SimpleNamespace(header=header,width=160,height=120,k=[100.,0,79.5,0,100.,59.5,0,0,1],
        d=[0.]*5,r=list(np.eye(3).ravel()),p=[100.,0,79.5,0,0,100.,59.5,0,0,0,1,0],
        binning_x=0,binning_y=0,roi=SimpleNamespace(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))
    node.on_depth_info(msg)
    image=SimpleNamespace(header=header,width=160,height=120,step=320,encoding='16UC1',
        is_bigendian=0,data=np.full((120,160),2000,dtype='<u2').tobytes())
    node.on_depth_image(image)
    assert node.depth_path.status(10.)['reason']=='ready'
    assert node.depth_path.view(10.,(0.,0.,0.)).query_many(np.array([1.]),np.array([0.]))[0][0]
    image.data=b'bad'
    node.on_depth_image(image)
    assert node.depth_path.view(10.,(0.,0.,0.)) is None


def test_control_rejects_depth_that_expires_during_computation():
    from follower_engine import FollowerEngine
    from follower_config import FollowerConfig
    cfg=FollowerConfig();cfg.recovery.enabled=False
    clock=[10.]
    e=FollowerEngine(cfg,now=lambda:clock[0],ros_time=lambda:clock[0],dry_run=True,
                     simulated_odometry=True)
    e.print_dashboard=lambda _:None
    e.people.confirm_hits=1
    e.people.add_camera([dict(x=2.2,y=0.,conf=.9)],10.,10.)
    e.feedback_stamp=e.scan_stamp=10.;e.feedback_healthy=True
    e.scan_evidence=scan();e.scan_points=list(e.scan_evidence.points);e.min_front_scan=5.
    e.depth_path.calibration=calibration();e.depth_path.intrinsics=intrinsics()
    e.depth_path.observe(np.full((120,160),2.),10.,(0.,0.,0.),'camera_rgb_optical_frame',10.,10.)
    original=e.recovery.clearance
    def slow_clearance(*args,**kw):
        result=original(*args,**kw)
        clock[0]=10.3
        return result
    e.recovery.clearance=slow_clearance
    e.step()
    assert e.cmd_vx==0.
    assert e.limit_reason=='depth_expired_during_control'
