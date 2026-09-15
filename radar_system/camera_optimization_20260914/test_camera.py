import sys,pathlib,importlib.util,types,threading,math,ast,textwrap
import numpy as np
root=pathlib.Path(sys.argv[1]);sys.path.insert(0,str(root))
modern=(root/'camera_pipeline.py').exists()
if modern:
    import camera_pipeline as p
else:
    spec=importlib.util.spec_from_file_location('baseline_ai',root/'ai_3d_detector.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    source=(root/'ai_3d_detector.py').read_text()
    block=source[source.index('                    xc ='):source.index('                    # 绘制矩形框')]
    namespace=dict(np=np,math=math)
    exec('def baseline_range(depth,box,k):\n    h,w=depth.shape\n    x1,y1,x2,y2=box\n    self=type("K",(),dict(fx=k[0],fy=k[4],cx=k[2],cy=k[5]))()\n'+textwrap.indent(textwrap.dedent(block),'    ')+'\n    return dict(valid=dist_m is not None and .2 < dist_m < 12,z=Z,distance=dist_m)\n',namespace)
    legacy_range=namespace['baseline_range']

def msg(arr,encoding='16UC1',padding=0,big=0):
    h,w=arr.shape[:2];raw=b''.join(row.tobytes()+b'\0'*padding for row in arr)
    return types.SimpleNamespace(data=raw,width=w,height=h,step=len(raw)//h,encoding=encoding,is_bigendian=big)
def decode(m):
    if modern:return p.image_array(m)
    dummy=types.SimpleNamespace(lock=threading.Lock(),latest_depth=None,latest_rgb=None,frame_seq=0)
    m.header=None
    if m.encoding.lower() in ('rgb8','bgr8'):
        old.AI3DDetectorNode.rgb_cb(dummy,m);return dummy.latest_rgb
    old.AI3DDetectorNode.depth_cb(dummy,m)
    return dummy.latest_depth.astype(np.float32)/1000 if dummy.latest_depth is not None else None
K=[100,0,40,0,100,40,0,0,1]
def sample(depth):return p.range_target(depth,(24,24,56,56),K) if modern else legacy_range(depth*1000,(24,24,56,56),K)
checks=[]
def check(name,fn):
    try:fn();checks.append((name,True))
    except Exception as e:checks.append((name,False));print('FAIL '+name+': '+str(e))
def assert_(condition,text='contract failed'):
    if not condition:raise AssertionError(text)
def decode_test(encoding,padding,big):
    dtype=('>u2' if big else '<u2') if encoding=='16UC1' else '<f4'
    value=2000 if encoding=='16UC1' else 2
    arr=np.full((12,13),value,dtype=dtype);result=decode(msg(arr,encoding,padding,big))
    assert_(result is not None and result.shape==(12,13) and np.allclose(result,2))
check('tight_mm',lambda:decode_test('16UC1',0,0))
check('row_padding',lambda:decode_test('16UC1',6,0))
check('float_meters',lambda:decode_test('32FC1',0,0))
check('big_endian',lambda:decode_test('16UC1',0,1))
check('plane_2m',lambda:assert_(abs(sample(np.full((80,80),2,dtype=np.float32))['z']-2)<.001))
def sparse():
    a=np.zeros((80,80),np.float32);a[39,38:44]=2
    assert_(not sample(a)['valid'],'six pixels in mostly empty ROI reported a distance')
check('sparse_roi_rejected',sparse)
def mixed():
    a=np.ones((80,80),np.float32);a[:,40:]=3
    assert_(not sample(a)['valid'],'two surfaces averaged to nonexistent surface')
check('mixed_surfaces_rejected',mixed)
def invalid():
    a=np.full((80,80),65.535,np.float32)
    assert_(not sample(a)['valid'],'invalid/saturated depth accepted')
check('invalid_depth_rejected',invalid)
def gate(which):
    if not modern:raise AssertionError('baseline uses latest unpaired buffers')
    rgb=dict(array=np.zeros((80,80,3),np.uint8),frame='rgb',stamp=1.)
    depth=dict(array=np.ones((80,80),np.float32),frame='rgb',stamp=1.01)
    info=dict(w=80,h=80,frame='rgb',k=K,d=[])
    assert_(p.alignment_reason(rgb,depth,info,info) is None)
    if which=='time':depth['stamp']=2
    elif which=='frame':depth['frame']='ir'
    elif which=='calibration':info=dict(info,k=[0]*9)
    assert_(p.alignment_reason(rgb,depth,info,info) is not None)
for value in ['time','frame','calibration']:check('guard_'+value,lambda v=value:gate(v))
def nms():
    d=[('Chair',.9,1,1,30,30),('Chair',.8,2,2,30,30)]
    assert_(len(p.filter_detections(d) if modern else d)==1)
check('duplicate_boxes_removed',nms)
def rgb():
    a=np.zeros((3,4,3),np.uint8);a[...,0]=255
    out=decode(msg(a,'bgr8'));assert_(out is not None and out[0,0].tolist()==[0,0,255])
check('bgr_rgb_conversion',rgb)
def heat():
    if not modern:
        assert_('numeric_depth_legend' in (root/'board_radar_gui.py').read_text(),'no numeric color scale / nonuniform layout')
    else:
        a=np.linspace(.2,5.5,80*80,dtype=np.float32).reshape(80,80);a[:4,:4]=np.nan;old=a.copy()
        image=p.heatmap_rgb(a);assert_(image.shape==(80,80,3));assert_(image[0,0].tolist()==[18,23,32]);assert_(np.allclose(a,old,equal_nan=True))
        assert_(not np.array_equal(image[10,10],image[70,70]))
check('heatmap_numeric_legend_and_mask',heat)
passed=sum(ok for _,ok in checks)
print(f'checks={len(checks)} passed={passed} failed={len(checks)-passed}')
sys.exit(0 if passed==len(checks) else 1)
