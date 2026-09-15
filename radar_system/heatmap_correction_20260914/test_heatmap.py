import sys,importlib.util,pathlib,types,json,hashlib
import numpy as np
p=pathlib.Path(sys.argv[1]);spec=importlib.util.spec_from_file_location('heat_target',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
checks=[]
def test(name,fn):
    try:fn();checks.append(True)
    except Exception as e:checks.append(False);print('FAIL '+name+': '+str(e))
def require(v,msg):
    if not v:raise AssertionError(msg)
d=np.tile(np.linspace(300,1800,120,dtype=np.uint16),(80,1));d[20:40,30:45]=0;d[10,10]=0
render=lambda depth,rgb=None:m.render_depth_heatmap(depth,rgb)[0][:depth.shape[0]]
def independent():require(np.array_equal(render(d,np.zeros((80,120,3),np.uint8)),render(d,np.full((80,120,3),255,np.uint8))),'RGB content changes depth colors')
def holes():require(np.all(render(d)[d==0]==m.HEAT_BG_RGB),'invalid pixels painted as returns')
def fixed():
    a=np.full((80,120),1000,np.uint16);b=a.copy();b[:30]=4000
    require(np.array_equal(render(a)[40,50],render(b)[40,50]),'same depth changes color with scene histogram')
def source():
    before=d.copy();render(d);require(np.array_equal(before,d),'source modified')
def scale():
    _,near,far=m.render_depth_heatmap(d);require(near==200 and far==2000,'default scale is not fixed 0.2–2m')
def boundaries():
    a=np.full((80,120),1000,np.uint16);a[0,0]=65535;a[1,1]=200
    out=render(a);require(np.array_equal(out[0,0],m.HEAT_BG_RGB),'invalid pixel received a depth color');require(not np.array_equal(out[1,1],m.HEAT_BG_RGB),'200mm endpoint hidden')
def colors():
    a=np.full((80,120),200,np.uint16);a[:,60:]=2000
    out=render(a);require(out[40,20,0]>out[40,20,2] and out[40,90,2]>out[40,90,0],'near red/far blue reversed')
def depth_types():
    for dtype,enc,factor in [('>u2','16UC1',1),('<f4','32FC1',.001)]:
        a=np.full((10,12),1200*factor,dtype=dtype);data=b''.join(row.tobytes()+b'\0'*8 for row in a)
        msg=types.SimpleNamespace(data=data,width=12,height=10,encoding=enc,is_bigendian=dtype.startswith('>'),step=len(data)//10)
        out=m.decode_depth_mm(msg);require(np.allclose(out,1200),'stride/unit/byte-order decoding wrong')
def center():
    a=np.full((40,40),1000.,np.float32);a[20,20]=0
    require(m.depth_center_mm(a)==1000,'single invalid center defeats valid neighborhood')
    a[:,20:]=3000;require(m.depth_center_mm(a)==0,'mixed surfaces return invented midpoint')
for name,fn in [('no_rgb_mix',independent),('no_hole_fill',holes),('stable_metric_color',fixed),('source_unchanged',source),('fixed_range',scale),('validity_endpoints',boundaries),('red_near_blue_far',colors),('depth_encoding',depth_types),('robust_center',center)]:test(name,fn)
# Exactly the same captured real sensor matrix.
base=pathlib.Path('/tmp/camera-baseline');meta=json.loads((base/'meta.json').read_text());x=meta['images']['depth'];raw=(base/'depth.bin').read_bytes()
depth=np.frombuffer(raw,np.uint16).reshape(x['h'],x['w'])
rgb=np.frombuffer((base/'rgb.bin').read_bytes(),np.uint8).reshape(x['h'],x['w'],3)
out,near,far=m.render_depth_heatmap(depth,rgb)
print('REPLAY shape=%s near=%.1f far=%.1f source_sha256=%s' % (str(out.shape),near,far,hashlib.sha256(raw).hexdigest()))
print('checks=%d passed=%d failed=%d'%(len(checks),sum(checks),len(checks)-sum(checks)))
sys.exit(0 if all(checks) else 1)
