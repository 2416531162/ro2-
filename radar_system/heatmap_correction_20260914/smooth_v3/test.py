import sys,pathlib,importlib.util,time,json,hashlib
import numpy as np
p=pathlib.Path(sys.argv[1]);sys.path.insert(0,str(p.parent));spec=importlib.util.spec_from_file_location('smooth_target',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
checks=[]
def check(name,fn):
    try:assert fn();checks.append(True)
    except Exception as e:checks.append(False);print('FAIL '+name)
rng=np.random.default_rng(42);raw=(1000+rng.normal(0,15,(64,80))).astype(np.float32);raw[30,30]=0;raw[10:22,10:25]=0
copy=raw.copy()
def proc():return m.prepare_display_depth(raw)
check('display_copy_unchanged_source',lambda:(proc() is not None) and np.array_equal(raw,copy))
check('reduced_plane_noise',lambda:np.std(proc()[0][40:60,40:60])<np.std(raw[40:60,40:60])*.8)
check('small_hole_explicit_mask',lambda:proc()[2][30,30] and not proc()[1][30,30])
check('large_hole_stays_missing',lambda:not np.any(proc()[2][10:22,10:25]) and np.all(proc()[0][10:22,10:25]==0))
def edge():
    a=np.full((40,40),1000,np.float32);a[:,20:]=3000;d,valid,est=m.prepare_display_depth(a)
    return np.allclose(d,a,atol=.001) and not np.any(est)
check('discontinuity_preserved',edge)
def mask():
    image,stats=m.render_depth_display(raw)
    return stats['estimated_pixels']>=1 and image.shape==(118,80,3) and np.any(np.all(image[30:31,30:31]==[210,215,225],axis=2))
check('estimate_hatch_visible',mask)
def rawmode():
    a,stats=m.render_depth_display(raw,smooth=False);b,_,_=m.render_depth_heatmap(raw)
    return np.array_equal(a,b) and stats['estimated_pixels']==0
check('raw_mode_exact',rawmode)
def metric():
    before=m.depth_center_mm(raw);quality=m.depth_quality(raw,2000);m.render_depth_display(raw)
    return before==m.depth_center_mm(raw) and quality==m.depth_quality(raw,2000) and np.array_equal(raw,copy)
check('measurement_invariant',metric)
base=pathlib.Path('/tmp/heatmap-live-data');meta=json.loads((base/'meta.json').read_text());x=meta['images']['depth'];data=(base/'depth.bin').read_bytes();frame=np.frombuffer(data,np.uint16).reshape(x['h'],x['w']);times=[]
for i in range(5):
    start=time.monotonic()
    if hasattr(m,'render_depth_display'):image,stats=m.render_depth_display(frame)
    else:image,_,_=m.render_depth_heatmap(frame);stats={}
    times.append((time.monotonic()-start)*1000)
print('checks=%d passed=%d failed=%d'%(len(checks),sum(checks),len(checks)-sum(checks)))
print('REPLAY median_ms=%.1f estimated_pixels=%d source_sha256=%s'%(np.median(times),stats.get('estimated_pixels',0),hashlib.sha256(data).hexdigest()))
sys.exit(0 if all(checks) else 1)
