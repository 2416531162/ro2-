"""RGB-D decoding, quality-gated surface ranging and display-only heat mapping."""
import math
import cv2
import numpy as np

DEPTH_MIN, DEPTH_MAX = 0.2, 5.5
MAX_SYNC_S = 0.04


def image_array(msg):
    """Honor ROS encoding, byte order and row padding; return owned native data."""
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        dtype, channels = np.dtype('u1'), 3
    elif enc in ('16uc1', 'mono16'):
        dtype, channels = np.dtype('>u2' if msg.is_bigendian else '<u2'), 1
    elif enc == '32fc1':
        dtype, channels = np.dtype('>f4' if msg.is_bigendian else '<f4'), 1
    else:
        raise ValueError('unsupported image encoding: '+msg.encoding)
    row_bytes = msg.width * channels * dtype.itemsize
    if msg.width <= 0 or msg.height <= 0 or msg.step < row_bytes or len(msg.data) < msg.step*msg.height:
        raise ValueError('invalid image dimensions/stride/buffer')
    shape = (msg.height, msg.width, channels) if channels == 3 else (msg.height, msg.width)
    strides = (msg.step, channels*dtype.itemsize, dtype.itemsize) if channels == 3 else (msg.step, dtype.itemsize)
    arr = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    if channels == 3:
        return np.ascontiguousarray(arr[..., ::-1] if enc == 'bgr8' else arr).copy()
    # ROS depth: uint16 millimeters, float32 meters.
    depth = arr.astype(np.float32)
    if enc != '32fc1': depth *= .001
    depth[~np.isfinite(depth) | (depth < DEPTH_MIN) | (depth > DEPTH_MAX)] = np.nan
    return depth


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec/1e9


def alignment_reason(rgb, depth, rgb_info, depth_info):
    if depth is None: return 'no_depth'
    if abs(rgb['stamp']-depth['stamp']) > MAX_SYNC_S: return 'unsynchronized'
    if not rgb['frame'] or rgb['frame'] != depth['frame']: return 'unaligned'
    if rgb['array'].shape[:2] != depth['array'].shape: return 'shape_mismatch'
    h,w = depth['array'].shape
    for info in [rgb_info,depth_info]:
        if not info or info['w'] != w or info['h'] != h or info['frame'] != rgb['frame']:
            return 'no_calibration'
        if not all(math.isfinite(float(v)) for v in info['k']) or info['k'][0] <= 0 or info['k'][4] <= 0:
            return 'no_calibration'
        if any(abs(x)>1e-6 for x in info.get('d', [])):
            return 'requires_rectification'
    if not np.allclose(rgb_info['k'],depth_info['k'],rtol=.002,atol=.05): return 'unaligned_intrinsics'
    return None


def surface_sample(depth, x, y, half_x=8, half_y=8):
    """Robust central-surface estimate, NOT segmentation or an object's centroid."""
    h,w=depth.shape
    x,y=int(x),int(y)
    if not (0<=x<w and 0<=y<h): return dict(valid=False,reason='outside_image')
    roi=depth[max(0,y-half_y):min(h,y+half_y+1),max(0,x-half_x):min(w,x+half_x+1)]
    valid=np.isfinite(roi)&(roi>=DEPTH_MIN)&(roi<=DEPTH_MAX)
    values=roi[valid]
    fraction=float(values.size/roi.size)
    result=dict(valid=False,reason='insufficient_depth',valid_ratio=round(fraction,3),samples=int(values.size),u=x,v=y)
    if values.size<12 or fraction<.35: return result
    median=float(np.median(values)); q25,q75=np.percentile(values,[25,75])
    # Do not silently report one of two substantial foreground/background layers.
    if q75-q25>max(.12,median*.10):
        result['reason']='mixed_surfaces'; return result
    tolerance=max(.025,median*.03)
    inliers=values[np.abs(values-median)<=tolerance]
    if inliers.size/values.size<.6:
        result['reason']='unstable_depth'; return result
    z=float(np.median(inliers)); mad=float(np.median(np.abs(inliers-z)))
    result.update(valid=True,reason='ok',z=round(z,4),spread_mm=round(mad*1000,1),samples=int(inliers.size))
    return result


def range_target(depth, box, k):
    x1,y1,x2,y2=box; u=(x1+x2)//2; v=(y1+y2)//2
    sample=surface_sample(depth,u,v,max(4,min(24,int((x2-x1)*.15))),max(4,min(24,int((y2-y1)*.15))))
    if sample['valid']:
        z=sample['z']; x=(u-k[2])*z/k[0]; y=(v-k[5])*z/k[4]
        sample.update(x=round(x,4),y=round(y,4),distance=round(math.sqrt(x*x+y*y+z*z),4))
    return sample


def heatmap_rgb(depth, far=4.5, near=DEPTH_MIN):
    if not near<far<=DEPTH_MAX: raise ValueError('invalid heatmap scale')
    valid=np.isfinite(depth)&(depth>=DEPTH_MIN)&(depth<=DEPTH_MAX)
    clean=np.where(valid,depth,0).astype(np.float32)
    med=cv2.medianBlur(clean,3)
    # Only denoise close-valued valid surfaces for DISPLAY; no hole filling,
    # boundary blending or mutation of the source used for measurement.
    smooth=np.where(valid&(med>=DEPTH_MIN)&(np.abs(med-clean)<np.maximum(.025,clean*.02)),med,clean)
    scaled=np.clip((far-smooth)/(far-near),0,1)
    colors=cv2.applyColorMap(np.rint(scaled*255).astype(np.uint8),cv2.COLORMAP_TURBO)
    colors=cv2.cvtColor(colors,cv2.COLOR_BGR2RGB)
    colors[~valid]=(18,23,32)
    return colors


def color_lut():
    return cv2.cvtColor(cv2.applyColorMap(np.arange(255,-1,-1,dtype=np.uint8).reshape(1,256),cv2.COLORMAP_TURBO),cv2.COLOR_BGR2RGB)[0]


def filter_detections(detections, threshold=.5):
    """Class-aware greedy NMS; keeps overlapping different categories."""
    result=[]
    for item in sorted(detections,key=lambda d:d[1],reverse=True):
        label,conf,x1,y1,x2,y2,*_=item
        if conf<threshold or x2-x1<6 or y2-y1<6:continue
        area=(x2-x1)*(y2-y1)
        duplicate=False
        for other in result:
            if other[0]!=label:continue
            ix=max(0,min(x2,other[4])-max(x1,other[2]));iy=max(0,min(y2,other[5])-max(y1,other[3]))
            inter=ix*iy; union=area+(other[4]-other[2])*(other[5]-other[3])-inter
            if inter/max(1,union)>.45:duplicate=True;break
        if not duplicate:result.append(item)
    return result
