"""Conservative RGB-D person ranging, independent of ROS and OpenCV."""
import numpy as np


class DepthMeasurement:
    DEPTH_MIN_MM = 150.0
    DEPTH_MAX_MM = 6000.0
    DEPTH_INSET = 0.20          # bbox 四边各内缩 20%,避开边缘穿透到背景
    DEPTH_PERCENTILE = 20.0     # 取第 20 百分位而非中位数,保守偏近
    DEPTH_MIN_VALID_RATIO = 0.15
    DEPTH_MIN_PIXELS = 30
    DEPTH_MAX_PAIR_AGE_S = 0.15  # RGB 与深度帧的最大允许时间差

    @classmethod
    def robust_depth(cls, depth, x1, y1, x2, y2):
        """返回 (Z_米, 有效像素占比);数据不足以下结论时返回 (None, ratio)。"""
        h, w = depth.shape[:2]
        bw, bh = x2 - x1, y2 - y1
        if bw <= 0 or bh <= 0:
            return None, 0.0
        ix1 = max(0, int(x1 + bw * cls.DEPTH_INSET))
        ix2 = min(w, int(x2 - bw * cls.DEPTH_INSET))
        iy1 = max(0, int(y1 + bh * cls.DEPTH_INSET))
        iy2 = min(h, int(y2 - bh * cls.DEPTH_INSET))
        if ix2 - ix1 < 2 or iy2 - iy1 < 2:
            return None, 0.0

        roi = depth[iy1:iy2, ix1:ix2]
        if roi.size < cls.DEPTH_MIN_PIXELS:
            return None, 0.0

        mask = (roi > cls.DEPTH_MIN_MM) & (roi < cls.DEPTH_MAX_MM)
        valid = roi[mask]
        ratio = float(len(valid)) / float(roi.size)
        # 有效像素太稀疏说明这一块深度图本身就不可信 (逆光/黑衣/玻璃),
        # 此时任何统计量都是噪声,直接判为无效观测。
        if ratio < cls.DEPTH_MIN_VALID_RATIO or len(valid) < cls.DEPTH_MIN_PIXELS:
            return None, ratio
        return float(np.percentile(valid, cls.DEPTH_PERCENTILE)) / 1000.0, ratio



def decode_depth(msg):
    """Decode padded/endian-aware ROS depth images to millimetres."""
    encoding = msg.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        scale = 1.
    elif encoding == '32fc1':
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        scale = 1000.
    else:
        raise ValueError('Unsupported depth encoding: '+msg.encoding)
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.step*msg.height:
        raise ValueError('Invalid depth image stride/buffer')
    image = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                       strides=(msg.step,dtype.itemsize))
    return image if scale == 1. else image.astype(np.float32) * scale


def decode_rgb(msg):
    if msg.encoding.lower() not in ('rgb8','bgr8'):
        raise ValueError('Unsupported color encoding: '+msg.encoding)
    if msg.step < msg.width*3 or len(msg.data) < msg.step*msg.height:
        raise ValueError('Invalid color image stride/buffer')
    image = np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=msg.data,
                       strides=(msg.step,3,1))
    return image[:,:,::-1] if msg.encoding.lower()=='bgr8' else image
