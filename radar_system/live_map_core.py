"""Bounded, ROS-independent map display primitives. Never used for collision control."""
import math
import struct
import zlib
import numpy as np


def yaw(q):
    return math.atan2(2 * (q.x*q.y + q.w*q.z), 1 - 2 * (q.y*q.y + q.z*q.z))


def transform_points(points, translation, quaternion):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = points @ matrix.T + np.asarray(translation)
    if not np.isfinite(result).all():
        raise ValueError('non-finite coordinates')
    return result


def optical_to_base(x, y, z, camera_x=.54, pitch=math.radians(15)):
    # Optical +x right,+y down,+z forward; base +x forward,+y left.
    return (camera_x + z*math.cos(pitch) - y*math.sin(pitch), -x)


def following_point(robot, person, clearance=1.2, front=.67):
    """Candidate rear-axle pose only, NOT a validated navigation goal.

    The standoff includes front overhang. Near the person, hold current pose;
    do not generate an unnecessary reverse goal or target the person's body.
    """
    dx, dy = person[0]-robot[0], person[1]-robot[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return (robot[0], robot[1], robot[2])
    heading = math.atan2(dy, dx)
    travel = max(0., d-clearance-front)
    return (robot[0]+dx/d*travel, robot[1]+dy/d*travel, heading)


def grid_to_world(gx, gy, resolution, ox, oy, heading):
    x, y = gx*resolution, gy*resolution
    return ox+x*math.cos(heading)-y*math.sin(heading), oy+x*math.sin(heading)+y*math.cos(heading)


def png_gray(image):
    image = np.ascontiguousarray(image, dtype=np.uint8)
    h, w = image.shape
    def chunk(kind, data):
        return struct.pack('!I', len(data))+kind+data+struct.pack('!I', zlib.crc32(kind+data)&0xffffffff)
    raw = b''.join(b'\x00'+row.tobytes() for row in image)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', w,h,8,0,0,0,0))
            + chunk(b'IDAT', zlib.compress(raw, 3)) + chunk(b'IEND', b''))


def encode_grid(data, width, height, resolution, max_side=1024):
    """Conservative display pooling: never skip a thin occupied wall.

    Rows remain ROS bottom-up; renderers flip them geometrically. Unknown cells
    remain gray, observed free white, occupied dark. Full map is kept by SLAM;
    this PNG is a lossy display product and must never be saved as a Nav2 map.
    """
    if (width <= 0 or height <= 0 or width*height > 16_000_000
            or len(data) != width*height or not math.isfinite(resolution) or resolution <= 0):
        raise ValueError('invalid or oversized occupancy grid')
    a = np.asarray(data, dtype=np.int16).reshape(height, width)
    if np.any((a < -1) | (a > 100)):
        raise ValueError('invalid occupancy values')
    step = max(1, math.ceil(max(width,height)/max_side))
    h, w = math.ceil(height/step), math.ceil(width/step)
    occupied = np.zeros((h*step, w*step), dtype=bool)
    free = np.zeros_like(occupied)
    occupied[:height,:width] = a >= 65
    free[:height,:width] = (a >= 0) & (a <= 25)
    occ = occupied.reshape(h,step,w,step).any(axis=(1,3))
    clear = free.reshape(h,step,w,step).all(axis=(1,3))
    image = np.full((h,w), 196, dtype=np.uint8)
    image[clear], image[occ] = 248, 44
    return png_gray(image), dict(width=width,height=height,resolution=resolution,
                                image_width=w,image_height=h,step=step,
                                known_area_m2=round(float(np.count_nonzero(a>=0))*resolution**2,2),
                                occupied_cells=int(np.count_nonzero(a>=65)))


def decode_depth(msg):
    encoding = msg.encoding.lower()
    if encoding in ('16uc1', 'mono16'):
        dtype, scale = np.dtype('>u2' if msg.is_bigendian else '<u2'), .001
    elif encoding == '32fc1':
        dtype, scale = np.dtype('>f4' if msg.is_bigendian else '<f4'), 1.
    else:
        raise ValueError('unsupported depth encoding: '+msg.encoding)
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.height*msg.step:
        raise ValueError('invalid depth stride/buffer')
    return np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                      strides=(msg.step,dtype.itemsize)).astype(np.float32)*scale


def body_self_hit_mask(points,front=.67,rear=.18,half_width=.335):
    """Only the measured physical body, no inflated safety margin."""
    p=np.asarray(points).reshape(-1,3)
    return (p[:,0]>=-rear)&(p[:,0]<=front)&(np.abs(p[:,1])<=half_width)


def recent_stamp(now,stamp,max_age=.6):
    return math.isfinite(stamp) and stamp>0 and -.1<=now-stamp<=max_age
