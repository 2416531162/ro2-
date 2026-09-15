import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

rclpy.init()
node = rclpy.create_node('yolo_readonly_capture')
received = {}
for key, topic, cls in [('rgb', '/camera/rgb/image_raw', Image),
                        ('depth', '/camera/depth_raw/image', Image),
                        ('info', '/camera/rgb/camera_info', CameraInfo)]:
    node.create_subscription(cls, topic, lambda msg, key=key: received.update({key: msg}), qos_profile_sensor_data)
deadline = time.monotonic()+12
while len(received) < 3 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.2)
assert len(received) == 3, 'RGB/depth/calibration topics missing'
rgb, depth, info = received['rgb'], received['depth'], received['info']
assert rgb.encoding == 'rgb8' and depth.encoding in ('16UC1', 'mono16')
rgb_array = np.ndarray((rgb.height, rgb.width, 3), np.uint8, buffer=bytes(rgb.data), strides=(rgb.step, 3, 1)).copy()
depth_array = np.ndarray((depth.height, depth.width), '>u2' if depth.is_bigendian else '<u2', buffer=bytes(depth.data), strides=(depth.step, 2)).astype(np.uint16)
p = Path(__file__).resolve().parent
np.savez_compressed(p/'fixture.npz', rgb=rgb_array, depth=depth_array, k=np.array(info.k))
cv2.imwrite(str(p/'fixture.jpg'), cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))
stamp = lambda m: m.header.stamp.sec+m.header.stamp.nanosec/1e9
result = dict(rgb_shape=rgb_array.shape, depth_shape=depth_array.shape,
              rgb_frame=rgb.header.frame_id, depth_frame=depth.header.frame_id,
              timestamp_delta_ms=round(abs(stamp(rgb)-stamp(depth))*1000, 1))
(p/'fixture.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
