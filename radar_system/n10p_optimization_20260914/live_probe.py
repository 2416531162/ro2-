import json, math, statistics, time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
rclpy.init(); n=Node('n10p_optimization_probe'); rows=[]; statuses=[]; latest=[None]
def cb(m):
    now=time.monotonic(); age=n.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9
    rows.append((now,m.scan_time,age,sum(math.isfinite(r) for r in m.ranges)))
    latest[0]=dict(ranges=[float(r) if math.isfinite(r) else None for r in m.ranges],range_min=m.range_min,range_max=m.range_max,angle_min=m.angle_min,angle_increment=m.angle_increment,scan_time=m.scan_time)
n.create_subscription(LaserScan,'/scan',cb,qos_profile_sensor_data)
n.create_subscription(String,'/lidar/status',lambda m:statuses.append(json.loads(m.data)),10)
t=time.monotonic()
while time.monotonic()-t<5: rclpy.spin_once(n,timeout_sec=.1)
rate=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0
out=dict(messages=len(rows),hz=round(rate,2),scan_time_median=round(statistics.median(r[1] for r in rows),4) if rows else 0,age_median_ms=round(statistics.median(r[2] for r in rows)*1000,1) if rows else None,valid_min=min((r[3] for r in rows),default=0),valid_max=max((r[3] for r in rows),default=0),status=statuses[-1] if statuses else None)
print(json.dumps(out,ensure_ascii=False))
if latest[0]: open(sys.argv[1],'w').write(json.dumps(latest[0]))
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if len(rows)>10 else 1)
