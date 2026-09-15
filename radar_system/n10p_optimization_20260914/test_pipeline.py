#!/usr/bin/env python3
"""Identical regression contracts for the saved baseline and modified driver."""
import copy, importlib.util, math, pathlib, sys, types
root = pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
clock = [100.0]
class Message:
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0,nanosec=0),frame_id='')
class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(copy.deepcopy(msg))
class Node:
    def __init__(self,*a): self.timers=[]
    def create_publisher(self,*a): return Publisher()
    def create_timer(self,period,fn): self.timers.append((period,fn)); return fn
    def get_logger(self): return types.SimpleNamespace(info=lambda *a,**k:None,warn=lambda *a,**k:None)
    def get_clock(self):
        return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(clock[0]*1e9),to_msg=lambda:types.SimpleNamespace(sec=int(clock[0]),nanosec=0)))
class Serial:
    def __init__(self,*a,**k): self.data=bytearray()
    @property
    def in_waiting(self): return len(self.data)
    def read(self,n): b=bytes(self.data[:n]); del self.data[:n]; return b
    def reset_input_buffer(self): self.data.clear()
    def close(self): pass
for name, attrs in {
 'serial':dict(Serial=Serial,SerialException=OSError), 'rclpy':{},
 'rclpy.node':dict(Node=Node),'rclpy.qos':dict(QoSProfile=lambda **k:k),
 'sensor_msgs':{},'sensor_msgs.msg':dict(LaserScan=Message),
 'std_msgs':{},'std_msgs.msg':dict(String=Message)}.items():
    m=types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name]=m
spec=importlib.util.spec_from_file_location('driver_test', root/'real_lidar_node.py')
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
m.time=types.SimpleNamespace(time=lambda:clock[0],monotonic=lambda:clock[0],sleep=lambda n:None)
def packet(angle=0, span=10, first=2000, second=0):
    p=bytearray(108); p[:5]=bytes.fromhex('a55a6c1042')
    p[5:7]=int(angle*100).to_bytes(2,'big'); p[105:107]=int((angle+span)%360*100).to_bytes(2,'big')
    for i in range(16):
        off=7+i*6; p[off:off+6]=first.to_bytes(2,'big')+b'\x14'+second.to_bytes(2,'big')+b'\x10'
    p[-1]=sum(p[:-1])&255
    return bytes(p)
def node(): clock[0]=100.; return m.RealLidarNode()
def feed(n,b,dt=.003): clock[0]+=dt; n.ser.data.extend(b); n.spin_serial()
def ticks(n,steps=100):
    for i in range(steps):
        clock[0]+=.01; n.spin_serial()
        if hasattr(n,'pub_timer') and i%10==0: n.publish_scan()
def scans(n,first=2000,second=0):
    for _ in range(3):
        for angle in range(0,360,10): feed(n,packet(angle,9.8,first,second))
def bins(n): return n.assembler.bins if hasattr(n,'assembler') else n.bins
checks=[]
def test(name,fn):
    try: fn(); checks.append((name,True))
    except Exception as e: checks.append((name,False)); print('FAIL '+name+': '+str(e))
def equal_size():
    n=node(); scans(n); assert n.pub.messages and len(n.pub.messages[-1].ranges)==720
    assert n.pub.messages[-1].angle_increment>0

def crc_resync():
    n=node(); feed(n,packet(350)); feed(n,packet(0))
    before=len(bins(n)); bad=packet(20); feed(n,bad[:50]+bad[51:]+packet(100))
    assert any(515<=k<=521 for k in bins(n)), 'valid frame after dropped byte lost'

def invalid_angle():
    n=node(); feed(n,packet(350)); feed(n,packet(0)); before=dict(bins(n)); feed(n,packet(100,50))
    assert bins(n)==before, 'invalid span was replaced by invented 15 degrees'

def fallback():
    n=node(); scans(n,0,1500)
    assert n.pub.messages and any(abs(r-1.5)<.001 for r in n.pub.messages[-1].ranges), 'second echo ignored'

def no_repeat():
    n=node(); scans(n); count=len(n.pub.messages); ticks(n)
    assert len(n.pub.messages)==count, 'old scan republished with fresh timestamp'

def empty_scan():
    n=node(); scans(n,0,0)
    assert n.pub.messages and all(math.isinf(r) for r in n.pub.messages[-1].ranges), 'empty sweep suppressed'

def scan_period():
    n=node(); scans(n)
    assert .10<n.pub.messages[-1].scan_time<.12, 'scan_time is elapsed since reset, not revolution duration'

def sparse_clear():
    n=node(); scans(n); feed(n,packet(10),dt=.6)
    if hasattr(n,'assembler'): assert n.assembler.started is None
    else: assert not n.display_bins, 'stale sweep survives stream gap'

def sectors():
    if hasattr(m,'N10PDecoder'):
        from n10p_pipeline import scan_payload,project_point
        r=[math.inf]*720; r[181]=.4; r[540]=2
        d=scan_payload(r,.15,12)
        assert d['left']==.4 and d['right']==2 and d['count']==2
        assert project_point(math.pi/2,1,100,100,50)[0]<100
        assert scan_payload([math.inf]*720,.15,12)['count']==0
    else:
        s=(root/'board_radar_gui.py').read_text()
        assert 'project_point' in s and 'scan_payload' in s, 'direction mirrored; sectors skip half-degree bins'

def bounded():
    n=node(); feed(n,b'\0'*8192+b'\xa5'); feed(n,packet(350)[1:]); feed(n,packet(0))
    if hasattr(n,'decoder'): assert len(n.decoder.buffer)<108
    else: assert len(n.buf)<108
for name,fn in [('720_bins',equal_size),('byte_resync',crc_resync),('reject_bad_angles',invalid_angle),
                ('second_echo',fallback),('no_stale_republish',no_repeat),('empty_sweep',empty_scan),
                ('revolution_time',scan_period),('gap_reset',sparse_clear),('sector_and_projection',sectors),('bounded_buffer',bounded)]: test(name,fn)
# Replay the actual same 3-second serial capture through both versions.
n=node(); raw=(pathlib.Path(__file__).parent/'n10p-raw.bin').read_bytes()
for offset in range(0,len(raw),180): feed(n,raw[offset:offset+180],dt=3*180/len(raw))
valid=[sum(math.isfinite(r) for r in msg.ranges) for msg in n.pub.messages]
print('REPLAY bytes=%d scans=%d valid_min=%d valid_max=%d' % (len(raw),len(valid),min(valid,default=0),max(valid,default=0)))
passed=sum(ok for _,ok in checks)
print('checks=%d passed=%d failed=%d' % (len(checks),passed,len(checks)-passed))
sys.exit(0 if passed==len(checks) else 1)
