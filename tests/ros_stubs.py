"""最小 ROS 2 替身,让三维建图节点能在没有 rclpy 的环境里被单元测试。

**这不是 ROS 集成测试。** 它只替换消息容器、Node 基类和 TF 缓存的接口形状,
用来验证我们自己的判断逻辑 —— 尤其是「查不到 TF 就丢帧」这一条。
序列化、QoS、执行器、实车行为都不在覆盖范围内。
"""
import sys
import types


class _Field:
    def __init__(self, name='', offset=0, datatype=0, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class PointField(_Field):
    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = 1, 2, 3, 4, 5, 6, 7, 8


class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class _Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = _Stamp()


class PointCloud2:
    def __init__(self):
        self.header = _Header()
        self.height = self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = self.row_step = 0
        self.data = b''
        self.is_dense = True


class Image:
    def __init__(self):
        self.header = _Header()
        self.height = self.width = self.step = 0
        self.encoding = ''
        self.is_bigendian = 0
        self.data = b''


class String:
    def __init__(self, data=''):
        self.data = data


class TransformException(Exception):
    pass


class Time:
    def __init__(self, seconds=0.0, nanoseconds=0):
        self.nanoseconds = int(nanoseconds or seconds * 1e9)

    @classmethod
    def from_msg(cls, stamp):
        return cls(nanoseconds=stamp.sec * 10**9 + stamp.nanosec)

    def to_msg(self):
        return _Stamp(self.nanoseconds // 10**9, self.nanoseconds % 10**9)


class Duration:
    def __init__(self, seconds=0.0, nanoseconds=0):
        self.nanoseconds = int(nanoseconds or seconds * 1e9)


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class FakeTransform:
    def __init__(self, translation=(0., 0., 0.), rotation=(0., 0., 0., 1.), stamp_s=0.0):
        self.transform = types.SimpleNamespace(
            translation=_Vec(*translation), rotation=_Quat(*rotation))
        self.header = _Header()
        self.header.stamp = _Stamp(int(stamp_s), int(round((stamp_s % 1) * 1e9)))
        self.child_frame_id = ''


class FakeBuffer:
    """可编程 TF 缓存。默认所有查询都失败 —— 和真车刚上电时一样。"""

    def __init__(self, cache_time=None, **kw):
        self.table = {}          # (target, source) -> FakeTransform 或 Exception
        self.calls = []
        self.cache_time = cache_time

    def set(self, target, source, value):
        self.table[(target, source)] = value

    def lookup_transform(self, target, source, stamp, timeout=None):
        self.calls.append((target, source, getattr(stamp, 'nanoseconds', None)))
        value = self.table.get((target, source))
        if value is None:
            raise TransformException('no transform %s <- %s' % (target, source))
        if isinstance(value, Exception):
            raise value
        return value


class _Auto:
    """自动生长的消息字段。

    真实 ROS 消息的字段是提前定义好的嵌套结构(msg.pose.position.x)。
    替身里挨个声明既啰嗦又容易漏,所以让未知字段在第一次访问时自己长出来,
    同时支持下标(covariance[0]=...)。
    """

    def __init__(self, **kw):
        object.__setattr__(self, '_fields', {})
        object.__setattr__(self, '_items', {})
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattr__(self, name):
        if name.startswith('_'):
            raise AttributeError(name)
        fields = object.__getattribute__(self, '_fields')
        if name not in fields:
            fields[name] = _Auto()
        return fields[name]

    def __setattr__(self, name, value):
        object.__getattribute__(self, '_fields')[name] = value

    def __getitem__(self, key):
        return object.__getattribute__(self, '_items').setdefault(key, 0.0)

    def __setitem__(self, key, value):
        object.__getattribute__(self, '_items')[key] = value


def _simple(name):
    """造一个字段可随便赋值的消息类,够构造与赋值用。"""
    def __init__(self, **kw):
        _Auto.__init__(self)
        self.header = _Header()
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (_Auto,), {'__init__': __init__})


class _Logger:
    def info(self, *a): pass
    def warn(self, *a): pass
    def warning(self, *a): pass
    def error(self, *a): pass


class _Clock:
    def __init__(self):
        self.seconds = 1000.0

    def now(self):
        return Time(seconds=self.seconds)


class Node:
    def __init__(self, name='node'):
        self._name = name
        self._params = {}
        self.subscriptions_ = []
        self.publishers_ = {}
        self.timers = []
        self._clock = _Clock()

    def declare_parameter(self, name, value):
        self._params[name] = value
        return types.SimpleNamespace(value=value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def set_param(self, name, value):
        self._params[name] = value

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subscriptions_.append((topic, cb))
        return types.SimpleNamespace(topic=topic)

    def create_publisher(self, msg_type, topic, qos):
        pub = types.SimpleNamespace(topic=topic, sent=[])
        pub.publish = pub.sent.append
        self.publishers_[topic] = pub
        return pub

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return types.SimpleNamespace(period=period)

    def create_client(self, srv_type, name):
        client = types.SimpleNamespace(name=name, calls=[])
        client.service_is_ready = lambda: False
        client.call_async = client.calls.append
        return client

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


def install():
    """把替身塞进 sys.modules。必须在 import live_3d_map_node 之前调用。"""
    if 'rclpy' in sys.modules and getattr(sys.modules['rclpy'], '_is_stub', False):
        return

    rclpy = types.ModuleType('rclpy'); rclpy._is_stub = True
    rclpy.init = lambda *a, **k: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None

    node_mod = types.ModuleType('rclpy.node'); node_mod.Node = Node
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **k: types.SimpleNamespace(**k)
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1, KEEP_ALL=2)
    qos.qos_profile_sensor_data = object()
    time_mod = types.ModuleType('rclpy.time'); time_mod.Time = Time
    dur_mod = types.ModuleType('rclpy.duration'); dur_mod.Duration = Duration

    sensor = types.ModuleType('sensor_msgs'); sensor_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msg.PointCloud2, sensor_msg.PointField = PointCloud2, PointField
    sensor_msg.Image = Image
    sensor_msg.LaserScan = _simple('LaserScan')
    sensor_msg.CameraInfo = _simple('CameraInfo')
    sensor.msg = sensor_msg

    std = types.ModuleType('std_msgs'); std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = String
    for name in ('Float32', 'Float64', 'Bool', 'Int32'):
        setattr(std_msg, name, _simple(name))
    std.msg = std_msg

    # 跟随节点要用 /wheeltec/arm (SetBool) 与 /wheeltec/stop (Trigger)
    srv = types.ModuleType('std_srvs'); srv_msg = types.ModuleType('std_srvs.srv')
    for name in ('SetBool', 'Trigger'):
        service = _simple(name)
        service.Request = _simple(name + '.Request')
        service.Response = _simple(name + '.Response')
        setattr(srv_msg, name, service)
    srv.srv = srv_msg

    nav = types.ModuleType('nav_msgs'); nav_msg = types.ModuleType('nav_msgs.msg')
    nav_msg.OccupancyGrid = _simple('OccupancyGrid')
    nav_msg.Path = _simple('Path')
    nav_msg.Odometry = _simple('Odometry')
    nav.msg = nav_msg

    geo = types.ModuleType('geometry_msgs'); geo_msg = types.ModuleType('geometry_msgs.msg')
    for name in ('PoseStamped', 'PoseWithCovarianceStamped', 'Point',
                 'Quaternion', 'Twist', 'TransformStamped'):
        setattr(geo_msg, name, _simple(name))
    geo.msg = geo_msg

    vis = types.ModuleType('visualization_msgs'); vis_msg = types.ModuleType('visualization_msgs.msg')
    vis_msg.MarkerArray = _simple('MarkerArray')
    vis_msg.Marker = _simple('Marker')
    vis.msg = vis_msg

    tf2 = types.ModuleType('tf2_ros')
    tf2.Buffer, tf2.TransformException = FakeBuffer, TransformException
    tf2.TransformListener = lambda buffer, node, **k: types.SimpleNamespace()

    rclpy.node, rclpy.qos, rclpy.time, rclpy.duration = node_mod, qos, time_mod, dur_mod
    sys.modules.update({
        'rclpy': rclpy, 'rclpy.node': node_mod, 'rclpy.qos': qos,
        'rclpy.time': time_mod, 'rclpy.duration': dur_mod,
        'sensor_msgs': sensor, 'sensor_msgs.msg': sensor_msg,
        'std_msgs': std, 'std_msgs.msg': std_msg, 'tf2_ros': tf2,
        'std_srvs': srv, 'std_srvs.srv': srv_msg,
        'nav_msgs': nav, 'nav_msgs.msg': nav_msg,
        'geometry_msgs': geo, 'geometry_msgs.msg': geo_msg,
        'visualization_msgs': vis, 'visualization_msgs.msg': vis_msg,
    })
