#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业自动记录（排查问题用）：每次点「开始作业」（车进入 SELECTING / RUNNING），自动把这次作业的全部数据存下来。
手动页的「IMU 直行」（/straight/status 的 state 为 running）也一样记，按任务名「IMU直行」加开始时间单独成一个作业。

不碰控制：只订阅话题、写文件，本进程出错或退出都不影响开车。

    订阅  /line/status、/straight/status（判断作业开始、结束）和 TOPICS 里的话题（原始字节原样写盘；几个状态话题另外解码算摘要）
    发布  /jobrec/status（1 Hz，String JSON：在不在记、记到哪、占多少空间、出错原因）

每个作业一个目录 ~/rtk/jobs/<开始时间>_<作业名>_<id 前 8 位>/：
    README.md         数据说明（给事后分析的人 / AI：每个文件、每个话题是什么、单位、时钟，怎么读）
    job.json          作业信息、结果、每段的起止和摘要（横偏、RTK 解算状态占比、控制权切换、底盘故障……）
    context/segNN/    这一段开始时的配置快照：版本号（release.txt、发布清单 manifest.json 带源码提交）、robot 配置、
                      oid.yaml、fusion.json、route.json、line_state.json
    segNN/            这一段的 ROS 2 bag（mcap，zstd 压缩）：点开始前 PRE_S 秒到停下后 POST_S 秒
一次「开始」或「继续」是一段；同一个作业暂停后再继续，接着往同一个目录里加段（记录进程重启过也按作业 id 找回目录）。
没有作业记录的任务（网页上的 A—B 直线）按任务名加开始时间单独成一个作业。
总量超过 MAX_TOTAL_MB 或磁盘剩余不到 MIN_FREE_MB 时删最旧的作业目录（正在记的不删）。
"""
import collections
import json
import os
import re
import shutil
import time

JOBS_DIR = os.path.expanduser(os.environ.get('RK3588_JOBS_DIR', '~/rtk/jobs'))
PRE_S = 10.0              # 点开始之前留多久（起步前的 RTK、控制权状态）
POST_S = 5.0              # 停下之后再记多久（停车过程、最后的状态）
STATUS_LOST_S = 10.0      # 这么久没有 /line/status（画线节点挂了）：结束这一段
MAX_TOTAL_MB = 2048
MIN_FREE_MB = 1024
ACTIVE_PHASES = ('SELECTING', 'RUNNING')
STRAIGHT_TASK = 'IMU直行'   # IMU 定向直行在跑时当成这个任务（目录名、job.json 的 label）
ENDED_STATES = ('DONE', 'CANCELLED')
MAX_MODE_CHANGES = 500
MAX_MESSAGES = 200

RELEASE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 记哪些话题（不记相机图像、雷达：太大，画线也用不到）
TOPICS = {
    '/line/status': 'std_msgs/msg/String',
    '/line/trace': 'std_msgs/msg/String',
    '/line/command': 'std_msgs/msg/String',
    '/navigation/command': 'std_msgs/msg/String',
    '/manual/command': 'std_msgs/msg/String',
    '/straight/status': 'std_msgs/msg/String',
    '/motion/status': 'std_msgs/msg/String',
    '/oid/status': 'std_msgs/msg/String',
    '/oid/trace': 'std_msgs/msg/String',
    '/wheeltec/status': 'std_msgs/msg/String',
    '/gnss/status': 'std_msgs/msg/String',
    '/gnss/raw': 'std_msgs/msg/String',
    '/gnss/fix': 'sensor_msgs/msg/NavSatFix',
    '/gnss/heading': 'sensor_msgs/msg/Imu',
    '/imu': 'sensor_msgs/msg/Imu',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/imu/status': 'std_msgs/msg/String',
    '/imu762/data': 'sensor_msgs/msg/Imu',
    '/imu/selected/data': 'sensor_msgs/msg/Imu',
    '/odom': 'nav_msgs/msg/Odometry',
    '/wheel/odometry': 'std_msgs/msg/String',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/voltage': 'std_msgs/msg/Float32',
    '/rosout': 'rcl_interfaces/msg/Log',
}
# 这几个是 String JSON，另外解码算摘要
SUMMARY_TOPICS = ('/line/status', '/gnss/status', '/motion/status', '/oid/status', '/straight/status')


def context_files():
    """每段开始时拷进 context/segNN/ 的配置文件（不存在的跳过）。"""
    home = os.path.expanduser('~')
    return [os.environ.get('RK3588_ROBOT_CONFIG', os.path.join(RELEASE_ROOT, 'robot_core', 'robot.json')),
            os.path.join(RELEASE_ROOT, 'oid_protocol', 'oid.yaml'),
            os.path.join(RELEASE_ROOT, 'rion_protocol', 'hda536t.yaml'),
            os.path.join(home, 'rtk', 'fusion.json'),
            os.path.join(home, 'rtk', 'route.json'),
            os.path.join(home, 'rtk', 'line_state.json'),
            os.path.join(home, 'rtk', 'route_operations.json'),
            '/etc/rk3588/runtime.env']


def iso(t):
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)) + ('.%03d' % int((t % 1) * 1000))


def dir_size(path):
    total = 0
    for base, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(base, name))
            except OSError:
                pass
    return total


README = '''# 作业记录（job_recorder 自动记）

给事后分析用（为什么走不直、为什么停车、定位跳……）。一次「开始」或「继续」是一段（segNN）。

## 文件
- `job.json`：作业信息、结果（state / reason）、每段起止时间、摘要（横偏 e 的分布、RTK 解算状态占比、控制权切换、
  底盘故障、电压……）。先看它找到出问题的段和大概时刻。
- `context/segNN/`：这一段开始时的配置快照。`manifest.json` 的 `source_commit` 是车上跑的源码提交（GitHub
  2416531162/ro2-），`robot*.json` 车身与控制参数，`fusion.json` 定位标定，`route.json` 路线，`oid.yaml` 底盘驱动参数。
- `segNN/`：ROS 2 bag（mcap，zstd 压缩，CDR 序列化）。Python 读：`pip install mcap mcap-ros2-support`，
  `mcap_ros2.reader.read_ros2_messages(path)`；String 话题的 `data` 是 JSON。

## 时钟
- bag 里每条消息的记录时间是 ROS 时钟（墙上时间，ns）。
- JSON 里的 `t`、`t_rx` 是车上的单调时钟（秒），`/line/trace`、`/oid/trace`、`/gnss/raw` 三者同一个时钟，可以直接对齐。
- `/gnss/raw` 的 `t` 是按 GPS 时间换算的测量时刻，`t_rx` 是车收到的时刻（`t_rx - t` 是传输延迟 + 抖动）。

## 话题（看「看到的 → 算的 → 发的 → 车做的」）
- 看到的：`/gnss/raw`（每条 RTK 测量：fix 主天线、fix2 副天线、heading 双天线测向、vel；sol 解算状态，NARROW_INT 才算固定）、
  `/gnss/status`（2 Hz，数据来源 source：usb / tablet WiFi / bluetooth / can）、`/imu`、`/imu/data`（陀螺、IMU 航向角）、
  `/odom`（兼容轮速里程计，约 50 Hz）、`/wheel/odometry`（Gen5 对齐轮程，JSON；time_basis=host_receive，
  start→t 区间轮程 distance 米 / turn 弧度，speed/yaw_rate 为终点即时值；quality 区分 encoder/speed/pending/invalid，
  两轮 speed_t/position_t 是接收时刻，epoch 区分驱动重连，seq 去重；详见 docs/WHEEL_ODOMETRY.md）。
- 算的：`/line/trace`（50 Hz，每个控制周期一条，作业时才有）：`pose` 是控制用的融合位姿（x 东、y 北，米，原点是 `/line/status` 的 origin 经纬度；yaw 弧度，
  东 0 逆时针；sigma_xy / sigma_yaw 不确定度；hold_* 是 RTK 断了靠推算的时长 / 路程 / 转角；usable 能不能控制），
  `tracker` 是跟踪器的看法（e 横偏米，车在线左侧为正；heading_err_deg；kappa_ref 路线曲率、kappa 下发曲率、integral 积分；
  section / gear / pivoting 分段、前进倒车、原地转），`vx`、`wz` 是这一拍算出来的，`sent` 是否真的发出去了。
  `/line/status`（5 Hz）带 phase、message、diag（定位跳动计数等）和路线。
- 发的：`/navigation/command`（发给底盘仲裁的 vx、wz）。
- 车做的：`/oid/trace`（约 50 Hz）：`v_cmd` / `w_cmd` 限速后发给底盘的、`erpm_sent` / `erpm_read` 左右轮下发 / 读回转速、
  `wheels_mps` 读回的左右轮速、`v` / `w` 里程计、`pos_cdeg` 左右电机轴位置（0.01°，累计，轮毂直驱即轮子转角）。`/oid/status`（2 Hz）驱动器故障、通信错误。`/motion/status` 控制权、急停、健康。

## 分析走不直的常用对照
1. `/line/trace` 的 `tracker.e`（横偏）随时间：是慢漂（航向 / 标定）、周期摆（增益 / 延迟）、还是跳（定位）。
2. `pose.yaw` 对比 `/gnss/raw` heading（双天线）和 `/imu/data` 航向：融合航向是不是偏了、偏多少。
3. `wz`（算的）→ `/oid/trace` `w_cmd`（限速后）→ `w`（读回）：转向有没有被限、延迟多少、左右轮是否一侧跟不上。
4. `/gnss/raw` 的 `sol` 和 `t_rx - t`：解算状态掉了没有、数据有没有晚到 / 断流（蓝牙、WiFi、CAN 转发）。
'''


def write_readme(path):
    try:
        with open(os.path.join(path, 'README.md'), 'w', encoding='utf-8') as fh:
            fh.write(README)
    except OSError:
        pass


def write_json(path, value):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(value, fh, ensure_ascii=False, indent=1, default=str)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class JobWatch:
    """由 /line/status 判断作业的开始、结束。update() 按顺序返回要做的事：
    ('open_job', key, job) / ('open_segment',) / ('close_segment', 为什么) / ('close_job', 最后看到的作业记录)"""

    def __init__(self, post_s=POST_S, wall=time.time):
        self.post_s, self.wall = post_s, wall
        self.key = None            # 当前作业：作业记录的 id；没有作业记录的任务是 task-<任务>-<开始时间>
        self.job = None            # 当前作业最近一次的作业记录（operations.job）
        self.recording = False
        self.idle_since = None

    @staticmethod
    def _job(status):
        ops = status.get('operations')
        job = ops.get('job') if isinstance(ops, dict) else None
        return job if isinstance(job, dict) and isinstance(job.get('id'), str) else None

    def update(self, status, now):
        out = []
        job = self._job(status)
        if status.get('phase') in ACTIVE_PHASES:
            # 作业记录只属于路线任务；跑完的记录会一直留着（DONE），不能把后面的 A—B 直线算进去
            own = job if job and status.get('task') == 'route' and job.get('state') not in ENDED_STATES else None
            if own:
                key = own['id']
            elif self.recording and self.key and self.key.startswith('task-'):
                key = self.key
            else:
                key = 'task-%s-%s' % (status.get('task') or 'line', time.strftime('%Y%m%d_%H%M%S', time.localtime(self.wall())))
            if key != self.key:
                if self.recording:
                    out.append(('close_segment', 'job_change'))
                    self.recording = False
                if self.key is not None:
                    out.append(('close_job', self.job))
                self.key, self.job = key, None
                out.append(('open_job', key, own))
            if own:
                self.job = own
            if not self.recording:
                out.append(('open_segment',))
                self.recording = True
            self.idle_since = None
            return out
        if job and job['id'] == self.key:
            self.job = job
        if self.recording:
            if self.idle_since is None:
                self.idle_since = now
            if now - self.idle_since < self.post_s:
                return out
            out.append(('close_segment', 'idle'))
            self.recording = False
        if self.key is not None:
            same = job is not None and job['id'] == self.key
            if self.key.startswith('task-') or not same or job.get('state') in ENDED_STATES:
                out.append(('close_job', self.job))
                self.key = self.job = None
        return out

    def lost(self):
        """/line/status 断了：结束这一段（作业不结束，恢复后接着记）。"""
        if not self.recording:
            return []
        self.recording, self.idle_since = False, None
        return [('close_segment', 'line_status_lost')]


def _percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


class Summary:
    """一段的摘要（边记边算），写进 job.json。t 是这一段开始后的秒数。"""

    def __init__(self):
        self.e = []                          # 跟踪中的横偏 |e|，米（/line/status 5 Hz）
        self.s_max = 0.0
        self.tracker_last = None
        self.messages = []                   # 画线节点提示的变化 [t, phase, message]
        self.jumps = None                    # 定位跳动、位置重置次数 [首条, 末条]
        self.resets = None
        self.fix = collections.Counter()     # /gnss/status：各解算状态的条数
        self.heading = collections.Counter()  # 双天线航向有效 / 无效的条数
        self.sats_min = None
        self.diff_age_max = None
        self.sources = collections.Counter()
        self.modes = []                      # 控制权切换 [t, mode, reason, waiting_stationary]
        self.health = collections.Counter()  # 不健康的原因
        self.chassis_faults = collections.Counter()
        self.silent = collections.Counter()
        self.link_errors = None              # 底盘通信错误 [首条, 末条]
        self.io_errors = None
        self.voltage_min = None
        self._mode = None
        self.straight = {}                   # IMU 直行：这一段里跑过的每一次（作业键 → 最后一条状态）

    @staticmethod
    def _span(old, value):
        if not isinstance(value, (int, float)):
            return old
        return [value, value] if old is None else [old[0], value]

    def feed(self, topic, d, t):
        if not isinstance(d, dict):
            return
        t = round(t, 2)
        if topic == '/line/status':
            tr = d.get('tracker') or {}
            if d.get('phase') == 'RUNNING' and tr.get('state') == 'RUNNING' and isinstance(tr.get('e'), (int, float)):
                self.e.append(abs(tr['e']))
            if isinstance(tr.get('s'), (int, float)):
                self.s_max = max(self.s_max, tr['s'])
            self.tracker_last = {k: tr.get(k) for k in ('state', 'reason', 's', 'length', 'section', 'sections', 'rms', 'max')
                                 if k in tr}
            msg = [d.get('phase'), d.get('message')]
            if (not self.messages or self.messages[-1][1:] != msg) and len(self.messages) < MAX_MESSAGES:
                self.messages.append([t] + msg)
            diag = d.get('diag') or {}
            self.jumps = self._span(self.jumps, diag.get('jumps'))
            self.resets = self._span(self.resets, diag.get('position_resets'))
        elif topic == '/gnss/status':
            self.fix[str(d.get('rtk'))] += 1
            self.heading['ok' if d.get('heading_ok') else 'invalid'] += 1
            self.sources[str(d.get('source'))] += 1
            if isinstance(d.get('sats'), (int, float)):
                self.sats_min = d['sats'] if self.sats_min is None else min(self.sats_min, d['sats'])
            if isinstance(d.get('diff_age'), (int, float)):
                self.diff_age_max = d['diff_age'] if self.diff_age_max is None else max(self.diff_age_max, d['diff_age'])
        elif topic == '/motion/status':
            mode = (d.get('mode'), d.get('reason'))
            if mode != self._mode and len(self.modes) < MAX_MODE_CHANGES:
                self.modes.append([t, d.get('mode'), d.get('reason'), d.get('waiting_stationary')])
            self._mode = mode
            if d.get('healthy') is False:
                for f in d.get('health_faults') or ['unhealthy']:
                    self.health[str(f)] += 1
        elif topic == '/straight/status':
            key = json.dumps(d.get('run'))
            # 只收这一段里真跑过的；停了以后状态一直停在最后那次，别的作业段里不算
            if d.get('state') == 'running' and key not in self.straight and len(self.straight) < MAX_MESSAGES:
                self.straight[key] = dict(run=d.get('run'), start_t=t)
            if key in self.straight:
                self.straight[key].update({k: d.get(k) for k in ('state', 'reason', 'target_deg', 'error_deg',
                                                                  'max_error_deg', 'within_pct', 'elapsed_s')})
                self.straight[key]['last_t'] = t
        elif topic == '/oid/status':
            tel = d.get('telemetry') or {}
            for side, text in (tel.get('faults') or {}).items():
                self.chassis_faults['%s:%s' % (side, text)] += 1
            for side in tel.get('silent') or []:
                self.silent[str(side)] += 1
            self.link_errors = self._span(self.link_errors, d.get('link_errors'))
            self.io_errors = self._span(self.io_errors, d.get('io_errors'))
            v = tel.get('voltage')
            if isinstance(v, (int, float)) and v == v:
                self.voltage_min = v if self.voltage_min is None else min(self.voltage_min, v)

    def to_dict(self):
        n_fix = sum(self.fix.values())
        n_head = sum(self.heading.values())
        delta = lambda span: None if span is None else span[1] - span[0]
        return dict(
            track=dict(samples=len(self.e), e_max_m=_r(max(self.e) if self.e else None),
                       e_p95_m=_r(_percentile(self.e, .95)), e_mean_m=_r(sum(self.e) / len(self.e) if self.e else None),
                       s_max_m=_r(self.s_max, 3), last=self.tracker_last),
            line_messages=self.messages,
            rtk=dict(fix_share={k: round(v / n_fix, 3) for k, v in self.fix.items()} if n_fix else {},
                     heading_ok_share=round(self.heading['ok'] / n_head, 3) if n_head else None,
                     sats_min=self.sats_min, diff_age_max_s=self.diff_age_max, source=dict(self.sources),
                     jumps=delta(self.jumps), position_resets=delta(self.resets)),
            motion=dict(changes=self.modes, unhealthy=dict(self.health)),
            chassis=dict(faults=dict(self.chassis_faults), silent=dict(self.silent), link_errors=delta(self.link_errors),
                         io_errors=delta(self.io_errors), voltage_min=self.voltage_min),
            straight=list(self.straight.values()))


def _r(v, digits=4):
    return None if v is None else round(v, digits)


class BagWriter:
    """一段 ROS 2 bag（mcap，zstd 压缩）。rosbag2_py 只在车上有；测试换成假的 writer。"""

    def __init__(self, uri):
        import rosbag2_py
        self._bag = rosbag2_py
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap', storage_preset_profile='zstd_fast'),
                         rosbag2_py.ConverterOptions('cdr', 'cdr'))
        self.topics = {}

    def write(self, topic, type_name, data, t_ns):
        if topic not in self.topics:
            self.topics[topic] = len(self.topics)
            try:                                   # Jazzy：TopicMetadata(id, name, type, format)
                meta = self._bag.TopicMetadata(self.topics[topic], topic, type_name, 'cdr')
            except TypeError:                      # Humble（旧 Jetson）：没有 id
                meta = self._bag.TopicMetadata(topic, type_name, 'cdr')
            self.writer.create_topic(meta)
        self.writer.write(topic, data, t_ns)

    def close(self):
        self.writer.close()


def prune(root, keep=None, max_total_mb=MAX_TOTAL_MB, min_free_mb=MIN_FREE_MB, log=print):
    """删最旧的作业目录，直到总量不超过 max_total_mb、磁盘剩余不少于 min_free_mb；keep（正在记的）不删。"""
    try:
        names = sorted(n for n in os.listdir(root) if os.path.isdir(os.path.join(root, n)))
    except OSError:
        return []
    sizes = {n: dir_size(os.path.join(root, n)) for n in names}
    removed = []
    for name in names:
        total = sum(sizes.values())
        try:
            free = shutil.disk_usage(root).free
        except OSError:
            free = float('inf')
        if total <= max_total_mb * 2 ** 20 and free >= min_free_mb * 2 ** 20:
            break
        path = os.path.join(root, name)
        if keep and os.path.abspath(path) == os.path.abspath(keep):
            continue
        shutil.rmtree(path, ignore_errors=True)
        sizes.pop(name)
        removed.append(name)
        log('作业记录超出空间，删掉最旧的：%s' % name)
    return removed


class JobRecorder:
    """记录逻辑（不依赖 ROS）：on_message 喂每条消息（序列化后的字节），tick 每秒调一次。"""

    def __init__(self, root=JOBS_DIR, writer=BagWriter, pre_s=PRE_S, post_s=POST_S, max_total_mb=MAX_TOTAL_MB,
                 min_free_mb=MIN_FREE_MB, files=context_files, clock=time.monotonic, wall=time.time, log=print):
        self.root, self.writer_cls, self.pre_ns = root, writer, int(pre_s * 1e9)
        self.max_total_mb, self.min_free_mb = max_total_mb, min_free_mb
        self.files, self.clock, self.wall, self.log = files, clock, wall, log
        self.watch = JobWatch(post_s, wall)
        self.ring = collections.deque()
        self.dir = self.info = None
        self.segment = self.seg = self.summary = None
        self.last_status = None
        self.line = self.straight = None    # 最近一条 /line/status、/straight/status
        self.error = None
        os.makedirs(root, exist_ok=True)
        prune(root, None, max_total_mb, min_free_mb, log)

    # ---- 消息 ----
    def on_message(self, topic, type_name, data, t_ns, decoded=None):
        now = self.clock()
        if self.segment is not None:
            self._write(topic, type_name, data, t_ns)
        else:
            self.ring.append((t_ns, topic, type_name, data))
            while self.ring and self.ring[0][0] < t_ns - self.pre_ns:
                self.ring.popleft()
        if topic in ('/line/status', '/straight/status') and isinstance(decoded, dict):
            self.last_status = now
            if topic == '/line/status':
                self.line = decoded
            else:
                self.straight = decoded
            for action in self.watch.update(self._merged(), now):
                self._act(action)
        if self.segment is not None and decoded is not None:
            self.summary.feed(topic, decoded, now - self.seg['t0'])

    def _merged(self):
        """给 JobWatch 的状态：路线作业在跑用它；没有路线作业、IMU 直行在跑时当成任务 STRAIGHT_TASK。"""
        line = self.line or {}
        if line.get('phase') in ACTIVE_PHASES:
            return line
        if (self.straight or {}).get('state') == 'running':
            return {'phase': 'RUNNING', 'task': STRAIGHT_TASK}
        return line

    def tick(self):
        now = self.clock()
        if self.segment is not None and (self.last_status is None or now - self.last_status > STATUS_LOST_S):
            for action in self.watch.lost():
                self._act(action)
        if self.segment is not None:
            try:
                low = shutil.disk_usage(self.root).free < self.min_free_mb * 2 ** 20 / 2
            except OSError:
                low = False
            if low:
                self.error = '磁盘快满了，停止记录'
                self._close_segment('disk_low')

    def shutdown(self):
        if self.segment is not None:
            self._close_segment('recorder_stopped')

    # ---- 作业、段 ----
    def _act(self, action):
        try:
            if action[0] == 'open_job':
                self._open_job(action[1], action[2])
            elif action[0] == 'open_segment':
                self._open_segment()
            elif action[0] == 'close_segment':
                self._close_segment(action[1])
            elif action[0] == 'close_job':
                self._close_job(action[1])
        except Exception as exc:          # 记录出错不能影响别的：记下原因，这一段不记了
            self.error = '%s：%s' % (action[0], exc)
            self.log('作业记录出错：%s' % self.error)
            writer, self.segment, self.seg, self.summary = self.segment, None, None, None
            try:
                if writer is not None:
                    writer.close()
            except Exception:
                pass

    def _find(self, key):
        tail = '_' + re.sub(r'[^0-9A-Za-z-]', '', key)[:8]
        for name in sorted(os.listdir(self.root), reverse=True):
            path = os.path.join(self.root, name)
            if name.endswith(tail) and os.path.isfile(os.path.join(path, 'job.json')):
                try:
                    with open(os.path.join(path, 'job.json'), encoding='utf-8') as fh:
                        info = json.load(fh)
                except (OSError, ValueError):
                    continue
                if info.get('id') == key:
                    return path, info
        return None, None

    def _open_job(self, key, job):
        path, info = (None, None) if key.startswith('task-') else self._find(key)
        if path is None:
            if key.startswith('task-'):
                label = key.split('-')[1]            # 没有作业记录的任务：用任务名（line）
            else:
                label = (job or {}).get('label') or '作业'
            safe = re.sub(r'[^\w-]', '', str(label))[:20] or 'job'
            name = '%s_%s_%s' % (time.strftime('%Y%m%d_%H%M%S', time.localtime(self.wall())), safe,
                                 re.sub(r'[^0-9A-Za-z-]', '', key)[:8])
            path = os.path.join(self.root, name)
            os.makedirs(path, exist_ok=True)
            info = dict(id=key, label=label, created=iso(self.wall()), segments=[], result=None)
            write_readme(path)
        info['job'] = {k: v for k, v in (job or info.get('job') or {}).items() if k not in ('plan', 'profile')}
        self.dir, self.info = path, info
        self._save()
        self.log('作业记录：%s' % path)

    def _open_segment(self):
        if self.dir is None:
            return
        n = len(self.info['segments']) + 1
        name = 'seg%02d' % n
        self._snapshot(os.path.join(self.dir, 'context', name))
        self.segment = self.writer_cls(os.path.join(self.dir, name))
        self.seg = dict(name=name, start=iso(self.wall()), t0=self.clock(), messages=0, bytes=0)
        self.summary = Summary()
        self.info['segments'].append(dict(name=name, start=self.seg['start'], end=None))
        self.error = None
        ring, self.ring = self.ring, collections.deque()
        for t_ns, topic, type_name, data in ring:     # 点开始之前的 PRE_S 秒
            self._write(topic, type_name, data, t_ns)
        self._save()

    def _write(self, topic, type_name, data, t_ns):
        if self.segment is None:            # 这一段刚因写入出错关掉了
            return
        try:
            self.segment.write(topic, type_name, data, t_ns)
        except Exception as exc:
            self.error = '写入失败：%s' % exc
            self.log('作业记录写入失败，这一段不记了：%s' % exc)
            self._close_segment('write_error')
            return
        self.seg['messages'] += 1
        self.seg['bytes'] += len(data)

    def _close_segment(self, why):
        seg, writer, self.segment = self.seg, self.segment, None
        if seg is None:
            return
        try:
            if writer is not None:
                writer.close()
        except Exception as exc:
            self.error = '关闭失败：%s' % exc
        entry = self.info['segments'][-1]
        entry.update(end=iso(self.wall()), seconds=round(self.clock() - seg['t0'], 1), closed_by=why,
                     messages=seg['messages'], raw_bytes=seg['bytes'],
                     disk_bytes=dir_size(os.path.join(self.dir, seg['name'])), error=self.error,
                     summary=self.summary.to_dict() if self.summary else None)
        self.seg = self.summary = None
        self._save()

    def _close_job(self, job):
        if self.dir is None:
            return
        if self.segment is not None:
            self._close_segment('job_end')
        if job:
            self.info['job'] = {k: v for k, v in job.items() if k not in ('plan', 'profile')}
        self.info['result'] = dict(state=(job or {}).get('state'), reason=(job or {}).get('reason'), closed=iso(self.wall()))
        self._save()
        self.log('作业记录结束：%s（%s）' % (self.dir, self.info['result']['state']))
        keep, self.dir, self.info = self.dir, None, None
        prune(self.root, keep, self.max_total_mb, self.min_free_mb, self.log)

    def _snapshot(self, path):
        os.makedirs(path, exist_ok=True)
        with open(os.path.join(path, 'release.txt'), 'w', encoding='utf-8') as fh:
            fh.write('%s\n%s\n' % (os.path.realpath('/opt/rk3588/current'), RELEASE_ROOT))
        manifest = os.path.join(RELEASE_ROOT, 'manifest.json')      # 发布清单：source_commit 是车上跑的源码提交
        if os.path.isfile(manifest):
            try:
                shutil.copy2(manifest, os.path.join(path, 'manifest.json'))
            except OSError:
                pass
        for src in self.files():
            if src and os.path.isfile(src):
                try:
                    shutil.copy2(src, os.path.join(path, os.path.basename(src)))
                except OSError:
                    pass

    def _save(self):
        if self.dir is not None:
            write_json(os.path.join(self.dir, 'job.json'), self.info)

    def status(self):
        try:
            usage = shutil.disk_usage(self.root)
            free_mb = round(usage.free / 2 ** 20)
        except OSError:
            free_mb = None
        seg = self.seg
        return dict(recording=self.segment is not None, dir=self.dir, job_id=self.watch.key,
                    segment=seg['name'] if seg else None,
                    seconds=round(self.clock() - seg['t0'], 1) if seg else None,
                    raw_mb=round(seg['bytes'] / 2 ** 20, 2) if seg else None, error=self.error, free_mb=free_mb)


def attach(node, rec):
    """在 rclpy 节点上建订阅（原始字节）、/jobrec/status 发布和 1 Hz 定时器。"""
    from rclpy.qos import QoSProfile
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String

    clock = node.get_clock()

    def callback(topic, type_name):
        def cb(data):
            decoded = None
            if topic in SUMMARY_TOPICS:
                try:
                    decoded = json.loads(deserialize_message(data, String).data)
                except Exception:
                    decoded = None
            rec.on_message(topic, type_name, data, clock.now().nanoseconds, decoded)
        return cb

    for topic, type_name in TOPICS.items():
        node.create_subscription(get_message(type_name), topic, callback(topic, type_name), QoSProfile(depth=50), raw=True)
    pub = node.create_publisher(String, '/jobrec/status', 1)

    def every_second():
        rec.tick()
        pub.publish(String(data=json.dumps(rec.status(), ensure_ascii=False)))

    node.create_timer(1.0, every_second)
    return pub


def main():
    import signal
    import rclpy
    from rclpy.node import Node
    import sys
    sys.path.insert(0, RELEASE_ROOT)
    from robot_core.service_watchdog import watchdog

    watchdog.start("ros")
    rclpy.init()
    node = Node('job_recorder')
    rec = JobRecorder(log=node.get_logger().info)
    attach(node, rec)
    node.create_timer(0.5, lambda: watchdog.tick("ros"))

    def on_signal(_sig=None, _frame=None):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.close()
        rec.shutdown()          # 停服务（部署、关机）时把正在记的这一段收好尾
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
