#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK 走直线 / 走路线节点：融合定位 + LineMission，经运动仲裁的 NAVIGATION 通道控制底盘。

    订阅  /gnss/raw（rtk_node）、/imu、/odom、
          /imu/data（算好的姿态：横滚、俯仰和 IMU 自己的偏航。旧车是 imu_filter_madgwick 算的，只给 app 显示；
          Gen5 是 HDA536T 传感器自己算的，它的航向角还交给融合推算航向（融合 yaw_input = 'angle'，这时 /imu 不进融合））、
          /line/command（String JSON：mark/start/stop/clear/route_set/route_start/route_clear）
    发布  /navigation/command（经 MotionClient，带仲裁 epoch）、/line/status（5 Hz）

雷达防撞、指令超时、急停都在底盘驱动里，对 NAVIGATION 同样生效；本节点只在任务运行时发指令。
IMU 和轮速用消息头时间戳（驱动读到数据时打的）换算到本机单调时钟，GNSS 测量时刻由 rtk_node 换算。
"""
import copy
import dataclasses
import json
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import runtime_config  # noqa: F401  服务直接运行本脚本时把仓库根目录加进 sys.path（line_tracker 要用 robot_core）
from robot_core.config import CONTROL_DT
from robot_core.service_watchdog import watchdog
from line_mission import RTK_HOLD_S, LineMission
from paint_quality import RTK_LOST_PAINT_STOP_S, RTK_LOST_STOP_DELAY_S
from motion_client import MotionClient
import control_trace
from wheel_input import WheelInput
from attitude_controls import AttitudeControls
from imu_sources import ImuSources

SELECT_RETRY_S = 0.5
# 已取消的固定基线参数（天线间距改由两根天线固定解的平面实时距离核对）；旧版 fusion.json 里可能还有，读取时丢掉
# 已取消的参数：固定基线；2026-10-05 删掉的推算位移互检、测向基线带、双天线位置互检门限（用户：「原来的检查必须删除」）
OBSOLETE_FUSION_KEYS = ('baseline_m', 'baseline_tol_m', 'baseline_check',
                        'displacement_check_enabled', 'displacement_window_s', 'displacement_long_window_s',
                        'displacement_limit_m', 'displacement_confirm_s', 'heading_length_band_m',
                        'dual_check_m', 'dual_check_rate_m', 'dual_check_n', 'dual_ok_m', 'dual_ok_s', 'dual_stale_s')
# Gen5 主天线（ANT1）相对后轴中点的杠杆臂，车体系 (前, 左) 米。车上 ~/rtk/fusion.json 里是 2026-09-29 手量的
# GEN5_ANTENNA_HAND；2026-10-06 用 66 次原地转向（左右轮速对称、绕后轴中点转）里两根天线的固定解反推，ANT1 实际在
# 前 0.235、左 0.197（ANT2 前 0.234、右 0.212，间距 0.409 对得上），两根都比手量的靠后约 0.5 cm、靠右约 1 cm。
# 用户（2026-10-06）：「修复，改车端」。车上文件还是手量值时用实测值；文件里写了别的值照用。不改车上的文件，回滚版本即恢复。
GEN5_ANTENNA_HAND = (0.24, 0.2075)
GEN5_ANTENNA_MEASURED = (0.235, 0.197)
# 单天线流动站（平板转来，um982.Um982Stream.antenna_mode = single）装在现在双天线的右天线（ANT2）位置
# （2026-10-07 用户：右天线前 0.234 m、右 0.212 m，即上面 GEN5_ANTENNA_MEASURED 同一次反推的 ANT2）。车体系 (前, 左) 米。
# 喷头参数按后轴中心存，换单天线照用
SINGLE_ANTENNA_XY = (0.234, -0.212)
# 单天线没有测向：车头 = IMU 航向角 + 偏移，偏移按卫星位移对 IMU + 轮程推算位移的夹角定、修（gnss_fusion._imu_course，
# 2026-10-07 用户：「用 imu 的航向角做航向，偏移来修正」；原来按「走直」的轨迹回归定，全链路仿真里实车拟合底盘低速
# 车身摆 ±1–4°/s，一次也定不出）。起步往前走 0.5 m 定第一次（用户 10-07 定；仿真车头误差中位 0.3°、最差 0.7°），
# 之后每走 SINGLE_ALIGN_M 修一次（直线、弧线、原地转后都修）
SINGLE_COURSE_INIT_M = 0.5
SINGLE_ALIGN_M = 1.0
# 单天线没有测向，融合估不了 IMU 航向角的漂移速度：起始不确定度按实测量级给（10-06 晚 IMU 换算漂约 0.4°/min），
# 不按双天线的 0.2°/s（那样车头不确定度每秒涨 0.2°，起步倒车接入十几秒就超门槛停车，仿真里 1/3 的趟卡在起步）
SINGLE_BIAS_SIGMA_DEG_S = 0.01
MAX_STAMP_AGE_S = 0.5       # 消息头时间戳比现在早超过这么多（或在未来），就不信它，改用收到时刻
ATTITUDE_MAX_AGE_S = 0.5    # /imu/data 超过这么久没来，就不报姿态（app 显示 —）
TILT_ATTACH_S = 0.1         # 旧车 /imu 陀螺帧带上这么久以内收到的 /imu/data 横滚俯仰，给融合做倾斜补偿


def attitude_deg(x, y, z, w):
    """四元数 -> (横滚, 俯仰, 偏航)，度。ROS REP-103 车体系 x 车头、y 左、z 上：
    横滚绕 x 轴，左侧抬起（右侧压低）为正；俯仰绕 y 轴，车头压低为正；偏航绕 z 轴，左转为正（−180…180）。
    imu_filter_madgwick 不用磁力计时偏航没有绝对参考：0 大致是它启动时的车头方向，随陀螺零偏漂移。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class LineNode(Node):
    def __init__(self, mission, clock=time.monotonic, fusion_path=None):
        super().__init__('line_node')
        self.mission, self.clock = mission, clock
        self.motion = MotionClient(self, 'navigation', timeout_s=1.0)
        self.pub_status = self.create_publisher(String, '/line/status', 1)
        # 直行和定位共用当前选源；旧 CAN IMU 的原话题继续供对比记录使用。
        self.pub_selected_imu = self.create_publisher(Imu,'/imu/selected/data',20)
        self.pub_imu762 = self.create_publisher(Imu,'/imu762/data',20)
        # 每个控制周期一条控制记录（作业时才发），作业记录存下来事后分析，见 control_trace
        self.pub_trace = self.create_publisher(String, '/line/trace', 50)
        self.create_subscription(String, '/gnss/raw', self.on_gnss, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 20)
        self.create_subscription(Imu, '/imu/data', self.on_attitude, 10)
        self.create_subscription(String,'/imu/status',self.on_imu_status,1)
        self._attitude = None   # (收到时刻, 横滚°, 俯仰°, 偏航°)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/line/command', self.on_command, 10)
        # 融合用 IMU 航向角（Gen5）：从 /imu/data 取航向角和陀螺交给融合，/imu 不再进融合（同一帧不送两遍）
        config = getattr(getattr(mission, 'fusion', None), 'config', None)
        self.imu_yaw_input = getattr(config, 'yaw_input', 'rate') == 'angle'
        self.wheel_input = (WheelInput(self.clock(), getattr(config, 'sensor_timeout_s', .2))
                            if getattr(config, 'wheel_increment_enabled', False) else None)
        self.create_subscription(String, '/wheel/odometry', self.on_wheel_motion, 50)
        self.attitude_controls = AttitudeControls(mission, fusion_path)
        supported = bool(getattr(mission,'differential',False))
        source_path = os.path.join(os.path.dirname(fusion_path),'imu_source.json') if supported and fusion_path else None
        self.imu_sources = ImuSources(supported,source_path)
        self._imu_switch_at = -math.inf
        self._imu_switch_stamp_ns = 0
        self._imu762_version = -1
        self._imu762_reader = None
        if supported:
            from rion_protocol.imu762_stream import Imu762Stream
            self._imu762_reader = Imu762Stream(clock)
        self.attitude_controls.select_imu_source(self.imu_sources.active)
        self.last_result = None
        self._select_at = None
        self.create_timer(CONTROL_DT, self.control)
        self.create_timer(0.2, self.publish_status)

    def on_gnss(self, msg):
        try:
            self.mission.on_gnss(json.loads(msg.data))
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warn('坏的 GNSS 消息：%s' % exc)

    def msg_time(self, msg):
        """消息头时间戳（底盘驱动读到数据时打的 ROS 时间）换算到本机单调时钟，去掉 DDS 传输和排队的延迟。
        没有时间戳或不可信时用收到时刻。"""
        now = self.clock()
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is None or not getattr(stamp, 'sec', 0):
            return now
        age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        return now - age if 0.0 <= age <= MAX_STAMP_AGE_S else now

    def on_imu(self, msg):
        if getattr(self,'imu_sources',None) is not None and self.imu_sources.active != 'hda536t':
            return
        if self.imu_yaw_input:
            return
        gz = msg.angular_velocity.z
        if not math.isfinite(gz):
            return
        a = self._attitude
        if a is not None and 0.0 <= self.clock() - a[0] <= TILT_ATTACH_S:
            self.mission.on_imu(self.msg_time(msg), gz, roll=math.radians(a[1]), pitch=math.radians(a[2]))
        else:
            self.mission.on_imu(self.msg_time(msg), gz)

    def on_attitude(self, msg):
        self.on_source_attitude(msg,'hda536t')

    def on_imu_status(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data,dict):self.imu_sources.driver['hda536t'] = data
        except (ValueError,TypeError):
            pass

    def on_source_attitude(self, msg, source):
        """记下姿态给状态用；融合用 IMU 航向角时再把航向角、横滚俯仰（倾斜补偿）和陀螺交给融合。
        orientation_covariance[0] = -1 表示消息不带姿态（ROS 约定）：不显示，交给融合时也只给陀螺。"""
        try:
            has_orientation = float(msg.orientation_covariance[0]) >= 0
        except (TypeError, IndexError, ValueError, AttributeError):
            has_orientation = True
        att = None
        if has_orientation:
            o = msg.orientation
            att = attitude_deg(o.x, o.y, o.z, o.w)
        if not math.isfinite(msg.angular_velocity.z):return
        sources = getattr(self,'imu_sources',None)
        if sources is not None:
            stamp = getattr(getattr(msg,'header',None),'stamp',None)
            stamp_ns = stamp.sec*1000000000+stamp.nanosec if stamp is not None else 0
            # msg_time 对过旧时间戳会回退到当前时间；切换屏障直接看原始时间戳。
            if stamp_ns and stamp_ns < getattr(self,'_imu_switch_stamp_ns',0):return
            sources.observe(source,self.msg_time(msg),att,msg.angular_velocity.z)
            if source != sources.active or self.msg_time(msg) < getattr(self,'_imu_switch_at',-math.inf):
                return
        if att is not None:
            self._attitude = (self.msg_time(msg),*att)
            if getattr(self,'pub_selected_imu',None) is not None:
                selected = copy.deepcopy(msg)
                selected.header.frame_id = 'selected_'+source
                self.pub_selected_imu.publish(selected)
        if self.imu_yaw_input or source == 'imu762':
            gz = msg.angular_velocity.z
            if math.isfinite(gz):
                if att is None:
                    self.mission.on_imu(self.msg_time(msg), gz, None)
                elif self.imu_yaw_input:
                    self.mission.on_imu(self.msg_time(msg), gz, math.radians(att[2]),
                                        roll=math.radians(att[0]), pitch=math.radians(att[1]))
                else:
                    self.mission.on_imu(self.msg_time(msg),gz,roll=math.radians(att[0]),pitch=math.radians(att[1]))

    def poll_imu762(self):
        if getattr(self,'_imu762_reader',None) is None:return
        version,stamp,frame,driver = self._imu762_reader.snapshot()
        self.imu_sources.driver['imu762'] = driver
        if frame is not None:
            driver['temp_c'] = frame.values.get('temperature_c')
        if version == self._imu762_version:return
        self._imu762_version = version
        if frame is None or stamp is None or not 0 <= self.clock()-stamp <= .2:return
        from rion_protocol.imu762_codec import to_ros
        converted = to_ros(frame)
        if 'angular_velocity' not in converted or 'orientation' not in converted:return
        msg = Imu()
        # 使用采集时刻，而不是发布时刻，防止积压数据被当成新鲜数据。
        ns = self.get_clock().now().nanoseconds-int((self.clock()-stamp)*1e9)
        msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(ns,1000000000)
        msg.header.frame_id = 'imu762_link'
        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z = converted['angular_velocity']
        msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w = converted['orientation']
        if 'linear_acceleration' in converted:
            msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z = converted['linear_acceleration']
        else:msg.linear_acceleration_covariance[0] = -1.
        self.pub_imu762.publish(msg)
        self.on_source_attitude(msg,'imu762')

    def select_imu(self, cmd):
        request_id = cmd.get('id')
        if request_id in self.imu_sources.results:return self.imu_sources.results[request_id]
        try:
            if not request_id:raise ValueError('IMU 切换必须带指令 id')
            blocked = self.imu_switch_blocker()
            if blocked:raise ValueError(blocked)
            source = cmd.get('source')
            if self.imu_sources.select(source,self.clock()):
                self._imu_switch_at = self.clock()
                self._imu_switch_stamp_ns = self.get_clock().now().nanoseconds
                self.mission.fusion.reset_imu_source()
                self.mission.pose = None
                self.mission._imu = None
                from line_cascade import ImuAlignment
                self.mission._imu_alignment = ImuAlignment()
                self._attitude = None
                self.attitude_controls.select_imu_source(source)
            result = True,'已选择 '+('IMU762' if source=='imu762' else 'HDA536T')+'，等待定位重新稳定'
        except (ValueError,TypeError,OSError) as exc:
            result = False,'IMU 切换未完成：'+str(exc)
        if request_id:
            self.imu_sources.results[request_id] = result
            if len(self.imu_sources.results)>32:del self.imu_sources.results[next(iter(self.imu_sources.results))]
        return result

    def imu_switch_blocker(self):
        motion = self.motion.state if self.motion.fresh() else None
        blocked = self.attitude_controls.blocker(self.clock(),motion)
        if blocked:return blocked
        if not motion or motion.get('mode') != 'IDLE':return '请先锁定手动控制并等待车辆空闲'
        return None

    def attitude_status(self):
        a = self._attitude
        if a is None or not 0.0 <= self.clock() - a[0] <= ATTITUDE_MAX_AGE_S:
            return None
        # imu 组件跟着底盘选：差速车（Gen5）是 HDA536T 驱动发的传感器姿态，旧车是 imu_filter_madgwick
        roll, pitch = self.mission.fusion.tilt_angles(math.radians(a[1]), math.radians(a[2]))
        return {'roll_deg': round(math.degrees(roll), 2), 'pitch_deg': round(math.degrees(pitch), 2), 'yaw_deg': round(a[3], 2),
                'source': getattr(getattr(self,'imu_sources',None),'active','hda536t')
                          if getattr(self.mission,'differential',False) else 'imu_madgwick'}

    def on_odom(self, msg):
        v, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if math.isfinite(v) and math.isfinite(wz):
            if self.wheel_input is None:
                t = self.msg_time(msg)
                self.mission.on_wheel(t, v, wz)
                self.attitude_controls.observe_wheel(t, v, wz)
            else:
                self.wheel_input.legacy(self.msg_time(msg), v, wz)

    def on_wheel_motion(self, msg):
        if self.wheel_input is None:
            return
        try:
            sample = self.wheel_input.decode(json.loads(msg.data), self.clock())
            if sample is not None:
                self.mission.on_wheel_motion(sample)
                self.attitude_controls.observe_sample(sample)
        except (ValueError, TypeError, KeyError, OverflowError):
            # 坏帧既不刷新轮速时刻，也不重新打接收时间来掩盖积压。
            return

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError('not an object')
        except ValueError:
            self.last_result = dict(ok=False, message='指令格式错误', id=None)
            return
        if cmd.get('cmd') == 'imu_source':
            ok,message = self.select_imu(cmd)
        elif cmd.get('cmd') in ('tilt', 'imu_zero'):
            ok, message = self.attitude_controls.command(cmd, self.clock(),
                self.motion.state if self.motion.fresh() else None, self._attitude)
        else:
            ok, message = self.mission.command(cmd, self.clock())
        self.last_result = dict(ok=ok, message=message, id=cmd.get('id'), cmd=cmd.get('cmd'))
        self.publish_status()

    def mode(self):
        return self.motion.state.get('mode', 'UNKNOWN') if self.motion.fresh() else 'UNKNOWN'

    def control(self):
        self.poll_imu762()
        now = self.clock()
        if self.wheel_input is not None:
            sample = self.wheel_input.fallback(now)
            if sample is not None:
                self.mission.on_wheel_motion(sample)
                self.attitude_controls.observe_sample(sample)
        mode = self.mode()
        vx, wz, select = self.mission.tick(now, mode)
        if select is True:
            if self._select_at is None or now - self._select_at > SELECT_RETRY_S:
                if self.motion.select(True) is not None:
                    self._select_at = now
        elif select is False:
            self._select_at = None
            self.motion.publish(0.0, 0.0)
            self.motion.select(False)
        # 任务已结束 / 已暂停但还在渐停（mission.braking）也照常下发，不然下一拍就停发、驱动器按 3 m/s² 刹停
        sent = (self.mission.phase in ('RUNNING', 'PAUSING', 'ORIENTING') or self.mission.braking) and self.motion.active()
        if sent:
            self.motion.publish(vx, wz)
        self.publish_trace(now, mode, vx, wz, select, sent)

    def publish_trace(self, now, mode, vx, wz, select, sent):
        """这一拍的控制记录（control_trace）；出错只丢这一条，不影响开车。"""
        try:
            rec = control_trace.record(self.mission, now, mode, vx, wz, select)
            if rec is not None:
                rec['sent'] = sent
                self.pub_trace.publish(String(data=json.dumps(rec, allow_nan=False, ensure_ascii=False)))
        except Exception:
            pass

    def publish_status(self):
        st = self.mission.status()
        if isinstance(st.get('tilt'), dict):
            st['tilt'].update(self.attitude_controls.status(self.clock(),
                self.motion.state if self.motion.fresh() else None, self._attitude))
        st['motion_mode'] = self.mode()
        st['last_result'] = self.last_result
        st['attitude'] = self.attitude_status()
        st['imu_sources'] = self.imu_sources.status(self.clock(),self.imu_switch_blocker())
        self.pub_status.publish(String(data=json.dumps(st, allow_nan=False, ensure_ascii=False)))

    def destroy_node(self):
        if getattr(self,'_imu762_reader',None) is not None:self._imu762_reader.close()
        return super().destroy_node()

    def stop_robot(self):
        self.mission.command({'cmd': 'stop'}, self.clock())
        self.motion.publish(0.0, 0.0)
        self.motion.select(False)


def load_fusion_config(gnss_fusion, path):
    """标定结果（安装偏差等）写在 path（~/rtk/fusion.json），键名同 FusionConfig 字段；没有文件就用默认值。
    已取消的固定基线参数（OBSOLETE_FUSION_KEYS）丢掉不用。"""
    overrides = {}
    if os.path.exists(path):
        with open(path, encoding='utf-8') as fh:
            overrides = {k: tuple(v) if isinstance(v, list) else v for k, v in json.load(fh).items()}
    for key in OBSOLETE_FUSION_KEYS:
        overrides.pop(key, None)
    # 差速车（原地转向）按共享底盘配置；标定文件里明确写了的照用
    from runtime_config import PROFILE
    overrides.setdefault('drive_differential', PROFILE['drive']['type'] == 'differential')
    if overrides['drive_differential']:
        antenna = overrides.get('antenna_xy')
        if antenna is None or (len(antenna) == 2 and all(isinstance(v, (int, float)) and abs(v - h) < 1e-6
                                                         for v, h in zip(antenna, GEN5_ANTENNA_HAND))):
            overrides['antenna_xy'] = GEN5_ANTENNA_MEASURED
        # Gen5（HDA536T）：航向推算用 IMU 自己算的航向角（2026-09-28 用户要求）；原地转向转角大，
        # 按 3% 刻度误差加航向不确定度，测向按 20 ms 时间同步误差随转速降权。
        # IMU 始终提供连续转动；可信双天线慢校直行漂移，明显转弯后及时收敛。
        overrides.setdefault('yaw_input', 'angle')
        overrides.setdefault('continuous_output', True)
        overrides.setdefault('wheel_increment_enabled', True)
        overrides.setdefault('sensor_failover_enabled', True)
        overrides.setdefault('wheel_scale_auto', True)
        overrides.setdefault('heading_sync_s', 0.02)
        # IMU 航向权重加大（2026-10-01 用户：「imu 的航向角还是非常稳的」「imu 的权重可以加大」）：航向过程噪声
        # 0.1 → 0.03°/√s，直走、画圆时测向的抖动、跳动多被 IMU 压住（实测空旷处 3 圈 1078°，IMU 航向角与双天线测向
        # 只差 0.4°）；测向先检查转动增量一致性，可信观测连续校准，稳定本身不触发航向硬重定。
        # 转角刻度不确定度保持 3%：慢慢画圆时 IMU 刻度很准，原地转（0.8 rad/s）却不是——当天 6 次测向可信的原地转，
        # IMU 与测向差 0.05%–1.3%（最多 2.2°）。曾改成 0.5%：转完 0–2 s 融合航向与测向之差 p95 矩形 0.33 → 0.59°、
        # 另一次 0.55 → 1.78°（转完要 1–2 s 才跟回测向）；3% 时 0.45 / 0.67°，直走的抖动照样少
        overrides.setdefault('yaw_scale_sigma', 0.03)
        overrides.setdefault('yaw_walk_deg_sqrt_s', 0.03)
        # 卫星位置不可信（双天线位置互检不过）或固定解中断（平板中转断流、浮点、单点）时只靠 IMU + 轮速推算走、不降速
        # （2026-10-06 晚用户：「即使浮点都可以靠 imu 继续」）；满 8 s 先停喷、再走 1 s 停车（10-07 用户），恢复固定解后接回
        overrides.setdefault('rtk_hold_s', RTK_LOST_PAINT_STOP_S + RTK_LOST_STOP_DELAY_S)
        overrides.setdefault('hold_budget_enabled', True)
        # 用户指定 500 ms：短时转发抖动不进入完整中断停车预算，与固定解超时窗口对齐。
        overrides.setdefault('hold_watch_s', 0.5)
        # 两根天线互检偏差超过 2 cm 就逐步少信卫星（空旷处 p95 1.4 cm 不受影响）；推算维持后每秒最多挪 4 cm 接回
        # （离线验证：靠楼那圈的卫星误差叠到空旷那圈上，横向偏差 p95 2.8 → 2.5 cm、最大 10.6 → 3.6 cm）
        overrides.setdefault('dual_weight_m', 0.02)
        overrides.setdefault('rejoin_rate_mps', 0.04)
        # 2026-10-02 用户：「修复融合算法，让误差更小」。全链路仿真（车上代码 + 按 10-02 实车校准的底盘与传感器）里，
        # 水泵横偏主要来自融合横向位置跟 RTK 跟得太慢：航向有一点误差，推算就把位置带偏一截（测向偏 1° 整条线约偏 1.1 cm）。
        # 在线学双天线测向的固定偏差并扣掉
        overrides.setdefault('heading_bias_learning', True)
        # RTK 固定解按逐帧抖动（约 1–2 mm）给观测噪声，不按接收机报的绝对精度 8 mm：RTK 正常时横向位置紧跟 RTK，
        # 航向误差就几乎不影响水泵横向位置（水泵与主天线前后对齐）；RTK 断了时推算误差怎么涨不变（推算维持照旧 5 s）。
        # 离群的固定解相对更离群，软门限降权更狠（仿真：两天线一起横跳 8 cm 1 s，水泵最大横偏 2.66 → 2.19 cm）
        overrides.setdefault('fix_floor_m', 0.002)
        overrides.setdefault('fix_sigma_scale', 0.25)
        # 但按实时估的 RTK 抖动（相邻两帧固定解相对推算的跳动）不小于它的 2 倍：靠楼多路径时自动少信 RTK
        # （真实回放 10-01 11:42 靠楼：固定 2 mm 时位置每 50 ms 额外跳动最大 1.8 cm，自适应后 0.97 cm，原融合 0.64 cm）
        overrides.setdefault('fix_jitter_k', 2.0)
        # 控制输出跟融合状态的平滑 0.25 → 0.10 s：横向纠偏少 0.15 s 滞后
        overrides.setdefault('output_position_tau_s', 0.10)
    return gnss_fusion.FusionConfig(**overrides)


def single_antenna_config(config):
    """双天线配置 → 单天线配置：杠杆臂换成右天线位置，按行驶轨迹定车头。差速车（Gen5）才有；别的车返回 None。
    单天线接收机的 $INS.NAVI 已经是它自己按杆长做完倾斜补偿的坐标（2026-10-07 用户：「输出的是倾斜后的补偿坐标，
    只需要告诉他杆长」，杆长在接收机上设），车端不再补，免得补两次；车头照旧用 IMU 航向角 + 行驶轨迹修正的偏移。"""
    if not config.drive_differential:
        return None
    return dataclasses.replace(config, antenna_xy=SINGLE_ANTENNA_XY, tilt_compensation_enabled=False,
                               course_init_distance_m=SINGLE_COURSE_INIT_M,
                               course_align_distance_m=SINGLE_ALIGN_M,
                               bias_initial_sigma_deg_s=SINGLE_BIAS_SIGMA_DEG_S,
                               # 没有卫星地速：停稳只看轮子（2026-10-07 用户：「只要轮子不动就行」）
                               ground_speed_still=False)


def main():
    import gnss_fusion   # 融合模块（协作开发，接口见 docs/GNSS_FUSION.md）
    home = os.path.expanduser('~')
    config = load_fusion_config(gnss_fusion, os.path.join(home, 'rtk', 'fusion.json'))
    fusion = gnss_fusion.GnssFusion(config)
    mission = LineMission(fusion, gnss_fusion, single_config=single_antenna_config(config),
                          state_path=os.path.join(home, 'rtk', 'line_state.json'),
                          log_dir=os.path.join(home, 'rtk', 'runs'),
                          route_path=os.path.join(home, 'rtk', 'route.json'))
    watchdog.start("ros")
    rclpy.init()
    node = LineNode(mission, fusion_path=os.path.join(home, 'rtk', 'fusion.json'))
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
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
