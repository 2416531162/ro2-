"""N10P 108-byte dual-echo decoding; ROS/Qt-independent replayable pipeline.

Wire reference: Lslidar/Lslidar_ROS2_driver, M10P/N10P branch.
No invented angles, interpolation of missing returns, or temporal smoothing.
"""
import math

FRAME = 108
BINS = 720
RANGE_MIN = 0.15
RANGE_MAX = 12.0
# Standard robot/ROS planar convention used by every downstream consumer:
# +X/front = 0 deg, +Y/left = 90 deg, rear = 180 deg, right = 270 deg.
DIRECTION_CENTERS_DEG = (
    ('front', 0),
    ('left', 90),
    ('back', 180),
    ('right', 270),
)


def be16(packet, offset):
    return int.from_bytes(packet[offset:offset + 2], 'big')


class N10PDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bytes_received = self.frames = self.crc_errors = 0
        self.angle_errors = self.discarded_bytes = self.echo_fallbacks = 0

    def feed(self, data):
        self.bytes_received += len(data)
        self.buffer.extend(data)
        while len(self.buffer) >= 2:
            offset = self.buffer.find(b'\xa5\x5a')
            if offset < 0:
                keep = 1 if self.buffer[-1] == 0xa5 else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b''
                return
            if offset:
                self.discarded_bytes += offset
                del self.buffer[:offset]
            if len(self.buffer) < FRAME:
                return
            packet = bytes(self.buffer[:FRAME])
            if packet[2] != FRAME or packet[3] != 16 or sum(packet[:-1]) & 255 != packet[-1]:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]  # search again, not a blind 108-byte skip
                continue
            del self.buffer[:FRAME]
            start, end = be16(packet, 5) / 100, be16(packet, 105) / 100
            span = (end - start) % 360
            if not (0 <= start <= 360 and 0 <= end <= 360 and 0.2 < span <= 40):
                self.angle_errors += 1
                continue
            self.frames += 1
            points = []
            for i in range(16):
                off = 7 + i * 6
                first, second = be16(packet, off) / 1000, be16(packet, off + 3) / 1000
                intensity = packet[off + 2]
                # Preserve first-return semantics. Second echo is used only when
                # first has no valid measurement; zero/FFFF never become hits.
                distance = first
                if not RANGE_MIN <= first <= RANGE_MAX:
                    if RANGE_MIN <= second <= RANGE_MAX:
                        distance, intensity = second, packet[off + 5]
                        self.echo_fallbacks += 1
                    else:
                        # REP-117: -inf = 有回波但近于量程下限(几乎总是车身自己的
                        # 结构件,雷达在这个方向上被挡住),+inf = 完全没有回波。
                        # 两者合并成 +inf 时,下游无法区分「被车身挡住」和
                        # 「前方可能有吸光物体」,只能都按未知处理。
                        near = 0 < first < RANGE_MIN or 0 < second < RANGE_MIN
                        distance, intensity = (-math.inf if near else math.inf), 0
                points.append(((start + span * i / 15) % 360, distance, intensity))
            yield points


def _rank(distance):
    """同一格多个采样的取舍:真实回波(近者优先) > 太近(-inf) > 无回波(+inf)。"""
    if math.isfinite(distance):
        return (0, distance)
    return (1, 0.0) if distance < 0 else (2, 0.0)


class SweepAssembler:
    def __init__(self):
        self.last_angle = self.started = self.last_packet = None
        self.bins = {}
        self.sweeps = self.gap_resets = 0

    def add(self, points, now):
        scans = []
        if self.last_packet is not None and now - self.last_packet > 0.3:
            self.last_angle = self.started = None
            self.bins = {}
            self.gap_resets += 1
        self.last_packet = now
        for angle, distance, intensity in points:
            if self.last_angle is not None and self.last_angle > 300 and angle < 60:
                if self.started is not None:
                    duration = now - self.started
                    if 0.03 <= duration <= 0.3:
                        # LaserScan ranges cannot distinguish an empty angular
                        # bin from a sampled ray with no echo. Reserve -1 in
                        # intensities for the former; N10P quality is 0..255.
                        ranges, intensities = [math.inf] * BINS, [-1.0] * BINS
                        sampled = [False] * BINS
                        for key, (r, quality) in self.bins.items():
                            ranges[key], intensities[key] = r, float(quality)
                            sampled[key] = True
                        self.sweeps += 1
                        scans.append({'ranges': ranges, 'intensities': intensities,
                                      'sampled': sampled,
                                      'scan_time': duration, 'received': now})
                self.bins = {}
                self.started = now
            self.last_angle = angle
            if self.started is not None:
                key = round(((360 - angle) % 360) * 2) % BINS
                old = self.bins.get(key)
                if old is None or _rank(distance) < _rank(old[0]):
                    self.bins[key] = (distance, intensity)
        return scans


def scan_payload(ranges, range_min, range_max, angle_min=0.0,
                 angle_increment=math.pi / 360, scan_time=0.0, age=0.0):
    sectors = {'front': [], 'left': [], 'back': [], 'right': []}
    valid = []
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        valid.append(distance)
        angle = math.degrees(angle_min + i * angle_increment) % 360
        for name, center in DIRECTION_CENTERS_DEG:
            if abs((angle - center + 180) % 360 - 180) <= 15:
                sectors[name].append(distance)
    return dict(ranges=list(ranges), range_min=range_min, range_max=range_max,
                angle_min=angle_min, angle_increment=angle_increment,
                scan_time=scan_time, source_age=max(0.0, age),
                min=min(valid, default=math.inf), count=len(valid), total=len(ranges),
                **{k: min(v, default=math.inf) for k, v in sectors.items()})


def project_point(angle, distance, cx, cy, scale):
    # ROS +Y is left: 90 degrees must agree with the left distance card.
    return cx - math.sin(angle) * distance * scale, cy - math.cos(angle) * distance * scale


def scan_coverage(ranges, sampled, bin_deg=0.5, min_gap_deg=3.0):
    """Preserve acquisition provenance without changing LaserScan ranges.

    Summarize contiguous invalid sectors in published angular coordinates.
    Empty bins are not the same as samples received with no usable echo.
    """
    if len(ranges) != len(sampled):
        raise ValueError("ranges and sampled must have equal lengths")
    n = len(ranges)
    def kind(i):
        if not sampled[i]:
            return 'unsampled'
        r = ranges[i]
        if math.isfinite(r) and RANGE_MIN <= r <= RANGE_MAX:
            return 'valid'
        return 'too_near' if r == -math.inf else 'no_valid_echo'
    kinds = [kind(i) for i in range(n)]
    counts = {name: kinds.count(name) for name in
              ('valid', 'unsampled', 'too_near', 'no_valid_echo')}
    if not n:
        return dict(counts=counts, gaps=[])
    # Start immediately after a good return so a gap crossing zero stays whole.
    start = next((i + 1 for i in range(n) if kinds[i] == 'valid'), 0) % n
    groups, group = [], []
    for offset in range(n):
        i = (start + offset) % n
        if kinds[i] != 'valid':
            group.append(i)
        elif group:
            groups.append(group)
            group = []
    if group:
        groups.append(group)
    gaps = []
    for group in groups:
        if len(group) * bin_deg < min_gap_deg:
            continue
        gaps.append(dict(start_deg=round((group[0]*bin_deg + 180) % 360 - 180, 2),
                         end_deg=round((group[-1]*bin_deg + 180) % 360 - 180, 2),
                         width_deg=round(len(group)*bin_deg, 2),
                         unsampled=sum(kinds[i] == 'unsampled' for i in group),
                         too_near=sum(kinds[i] == 'too_near' for i in group),
                         no_valid_echo=sum(kinds[i] == 'no_valid_echo' for i in group)))
    return dict(counts=counts, gaps=gaps)
