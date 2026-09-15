"""N10P 108-byte dual-echo decoding; ROS/Qt-independent replayable pipeline.

Wire reference: Lslidar/Lslidar_ROS2_driver, M10P/N10P branch.
No invented angles, interpolation of missing returns, or temporal smoothing.
"""
import math

FRAME = 108
BINS = 720
RANGE_MIN = 0.15
RANGE_MAX = 12.0


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
                        distance, intensity = math.inf, 0
                points.append(((start + span * i / 15) % 360, distance, intensity))
            yield points


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
                        ranges, intensities = [math.inf] * BINS, [0.0] * BINS
                        for key, (r, quality) in self.bins.items():
                            ranges[key], intensities[key] = r, float(quality)
                        self.sweeps += 1
                        scans.append({'ranges': ranges, 'intensities': intensities,
                                      'scan_time': duration, 'received': now})
                self.bins = {}
                self.started = now
            self.last_angle = angle
            if self.started is not None:
                key = round(((360 - angle) % 360) * 2) % BINS
                old = self.bins.get(key)
                if old is None or distance < old[0]:
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
        for name, center in [('front', 0), ('left', 90), ('back', 180), ('right', 270)]:
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
