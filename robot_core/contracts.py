"""Transport-independent perception and local-pose inputs."""
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class ScanFrame:
    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp: float = 0.0
    sampled: Optional[Sequence[bool]] = None


@dataclass(frozen=True)
class LocalPose:
    stamp: float
    x: float
    y: float
    yaw: float
    frame: str = 'odom'
    child_frame: str = 'base_link'
