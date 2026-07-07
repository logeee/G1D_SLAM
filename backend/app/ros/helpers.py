# Auto-extracted from scripts/base_sensor_visual_server.py (verbatim).
# Constants + pure helpers + QoS factory. No ROS message imports here so the
# state/* modules can depend on it without pulling in the ROS message types.
from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


SENSOR_TYPE_NAMES = {
    -1: "UNKNOWN",
    0: "BUMPER",
    1: "CLIFF",
    2: "SONAR",
    3: "DEPTH_CAMERA",
    4: "WALL_SENSOR",
    5: "MAG_TAPE_DETECTOR",
}

IMPACT_TYPE_NAMES = {
    -1: "UNKNOWN",
    0: "DIGITAL",
    1: "ANALOG",
}

SLAMWARE_MOVE_OPTION_WITH_YAW = 32
SLAMWARE_MOVE_OPTION_KEY_POINTS = 8


def finite_or_none(value: Any, digits: Optional[int] = None) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return round(f, digits) if digits is not None else f


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Dict[str, float]:
    half = float(yaw) / 2.0
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def normalize_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def make_reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
