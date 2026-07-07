# Auto-extracted from scripts/base_sensor_visual_server.py (verbatim).
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path as FsPath
from typing import Any, Dict, Iterable, List, Optional, Tuple
import urllib.error
import urllib.request

from ..ros.helpers import (
    SENSOR_TYPE_NAMES,
    IMPACT_TYPE_NAMES,
    SLAMWARE_MOVE_OPTION_WITH_YAW,
    SLAMWARE_MOVE_OPTION_KEY_POINTS,
    finite_or_none,
    yaw_from_quaternion,
    quaternion_from_yaw,
    normalize_angle_rad,
    now_iso,
    make_reliable_qos,
)


class SharedState:
    def __init__(self, max_track: int) -> None:
        self.lock = threading.RLock()
        self.started_at = time.time()
        self.scan: Optional[Dict[str, Any]] = None
        self.map: Optional[Dict[str, Any]] = None
        self.odom: Optional[Dict[str, Any]] = None
        self.sensors: Optional[Dict[str, Any]] = None
        self.point_cloud: Optional[Dict[str, Any]] = None
        self.global_plan_path: Optional[Dict[str, Any]] = None
        self.robot_basic_state: Optional[Dict[str, Any]] = None
        self.slamware_state: Optional[Dict[str, Any]] = None
        self.last_relocalization_command: Optional[Dict[str, Any]] = None
        self.arm_task_status: Optional[Dict[str, Any]] = None
        self.last_arm_task_command: Optional[Dict[str, Any]] = None
        self.last_navigation_command: Optional[Dict[str, Any]] = None
        self.fault_snapshots: deque[Dict[str, Any]] = deque(maxlen=120)
        self.track: deque[Dict[str, float]] = deque(maxlen=max_track)
        self.seq = {
            "scan": 0,
            "map": 0,
            "odom": 0,
            "sensors": 0,
            "point_cloud": 0,
            "global_plan_path": 0,
            "robot_basic_state": 0,
            "slamware_state": 0,
            "relocalization_command": 0,
            "navigation_command": 0,
            "arm_task_status": 0,
            "arm_task_command": 0,
            "fault_snapshot": 0,
        }

    def snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self.lock:
            return {
                "ok": True,
                "server_time": now,
                "uptime_s": round(now - self.started_at, 3),
                "seq": dict(self.seq),
                "scan": self.scan,
                "map": self.map,
                "odom": self.odom,
                "track": list(self.track),
                "sensors": self.sensors,
                "point_cloud": self.point_cloud,
                "navigation": {
                    "global_plan_path": self.global_plan_path,
                    "robot_basic_state": self.robot_basic_state,
                    "slamware_state": self.slamware_state,
                    "last_relocalization_command": self.last_relocalization_command,
                    "last_command": self.last_navigation_command,
                },
                "arm_control": {
                    "last_status": self.arm_task_status,
                    "last_command": self.last_arm_task_command,
                },
                "fault_snapshots": list(self.fault_snapshots)[-40:],
                "freshness_s": {
                    "scan": self._age(self.scan, now),
                    "map": self._age(self.map, now),
                    "odom": self._age(self.odom, now),
                    "sensors": self._age(self.sensors, now),
                    "point_cloud": self._age(self.point_cloud, now),
                    "global_plan_path": self._age(self.global_plan_path, now),
                    "robot_basic_state": self._age(self.robot_basic_state, now),
                    "slamware_state": self._age(self.slamware_state, now),
                    "arm_task_status": self._age(self.arm_task_status, now),
                },
            }

    @staticmethod
    def _age(item: Optional[Dict[str, Any]], now: float) -> Optional[float]:
        if not item:
            return None
        stamp = item.get("received_at")
        if stamp is None:
            return None
        return round(now - float(stamp), 3)
