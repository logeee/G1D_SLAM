#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight web dashboard for Unitree base sensors.

The server subscribes to existing ROS2 topics and exposes a read-only browser
view.  It is intentionally dependency-light: rclpy + Python stdlib only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import threading
import time
import uuid
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from slamware_ros_sdk.msg import (
    BasicSensorValueDataArray,
    CancelActionRequest,
    ClearMapRequest,
    LocalizationMovement,
    MapKind,
    MoveToLocationsRequest,
    RecoverLocalizationRequest,
    RobotBasicState,
    SetMapLocalizationRequest,
    SetMapUpdateRequest,
    SyncMapRequest,
)
from slamware_ros_sdk.srv import SyncGetStcm, SyncSetStcm
from std_msgs.msg import String


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


class SavedPointStore:
    def __init__(self, path: str) -> None:
        self.path = FsPath(path)
        self.lock = threading.RLock()
        self.data: Dict[str, Any] = {"version": 1, "points": []}
        self.load()

    def load(self) -> None:
        with self.lock:
            if not self.path.exists():
                self.data = {"version": 1, "points": []}
                return
            try:
                loaded = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                loaded = {"version": 1, "points": []}
            points = loaded.get("points") if isinstance(loaded, dict) else []
            if not isinstance(points, list):
                points = []
            version = 1
            if isinstance(loaded, dict):
                try:
                    version = int(loaded.get("version", 1))
                except (TypeError, ValueError):
                    version = 1
            self.data = {"version": version, "points": []}
            for item in points:
                point = self.normalize_point(item, existing=None, require_xy=True)
                if point["ok"]:
                    self.data["points"].append(point["point"])

    def list_payload(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "ok": True,
                "path": str(self.path),
                "count": len(self.data["points"]),
                "points": [dict(point) for point in self.data["points"]],
            }

    def record_current(self, odom: Optional[Dict[str, Any]], payload: Dict[str, Any]) -> Dict[str, Any]:
        if not odom or odom.get("x") is None or odom.get("y") is None:
            return {"ok": False, "error": "current odom is unavailable"}
        now = now_iso()
        name = str(payload.get("name") or "").strip() or f"Point {now.replace('T', ' ')}"
        point_payload = {
            "name": name,
            "x": odom.get("x"),
            "y": odom.get("y"),
            "yaw": odom.get("yaw", 0.0),
            "note": payload.get("note", ""),
            "actions": payload.get("actions", []),
        }
        normalized = self.normalize_point(point_payload, existing=None, require_xy=True)
        if not normalized["ok"]:
            return normalized
        point = normalized["point"]
        point["id"] = self.new_id()
        point["created_at"] = now
        point["updated_at"] = now
        point["source"] = "current_odom"
        with self.lock:
            self.data["points"].append(point)
            self.write_locked()
        return {"ok": True, "point": point}

    def upsert(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        point_id = str(payload.get("id") or "").strip()
        with self.lock:
            existing = self.find_locked(point_id) if point_id else None
            normalized = self.normalize_point(payload, existing=existing, require_xy=True)
            if not normalized["ok"]:
                return normalized
            point = normalized["point"]
            now = now_iso()
            if existing:
                point["id"] = existing["id"]
                point["created_at"] = existing.get("created_at") or now
                point["updated_at"] = now
                point["source"] = existing.get("source") or "manual"
                self.data["points"] = [point if item.get("id") == point["id"] else item for item in self.data["points"]]
            else:
                point["id"] = self.new_id()
                point["created_at"] = now
                point["updated_at"] = now
                point["source"] = str(payload.get("source") or "manual")
                self.data["points"].append(point)
            self.write_locked()
        return {"ok": True, "point": point}

    def delete(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        point_id = str(payload.get("id") or "").strip()
        if not point_id:
            return {"ok": False, "error": "missing point id"}
        with self.lock:
            before = len(self.data["points"])
            self.data["points"] = [point for point in self.data["points"] if point.get("id") != point_id]
            if len(self.data["points"]) == before:
                return {"ok": False, "error": "point not found", "id": point_id}
            self.write_locked()
        return {"ok": True, "deleted_id": point_id}

    def normalize_point(
        self,
        payload: Any,
        existing: Optional[Dict[str, Any]],
        require_xy: bool,
    ) -> Dict[str, Any]:
        if not isinstance(payload, dict):
            return {"ok": False, "error": "point payload must be an object"}
        x = finite_or_none(payload.get("x"), 5)
        y = finite_or_none(payload.get("y"), 5)
        if require_xy and (x is None or y is None):
            return {"ok": False, "error": "point x/y are required"}
        yaw = self.resolve_yaw(payload, existing)
        if yaw is None:
            return {"ok": False, "error": "invalid yaw/yaw_deg"}
        name = str(payload.get("name") or (existing or {}).get("name") or "Point").strip() or "Point"
        note = str(payload.get("note") if payload.get("note") is not None else (existing or {}).get("note", ""))
        actions = payload.get("actions", (existing or {}).get("actions", []))
        if not isinstance(actions, list):
            return {"ok": False, "error": "actions must be a JSON list"}
        point = {
            "id": str((existing or {}).get("id") or payload.get("id") or ""),
            "name": name[:80],
            "x": x,
            "y": y,
            "yaw": round(yaw, 6),
            "yaw_deg": round(math.degrees(yaw), 3),
            "note": note[:500],
            "actions": actions,
        }
        for key in ("created_at", "updated_at", "source"):
            if existing and existing.get(key) is not None:
                point[key] = existing[key]
            elif payload.get(key) is not None:
                point[key] = payload[key]
        return {"ok": True, "point": point}

    @staticmethod
    def resolve_yaw(payload: Dict[str, Any], existing: Optional[Dict[str, Any]]) -> Optional[float]:
        raw_yaw = payload.get("yaw")
        raw_yaw_deg = payload.get("yaw_deg")
        if raw_yaw is not None:
            yaw = finite_or_none(raw_yaw)
            return normalize_angle_rad(float(yaw)) if yaw is not None else None
        if raw_yaw_deg is not None:
            yaw_deg = finite_or_none(raw_yaw_deg)
            return normalize_angle_rad(math.radians(float(yaw_deg))) if yaw_deg is not None else None
        if existing and existing.get("yaw") is not None:
            yaw = finite_or_none(existing.get("yaw"))
            return normalize_angle_rad(float(yaw)) if yaw is not None else None
        return 0.0

    def find_locked(self, point_id: str) -> Optional[Dict[str, Any]]:
        for point in self.data["points"]:
            if point.get("id") == point_id:
                return point
        return None

    def write_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp_path, self.path)

    @staticmethod
    def new_id() -> str:
        return "pt_" + uuid.uuid4().hex[:12]


class RelocalizationAnchorStore:
    def __init__(self, path: str) -> None:
        self.path = FsPath(path)
        self.lock = threading.RLock()
        self.data: Dict[str, Any] = {"version": 1, "anchor": None}
        self.load()

    def load(self) -> None:
        with self.lock:
            if not self.path.exists():
                self.data = {"version": 1, "anchor": None}
                return
            try:
                loaded = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                loaded = {"version": 1, "anchor": None}
            anchor = loaded.get("anchor") if isinstance(loaded, dict) else None
            self.data = {
                "version": 1,
                "anchor": self.normalize_anchor(anchor) if isinstance(anchor, dict) else None,
            }

    def normalize_anchor(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        x = finite_or_none(payload.get("x"), 5)
        y = finite_or_none(payload.get("y"), 5)
        yaw = finite_or_none(payload.get("yaw"))
        if x is None or y is None:
            return None
        if yaw is None:
            yaw_deg = finite_or_none(payload.get("yaw_deg"))
            yaw = math.radians(float(yaw_deg)) if yaw_deg is not None else 0.0
        yaw = normalize_angle_rad(float(yaw))
        return {
            "x": x,
            "y": y,
            "z": finite_or_none(payload.get("z"), 5) or 0.0,
            "yaw": finite_or_none(yaw, 6),
            "yaw_deg": finite_or_none(math.degrees(yaw), 3),
            "frame_id": str(payload.get("frame_id") or ""),
            "child_frame_id": str(payload.get("child_frame_id") or ""),
            "source": str(payload.get("source") or "manual"),
            "saved_at": str(payload.get("saved_at") or now_iso()),
        }

    def list_payload(self, current_odom: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self.lock:
            return {
                "ok": True,
                "path": str(self.path),
                "anchor": dict(self.data["anchor"]) if self.data.get("anchor") else None,
                "current_odom": current_odom,
            }

    def save_from_odom(self, odom: Optional[Dict[str, Any]], source: str = "manual") -> Dict[str, Any]:
        if not odom or odom.get("x") is None or odom.get("y") is None:
            return {"ok": False, "error": "current odom is unavailable"}
        payload = dict(odom)
        payload["source"] = source
        payload["saved_at"] = now_iso()
        anchor = self.normalize_anchor(payload)
        if not anchor:
            return {"ok": False, "error": "invalid odom for relocalization anchor", "odom": odom}
        with self.lock:
            self.data = {"version": 1, "anchor": anchor}
            self.write_locked()
        return {"ok": True, "anchor": anchor, "path": str(self.path)}

    def get_anchor(self) -> Optional[Dict[str, Any]]:
        with self.lock:
            return dict(self.data["anchor"]) if self.data.get("anchor") else None

    def write_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp_path, self.path)


class FaultSnapshotLogger:
    CLOSE_THRESHOLDS_M = (0.45, 0.60, 0.80, 1.00)

    def __init__(self, state: SharedState, log_path: str, max_recent_errors: int = 12) -> None:
        self.state = state
        self.log_path = FsPath(log_path)
        self.max_recent_errors = max(0, int(max_recent_errors))
        self.lock = threading.RLock()
        self.last_by_key: Dict[str, float] = {}

    def list_payload(self) -> Dict[str, Any]:
        with self.state.lock:
            snapshots = list(self.state.fault_snapshots)
        return {
            "ok": True,
            "count": len(snapshots),
            "log_path": str(self.log_path),
            "snapshots": snapshots,
        }

    def clear(self) -> Dict[str, Any]:
        with self.state.lock:
            self.state.fault_snapshots.clear()
        return {"ok": True, "cleared": True, "log_path": str(self.log_path)}

    def capture_manual(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        reason = str(payload.get("reason") or "manual_fault_snapshot").strip() or "manual_fault_snapshot"
        extra = dict(payload)
        extra.pop("reason", None)
        snapshot = self.capture(reason, extra=extra, force=True)
        return {"ok": True, "snapshot": snapshot, "log_path": str(self.log_path)}

    def capture(
        self,
        reason: str,
        extra: Optional[Dict[str, Any]] = None,
        throttle_key: Optional[str] = None,
        min_interval_s: float = 0.0,
        force: bool = False,
    ) -> Dict[str, Any]:
        now = time.time()
        key = throttle_key or reason
        with self.lock:
            if not force and min_interval_s > 0:
                last = self.last_by_key.get(key)
                if last is not None and now - last < min_interval_s:
                    with self.state.lock:
                        if self.state.fault_snapshots:
                            return self.state.fault_snapshots[-1]
                    return {"ok": False, "skipped": True, "reason": reason}
                self.last_by_key[key] = now
        snapshot = self.build_snapshot(reason, extra or {}, now)
        with self.state.lock:
            self.state.seq["fault_snapshot"] += 1
            snapshot["seq"] = self.state.seq["fault_snapshot"]
            self.state.fault_snapshots.append(snapshot)
        self.append_log_line(snapshot)
        return snapshot

    def build_snapshot(self, reason: str, extra: Dict[str, Any], now: float) -> Dict[str, Any]:
        with self.state.lock:
            scan = self.state.scan
            odom = self.state.odom
            sensors = self.state.sensors
            plan = self.state.global_plan_path
            command = self.state.last_navigation_command
            robot_basic_state = self.state.robot_basic_state
            slamware_state = self.state.slamware_state
            seq = dict(self.state.seq)
            track_tail = list(self.state.track)[-20:]

        snapshot = {
            "ok": True,
            "reason": str(reason),
            "captured_at": now_iso(),
            "captured_time": round(now, 3),
            "seq": None,
            "state_seq": seq,
            "extra": self.safe_json(extra),
            "odom": self.compact_odom(odom),
            "goal": self.compute_goal_summary(command, odom, extra),
            "navigation": {
                "last_command": self.compact_command(command, now),
                "global_plan_path": self.compact_plan(plan, now),
                "robot_basic_state": self.compact_timed(robot_basic_state, now),
                "slamware_state": self.compact_timed(slamware_state, now),
            },
            "scan": self.compact_scan(scan, now),
            "sensors": self.compact_sensors(sensors, now),
            "track_tail": [
                {
                    "x": finite_or_none(p.get("x"), 4),
                    "y": finite_or_none(p.get("y"), 4),
                    "yaw_deg": finite_or_none(math.degrees(float(p.get("yaw", 0.0))), 2),
                    "age_s": finite_or_none(now - float(p.get("t", now)), 2),
                }
                for p in track_tail
            ],
            "recent_slamware_errors": self.read_recent_slamware_errors(),
        }
        return snapshot

    @staticmethod
    def safe_json(value: Any) -> Any:
        try:
            return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))
        except (TypeError, ValueError):
            return str(value)

    @staticmethod
    def compact_timed(item: Optional[Dict[str, Any]], now: float) -> Optional[Dict[str, Any]]:
        if not item:
            return None
        output = {k: v for k, v in item.items() if k not in ("ranges", "data", "poses")}
        if item.get("received_at") is not None:
            output["age_s"] = finite_or_none(now - float(item["received_at"]), 3)
        return output

    @staticmethod
    def compact_odom(odom: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not odom:
            return None
        return {
            "received_at": odom.get("received_at"),
            "topic": odom.get("topic"),
            "frame_id": odom.get("frame_id"),
            "x": finite_or_none(odom.get("x"), 5),
            "y": finite_or_none(odom.get("y"), 5),
            "z": finite_or_none(odom.get("z"), 5),
            "yaw": finite_or_none(odom.get("yaw"), 6),
            "yaw_deg": finite_or_none(odom.get("yaw_deg"), 3),
            "seq": odom.get("seq"),
        }

    @staticmethod
    def compact_command(command: Optional[Dict[str, Any]], now: float) -> Optional[Dict[str, Any]]:
        if not command:
            return None
        keep = [
            "received_at",
            "type",
            "seq",
            "dry_run",
            "waypoints",
            "yaw",
            "yaw_deg",
            "yaw_source",
            "move_option_flags",
            "with_yaw",
            "direct_no_avoidance",
            "key_points_mode",
            "navigation_mode",
            "raw_cmd_vel",
            "raw_nav_id",
            "raw_nav_status",
            "raw_nav_error",
            "published_topic",
        ]
        output = {k: command.get(k) for k in keep if k in command}
        if command.get("received_at") is not None:
            output["age_s"] = finite_or_none(now - float(command["received_at"]), 3)
        safety = command.get("safety")
        if safety:
            output["safety"] = {
                "blockers": safety.get("blockers", []),
                "warnings": safety.get("warnings", []),
                "waypoint_checks": safety.get("waypoint_checks", []),
            }
        return output

    @staticmethod
    def compact_plan(plan: Optional[Dict[str, Any]], now: float) -> Optional[Dict[str, Any]]:
        if not plan:
            return None
        output = {
            "received_at": plan.get("received_at"),
            "frame_id": plan.get("frame_id"),
            "total_poses": plan.get("total_poses"),
            "sampled_poses": plan.get("sampled_poses"),
            "seq": plan.get("seq"),
        }
        if plan.get("received_at") is not None:
            output["age_s"] = finite_or_none(now - float(plan["received_at"]), 3)
        poses = plan.get("poses") or []
        if poses:
            output["first_pose"] = poses[0]
            output["last_pose"] = poses[-1]
        return output

    def compact_scan(self, scan: Optional[Dict[str, Any]], now: float) -> Optional[Dict[str, Any]]:
        if not scan:
            return None
        ranges = scan.get("ranges") or []
        angle_min = finite_or_none(scan.get("angle_min"))
        angle_increment = finite_or_none(scan.get("angle_increment"))
        valid = []
        if angle_min is not None and angle_increment is not None:
            for idx, raw in enumerate(ranges):
                value = finite_or_none(raw, 4)
                if value is None:
                    continue
                angle = float(angle_min) + idx * float(angle_increment)
                deg = self.normalize_deg(math.degrees(angle))
                valid.append((float(value), deg, idx))
        closest = sorted(valid, key=lambda item: item[0])[:16]
        return {
            "received_at": scan.get("received_at"),
            "age_s": finite_or_none(now - float(scan.get("received_at", now)), 3)
            if scan.get("received_at") is not None
            else None,
            "frame_id": scan.get("frame_id"),
            "count": scan.get("count"),
            "valid_count": scan.get("valid_count"),
            "min_range": scan.get("min_range"),
            "range_min": scan.get("range_min"),
            "range_max": scan.get("range_max"),
            "closest_points": [
                {"range_m": finite_or_none(value, 4), "angle_deg": finite_or_none(deg, 1), "index": idx}
                for value, deg, idx in closest
            ],
            "close_counts": {
                f"lt_{str(th).replace('.', '_')}m": sum(1 for value, _deg, _idx in valid if value < th)
                for th in self.CLOSE_THRESHOLDS_M
            },
        }

    @staticmethod
    def compact_sensors(sensors: Optional[Dict[str, Any]], now: float) -> Optional[Dict[str, Any]]:
        if not sensors:
            return None
        items = []
        for item in sensors.get("items", []):
            items.append(
                {
                    "id": item.get("id"),
                    "sensor_type": item.get("sensor_type"),
                    "sensor_type_name": item.get("sensor_type_name"),
                    "impact_type_name": item.get("impact_type_name"),
                    "pose": item.get("pose"),
                    "is_in_impact": item.get("is_in_impact"),
                    "value": item.get("value"),
                }
            )
        hits = [item for item in items if item.get("is_in_impact")]
        return {
            "received_at": sensors.get("received_at"),
            "age_s": finite_or_none(now - float(sensors.get("received_at", now)), 3)
            if sensors.get("received_at") is not None
            else None,
            "count": sensors.get("count"),
            "hits": hits,
            "items": items,
            "seq": sensors.get("seq"),
        }

    def compute_goal_summary(
        self,
        command: Optional[Dict[str, Any]],
        odom: Optional[Dict[str, Any]],
        extra: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        goal = None
        waypoints = command.get("waypoints") if command else None
        if isinstance(waypoints, list) and waypoints:
            goal = waypoints[-1]
        action = extra.get("action") if isinstance(extra, dict) else None
        if not goal and isinstance(action, dict) and action.get("x") is not None and action.get("y") is not None:
            goal = {"x": action.get("x"), "y": action.get("y"), "yaw_deg": action.get("yawDeg")}
        if not goal:
            return None
        output = {
            "target": {
                "x": finite_or_none(goal.get("x"), 5),
                "y": finite_or_none(goal.get("y"), 5),
                "yaw_deg": finite_or_none(goal.get("yaw_deg", goal.get("yawDeg")), 3),
            }
        }
        if odom and odom.get("x") is not None and odom.get("y") is not None:
            dx = float(goal.get("x", 0.0)) - float(odom.get("x", 0.0))
            dy = float(goal.get("y", 0.0)) - float(odom.get("y", 0.0))
            bearing = math.atan2(dy, dx)
            yaw = finite_or_none(odom.get("yaw"))
            output.update(
                {
                    "distance_m": finite_or_none(math.hypot(dx, dy), 4),
                    "bearing_map_deg": finite_or_none(math.degrees(bearing), 2),
                    "bearing_robot_deg": finite_or_none(math.degrees(normalize_angle_rad(bearing - float(yaw))), 2)
                    if yaw is not None
                    else None,
                }
            )
        return output

    @staticmethod
    def normalize_deg(deg: float) -> float:
        return ((deg + 180.0) % 360.0) - 180.0

    def read_recent_slamware_errors(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {"log_path": None, "lines": []}
        if self.max_recent_errors <= 0:
            return result
        try:
            log_dir = FsPath("/unitree/var/log/slamware_service_pc4")
            logs = sorted(log_dir.glob("slamware_*.log"), key=lambda p: p.stat().st_mtime)
            if not logs:
                return result
            log_path = logs[-1]
            result["log_path"] = str(log_path)
            with log_path.open("rb") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 262144), os.SEEK_SET)
                text = f.read().decode("utf-8", "replace")
            interesting = []
            for line in text.splitlines():
                if (
                    "[ERROR]" in line
                    or "OperationFailException" in line
                    or "bad_alloc" in line
                    or "PathFindFail" in line
                    or "Exception" in line
                ):
                    interesting.append(line[-800:])
            result["lines"] = interesting[-self.max_recent_errors :]
        except Exception as exc:
            result["error"] = str(exc)
        return result

    def append_log_line(self, snapshot: Dict[str, Any]) -> None:
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(snapshot, ensure_ascii=False, allow_nan=False, separators=(",", ":")))
                f.write("\n")
        except Exception as exc:
            snapshot["log_write_error"] = str(exc)


class BaseSensorNode(Node):
    def __init__(
        self,
        state: SharedState,
        scan_topic: str,
        map_topic: str,
        odom_topics: Iterable[str],
        sensors_topic: str,
        pointcloud_topics: Iterable[str],
        move_to_locations_topic: str,
        cancel_action_topic: str,
        set_pose_topic: str,
        recover_localization_topic: str,
        set_map_localization_topic: str,
        set_map_update_topic: str,
        clear_map_topic: str,
        sync_get_stcm_service: str,
        sync_set_stcm_service: str,
        maps_dir: str,
        sync_get_stcm_timeout_sec: float,
        sync_set_stcm_timeout_sec: float,
        cmd_vel_topic: str,
        global_plan_path_topic: str,
        robot_basic_state_topic: str,
        slamware_state_topic: str,
        arm_command_topic: str,
        arm_status_topic: str,
        arm_task_timeout_sec: float,
        arm_stop_phases: Iterable[str],
        column_control_bin: str,
        column_control_workdir: str,
        column_control_interface: str,
        column_control_libdir: str,
        column_height_timeout_sec: float,
        column_height_min_m: float,
        column_height_max_m: float,
        lift_height_url: str,
        lift_height_timeout_sec: float,
        raw_nav_linear_speed_mps: float,
        raw_nav_angular_speed_radps: float,
        raw_nav_position_tolerance_m: float,
        raw_nav_yaw_tolerance_deg: float,
        min_localization_quality: int,
        relocalization_store: RelocalizationAnchorStore,
        relocalization_search_radius_m: float,
        relocalization_max_time_ms: int,
        relocalization_movement: str,
        max_cloud_points: int,
        fault_log_path: str,
    ) -> None:
        super().__init__("base_sensor_visual_server")
        self.state = state
        self.fault_logger = FaultSnapshotLogger(state, fault_log_path)
        self.max_cloud_points = max(10, int(max_cloud_points))
        self.min_localization_quality = int(min_localization_quality)
        self.relocalization_store = relocalization_store
        self.relocalization_search_radius_m = max(0.05, min(3.0, float(relocalization_search_radius_m)))
        self.relocalization_max_time_ms = max(1000, min(60000, int(relocalization_max_time_ms)))
        self.relocalization_movement = str(relocalization_movement or "NO_MOVE").strip().upper()
        self.set_pose_topic = str(set_pose_topic)
        self.recover_localization_topic = str(recover_localization_topic)
        self.set_map_localization_topic = str(set_map_localization_topic)
        self.set_map_update_topic = str(set_map_update_topic)
        self.clear_map_topic = str(clear_map_topic)
        self.sync_get_stcm_service = str(sync_get_stcm_service)
        self.sync_set_stcm_service = str(sync_set_stcm_service)
        self.maps_dir = FsPath(maps_dir)
        self.sync_get_stcm_timeout_sec = max(3.0, min(120.0, float(sync_get_stcm_timeout_sec)))
        self.sync_set_stcm_timeout_sec = max(3.0, min(120.0, float(sync_set_stcm_timeout_sec)))
        self.last_mapping_command: Optional[Dict[str, Any]] = None
        self.last_map_save: Optional[Dict[str, Any]] = None
        self.last_map_load: Optional[Dict[str, Any]] = None
        self.arm_command_topic = arm_command_topic
        self.arm_status_topic = arm_status_topic
        self.arm_task_timeout_sec = max(1.0, float(arm_task_timeout_sec))
        self.arm_stop_phases = [str(phase).strip().upper() for phase in arm_stop_phases if str(phase).strip()]
        if not self.arm_stop_phases:
            self.arm_stop_phases = ["RESET"]
        self.column_control_bin = str(column_control_bin)
        self.column_control_workdir = str(column_control_workdir)
        self.column_control_interface = str(column_control_interface)
        self.column_control_libdir = str(column_control_libdir)
        self.column_height_timeout_sec = max(1.0, float(column_height_timeout_sec))
        self.column_height_min_m = float(column_height_min_m)
        self.column_height_max_m = float(column_height_max_m)
        self.lift_height_url = str(lift_height_url or "").strip()
        self.lift_height_timeout_sec = max(0.2, min(5.0, float(lift_height_timeout_sec)))
        self.raw_nav_linear_speed_mps = max(0.02, min(0.35, float(raw_nav_linear_speed_mps)))
        self.raw_nav_angular_speed_radps = max(0.05, min(1.2, float(raw_nav_angular_speed_radps)))
        self.raw_nav_position_tolerance_m = max(0.03, min(0.3, float(raw_nav_position_tolerance_m)))
        self.raw_nav_yaw_tolerance_rad = math.radians(max(1.0, min(20.0, float(raw_nav_yaw_tolerance_deg))))
        self.raw_nav_lock = threading.Lock()
        self.raw_nav_stop = threading.Event()
        self.raw_nav_thread: Optional[threading.Thread] = None
        self.raw_nav_id: Optional[str] = None
        self.nav_monitor_lock = threading.RLock()
        self.nav_monitor_seq: Optional[int] = None
        self.nav_monitor_last_pose: Optional[Dict[str, Any]] = None
        self.nav_monitor_last_motion_at = time.time()
        self.cancel_monitor_seq: Optional[int] = None
        self.cancel_monitor_pose: Optional[Dict[str, Any]] = None
        self.arm_status_condition = threading.Condition()
        self.arm_status_by_task_id: Dict[str, List[Dict[str, Any]]] = {}
        qos = make_reliable_qos(depth=10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, qos)
        for topic in odom_topics:
            if topic:
                self.create_subscription(Odometry, topic, self.make_odom_cb(topic), qos)
        self.create_subscription(BasicSensorValueDataArray, sensors_topic, self.on_sensors, qos)
        self.create_subscription(Path, global_plan_path_topic, self.on_global_plan_path, qos)
        self.create_subscription(RobotBasicState, robot_basic_state_topic, self.on_robot_basic_state, qos)
        self.create_subscription(String, slamware_state_topic, self.on_slamware_state, qos)
        self.create_subscription(String, arm_status_topic, self.on_arm_task_status, qos)
        pointcloud_topics = [topic for topic in pointcloud_topics if topic]
        for topic in pointcloud_topics:
            self.create_subscription(PointCloud2, topic, self.make_point_cloud_cb(topic), qos)
        self.move_to_locations_pub = self.create_publisher(MoveToLocationsRequest, move_to_locations_topic, qos)
        self.cancel_action_pub = self.create_publisher(CancelActionRequest, cancel_action_topic, qos)
        self.set_pose_pub = self.create_publisher(Pose, set_pose_topic, qos)
        self.recover_localization_pub = self.create_publisher(RecoverLocalizationRequest, recover_localization_topic, qos)
        self.set_map_localization_pub = self.create_publisher(SetMapLocalizationRequest, set_map_localization_topic, qos)
        self.set_map_update_pub = self.create_publisher(SetMapUpdateRequest, set_map_update_topic, qos)
        self.clear_map_pub = self.create_publisher(ClearMapRequest, clear_map_topic, qos)
        self.sync_map_topic = (
            clear_map_topic.rsplit("/", 1)[0] + "/sync_map" if "/" in clear_map_topic else "sync_map"
        )
        self.sync_map_pub = self.create_publisher(SyncMapRequest, self.sync_map_topic, qos)
        self.sync_get_stcm_client = self.create_client(SyncGetStcm, sync_get_stcm_service)
        self.sync_set_stcm_client = self.create_client(SyncSetStcm, sync_set_stcm_service)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.arm_task_command_pub = self.create_publisher(String, arm_command_topic, qos)
        self.get_logger().info(
            f"subscribed scan={scan_topic} map={map_topic} sensors={sensors_topic} "
            f"pointclouds={pointcloud_topics} plan={global_plan_path_topic} "
            f"cmd_vel={cmd_vel_topic} arm_status={arm_status_topic} "
            f"relocalization={recover_localization_topic}"
        )

    def relocalization_status(self) -> Dict[str, Any]:
        with self.state.lock:
            current_odom = dict(self.state.odom) if self.state.odom else None
            basic = dict(self.state.robot_basic_state) if self.state.robot_basic_state else None
            last = dict(self.state.last_relocalization_command) if self.state.last_relocalization_command else None
        payload = self.relocalization_store.list_payload(current_odom=current_odom)
        payload.update(
            {
                "robot_basic_state": basic,
                "last_relocalization_command": last,
                "default_search_radius_m": self.relocalization_search_radius_m,
                "default_max_time_ms": self.relocalization_max_time_ms,
                "default_movement": self.relocalization_movement,
            }
        )
        return payload

    def save_relocalization_anchor(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        with self.state.lock:
            odom = dict(self.state.odom) if self.state.odom else None
        source = str(payload.get("source") or "manual").strip() or "manual"
        return self.relocalization_store.save_from_odom(odom, source=source)

    @staticmethod
    def resolve_localization_movement(value: Any, fallback: str = "NO_MOVE") -> Tuple[int, str]:
        raw = str(value or fallback or "NO_MOVE").strip().upper()
        mapping = {
            "NO_MOVE": int(LocalizationMovement.NO_MOVE),
            "ROTATE_ONLY": int(LocalizationMovement.ROTATE_ONLY),
            "ANY": int(LocalizationMovement.ANY),
        }
        if raw not in mapping:
            raw = "NO_MOVE"
        return mapping[raw], raw

    def run_relocalization(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        anchor_payload = payload.get("anchor")
        anchor = self.relocalization_store.normalize_anchor(anchor_payload) if isinstance(anchor_payload, dict) else None
        if not anchor:
            anchor = self.relocalization_store.get_anchor()
        if not anchor:
            return {"ok": False, "error": "no relocalization anchor; save current pose as anchor first"}

        radius = finite_or_none(payload.get("search_radius_m", payload.get("searchRadiusM", self.relocalization_search_radius_m)), 3)
        if radius is None:
            radius = self.relocalization_search_radius_m
        radius = max(0.05, min(3.0, float(radius)))
        max_time_ms = finite_or_none(payload.get("max_time_ms", payload.get("maxTimeMs", self.relocalization_max_time_ms)))
        if max_time_ms is None:
            max_time_ms = self.relocalization_max_time_ms
        max_time_ms = max(1000, min(60000, int(max_time_ms)))
        movement_type, movement_name = self.resolve_localization_movement(
            payload.get("movement", payload.get("movement_type", payload.get("movementType", self.relocalization_movement))),
            self.relocalization_movement,
        )
        dry_run = bool(payload.get("dry_run") or payload.get("dryRun"))
        use_set_pose = not (payload.get("set_pose") is False or payload.get("setPose") is False)
        enable_localization = not (payload.get("enable_localization") is False or payload.get("enableLocalization") is False)

        if not dry_run and enable_localization:
            map_localization = SetMapLocalizationRequest()
            map_localization.enabled = True
            self.set_map_localization_pub.publish(map_localization)

        pose_msg = Pose()
        pose_msg.position.x = float(anchor["x"])
        pose_msg.position.y = float(anchor["y"])
        pose_msg.position.z = float(anchor.get("z") or 0.0)
        q = quaternion_from_yaw(float(anchor.get("yaw") or 0.0))
        pose_msg.orientation.x = q["x"]
        pose_msg.orientation.y = q["y"]
        pose_msg.orientation.z = q["z"]
        pose_msg.orientation.w = q["w"]
        if not dry_run and use_set_pose:
            self.set_pose_pub.publish(pose_msg)

        req = RecoverLocalizationRequest()
        req.area.x = float(anchor["x"]) - radius
        req.area.y = float(anchor["y"]) - radius
        req.area.w = radius * 2.0
        req.area.h = radius * 2.0
        req.options.max_time_ms.is_valid = True
        req.options.max_time_ms.value = int(max_time_ms)
        req.options.mvmt_type.is_valid = True
        req.options.mvmt_type.value.type = int(movement_type)
        if not dry_run:
            self.recover_localization_pub.publish(req)

        command = {
            "received_at": time.time(),
            "type": "slamware_relocalization",
            "dry_run": dry_run,
            "anchor": anchor,
            "search_radius_m": finite_or_none(radius, 3),
            "area": {
                "x": finite_or_none(req.area.x, 3),
                "y": finite_or_none(req.area.y, 3),
                "w": finite_or_none(req.area.w, 3),
                "h": finite_or_none(req.area.h, 3),
            },
            "max_time_ms": int(max_time_ms),
            "movement": movement_name,
            "set_pose": use_set_pose,
            "enable_localization": enable_localization,
            "published_topics": {
                "set_pose": self.set_pose_topic,
                "recover_localization": self.recover_localization_topic,
                "set_map_localization": self.set_map_localization_topic,
            },
        }
        with self.state.lock:
            self.state.seq["relocalization_command"] += 1
            command["seq"] = self.state.seq["relocalization_command"]
            self.state.last_relocalization_command = command
        return {"ok": True, "relocalization_started": not dry_run, "dry_run": dry_run, "command": command}

    @staticmethod
    def _explorer_map_kind() -> MapKind:
        kind = MapKind()
        kind.kind = int(MapKind.EXPLORERMAP)
        return kind

    def start_mapping(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        clear_value = payload.get("clear", True)
        clear = not (clear_value is False or str(clear_value).strip().lower() in ("false", "0", "no", "off"))
        if clear:
            # A previously loaded map is "held" by localization mode, so a bare
            # clear_map has no effect. Release it first (localization off + map
            # update off), then clear, then re-enter SLAM (map update + localization on).
            loc_off = SetMapLocalizationRequest()
            loc_off.enabled = False
            self.set_map_localization_pub.publish(loc_off)
            time.sleep(0.3)

            update_off = SetMapUpdateRequest()
            update_off.enabled = False
            update_off.kind = self._explorer_map_kind()
            self.set_map_update_pub.publish(update_off)
            time.sleep(0.3)

            clear_msg = ClearMapRequest()
            clear_msg.kind = self._explorer_map_kind()
            self.clear_map_pub.publish(clear_msg)
            time.sleep(0.5)

            # The ROS map worker updates incrementally, so it keeps publishing the
            # cached grid after a clear. Force a full resync to reflect the wipe.
            self.sync_map_pub.publish(SyncMapRequest())
            time.sleep(0.5)

        update = SetMapUpdateRequest()
        update.enabled = True
        update.kind = self._explorer_map_kind()
        self.set_map_update_pub.publish(update)
        time.sleep(0.2)

        loc_on = SetMapLocalizationRequest()
        loc_on.enabled = True
        self.set_map_localization_pub.publish(loc_on)
        time.sleep(0.2)
        self.sync_map_pub.publish(SyncMapRequest())

        command = {
            "received_at": time.time(),
            "type": "start_mapping",
            "enabled": True,
            "cleared": clear,
            "published_topics": {
                "set_map_update": self.set_map_update_topic,
                "clear_map": self.clear_map_topic if clear else None,
                "set_map_localization": self.set_map_localization_topic if clear else None,
            },
        }
        self.last_mapping_command = command
        return {"ok": True, "mapping": "started", "cleared": clear, "command": command}

    def stop_mapping(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        _ = payload
        update = SetMapUpdateRequest()
        update.enabled = False
        update.kind = self._explorer_map_kind()
        self.set_map_update_pub.publish(update)
        command = {
            "received_at": time.time(),
            "type": "stop_mapping",
            "enabled": False,
            "published_topics": {"set_map_update": self.set_map_update_topic},
        }
        self.last_mapping_command = command
        return {"ok": True, "mapping": "stopped", "command": command}

    def mapping_status(self) -> Dict[str, Any]:
        with self.state.lock:
            basic = dict(self.state.robot_basic_state) if self.state.robot_basic_state else None
            map_payload = dict(self.state.map) if self.state.map else None
        map_info = None
        if map_payload:
            map_info = {
                "width": map_payload.get("width"),
                "height": map_payload.get("height"),
                "resolution": map_payload.get("resolution"),
                "origin": map_payload.get("origin"),
            }
        return {
            "ok": True,
            "is_map_building_enabled": bool(basic.get("is_map_building_enabled")) if basic else None,
            "is_localization_enabled": bool(basic.get("is_localization_enabled")) if basic else None,
            "map": map_info,
            "last_mapping_command": self.last_mapping_command,
            "last_map_save": self.last_map_save,
            "last_map_load": self.last_map_load,
            "maps_dir": str(self.maps_dir),
            "saved_maps": self.list_saved_maps(),
        }

    @staticmethod
    def resolve_map_filename(name: Any) -> Tuple[Optional[str], Optional[str]]:
        raw = str(name or "").strip()
        if not raw:
            return None, "map name is required / 地图名称不能为空"
        if not raw.lower().endswith(".stcm"):
            raw = raw + ".stcm"
        filename = FsPath(raw).name
        if not filename or filename in (".stcm",) or ".." in filename:
            return None, "invalid map name / 地图名称无效"
        if "/" in filename or "\\" in filename:
            return None, "invalid map name / 地图名称无效"
        return filename, None

    def resolve_map_save_path(self, name: Any) -> Tuple[Optional[FsPath], Optional[str]]:
        filename, err = self.resolve_map_filename(name)
        if err:
            return None, err
        base = self.maps_dir.resolve()
        base.mkdir(parents=True, exist_ok=True)
        target = (base / filename).resolve()
        try:
            target.relative_to(base)
        except ValueError:
            return None, "invalid map path / 地图保存路径无效"
        return target, None

    def list_saved_maps(self) -> List[Dict[str, Any]]:
        base = self.maps_dir
        if not base.exists():
            return []
        items: List[Dict[str, Any]] = []
        for path in sorted(base.glob("*.stcm")):
            if not path.is_file():
                continue
            stat = path.stat()
            items.append(
                {
                    "name": path.name,
                    "path": str(path),
                    "size_bytes": int(stat.st_size),
                    "modified_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(stat.st_mtime)),
                }
            )
        return items

    def call_sync_get_stcm(self) -> Tuple[Optional[bytes], Optional[str]]:
        if not self.sync_get_stcm_client.wait_for_service(timeout_sec=2.0):
            return None, "sync_get_stcm service unavailable / 思岚地图导出服务不可用"
        request = SyncGetStcm.Request()
        future = self.sync_get_stcm_client.call_async(request)
        deadline = time.time() + self.sync_get_stcm_timeout_sec
        while not future.done():
            if time.time() > deadline:
                return None, "sync_get_stcm timed out / 从底盘导出地图超时"
            time.sleep(0.05)
        try:
            response = future.result()
        except Exception as exc:
            return None, f"sync_get_stcm failed / 导出地图失败: {exc}"
        if response is None:
            return None, "sync_get_stcm returned no response / 导出地图无响应"
        raw = bytes(response.raw_stcm)
        if not raw:
            return None, "empty map data from chassis / 底盘返回的地图数据为空"
        return raw, None

    def save_map(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        name = payload.get("name", payload.get("filename", payload.get("map_name")))
        target, err = self.resolve_map_save_path(name)
        if err:
            return {"ok": False, "error": err}
        raw, err = self.call_sync_get_stcm()
        if err:
            return {"ok": False, "error": err}
        assert target is not None
        tmp_path = target.with_suffix(target.suffix + ".tmp")
        try:
            tmp_path.write_bytes(raw)
            os.replace(tmp_path, target)
        except OSError as exc:
            try:
                if tmp_path.exists():
                    tmp_path.unlink()
            except OSError:
                pass
            return {"ok": False, "error": f"failed to write map file / 写入地图文件失败: {exc}"}
        result = {
            "ok": True,
            "name": target.name,
            "path": str(target),
            "size_bytes": len(raw),
            "saved_at": now_iso(),
            "service": self.sync_get_stcm_service,
        }
        self.last_map_save = result
        return result

    def build_load_robot_pose(self, payload: Optional[Dict[str, Any]] = None) -> Pose:
        payload = payload or {}
        pose_payload = payload.get("robot_pose", payload.get("pose"))
        if isinstance(pose_payload, dict):
            x = finite_or_none(pose_payload.get("x"), 5) or 0.0
            y = finite_or_none(pose_payload.get("y"), 5) or 0.0
            z = finite_or_none(pose_payload.get("z"), 5) or 0.0
            yaw = finite_or_none(pose_payload.get("yaw"))
            if yaw is None:
                yaw_deg = finite_or_none(pose_payload.get("yaw_deg"))
                yaw = math.radians(float(yaw_deg)) if yaw_deg is not None else 0.0
        else:
            with self.state.lock:
                odom = dict(self.state.odom) if self.state.odom else None
            if odom and odom.get("x") is not None and odom.get("y") is not None:
                x = float(odom["x"])
                y = float(odom["y"])
                z = float(odom.get("z") or 0.0)
                yaw = float(odom.get("yaw") or 0.0)
            else:
                x = y = z = yaw = 0.0
        pose_msg = Pose()
        pose_msg.position.x = float(x)
        pose_msg.position.y = float(y)
        pose_msg.position.z = float(z)
        q = quaternion_from_yaw(float(yaw))
        pose_msg.orientation.x = q["x"]
        pose_msg.orientation.y = q["y"]
        pose_msg.orientation.z = q["z"]
        pose_msg.orientation.w = q["w"]
        return pose_msg

    def call_sync_set_stcm(self, raw: bytes, robot_pose: Pose) -> Optional[str]:
        if not self.sync_set_stcm_client.wait_for_service(timeout_sec=2.0):
            return "sync_set_stcm service unavailable / 思岚地图加载服务不可用"
        request = SyncSetStcm.Request()
        request.raw_stcm = list(raw)
        request.robot_pose = robot_pose
        future = self.sync_set_stcm_client.call_async(request)
        deadline = time.time() + self.sync_set_stcm_timeout_sec
        while not future.done():
            if time.time() > deadline:
                return "sync_set_stcm timed out / 向底盘加载地图超时"
            time.sleep(0.05)
        try:
            future.result()
        except Exception as exc:
            return f"sync_set_stcm failed / 加载地图失败: {exc}"
        return None

    def load_map(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        name = payload.get("name", payload.get("filename", payload.get("map_name")))
        target, err = self.resolve_map_save_path(name)
        if err:
            return {"ok": False, "error": err}
        assert target is not None
        if not target.exists():
            return {"ok": False, "error": f"map file not found / 地图文件不存在: {target.name}"}
        try:
            raw = target.read_bytes()
        except OSError as exc:
            return {"ok": False, "error": f"failed to read map file / 读取地图文件失败: {exc}"}
        if not raw:
            return {"ok": False, "error": "map file is empty / 地图文件为空"}
        robot_pose = self.build_load_robot_pose(payload)
        err = self.call_sync_set_stcm(raw, robot_pose)
        if err:
            return {"ok": False, "error": err}
        result = {
            "ok": True,
            "name": target.name,
            "path": str(target),
            "size_bytes": len(raw),
            "loaded_at": now_iso(),
            "service": self.sync_set_stcm_service,
            "robot_pose": {
                "x": finite_or_none(robot_pose.position.x, 5),
                "y": finite_or_none(robot_pose.position.y, 5),
                "z": finite_or_none(robot_pose.position.z, 5),
                "yaw": finite_or_none(
                    yaw_from_quaternion(
                        robot_pose.orientation.x,
                        robot_pose.orientation.y,
                        robot_pose.orientation.z,
                        robot_pose.orientation.w,
                    ),
                    6,
                ),
            },
        }
        self.last_map_load = result
        return result

    def current_navigation_command(self, max_age_s: float = 300.0) -> Optional[Dict[str, Any]]:
        now = time.time()
        with self.state.lock:
            command = self.state.last_navigation_command
        if not command or command.get("dry_run"):
            return None
        if command.get("type") not in ("move_to_locations", "raw_cmd_vel_navigation"):
            return None
        received_at = command.get("received_at")
        if received_at is None or now - float(received_at) > max_age_s:
            return None
        return command

    def reset_navigation_monitor(self, command: Dict[str, Any]) -> None:
        with self.state.lock:
            odom = self.state.odom
        with self.nav_monitor_lock:
            self.nav_monitor_seq = int(command.get("seq") or -1)
            self.nav_monitor_last_pose = self.fault_logger.compact_odom(odom)
            self.nav_monitor_last_motion_at = time.time()

    def reset_cancel_monitor(self, command: Dict[str, Any]) -> None:
        with self.state.lock:
            odom = self.state.odom
        with self.nav_monitor_lock:
            self.cancel_monitor_seq = int(command.get("seq") or -1)
            self.cancel_monitor_pose = self.fault_logger.compact_odom(odom)

    def maybe_log_plan_zero(self, plan_payload: Dict[str, Any]) -> None:
        command = self.current_navigation_command(max_age_s=300.0)
        if not command or command.get("type") != "move_to_locations":
            return
        now = time.time()
        if int(plan_payload.get("total_poses") or 0) != 0:
            return
        if now - float(command.get("received_at") or now) < 1.0:
            return
        seq = command.get("seq", "unknown")
        self.fault_logger.capture(
            "global_plan_zero",
            extra={
                "plan_seq": plan_payload.get("seq"),
                "seconds_since_command": finite_or_none(now - float(command.get("received_at", now)), 3),
            },
            throttle_key=f"global_plan_zero:{seq}",
            min_interval_s=5.0,
        )

    def maybe_log_laser_close(self, scan_payload: Dict[str, Any]) -> None:
        command = self.current_navigation_command(max_age_s=300.0)
        if not command:
            return
        min_range = finite_or_none(scan_payload.get("min_range"))
        if min_range is None or float(min_range) > 0.60:
            return
        seq = command.get("seq", "unknown")
        self.fault_logger.capture(
            "laser_close_during_navigation",
            extra={"scan_seq": scan_payload.get("seq"), "min_range_m": min_range},
            throttle_key=f"laser_close:{seq}",
            min_interval_s=3.0,
        )

    def maybe_log_sensor_impacts(self, sensors_payload: Dict[str, Any]) -> None:
        hits = [item for item in sensors_payload.get("items", []) if item.get("is_in_impact")]
        if not hits:
            return
        command = self.current_navigation_command(max_age_s=300.0)
        nav_seq = command.get("seq") if command else "no_active_nav"
        hit_ids = ",".join(str(item.get("id")) for item in hits)
        self.fault_logger.capture(
            "sensor_impact",
            extra={"sensor_seq": sensors_payload.get("seq"), "hit_ids": hit_ids},
            throttle_key=f"sensor_impact:{nav_seq}:{hit_ids}",
            min_interval_s=2.0,
        )

    def observe_odom_for_faults(self, odom_payload: Dict[str, Any]) -> None:
        now = time.time()
        command = self.current_navigation_command(max_age_s=300.0)
        if command:
            seq = int(command.get("seq") or -1)
            with self.nav_monitor_lock:
                if self.nav_monitor_seq != seq:
                    self.nav_monitor_seq = seq
                    self.nav_monitor_last_pose = self.fault_logger.compact_odom(odom_payload)
                    self.nav_monitor_last_motion_at = now
                    return
                last_pose = self.nav_monitor_last_pose
                if last_pose and odom_payload.get("x") is not None and odom_payload.get("y") is not None:
                    moved = math.hypot(float(odom_payload["x"]) - float(last_pose.get("x") or 0.0),
                                       float(odom_payload["y"]) - float(last_pose.get("y") or 0.0))
                    yaw_delta = self.pose_yaw_delta_deg(odom_payload, last_pose)
                    if moved > 0.015 or yaw_delta > 1.5:
                        self.nav_monitor_last_pose = self.fault_logger.compact_odom(odom_payload)
                        self.nav_monitor_last_motion_at = now
                        return
                still_for = now - self.nav_monitor_last_motion_at
            if now - float(command.get("received_at") or now) > 4.0 and still_for >= 3.0:
                self.fault_logger.capture(
                    "odom_still_during_navigation",
                    extra={"still_for_s": finite_or_none(still_for, 3), "odom_seq": odom_payload.get("seq")},
                    throttle_key=f"odom_still:{seq}",
                    min_interval_s=5.0,
                )
            return

        with self.state.lock:
            last_command = self.state.last_navigation_command
        if not last_command or last_command.get("type") != "cancel_action":
            return
        age = now - float(last_command.get("received_at") or now)
        if age < 0.5 or age > 20.0:
            return
        with self.nav_monitor_lock:
            cancel_seq = int(last_command.get("seq") or -1)
            if self.cancel_monitor_seq != cancel_seq or not self.cancel_monitor_pose:
                self.cancel_monitor_seq = cancel_seq
                self.cancel_monitor_pose = self.fault_logger.compact_odom(odom_payload)
                return
            cancel_pose = self.cancel_monitor_pose
        if odom_payload.get("x") is None or odom_payload.get("y") is None:
            return
        moved_after_cancel = math.hypot(
            float(odom_payload["x"]) - float(cancel_pose.get("x") or 0.0),
            float(odom_payload["y"]) - float(cancel_pose.get("y") or 0.0),
        )
        yaw_after_cancel = self.pose_yaw_delta_deg(odom_payload, cancel_pose)
        if moved_after_cancel > 0.05 or yaw_after_cancel > 5.0:
            self.fault_logger.capture(
                "movement_after_cancel",
                extra={
                    "seconds_since_cancel": finite_or_none(age, 3),
                    "moved_after_cancel_m": finite_or_none(moved_after_cancel, 4),
                    "yaw_after_cancel_deg": finite_or_none(yaw_after_cancel, 2),
                    "odom_seq": odom_payload.get("seq"),
                },
                throttle_key=f"movement_after_cancel:{last_command.get('seq')}",
                min_interval_s=999.0,
            )

    @staticmethod
    def pose_yaw_delta_deg(a: Dict[str, Any], b: Dict[str, Any]) -> float:
        yaw_a = finite_or_none(a.get("yaw"))
        yaw_b = finite_or_none(b.get("yaw"))
        if yaw_a is not None and yaw_b is not None:
            return abs(math.degrees(normalize_angle_rad(float(yaw_a) - float(yaw_b))))
        deg_a = finite_or_none(a.get("yaw_deg"))
        deg_b = finite_or_none(b.get("yaw_deg"))
        if deg_a is None or deg_b is None:
            return 0.0
        return abs(math.degrees(normalize_angle_rad(math.radians(float(deg_a) - float(deg_b)))))

    def on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        ranges: List[Optional[float]] = []
        valid_ranges: List[float] = []
        for raw in msg.ranges:
            value = finite_or_none(raw, 4)
            if value is None or value < msg.range_min or value > msg.range_max:
                ranges.append(None)
            else:
                ranges.append(value)
                valid_ranges.append(value)

        payload = {
            "received_at": now,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "angle_min": finite_or_none(msg.angle_min, 6),
            "angle_max": finite_or_none(msg.angle_max, 6),
            "angle_increment": finite_or_none(msg.angle_increment, 8),
            "range_min": finite_or_none(msg.range_min, 4),
            "range_max": finite_or_none(msg.range_max, 4),
            "count": len(ranges),
            "valid_count": len(valid_ranges),
            "min_range": round(min(valid_ranges), 4) if valid_ranges else None,
            "ranges": ranges,
        }
        with self.state.lock:
            self.state.seq["scan"] += 1
            payload["seq"] = self.state.seq["scan"]
            self.state.scan = payload
        self.maybe_log_laser_close(payload)

    def on_map(self, msg: OccupancyGrid) -> None:
        now = time.time()
        info = msg.info
        payload = {
            "received_at": now,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "resolution": finite_or_none(info.resolution, 6),
            "width": int(info.width),
            "height": int(info.height),
            "origin": {
                "x": finite_or_none(info.origin.position.x, 6),
                "y": finite_or_none(info.origin.position.y, 6),
                "z": finite_or_none(info.origin.position.z, 6),
                "yaw": finite_or_none(
                    yaw_from_quaternion(
                        info.origin.orientation.x,
                        info.origin.orientation.y,
                        info.origin.orientation.z,
                        info.origin.orientation.w,
                    ),
                    6,
                ),
            },
            "data": list(msg.data),
        }
        with self.state.lock:
            self.state.seq["map"] += 1
            payload["seq"] = self.state.seq["map"]
            self.state.map = payload

    def make_odom_cb(self, topic: str):
        def on_odom(msg: Odometry) -> None:
            now = time.time()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            payload = {
                "received_at": now,
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
                "x": finite_or_none(p.x, 5),
                "y": finite_or_none(p.y, 5),
                "z": finite_or_none(p.z, 5),
                "yaw": finite_or_none(yaw, 6),
                "yaw_deg": finite_or_none(math.degrees(yaw), 3),
            }
            track_point = {
                "x": float(p.x),
                "y": float(p.y),
                "yaw": float(yaw),
                "t": now,
            }
            with self.state.lock:
                self.state.seq["odom"] += 1
                payload["seq"] = self.state.seq["odom"]
                self.state.odom = payload
                if math.isfinite(track_point["x"]) and math.isfinite(track_point["y"]):
                    self.state.track.append(track_point)
            self.observe_odom_for_faults(payload)

        return on_odom

    def on_sensors(self, msg: BasicSensorValueDataArray) -> None:
        now = time.time()
        sensors = []
        for item in msg.values_data:
            info = item.info
            pose = info.install_pose
            stype = int(info.sensor_type.type)
            itype = int(info.impact_type.type)
            raw_value = float(item.value.value)
            value_is_finite = math.isfinite(raw_value) and abs(raw_value) < 1.0e30
            sensors.append(
                {
                    "id": int(info.id),
                    "sensor_type": stype,
                    "sensor_type_name": SENSOR_TYPE_NAMES.get(stype, str(stype)),
                    "impact_type": itype,
                    "impact_type_name": IMPACT_TYPE_NAMES.get(itype, str(itype)),
                    "refresh_freq": finite_or_none(info.refresh_freq, 3),
                    "pose": {
                        "x": finite_or_none(pose.position.x, 5),
                        "y": finite_or_none(pose.position.y, 5),
                        "z": finite_or_none(pose.position.z, 5),
                    },
                    "is_in_impact": bool(item.value.is_in_impact),
                    "value": round(raw_value, 5) if value_is_finite else None,
                    "raw_value_is_finite": value_is_finite,
                }
            )

        payload = {"received_at": now, "count": len(sensors), "items": sensors}
        with self.state.lock:
            self.state.seq["sensors"] += 1
            payload["seq"] = self.state.seq["sensors"]
            self.state.sensors = payload
        self.maybe_log_sensor_impacts(payload)

    def on_global_plan_path(self, msg: Path) -> None:
        now = time.time()
        poses = []
        max_points = 2500
        step = max(1, math.ceil(len(msg.poses) / max_points)) if msg.poses else 1
        for stamped in msg.poses[::step]:
            p = stamped.pose.position
            q = stamped.pose.orientation
            poses.append(
                {
                    "x": finite_or_none(p.x, 5),
                    "y": finite_or_none(p.y, 5),
                    "z": finite_or_none(p.z, 5),
                    "yaw": finite_or_none(yaw_from_quaternion(q.x, q.y, q.z, q.w), 6),
                }
            )
        payload = {
            "received_at": now,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "total_poses": len(msg.poses),
            "sampled_poses": len(poses),
            "poses": poses,
        }
        with self.state.lock:
            self.state.seq["global_plan_path"] += 1
            payload["seq"] = self.state.seq["global_plan_path"]
            self.state.global_plan_path = payload
        self.maybe_log_plan_zero(payload)

    def on_robot_basic_state(self, msg: RobotBasicState) -> None:
        now = time.time()
        payload = {
            "received_at": now,
            "is_map_building_enabled": bool(msg.is_map_building_enabled),
            "is_localization_enabled": bool(msg.is_localization_enabled),
            "localization_quality": int(msg.localization_quality),
            "board_temperature": int(msg.board_temperature),
            "battery_percentage": int(msg.battery_percentage),
            "is_dc_in": bool(msg.is_dc_in),
            "is_charging": bool(msg.is_charging),
        }
        with self.state.lock:
            self.state.seq["robot_basic_state"] += 1
            payload["seq"] = self.state.seq["robot_basic_state"]
            self.state.robot_basic_state = payload

    def on_slamware_state(self, msg: String) -> None:
        now = time.time()
        payload = {"received_at": now, "state": str(msg.data)}
        with self.state.lock:
            self.state.seq["slamware_state"] += 1
            payload["seq"] = self.state.seq["slamware_state"]
            self.state.slamware_state = payload

    def on_arm_task_status(self, msg: String) -> None:
        now = time.time()
        raw = str(msg.data)
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            parsed = {"raw": raw, "parse_error": str(exc)}
        if not isinstance(parsed, dict):
            parsed = {"raw": raw, "parse_error": "status payload is not a JSON object"}
        payload = {
            "received_at": now,
            "raw": raw,
            "task_id": str(parsed.get("task_id") or ""),
            "phase": str(parsed.get("phase") or ""),
            "exec_status": finite_or_none(parsed.get("exec_status")),
            "status_text": str(parsed.get("status_text") or ""),
            "message": parsed.get("message"),
            "timestamp": finite_or_none(parsed.get("timestamp"), 3),
            "parsed": parsed,
        }
        with self.state.lock:
            self.state.seq["arm_task_status"] += 1
            payload["seq"] = self.state.seq["arm_task_status"]
            self.state.arm_task_status = payload
        task_id = payload["task_id"]
        if task_id:
            with self.arm_status_condition:
                history = self.arm_status_by_task_id.setdefault(task_id, [])
                history.append(payload)
                if len(history) > 30:
                    del history[:-30]
                if len(self.arm_status_by_task_id) > 100:
                    for old_task_id in list(self.arm_status_by_task_id.keys())[:-80]:
                        self.arm_status_by_task_id.pop(old_task_id, None)
                self.arm_status_condition.notify_all()

    def execute_arm_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        phase = str(payload.get("phase") or "").strip().upper()
        phase_map = {
            "ARM_PICK": "PICK",
            "PICK": "PICK",
            "ARM_PLACE": "PLACE",
            "PLACE": "PLACE",
            "ARM_RESET": "RESET",
            "RESET": "RESET",
        }
        phase = phase_map.get(phase, phase)
        if phase not in {"RESET", "PICK", "PLACE"}:
            return {"ok": False, "error": "invalid arm phase; expected RESET / PICK / PLACE", "received": payload}

        target_object = str(payload.get("target_object") or payload.get("targetObject") or "").strip()
        if phase == "PICK" and not target_object:
            return {"ok": False, "error": "target_object is required for PICK", "received": payload}
        if phase != "PICK":
            target_object = ""

        timeout = finite_or_none(payload.get("timeout_sec", payload.get("timeoutSec", self.arm_task_timeout_sec)), 3)
        if timeout is None:
            timeout = self.arm_task_timeout_sec
        timeout = max(1.0, min(600.0, float(timeout)))
        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            target_part = target_object.lower() if target_object else "none"
            task_id = f"arm_{phase.lower()}_{target_part}_{int(time.time() * 1000)}_{uuid.uuid4().hex[:6]}"

        command = {
            "task_id": task_id,
            "phase": phase,
            "target_object": target_object,
        }
        dry_run = bool(payload.get("dry_run") or payload.get("dryRun"))
        started_at = now_iso()
        command_meta = {
            "received_at": time.time(),
            "task_id": task_id,
            "phase": phase,
            "target_object": target_object,
            "published_topic": self.arm_command_topic,
            "status_topic": self.arm_status_topic,
            "dry_run": dry_run,
            "timeout_sec": timeout,
            "command": command,
        }
        with self.state.lock:
            self.state.seq["arm_task_command"] += 1
            command_meta["seq"] = self.state.seq["arm_task_command"]
            self.state.last_arm_task_command = command_meta

        if dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "type": "arm_task",
                "task_id": task_id,
                "phase": phase,
                "target_object": target_object,
                "command": command,
                "command_topic": self.arm_command_topic,
                "status_topic": self.arm_status_topic,
                "started_at": started_at,
                "finished_at": now_iso(),
            }

        msg = String()
        msg.data = json.dumps(command, ensure_ascii=False)
        with self.arm_status_condition:
            self.arm_status_by_task_id.pop(task_id, None)
        self.arm_task_command_pub.publish(msg)

        deadline = time.time() + timeout
        final_status: Optional[Dict[str, Any]] = None
        terminal_codes = {2, 3, 4}
        terminal_texts = {"DONE", "FAILED", "REJECTED"}
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            with self.arm_status_condition:
                self.arm_status_condition.wait(timeout=min(0.5, remaining))
                history = list(self.arm_status_by_task_id.get(task_id, []))
            if history:
                latest = history[-1]
                status_code = latest.get("exec_status")
                status_text = str(latest.get("status_text") or "").upper()
                if status_code in terminal_codes or status_text in terminal_texts:
                    final_status = latest
                    break

        with self.arm_status_condition:
            status_history = list(self.arm_status_by_task_id.get(task_id, []))
        if not final_status:
            return {
                "ok": False,
                "error": "arm task timeout waiting for terminal status",
                "type": "arm_task",
                "task_id": task_id,
                "phase": phase,
                "target_object": target_object,
                "command": command,
                "status_history": status_history,
                "last_status": status_history[-1] if status_history else None,
                "timeout_sec": timeout,
                "started_at": started_at,
                "finished_at": now_iso(),
            }

        status_text = str(final_status.get("status_text") or "").upper()
        exec_status = final_status.get("exec_status")
        ok = exec_status == 2 or status_text == "DONE"
        return {
            "ok": ok,
            "error": None if ok else f"arm task ended with {status_text or exec_status}",
            "type": "arm_task",
            "task_id": task_id,
            "phase": phase,
            "target_object": target_object,
            "command": command,
            "final_status": final_status,
            "status_history": status_history,
            "timeout_sec": timeout,
            "started_at": started_at,
            "finished_at": now_iso(),
        }

    def publish_arm_command(
        self,
        phase: str,
        target_object: str = "",
        task_id: Optional[str] = None,
        dry_run: bool = False,
    ) -> Dict[str, Any]:
        phase = str(phase or "").strip().upper()
        if not phase:
            return {"ok": False, "error": "empty arm phase"}
        target_object = str(target_object or "").strip()
        if not task_id:
            task_id = f"arm_{phase.lower()}_{int(time.time() * 1000)}_{uuid.uuid4().hex[:6]}"
        command = {
            "task_id": task_id,
            "phase": phase,
            "target_object": target_object,
        }
        command_meta = {
            "received_at": time.time(),
            "task_id": task_id,
            "phase": phase,
            "target_object": target_object,
            "published_topic": self.arm_command_topic,
            "status_topic": self.arm_status_topic,
            "dry_run": dry_run,
            "command": command,
        }
        with self.state.lock:
            self.state.seq["arm_task_command"] += 1
            command_meta["seq"] = self.state.seq["arm_task_command"]
            self.state.last_arm_task_command = command_meta
        if not dry_run:
            msg = String()
            msg.data = json.dumps(command, ensure_ascii=False)
            self.arm_task_command_pub.publish(msg)
        return {"ok": True, "dry_run": dry_run, "command": command_meta}

    def stop_all_actions(self, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        dry_run = bool(payload.get("dry_run") or payload.get("dryRun"))
        if dry_run:
            navigation = {
                "ok": True,
                "dry_run": True,
                "navigation_cancelled": False,
                "command": {
                    "type": "cancel_action",
                    "published_topic": "/slamware_ros_sdk_server_node/cancel_action",
                },
            }
        else:
            navigation = self.cancel_navigation()
        raw_phases = payload.get("arm_stop_phases", payload.get("armStopPhases", self.arm_stop_phases))
        if isinstance(raw_phases, str):
            phases = [item.strip().upper() for item in raw_phases.split(",") if item.strip()]
        elif isinstance(raw_phases, list):
            phases = [str(item).strip().upper() for item in raw_phases if str(item).strip()]
        else:
            phases = list(self.arm_stop_phases)
        if not phases:
            phases = ["RESET"]
        arm_commands = [self.publish_arm_command(phase, dry_run=dry_run) for phase in phases]
        return {
            "ok": bool(navigation.get("ok")) and all(item.get("ok") for item in arm_commands),
            "dry_run": dry_run,
            "navigation": navigation,
            "arm_stop_phases": phases,
            "arm_commands": arm_commands,
            "message": "published navigation cancel and arm stop/reset commands",
        }

    def read_lift_height_status(self) -> Dict[str, Any]:
        url = self.lift_height_url
        if not url:
            return {"ok": False, "error": "lift height url is empty"}
        started = time.time()
        try:
            req = urllib.request.Request(url, headers={"Accept": "application/json"})
            with urllib.request.urlopen(req, timeout=self.lift_height_timeout_sec) as resp:
                body = resp.read(65536).decode("utf-8", "replace")
                status_code = getattr(resp, "status", 200)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "url": url, "error": str(exc), "elapsed_ms": round((time.time() - started) * 1000.0, 1)}
        try:
            source_payload = json.loads(body)
        except json.JSONDecodeError as exc:
            return {"ok": False, "url": url, "http_status": status_code, "error": f"invalid json from lift height service: {exc}", "body_preview": body[:400], "elapsed_ms": round((time.time() - started) * 1000.0, 1)}
        if not isinstance(source_payload, dict):
            source_payload = {"value": source_payload}
        def pick_m(*keys: str) -> Optional[float]:
            for key in keys:
                value = finite_or_none(source_payload.get(key))
                if value is not None:
                    return float(value)
            return None
        physical_height_m = pick_m("physical_height_m", "physicalHeightM", "height_m", "heightM")
        hispeed_y_m = pick_m("hispeed_y_m", "raw_height_m", "rawHeightM", "sdk_height_m", "sdkHeightM")
        lift_offset_m = pick_m("lift_offset_m", "offset_m", "offsetM")
        full_travel_m = pick_m("full_travel_m", "fullTravelM")
        sdk_min_m = pick_m("sdk_min_m", "sdkMinM")
        sdk_max_m = pick_m("sdk_max_m", "sdkMaxM")
        physical_min_m = pick_m("physical_min_m", "physicalMinM")
        physical_max_m = pick_m("physical_max_m", "physicalMaxM")
        if physical_height_m is None and hispeed_y_m is not None and lift_offset_m is not None:
            physical_height_m = hispeed_y_m - lift_offset_m
        if lift_offset_m is None and hispeed_y_m is not None and physical_height_m is not None:
            lift_offset_m = hispeed_y_m - physical_height_m
        if physical_min_m is None:
            physical_min_m = 0.0
        if physical_max_m is None and full_travel_m is not None:
            physical_max_m = full_travel_m
        if sdk_min_m is None and lift_offset_m is not None and physical_min_m is not None:
            sdk_min_m = lift_offset_m + physical_min_m
        if sdk_max_m is None and lift_offset_m is not None and physical_max_m is not None:
            sdk_max_m = lift_offset_m + physical_max_m
        def mm(value: Optional[float]) -> Optional[float]:
            return round(value * 1000.0, 1) if value is not None and math.isfinite(value) else None
        return {"ok": True, "url": url, "http_status": status_code, "source": source_payload.get("source") or source_payload.get("service") or "lift_height_service", "physical_height_m": finite_or_none(physical_height_m, 6), "physical_height_mm": mm(physical_height_m), "hispeed_y_m": finite_or_none(hispeed_y_m, 6), "hispeed_y_mm": mm(hispeed_y_m), "lift_offset_m": finite_or_none(lift_offset_m, 6), "lift_offset_mm": mm(lift_offset_m), "full_travel_m": finite_or_none(full_travel_m, 6), "full_travel_mm": mm(full_travel_m), "sdk_min_m": finite_or_none(sdk_min_m, 6), "sdk_max_m": finite_or_none(sdk_max_m, 6), "physical_min_m": finite_or_none(physical_min_m, 6), "physical_max_m": finite_or_none(physical_max_m, 6), "data_age_sec": finite_or_none(source_payload.get("data_age_sec"), 3), "timestamp": source_payload.get("timestamp"), "elapsed_ms": round((time.time() - started) * 1000.0, 1), "raw": source_payload}

    def execute_column_height_action(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        lift_height_before: Optional[Dict[str, Any]] = None
        target_source = "raw"
        target_physical = finite_or_none(payload.get("target_physical_height_m", payload.get("targetPhysicalHeightM", payload.get("physical_height_m", payload.get("physicalHeightM")))), 4)
        if target_physical is not None:
            target_source = "physical"
            lift_height_before = self.read_lift_height_status()
            if not lift_height_before.get("ok"):
                return {"ok": False, "error": "cannot read current lift offset for physical height control", "target_physical_height_m": target_physical, "lift_height": lift_height_before, "received": payload}
            physical_min = finite_or_none(lift_height_before.get("physical_min_m")) or 0.0
            physical_max = finite_or_none(lift_height_before.get("physical_max_m"))
            full_travel = finite_or_none(lift_height_before.get("full_travel_m"))
            if physical_max is None:
                physical_max = full_travel if full_travel is not None else 0.427
            if target_physical < physical_min - 0.002 or target_physical > physical_max + 0.002:
                return {"ok": False, "error": f"target_physical_height_m out of range [{physical_min:.3f}, {physical_max:.3f}]", "target_physical_height_m": target_physical, "lift_height": lift_height_before, "received": payload}
            lift_offset = finite_or_none(lift_height_before.get("lift_offset_m"))
            if lift_offset is None:
                hispeed_y = finite_or_none(lift_height_before.get("hispeed_y_m")); physical_now = finite_or_none(lift_height_before.get("physical_height_m"))
                if hispeed_y is not None and physical_now is not None:
                    lift_offset = hispeed_y - physical_now
            if lift_offset is None:
                return {"ok": False, "error": "lift_offset_m unavailable; cannot convert physical height to raw SDK target", "target_physical_height_m": target_physical, "lift_height": lift_height_before, "received": payload}
            target = finite_or_none(target_physical + lift_offset, 4)
        else:
            target = finite_or_none(payload.get("target_height_m", payload.get("targetHeightM", payload.get("height_m", payload.get("heightM")))), 4)
            if target is None:
                return {"ok": False, "error": "target_physical_height_m or target_height_m is required", "received": payload}
        if target is None:
            return {"ok": False, "error": "converted raw target_height_m is invalid", "received": payload}
        if target_source == "raw" and (target < self.column_height_min_m or target > self.column_height_max_m):
            return {"ok": False, "error": (f"target_height_m out of range " f"[{self.column_height_min_m:.3f}, {self.column_height_max_m:.3f}]"), "target_height_m": target, "received": payload}
        if target_source == "physical" and lift_height_before:
            sdk_min = finite_or_none(lift_height_before.get("sdk_min_m")); sdk_max = finite_or_none(lift_height_before.get("sdk_max_m"))
            if sdk_min is not None and sdk_max is not None and (target < sdk_min - 0.02 or target > sdk_max + 0.02):
                return {"ok": False, "error": f"converted raw target_height_m out of SDK range [{sdk_min:.3f}, {sdk_max:.3f}]", "target_height_m": target, "target_physical_height_m": target_physical, "lift_height": lift_height_before, "received": payload}
        timeout = finite_or_none(payload.get("timeout_sec", payload.get("timeoutSec", self.column_height_timeout_sec)), 2)
        if timeout is None:
            timeout = self.column_height_timeout_sec
        timeout = max(1.0, min(180.0, float(timeout)))
        dry_run = bool(payload.get("dry_run") or payload.get("dryRun"))
        argv = [
            self.column_control_bin,
            self.column_control_interface,
            f"{target:.4f}",
        ]
        workdir = self.column_control_workdir or None
        env = os.environ.copy()
        if self.column_control_libdir:
            old_path = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = (
                self.column_control_libdir
                if not old_path
                else f"{self.column_control_libdir}:{old_path}"
            )
        started_at = now_iso()
        command = {
            "type": "column_height",
            "argv": argv,
            "cwd": workdir,
            "target_height_m": target,
            "target_raw_height_m": target,
            "target_physical_height_m": target_physical,
            "target_source": target_source,
            "timeout_sec": timeout,
            "dry_run": dry_run,
            "lift_height_before": lift_height_before,
        }
        if dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "type": "column_height",
                "command": command,
                "started_at": started_at,
                "finished_at": now_iso(),
            }
        try:
            result = subprocess.run(
                argv,
                cwd=workdir,
                env=env,
                text=True,
                capture_output=True,
                timeout=timeout,
                check=False,
            )
        except FileNotFoundError as exc:
            return {
                "ok": False,
                "error": f"column control binary not found: {exc}",
                "type": "column_height",
                "command": command,
                "started_at": started_at,
                "finished_at": now_iso(),
            }
        except subprocess.TimeoutExpired as exc:
            return {
                "ok": False,
                "error": "column height command timeout",
                "type": "column_height",
                "command": command,
                "returncode": None,
                "stdout": exc.stdout or "",
                "stderr": exc.stderr or "",
                "started_at": started_at,
                "finished_at": now_iso(),
            }
        ok = result.returncode == 0
        lift_height_after = self.read_lift_height_status() if not dry_run else None
        return {
            "ok": ok,
            "error": None if ok else f"column height command failed with returncode {result.returncode}",
            "type": "column_height",
            "command": command,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "started_at": started_at,
            "finished_at": now_iso(),
            "lift_height_before": lift_height_before,
            "lift_height_after": lift_height_after,
        }

    def start_navigation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        waypoints, parse_errors = self.parse_waypoints(payload.get("waypoints", payload.get("points", [])))
        if parse_errors:
            return {"ok": False, "error": "invalid waypoints", "errors": parse_errors}
        if not waypoints:
            return {"ok": False, "error": "no waypoints selected"}
        if len(waypoints) > 60:
            return {"ok": False, "error": "too many waypoints", "max_waypoints": 60}

        safety = self.check_navigation_safety(waypoints)
        if safety["blockers"]:
            self.fault_logger.capture(
                "navigation_start_blocked",
                extra={"waypoints": waypoints, "safety": safety},
                force=True,
            )
            return {"ok": False, "error": "navigation safety check failed", "safety": safety}

        yaw_result = self.resolve_navigation_yaw(payload, waypoints)
        if not yaw_result["ok"]:
            return yaw_result
        raw_cmd_vel = bool(
            payload.get("raw_cmd_vel")
            or payload.get("rawCmdVel")
            or payload.get("disable_obstacle_avoidance")
            or payload.get("disableObstacleAvoidance")
            or payload.get("navigation_mode") == "raw_cmd_vel_no_obstacle_avoidance"
        )
        if raw_cmd_vel:
            return self.start_raw_cmd_vel_navigation(payload, waypoints, yaw_result, safety)

        request = MoveToLocationsRequest()
        request.locations = [Point(x=p["x"], y=p["y"], z=0.0) for p in waypoints]
        request.yaw = float(yaw_result["yaw"])
        request.options.opt_flags.flags = int(request.options.opt_flags.flags) | SLAMWARE_MOVE_OPTION_WITH_YAW
        direct_no_avoidance = bool(
            payload.get("direct_no_avoidance")
            or payload.get("directNoAvoidance")
            or payload.get("key_points_mode")
            or payload.get("keyPointsMode")
        )
        if direct_no_avoidance:
            request.options.opt_flags.flags = int(request.options.opt_flags.flags) | SLAMWARE_MOVE_OPTION_KEY_POINTS

        speed_ratio = payload.get("speed_ratio", payload.get("speedRatio", 1.0))
        try:
            speed = max(0.05, min(1.0, float(speed_ratio)))
            request.options.speed_ratio.is_valid = True
            request.options.speed_ratio.value = speed
        except (TypeError, ValueError):
            return {"ok": False, "error": "invalid speed_ratio"}

        dry_run = bool(payload.get("dry_run"))
        now = time.time()
        command = {
            "received_at": now,
            "type": "move_to_locations",
            "dry_run": dry_run,
            "waypoints": waypoints,
            "yaw": finite_or_none(request.yaw, 5),
            "yaw_deg": finite_or_none(math.degrees(float(request.yaw)), 2),
            "yaw_source": yaw_result["source"],
            "move_option_flags": int(request.options.opt_flags.flags),
            "with_yaw": bool(int(request.options.opt_flags.flags) & SLAMWARE_MOVE_OPTION_WITH_YAW),
            "direct_no_avoidance": direct_no_avoidance,
            "key_points_mode": bool(int(request.options.opt_flags.flags) & SLAMWARE_MOVE_OPTION_KEY_POINTS),
            "navigation_mode": "direct_key_points_stop_on_obstacle" if direct_no_avoidance else "normal_slamware",
            "speed_ratio": finite_or_none(request.options.speed_ratio.value, 3)
            if request.options.speed_ratio.is_valid
            else None,
            "published_topic": "/slamware_ros_sdk_server_node/move_to_locations",
            "safety": safety,
        }
        if not dry_run:
            self.move_to_locations_pub.publish(request)
        with self.state.lock:
            self.state.seq["navigation_command"] += 1
            command["seq"] = self.state.seq["navigation_command"]
            self.state.last_navigation_command = command
        if not dry_run:
            self.reset_navigation_monitor(command)
        self.fault_logger.capture(
            "navigation_start",
            extra={"waypoints": waypoints, "yaw_result": yaw_result, "request": payload},
            force=True,
        )
        return {"ok": True, "navigation_started": not dry_run, "dry_run": dry_run, "command": command}

    def start_raw_cmd_vel_navigation(
        self,
        payload: Dict[str, Any],
        waypoints: List[Dict[str, float]],
        yaw_result: Dict[str, Any],
        safety: Dict[str, Any],
    ) -> Dict[str, Any]:
        dry_run = bool(payload.get("dry_run"))
        speed_ratio = finite_or_none(payload.get("speed_ratio", 1.0), 3)
        if speed_ratio is None:
            speed_ratio = 1.0
        speed_ratio = max(0.2, min(1.0, float(speed_ratio)))
        linear_speed = finite_or_none(payload.get("raw_linear_speed_mps", self.raw_nav_linear_speed_mps), 3)
        angular_speed = finite_or_none(payload.get("raw_angular_speed_radps", self.raw_nav_angular_speed_radps), 3)
        position_tolerance = finite_or_none(payload.get("raw_position_tolerance_m", self.raw_nav_position_tolerance_m), 3)
        yaw_tolerance_deg = finite_or_none(payload.get("raw_yaw_tolerance_deg", math.degrees(self.raw_nav_yaw_tolerance_rad)), 2)
        timeout = finite_or_none(payload.get("timeout_sec", payload.get("timeoutSec", max(30.0, 45.0 * len(waypoints)))), 2)
        linear_speed = max(0.02, min(0.35, float(linear_speed or self.raw_nav_linear_speed_mps))) * speed_ratio
        angular_speed = max(0.05, min(1.2, float(angular_speed or self.raw_nav_angular_speed_radps))) * speed_ratio
        position_tolerance = max(0.03, min(0.3, float(position_tolerance or self.raw_nav_position_tolerance_m)))
        yaw_tolerance_rad = math.radians(max(1.0, min(20.0, float(yaw_tolerance_deg or math.degrees(self.raw_nav_yaw_tolerance_rad)))))
        timeout = max(5.0, min(600.0, float(timeout or 60.0)))
        nav_id = f"raw_nav_{int(time.time() * 1000)}_{uuid.uuid4().hex[:6]}"
        command = {
            "received_at": time.time(),
            "type": "raw_cmd_vel_navigation",
            "dry_run": dry_run,
            "navigation_started": not dry_run,
            "waypoints": waypoints,
            "yaw": finite_or_none(float(yaw_result["yaw"]), 5),
            "yaw_deg": finite_or_none(math.degrees(float(yaw_result["yaw"])), 2),
            "yaw_source": yaw_result["source"],
            "navigation_mode": "raw_cmd_vel_no_obstacle_avoidance",
            "disable_obstacle_avoidance": True,
            "raw_cmd_vel": True,
            "raw_nav_id": nav_id,
            "raw_nav_status": "dry_run" if dry_run else "running",
            "raw_linear_speed_mps": finite_or_none(linear_speed, 3),
            "raw_angular_speed_radps": finite_or_none(angular_speed, 3),
            "raw_position_tolerance_m": finite_or_none(position_tolerance, 3),
            "raw_yaw_tolerance_deg": finite_or_none(math.degrees(yaw_tolerance_rad), 2),
            "timeout_sec": finite_or_none(timeout, 2),
            "published_topic": "/cmd_vel",
            "safety": safety,
        }
        terminal_statuses = {"done", "dry_run", "timeout", "cancelled", "error"}

        def record_command() -> None:
            with self.state.lock:
                self.state.seq["navigation_command"] += 1
                command["seq"] = self.state.seq["navigation_command"]
                self.state.last_navigation_command = command

        with self.state.lock:
            previous_command = dict(self.state.last_navigation_command) if self.state.last_navigation_command else None
        if dry_run:
            record_command()
            self.fault_logger.capture(
                "raw_navigation_start",
                extra={"waypoints": waypoints, "yaw_result": yaw_result, "request": payload},
                force=True,
            )
            return {"ok": True, "navigation_started": False, "dry_run": True, "command": command}

        active_thread = None
        active_id = None
        with self.raw_nav_lock:
            active_thread = self.raw_nav_thread
            active_id = self.raw_nav_id
        if active_thread and active_thread.is_alive():
            active_status = ""
            if previous_command and previous_command.get("raw_nav_id") == active_id:
                active_status = str(previous_command.get("raw_nav_status") or "")
            if active_status in terminal_statuses:
                active_thread.join(timeout=1.0)
                with self.raw_nav_lock:
                    if self.raw_nav_thread is active_thread and not active_thread.is_alive():
                        self.raw_nav_thread = None
                        if self.raw_nav_id == active_id:
                            self.raw_nav_id = None
            if active_thread.is_alive():
                self.fault_logger.capture(
                    "raw_navigation_start_blocked",
                    extra={"reason": "raw cmd_vel navigation is already running", "active_raw_nav_id": active_id},
                    force=True,
                )
                return {
                    "ok": False,
                    "error": "raw cmd_vel navigation is already running",
                    "active_raw_nav": {
                        "raw_nav_id": active_id,
                        "raw_nav_status": active_status or None,
                    },
                    "command": command,
                }

        with self.raw_nav_lock:
            if self.raw_nav_thread and self.raw_nav_thread.is_alive():
                self.fault_logger.capture(
                    "raw_navigation_start_blocked",
                    extra={"reason": "raw cmd_vel navigation is already running", "active_raw_nav_id": self.raw_nav_id},
                    force=True,
                )
                return {
                    "ok": False,
                    "error": "raw cmd_vel navigation is already running",
                    "active_raw_nav": {
                        "raw_nav_id": self.raw_nav_id,
                    },
                    "command": command,
                }
            self.raw_nav_stop.clear()
            self.raw_nav_id = nav_id
            self.raw_nav_thread = threading.Thread(
                target=self.raw_cmd_vel_worker,
                args=(nav_id, waypoints, float(yaw_result["yaw"]), linear_speed, angular_speed, position_tolerance, yaw_tolerance_rad, timeout),
                name="raw_cmd_vel_navigation",
                daemon=True,
            )
            record_command()
            self.reset_navigation_monitor(command)
            self.fault_logger.capture(
                "raw_navigation_start",
                extra={"waypoints": waypoints, "yaw_result": yaw_result, "request": payload},
                force=True,
            )
            self.raw_nav_thread.start()
        return {"ok": True, "navigation_started": True, "dry_run": False, "command": command}

    @staticmethod
    def clamp_abs(value: float, max_abs: float, min_abs: float = 0.0) -> float:
        if abs(value) < min_abs:
            return math.copysign(min_abs, value) if value else 0.0
        return max(-max_abs, min(max_abs, value))

    def publish_cmd_vel(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def update_raw_nav_status(self, nav_id: str, **updates: Any) -> None:
        with self.state.lock:
            current = self.state.last_navigation_command
            if current and current.get("raw_nav_id") == nav_id:
                current.update(updates)

    def get_current_odom(self) -> Optional[Dict[str, Any]]:
        with self.state.lock:
            return dict(self.state.odom) if self.state.odom else None

    def raw_cmd_vel_worker(
        self,
        nav_id: str,
        waypoints: List[Dict[str, float]],
        final_yaw: float,
        linear_speed: float,
        angular_speed: float,
        position_tolerance: float,
        yaw_tolerance_rad: float,
        timeout: float,
    ) -> None:
        started = time.time()
        status = "done"
        error = ""
        try:
            for idx, target in enumerate(waypoints):
                self.update_raw_nav_status(nav_id, raw_nav_status="running", raw_nav_target_index=idx)
                while not self.raw_nav_stop.is_set():
                    if time.time() - started > timeout:
                        status, error = "timeout", "raw cmd_vel navigation timeout"
                        return
                    odom = self.get_current_odom()
                    if not odom or time.time() - float(odom.get("received_at", 0.0)) > 1.0:
                        self.publish_cmd_vel(0.0, 0.0)
                        time.sleep(0.1)
                        continue
                    dx = float(target["x"]) - float(odom["x"])
                    dy = float(target["y"]) - float(odom["y"])
                    dist = math.hypot(dx, dy)
                    if dist <= position_tolerance:
                        self.publish_cmd_vel(0.0, 0.0)
                        break
                    yaw = float(odom.get("yaw") or 0.0)
                    target_yaw = math.atan2(dy, dx)
                    yaw_err = normalize_angle_rad(target_yaw - yaw)
                    if abs(yaw_err) > math.radians(25.0):
                        linear = 0.0
                    else:
                        linear = min(linear_speed, max(0.03, dist * 0.45)) * max(0.25, math.cos(yaw_err))
                    angular = self.clamp_abs(1.8 * yaw_err, angular_speed, min_abs=0.06)
                    self.publish_cmd_vel(linear, angular)
                    self.update_raw_nav_status(
                        nav_id,
                        raw_nav_status="running",
                        raw_nav_distance_m=finite_or_none(dist, 3),
                        raw_nav_heading_error_deg=finite_or_none(math.degrees(yaw_err), 1),
                    )
                    time.sleep(0.1)
            while not self.raw_nav_stop.is_set():
                if time.time() - started > timeout:
                    status, error = "timeout", "raw cmd_vel navigation timeout"
                    return
                odom = self.get_current_odom()
                if not odom or time.time() - float(odom.get("received_at", 0.0)) > 1.0:
                    self.publish_cmd_vel(0.0, 0.0)
                    time.sleep(0.1)
                    continue
                yaw_err = normalize_angle_rad(final_yaw - float(odom.get("yaw") or 0.0))
                if abs(yaw_err) <= yaw_tolerance_rad:
                    break
                angular = self.clamp_abs(1.8 * yaw_err, angular_speed, min_abs=0.06)
                self.publish_cmd_vel(0.0, angular)
                self.update_raw_nav_status(
                    nav_id,
                    raw_nav_status="final_yaw",
                    raw_nav_yaw_error_deg=finite_or_none(math.degrees(yaw_err), 1),
                )
                time.sleep(0.1)
            if self.raw_nav_stop.is_set():
                status, error = "cancelled", "raw cmd_vel navigation cancelled"
        finally:
            self.publish_cmd_vel(0.0, 0.0)
            self.update_raw_nav_status(nav_id, raw_nav_status=status, raw_nav_error=error, raw_nav_finished_at=now_iso())
            with self.raw_nav_lock:
                if self.raw_nav_id == nav_id:
                    self.raw_nav_id = None

    def cancel_navigation(self) -> Dict[str, Any]:
        raw_cancelled = self.cancel_raw_cmd_vel_navigation()
        self.cancel_action_pub.publish(CancelActionRequest())
        now = time.time()
        command = {
            "received_at": now,
            "type": "cancel_action",
            "published_topic": "/slamware_ros_sdk_server_node/cancel_action",
            "raw_cmd_vel_cancelled": raw_cancelled,
        }
        with self.state.lock:
            self.state.seq["navigation_command"] += 1
            command["seq"] = self.state.seq["navigation_command"]
            self.state.last_navigation_command = command
        self.reset_cancel_monitor(command)
        self.fault_logger.capture(
            "navigation_cancel",
            extra={"raw_cancelled": raw_cancelled},
            force=True,
        )
        return {"ok": True, "navigation_cancelled": True, "command": command}

    def cancel_raw_cmd_vel_navigation(self) -> bool:
        with self.raw_nav_lock:
            running = bool(self.raw_nav_thread and self.raw_nav_thread.is_alive())
            if running:
                self.raw_nav_stop.set()
        self.publish_cmd_vel(0.0, 0.0)
        return running

    def parse_waypoints(self, raw: Any) -> Tuple[List[Dict[str, float]], List[str]]:
        errors = []
        points = []
        if not isinstance(raw, list):
            return [], ["waypoints must be a list"]
        for idx, item in enumerate(raw):
            if isinstance(item, dict):
                x_raw = item.get("x")
                y_raw = item.get("y")
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                x_raw, y_raw = item[0], item[1]
            else:
                errors.append(f"waypoint {idx + 1} must have x/y")
                continue
            try:
                x = float(x_raw)
                y = float(y_raw)
            except (TypeError, ValueError):
                errors.append(f"waypoint {idx + 1} x/y must be numbers")
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                errors.append(f"waypoint {idx + 1} x/y must be finite")
                continue
            points.append({"x": round(x, 4), "y": round(y, 4)})
        return points, errors

    def resolve_navigation_yaw(self, payload: Dict[str, Any], waypoints: List[Dict[str, float]]) -> Dict[str, Any]:
        yaw_source = str(payload.get("yaw_source") or "").strip() or None
        if payload.get("yaw") is not None:
            try:
                yaw = normalize_angle_rad(float(payload.get("yaw")))
            except (TypeError, ValueError):
                return {"ok": False, "error": "invalid yaw"}
            return {"ok": True, "yaw": yaw, "source": yaw_source or "request_yaw_rad"}

        if payload.get("yaw_deg") is not None:
            try:
                yaw = normalize_angle_rad(math.radians(float(payload.get("yaw_deg"))))
            except (TypeError, ValueError):
                return {"ok": False, "error": "invalid yaw_deg"}
            return {"ok": True, "yaw": yaw, "source": yaw_source or "request_yaw_deg"}

        yaw, source = self.infer_final_yaw(waypoints)
        return {"ok": True, "yaw": yaw, "source": yaw_source or source}

    def infer_final_yaw(self, waypoints: List[Dict[str, float]]) -> Tuple[float, str]:
        if len(waypoints) >= 2:
            a = waypoints[-2]
            b = waypoints[-1]
            return normalize_angle_rad(math.atan2(b["y"] - a["y"], b["x"] - a["x"])), "server_auto_last_segment"
        with self.state.lock:
            odom = self.state.odom
        if odom and odom.get("yaw") is not None:
            return normalize_angle_rad(float(odom["yaw"])), "server_current_odom"
        return 0.0, "server_default_zero"

    def check_navigation_safety(self, waypoints: List[Dict[str, float]]) -> Dict[str, Any]:
        now = time.time()
        blockers: List[str] = []
        warnings: List[str] = []
        waypoint_checks = []

        with self.state.lock:
            map_payload = self.state.map
            odom = self.state.odom
            scan = self.state.scan
            sensors = self.state.sensors
            robot_basic_state = self.state.robot_basic_state
            slamware_state = self.state.slamware_state

        self.require_fresh("map", map_payload, now, 3.0, blockers)
        self.require_fresh("odom", odom, now, 1.0, blockers)
        self.require_fresh("scan", scan, now, 1.5, blockers)
        self.require_fresh("basic sensors", sensors, now, 3.0, blockers)
        self.require_fresh("robot basic state", robot_basic_state, now, 3.0, warnings)

        if sensors:
            hit_sensors = [item for item in sensors.get("items", []) if item.get("is_in_impact")]
            if hit_sensors:
                blocking_hits = [
                    item for item in hit_sensors
                    if int(item.get("sensor_type", -1)) != 2
                ]
                sonar_hits = [
                    item for item in hit_sensors
                    if int(item.get("sensor_type", -1)) == 2
                ]
                if blocking_hits:
                    ids = ", ".join(str(item.get("id")) for item in blocking_hits)
                    blockers.append(f"contact sensor impact active: {ids}")
                if sonar_hits:
                    ids = ", ".join(str(item.get("id")) for item in sonar_hits)
                    warnings.append(f"sonar impact ignored for navigation start: {ids}")

        if robot_basic_state:
            if not robot_basic_state.get("is_localization_enabled"):
                blockers.append("slamware localization is disabled")
            quality = robot_basic_state.get("localization_quality")
            if quality is not None and quality <= 0:
                msg = f"localization_quality is {quality}; verify localization before moving"
                if self.min_localization_quality >= 0 and int(quality) < self.min_localization_quality:
                    blockers.append(msg)
                else:
                    warnings.append(msg)

        if slamware_state and str(slamware_state.get("state", "")).lower() not in ("connected", "ok", "running"):
            warnings.append(f"slamware state is {slamware_state.get('state')}")

        if map_payload:
            for idx, point in enumerate(waypoints):
                waypoint_checks.append(self.check_waypoint_on_map(idx, point, map_payload, blockers, warnings))
            self.check_straight_segments_on_map(waypoints, map_payload, warnings)

        return {
            "ok": not blockers,
            "blockers": blockers,
            "warnings": warnings,
            "waypoint_checks": waypoint_checks,
            "min_localization_quality": self.min_localization_quality,
        }

    @staticmethod
    def require_fresh(
        name: str,
        item: Optional[Dict[str, Any]],
        now: float,
        max_age_s: float,
        output: List[str],
    ) -> None:
        if not item:
            output.append(f"{name} has no data")
            return
        stamp = item.get("received_at")
        if stamp is None:
            output.append(f"{name} has no timestamp")
            return
        age = now - float(stamp)
        if age > max_age_s:
            output.append(f"{name} is stale: {age:.2f}s > {max_age_s:.2f}s")

    def check_waypoint_on_map(
        self,
        idx: int,
        point: Dict[str, float],
        map_payload: Dict[str, Any],
        blockers: List[str],
        warnings: List[str],
    ) -> Dict[str, Any]:
        cell = self.world_to_map_cell(point["x"], point["y"], map_payload)
        result = {"index": idx, "x": point["x"], "y": point["y"], "cell": cell, "status": "ok", "value": None}
        if cell is None:
            result["status"] = "out_of_map"
            blockers.append(f"waypoint {idx + 1} is outside map")
            return result
        data = map_payload.get("data") or []
        width = int(map_payload.get("width") or 0)
        data_idx = cell["y"] * width + cell["x"]
        value = int(data[data_idx]) if 0 <= data_idx < len(data) else -1
        result["value"] = value
        if value > 70:
            result["status"] = "occupied"
            blockers.append(f"waypoint {idx + 1} is on occupied map cell: {value}")
        elif value < 0:
            result["status"] = "unknown"
            warnings.append(f"waypoint {idx + 1} is on unknown map cell")
        return result

    @staticmethod
    def world_to_map_cell(x: float, y: float, map_payload: Dict[str, Any]) -> Optional[Dict[str, int]]:
        resolution = float(map_payload.get("resolution") or 0.0)
        width = int(map_payload.get("width") or 0)
        height = int(map_payload.get("height") or 0)
        origin = map_payload.get("origin") or {}
        if resolution <= 0 or width <= 0 or height <= 0:
            return None
        mx = int(math.floor((x - float(origin.get("x") or 0.0)) / resolution))
        my = int(math.floor((y - float(origin.get("y") or 0.0)) / resolution))
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None
        return {"x": mx, "y": my}

    def check_straight_segments_on_map(
        self,
        waypoints: List[Dict[str, float]],
        map_payload: Dict[str, Any],
        warnings: List[str],
    ) -> None:
        if len(waypoints) < 2:
            return
        data = map_payload.get("data") or []
        width = int(map_payload.get("width") or 0)
        resolution = float(map_payload.get("resolution") or 0.0)
        if not data or width <= 0 or resolution <= 0:
            return
        for idx in range(len(waypoints) - 1):
            a = waypoints[idx]
            b = waypoints[idx + 1]
            dist = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            steps = max(2, int(math.ceil(dist / resolution)))
            occupied = 0
            unknown = 0
            for step in range(steps + 1):
                t = step / steps
                x = a["x"] + (b["x"] - a["x"]) * t
                y = a["y"] + (b["y"] - a["y"]) * t
                cell = self.world_to_map_cell(x, y, map_payload)
                if cell is None:
                    unknown += 1
                    continue
                data_idx = cell["y"] * width + cell["x"]
                value = int(data[data_idx]) if 0 <= data_idx < len(data) else -1
                if value > 70:
                    occupied += 1
                elif value < 0:
                    unknown += 1
            if occupied:
                warnings.append(
                    f"straight preview segment {idx + 1}->{idx + 2} crosses {occupied} occupied cells; "
                    "Slamware may plan around them"
                )
            elif unknown:
                warnings.append(f"straight preview segment {idx + 1}->{idx + 2} crosses unknown cells")

    def make_point_cloud_cb(self, topic: str):
        def on_point_cloud(msg: PointCloud2) -> None:
            payload = self.point_cloud_to_payload(topic, msg)
            with self.state.lock:
                self.state.seq["point_cloud"] += 1
                payload["seq"] = self.state.seq["point_cloud"]
                self.state.point_cloud = payload

        return on_point_cloud

    def point_cloud_to_payload(self, topic: str, msg: PointCloud2) -> Dict[str, Any]:
        now = time.time()
        fields = {field.name: field for field in msg.fields}
        points: List[List[float]] = []
        bounds = {
            "min_x": None,
            "max_x": None,
            "min_y": None,
            "max_y": None,
            "min_z": None,
            "max_z": None,
        }
        total_points = int(msg.width) * int(msg.height)
        xyz_ok = all(name in fields and int(fields[name].datatype) == 7 for name in ("x", "y", "z"))

        if xyz_ok and total_points > 0 and msg.point_step > 0:
            endian = ">" if msg.is_bigendian else "<"
            data = bytes(msg.data)
            x_off = int(fields["x"].offset)
            y_off = int(fields["y"].offset)
            z_off = int(fields["z"].offset)
            step = max(1, math.ceil(total_points / self.max_cloud_points))
            for idx in range(0, total_points, step):
                row = idx // int(msg.width) if msg.width else 0
                col = idx % int(msg.width) if msg.width else 0
                base = row * int(msg.row_step) + col * int(msg.point_step)
                try:
                    x = struct.unpack_from(endian + "f", data, base + x_off)[0]
                    y = struct.unpack_from(endian + "f", data, base + y_off)[0]
                    z = struct.unpack_from(endian + "f", data, base + z_off)[0]
                except (struct.error, ValueError):
                    continue
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                points.append([round(float(x), 4), round(float(y), 4), round(float(z), 4)])
                for axis, value in (("x", x), ("y", y), ("z", z)):
                    lo = "min_" + axis
                    hi = "max_" + axis
                    bounds[lo] = value if bounds[lo] is None else min(bounds[lo], value)
                    bounds[hi] = value if bounds[hi] is None else max(bounds[hi], value)

        return {
            "received_at": now,
            "topic": topic,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "width": int(msg.width),
            "height": int(msg.height),
            "total_points": total_points,
            "sampled_points": len(points),
            "fields": [field.name for field in msg.fields],
            "point_step": int(msg.point_step),
            "row_step": int(msg.row_step),
            "is_dense": bool(msg.is_dense),
            "xyz_supported": bool(xyz_ok),
            "bounds": {key: finite_or_none(value, 4) for key, value in bounds.items()},
            "points": points,
        }


FRONTEND_DIST_DIR = FsPath(__file__).resolve().parent.parent / "frontend" / "dist"

_DEV_PLACEHOLDER_HTML = (
    "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<title>Base Sensor Dashboard</title></head>"
    "<body style=\"font-family:system-ui;padding:40px;line-height:1.7;color:#152235\">"
    "<h1>\u524d\u7aef\u5c1a\u672a\u6784\u5efa</h1>"
    "<p><b>\u5f00\u53d1\u6a21\u5f0f</b>\uff1a\u8fd0\u884c <code>cd frontend &amp;&amp; npm run dev</code>\uff0c"
    "\u9ed8\u8ba4 <a href=\"http://localhost:5173\">http://localhost:5173</a>\uff0c"
    "Vite \u4f1a\u628a <code>/api</code> \u4ee3\u7406\u5230\u672c\u670d\u52a1\u3002</p>"
    "<p><b>\u751f\u4ea7\u6a21\u5f0f</b>\uff1a\u5148 <code>npm run build</code> \u751f\u6210 "
    "<code>frontend/dist/</code>\uff0c\u672c\u670d\u52a1\u4f1a\u81ea\u52a8\u6258\u7ba1\u3002</p>"
    "</body></html>"
)

_STATIC_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".ico": "image/x-icon",
    ".webp": "image/webp",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".map": "application/json; charset=utf-8",
    ".txt": "text/plain; charset=utf-8",
}


def load_frontend_index() -> Optional[bytes]:
    index_path = FRONTEND_DIST_DIR / "index.html"
    try:
        return index_path.read_bytes()
    except OSError:
        return None


def resolve_static_asset(url_path: str) -> Optional[Tuple[bytes, str]]:
    rel = url_path.lstrip("/")
    if not rel:
        return None
    base = FRONTEND_DIST_DIR.resolve()
    try:
        candidate = (base / rel).resolve()
        candidate.relative_to(base)  # guard against path traversal
    except (OSError, ValueError):
        return None
    if not candidate.is_file():
        return None
    content_type = _STATIC_CONTENT_TYPES.get(candidate.suffix.lower(), "application/octet-stream")
    try:
        return candidate.read_bytes(), content_type
    except OSError:
        return None


class DashboardHandler(BaseHTTPRequestHandler):
    state: SharedState
    node_ref: BaseSensorNode
    point_store: SavedPointStore

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            index_bytes = load_frontend_index()
            if index_bytes is None:
                self.write_bytes(_DEV_PLACEHOLDER_HTML.encode("utf-8"), "text/html; charset=utf-8")
            else:
                self.write_bytes(index_bytes, "text/html; charset=utf-8")
        elif parsed.path == "/api/state":
            self.write_json(self.state.snapshot())
        elif parsed.path == "/api/points":
            self.write_json(self.point_store.list_payload())
        elif parsed.path in ("/api/lift_height", "/api/column_height/current"):
            self.write_json(self.node_ref.read_lift_height_status())
        elif parsed.path == "/api/fault_snapshots":
            self.write_json(self.node_ref.fault_logger.list_payload())
        elif parsed.path in ("/api/relocalization/status", "/api/relocalization"):
            self.write_json(self.node_ref.relocalization_status())
        elif parsed.path == "/api/mapping/status":
            self.write_json(self.node_ref.mapping_status())
        elif parsed.path in ("/api/mapping/list", "/api/mapping/files"):
            self.write_json({"ok": True, "maps_dir": str(self.node_ref.maps_dir), "saved_maps": self.node_ref.list_saved_maps()})
        elif parsed.path == "/api/health":
            snap = self.state.snapshot()
            self.write_json(
                {
                    "ok": True,
                    "uptime_s": snap["uptime_s"],
                    "freshness_s": snap["freshness_s"],
                    "seq": snap["seq"],
                    "has_scan": snap["scan"] is not None,
                    "has_map": snap["map"] is not None,
                    "has_odom": snap["odom"] is not None,
                    "has_sensors": snap["sensors"] is not None,
                    "has_point_cloud": snap["point_cloud"] is not None,
                    "has_global_plan_path": snap["navigation"]["global_plan_path"] is not None,
                    "has_robot_basic_state": snap["navigation"]["robot_basic_state"] is not None,
                    "has_arm_task_status": snap["arm_control"]["last_status"] is not None,
                    "fault_snapshot_count": len(snap.get("fault_snapshots", [])),
                }
            )
        elif not parsed.path.startswith("/api/"):
            asset = resolve_static_asset(parsed.path)
            if asset is not None:
                data, content_type = asset
                self.write_bytes(data, content_type)
            else:
                index_bytes = load_frontend_index()
                if index_bytes is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "not found")
                else:
                    self.write_bytes(index_bytes, "text/html; charset=utf-8")
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/api/navigation/start", "/api/nav/start"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.start_navigation(payload))
        elif parsed.path in ("/api/navigation/cancel", "/api/nav/cancel", "/api/navigation/stop", "/api/nav/stop"):
            self.write_json(self.node_ref.cancel_navigation())
        elif parsed.path == "/api/points/record_current":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.point_store.record_current(self.state.snapshot().get("odom"), payload))
        elif parsed.path == "/api/points/upsert":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.point_store.upsert(payload))
        elif parsed.path == "/api/points/delete":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.point_store.delete(payload))
        elif parsed.path in ("/api/actions/stop_all", "/api/actions/stop"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.stop_all_actions(payload))
        elif parsed.path == "/api/relocalization/save_anchor":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.save_relocalization_anchor(payload))
        elif parsed.path in ("/api/relocalization/run", "/api/relocalization/start"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.run_relocalization(payload))
        elif parsed.path in ("/api/mapping/start", "/api/mapping/start_collection"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.start_mapping(payload))
        elif parsed.path in ("/api/mapping/stop", "/api/mapping/stop_collection"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.stop_mapping(payload))
        elif parsed.path in ("/api/mapping/save", "/api/mapping/save_map"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.save_map(payload))
        elif parsed.path in ("/api/mapping/load", "/api/mapping/load_map"):
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.load_map(payload))
        elif parsed.path in ("/api/mapping/list", "/api/mapping/files"):
            self.write_json({"ok": True, "maps_dir": str(self.node_ref.maps_dir), "saved_maps": self.node_ref.list_saved_maps()})
        elif parsed.path == "/api/fault_snapshots/log":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.node_ref.fault_logger.capture_manual(payload))
        elif parsed.path == "/api/fault_snapshots/clear":
            self.write_json(self.node_ref.fault_logger.clear())
        elif parsed.path == "/api/actions/execute":
            payload = self.read_json_body()
            if payload is None:
                return
            self.write_json(self.execute_action(payload))
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def execute_action(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        action_type = str(payload.get("type") or payload.get("action") or "").strip()
        if action_type in ("column_height", "column_lift", "g1d_column_height"):
            return self.node_ref.execute_column_height_action(payload)
        if action_type in ("arm_task", "arm_pick", "arm_place", "arm_reset"):
            arm_payload = dict(payload)
            if action_type == "arm_pick":
                arm_payload["phase"] = "PICK"
            elif action_type == "arm_place":
                arm_payload["phase"] = "PLACE"
            elif action_type == "arm_reset":
                arm_payload["phase"] = "RESET"
            return self.node_ref.execute_arm_task(arm_payload)
        if action_type != "fake_pick_xiongmao":
            return {
                "ok": False,
                "error": "unsupported action type; expected arm_task, column_height, or fake_pick_xiongmao",
                "received": payload,
            }
        duration = finite_or_none(payload.get("duration_sec", 5), 3)
        if duration is None:
            duration = 5.0
        duration = max(0.0, min(30.0, float(duration)))
        started_at = now_iso()
        time.sleep(duration)
        return {
            "ok": True,
            "type": action_type,
            "name": payload.get("name") or "拾取熊猫烟",
            "started_at": started_at,
            "finished_at": now_iso(),
            "duration_sec": duration,
            "simulated": True,
            "message": "fake action finished; replace this endpoint with arm control later",
        }

    def read_json_body(self) -> Optional[Dict[str, Any]]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(HTTPStatus.BAD_REQUEST, "invalid content length")
            return None
        if length <= 0:
            return {}
        if length > 65536:
            self.send_error(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "request body too large")
            return None
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.send_error(HTTPStatus.BAD_REQUEST, f"invalid json: {exc}")
            return None
        if not isinstance(payload, dict):
            self.send_error(HTTPStatus.BAD_REQUEST, "json body must be an object")
            return None
        return payload

    def write_json(self, obj: Dict[str, Any]) -> None:
        data = json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.write_bytes(data, "application/json; charset=utf-8")

    def write_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Web dashboard for base laser/map/odom/basic sensors.")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18083)
    parser.add_argument("--scan-topic", default="/slamware_ros_sdk_server_node/scan")
    parser.add_argument("--map-topic", default="/slamware_ros_sdk_server_node/map")
    parser.add_argument("--odom-topic", action="append", default=["/slamware_ros_sdk_server_node/odom"])
    parser.add_argument("--sensors-topic", default="/slamware_ros_sdk_server_node/basic_sensors_values")
    parser.add_argument("--pointcloud-topic", action="append", default=["/ele_clouds"])
    parser.add_argument("--move-to-locations-topic", default="/slamware_ros_sdk_server_node/move_to_locations")
    parser.add_argument("--cancel-action-topic", default="/slamware_ros_sdk_server_node/cancel_action")
    parser.add_argument("--set-pose-topic", default="/slamware_ros_sdk_server_node/set_pose")
    parser.add_argument("--recover-localization-topic", default="/slamware_ros_sdk_server_node/recover_localization")
    parser.add_argument("--set-map-localization-topic", default="/slamware_ros_sdk_server_node/set_map_localization")
    parser.add_argument("--set-map-update-topic", default="/slamware_ros_sdk_server_node/set_map_update")
    parser.add_argument("--clear-map-topic", default="/slamware_ros_sdk_server_node/clear_map")
    parser.add_argument("--sync-get-stcm-service", default="/sync_get_stcm")
    parser.add_argument("--sync-set-stcm-service", default="/sync_set_stcm")
    parser.add_argument("--maps-dir", default="data/map")
    parser.add_argument("--sync-get-stcm-timeout-sec", type=float, default=30.0)
    parser.add_argument("--sync-set-stcm-timeout-sec", type=float, default=30.0)
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--global-plan-path-topic", default="/slamware_ros_sdk_server_node/global_plan_path")
    parser.add_argument("--robot-basic-state-topic", default="/slamware_ros_sdk_server_node/robot_basic_state")
    parser.add_argument("--slamware-state-topic", default="/slamware_ros_sdk_server_node/state")
    parser.add_argument("--arm-command-topic", default="/arm_control/task_command")
    parser.add_argument("--arm-status-topic", default="/arm_control/task_status")
    parser.add_argument("--arm-task-timeout-sec", type=float, default=120.0)
    parser.add_argument(
        "--arm-stop-phases",
        default="RESET,SUCTION_STOP,MOTION_STOP",
        help="Comma-separated arm phases published by the Stop button.",
    )
    parser.add_argument("--column-control-bin", default="/home/unitree/unitree_sdk2/build/bin/g1d_height_control")
    parser.add_argument("--column-control-workdir", default="/home/unitree/unitree_sdk2/build")
    parser.add_argument("--column-control-interface", default="eth0")
    parser.add_argument("--column-control-libdir", default="/home/unitree/unitree_sdk2/thirdparty/lib/aarch64")
    parser.add_argument("--column-height-timeout-sec", type=float, default=30.0)
    parser.add_argument("--column-height-min-m", type=float, default=-0.053)
    parser.add_argument("--column-height-max-m", type=float, default=0.376)
    parser.add_argument("--lift-height-url", default="http://127.0.0.1:28089/api/basic_status")
    parser.add_argument("--lift-height-timeout-sec", type=float, default=1.0)
    parser.add_argument("--raw-nav-linear-speed-mps", type=float, default=0.12)
    parser.add_argument("--raw-nav-angular-speed-radps", type=float, default=0.45)
    parser.add_argument("--raw-nav-position-tolerance-m", type=float, default=0.08)
    parser.add_argument("--raw-nav-yaw-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--points-file", default="data/nav_points.json")
    parser.add_argument("--relocalization-anchor-file", default="data/relocalization_anchor.json")
    parser.add_argument("--relocalization-search-radius-m", type=float, default=0.6)
    parser.add_argument("--relocalization-max-time-ms", type=int, default=8000)
    parser.add_argument("--relocalization-movement", default="NO_MOVE", choices=["NO_MOVE", "ROTATE_ONLY", "ANY"])
    parser.add_argument(
        "--min-localization-quality",
        type=int,
        default=-1,
        help="Block navigation when localization_quality is below this value. -1 means warn only.",
    )
    parser.add_argument("--max-cloud-points", type=int, default=6000)
    parser.add_argument("--max-track", type=int, default=1200)
    parser.add_argument("--fault-log-path", default="data/navigation_fault_snapshots.jsonl")
    args = parser.parse_args()

    state = SharedState(max_track=args.max_track)
    point_store = SavedPointStore(args.points_file)
    relocalization_store = RelocalizationAnchorStore(args.relocalization_anchor_file)
    rclpy.init()
    node = BaseSensorNode(
        state=state,
        scan_topic=args.scan_topic,
        map_topic=args.map_topic,
        odom_topics=args.odom_topic,
        sensors_topic=args.sensors_topic,
        pointcloud_topics=args.pointcloud_topic,
        move_to_locations_topic=args.move_to_locations_topic,
        cancel_action_topic=args.cancel_action_topic,
        set_pose_topic=args.set_pose_topic,
        recover_localization_topic=args.recover_localization_topic,
        set_map_localization_topic=args.set_map_localization_topic,
        set_map_update_topic=args.set_map_update_topic,
        clear_map_topic=args.clear_map_topic,
        sync_get_stcm_service=args.sync_get_stcm_service,
        sync_set_stcm_service=args.sync_set_stcm_service,
        maps_dir=args.maps_dir,
        sync_get_stcm_timeout_sec=args.sync_get_stcm_timeout_sec,
        sync_set_stcm_timeout_sec=args.sync_set_stcm_timeout_sec,
        cmd_vel_topic=args.cmd_vel_topic,
        global_plan_path_topic=args.global_plan_path_topic,
        robot_basic_state_topic=args.robot_basic_state_topic,
        slamware_state_topic=args.slamware_state_topic,
        arm_command_topic=args.arm_command_topic,
        arm_status_topic=args.arm_status_topic,
        arm_task_timeout_sec=args.arm_task_timeout_sec,
        arm_stop_phases=args.arm_stop_phases.split(","),
        column_control_bin=args.column_control_bin,
        column_control_workdir=args.column_control_workdir,
        column_control_interface=args.column_control_interface,
        column_control_libdir=args.column_control_libdir,
        column_height_timeout_sec=args.column_height_timeout_sec,
        column_height_min_m=args.column_height_min_m,
        column_height_max_m=args.column_height_max_m,
        lift_height_url=args.lift_height_url,
        lift_height_timeout_sec=args.lift_height_timeout_sec,
        raw_nav_linear_speed_mps=args.raw_nav_linear_speed_mps,
        raw_nav_angular_speed_radps=args.raw_nav_angular_speed_radps,
        raw_nav_position_tolerance_m=args.raw_nav_position_tolerance_m,
        raw_nav_yaw_tolerance_deg=args.raw_nav_yaw_tolerance_deg,
        min_localization_quality=args.min_localization_quality,
        relocalization_store=relocalization_store,
        relocalization_search_radius_m=args.relocalization_search_radius_m,
        relocalization_max_time_ms=args.relocalization_max_time_ms,
        relocalization_movement=args.relocalization_movement,
        max_cloud_points=args.max_cloud_points,
        fault_log_path=args.fault_log_path,
    )

    stop_event = threading.Event()

    def ros_spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=ros_spin, name="ros_spin", daemon=True)
    ros_thread.start()

    handler_cls = type(
        "BoundDashboardHandler",
        (DashboardHandler,),
        {"state": state, "node_ref": node, "point_store": point_store},
    )
    server = ThreadingHTTPServer((args.bind, args.port), handler_cls)

    def shutdown(_signum: int, _frame: Any) -> None:
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(f"serving base sensor dashboard on http://{args.bind}:{args.port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        stop_event.set()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
