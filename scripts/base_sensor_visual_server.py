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

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from slamware_ros_sdk.msg import (
    BasicSensorValueDataArray,
    CancelActionRequest,
    MoveToLocationsRequest,
    RobotBasicState,
)
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
        self.arm_task_status: Optional[Dict[str, Any]] = None
        self.last_arm_task_command: Optional[Dict[str, Any]] = None
        self.last_navigation_command: Optional[Dict[str, Any]] = None
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
            "navigation_command": 0,
            "arm_task_status": 0,
            "arm_task_command": 0,
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
                    "last_command": self.last_navigation_command,
                },
                "arm_control": {
                    "last_status": self.arm_task_status,
                    "last_command": self.last_arm_task_command,
                },
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
        raw_nav_linear_speed_mps: float,
        raw_nav_angular_speed_radps: float,
        raw_nav_position_tolerance_m: float,
        raw_nav_yaw_tolerance_deg: float,
        min_localization_quality: int,
        max_cloud_points: int,
    ) -> None:
        super().__init__("base_sensor_visual_server")
        self.state = state
        self.max_cloud_points = max(10, int(max_cloud_points))
        self.min_localization_quality = int(min_localization_quality)
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
        self.raw_nav_linear_speed_mps = max(0.02, min(0.35, float(raw_nav_linear_speed_mps)))
        self.raw_nav_angular_speed_radps = max(0.05, min(1.2, float(raw_nav_angular_speed_radps)))
        self.raw_nav_position_tolerance_m = max(0.03, min(0.3, float(raw_nav_position_tolerance_m)))
        self.raw_nav_yaw_tolerance_rad = math.radians(max(1.0, min(20.0, float(raw_nav_yaw_tolerance_deg))))
        self.raw_nav_lock = threading.Lock()
        self.raw_nav_stop = threading.Event()
        self.raw_nav_thread: Optional[threading.Thread] = None
        self.raw_nav_id: Optional[str] = None
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
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.arm_task_command_pub = self.create_publisher(String, arm_command_topic, qos)
        self.get_logger().info(
            f"subscribed scan={scan_topic} map={map_topic} sensors={sensors_topic} "
            f"pointclouds={pointcloud_topics} plan={global_plan_path_topic} "
            f"cmd_vel={cmd_vel_topic} arm_status={arm_status_topic}"
        )

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

    def execute_column_height_action(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        target = finite_or_none(
            payload.get(
                "target_height_m",
                payload.get("targetHeightM", payload.get("height_m", payload.get("heightM"))),
            ),
            4,
        )
        if target is None:
            return {"ok": False, "error": "target_height_m is required", "received": payload}
        if target < self.column_height_min_m or target > self.column_height_max_m:
            return {
                "ok": False,
                "error": (
                    f"target_height_m out of range "
                    f"[{self.column_height_min_m:.3f}, {self.column_height_max_m:.3f}]"
                ),
                "target_height_m": target,
                "received": payload,
            }
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
            "timeout_sec": timeout,
            "dry_run": dry_run,
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

        speed_ratio = payload.get("speed_ratio")
        if speed_ratio is not None:
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
        with self.state.lock:
            self.state.seq["navigation_command"] += 1
            command["seq"] = self.state.seq["navigation_command"]
            self.state.last_navigation_command = command
        if dry_run:
            return {"ok": True, "navigation_started": False, "dry_run": True, "command": command}
        with self.raw_nav_lock:
            if self.raw_nav_thread and self.raw_nav_thread.is_alive():
                return {"ok": False, "error": "raw cmd_vel navigation is already running", "command": command}
            self.raw_nav_stop.clear()
            self.raw_nav_id = nav_id
            self.raw_nav_thread = threading.Thread(
                target=self.raw_cmd_vel_worker,
                args=(nav_id, waypoints, float(yaw_result["yaw"]), linear_speed, angular_speed, position_tolerance, yaw_tolerance_rad, timeout),
                name="raw_cmd_vel_navigation",
                daemon=True,
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
                ids = ", ".join(str(item.get("id")) for item in hit_sensors)
                blockers.append(f"bumper/sonar impact active: {ids}")

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


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Base Sensor Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fa;
      --panel: #ffffff;
      --line: #d8dee8;
      --text: #152235;
      --muted: #66758a;
      --ok: #16a34a;
      --warn: #dc2626;
      --blue: #2563eb;
      --amber: #d97706;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 18px;
      border-bottom: 1px solid var(--line);
      background: #fff;
      position: sticky;
      top: 0;
      z-index: 5;
    }
    h1 { margin: 0; font-size: 20px; letter-spacing: 0; }
    .status {
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      white-space: nowrap;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--warn); }
    .dot.ok { background: var(--ok); }
    main {
      display: grid;
      grid-template-columns: minmax(360px, 1.2fr) minmax(340px, 1fr);
      gap: 14px;
      padding: 14px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
      overflow: hidden;
    }
    .map-section { grid-column: 1 / -1; }
    .map-workflow-grid {
      display: grid;
      grid-template-columns: minmax(420px, 1fr) minmax(300px, 360px);
      gap: 12px;
      padding: 10px;
    }
    .map-workflow-grid .canvas-wrap { padding: 0; }
    .workflow-panel {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
      min-width: 0;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }
    .workflow-head {
      padding: 10px;
      border-bottom: 1px solid var(--line);
      display: flex;
      justify-content: space-between;
      gap: 10px;
      align-items: baseline;
    }
    .workflow-head strong { font-size: 15px; }
    .workflow-control-row {
      padding: 10px;
      border-bottom: 1px solid var(--line);
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      background: #f8fafc;
    }
    .workflow-control-row button {
      width: 100%;
      min-height: 38px;
    }
    .workflow-actions {
      padding: 10px;
      border-bottom: 1px solid var(--line);
      display: grid;
      gap: 8px;
    }
    .action-builder {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      align-items: end;
    }
    .action-builder label {
      display: grid;
      gap: 3px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }
    .action-builder select,
    .action-builder input {
      width: 100%;
      border: 1px solid #b9c6d8;
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
      background: #fff;
      color: var(--text);
    }
    .action-builder-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }
    .workflow-list {
      display: grid;
      gap: 8px;
      padding: 10px;
      max-height: 520px;
      overflow: auto;
      align-content: start;
    }
    .workflow-step {
      position: relative;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      padding: 12px;
      overflow: hidden;
      cursor: grab;
      min-height: 0;
    }
    .workflow-step:active { cursor: grabbing; }
    .workflow-step.dragging {
      opacity: 0.55;
      border-style: dashed;
    }
    .workflow-step::before {
      content: "";
      position: absolute;
      inset: 0 auto 0 0;
      width: var(--progress, 0%);
      background: rgba(37, 99, 235, 0.10);
      transition: width 0.35s ease;
      pointer-events: none;
    }
    .workflow-step.running {
      border-color: #60a5fa;
      box-shadow: 0 0 0 1px rgba(37, 99, 235, 0.18);
    }
    .workflow-step.done {
      border-color: #86efac;
      background: #f0fdf4;
    }
    .workflow-step.error {
      border-color: #fecaca;
      background: #fff7f7;
    }
    .workflow-step-content {
      position: relative;
      display: grid;
      gap: 7px;
      min-width: 0;
    }
    .workflow-title {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto auto;
      align-items: center;
      gap: 8px;
      font-weight: 750;
      min-width: 0;
    }
    .workflow-title-left {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .drag-handle {
      color: var(--muted);
      font-weight: 900;
      letter-spacing: 0;
      cursor: grab;
      user-select: none;
    }
    .workflow-delete {
      padding: 4px 9px;
      font-size: 12px;
      font-weight: 700;
      border-color: #fecaca;
      color: #b91c1c;
      background: #fff7f7;
      white-space: nowrap;
    }
    .workflow-detail {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.35;
      overflow-wrap: anywhere;
    }
    .workflow-progress-text {
      color: #7c8aa0;
      font-size: 12px;
      line-height: 1.2;
    }
    .workflow-badge {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-width: 44px;
      border-radius: 999px;
      padding: 2px 8px;
      background: #e2e8f0;
      color: #334155;
      font-size: 12px;
      white-space: nowrap;
    }
    .workflow-step.running .workflow-badge {
      background: #dbeafe;
      color: #1d4ed8;
    }
    .workflow-step.done .workflow-badge {
      background: #dcfce7;
      color: #15803d;
    }
    .workflow-step.error .workflow-badge {
      background: #fee2e2;
      color: #b91c1c;
    }
    .workflow-pulse {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #2563eb;
      display: inline-block;
      margin-right: 6px;
      visibility: hidden;
    }
    .workflow-step.running .workflow-pulse {
      animation: workflowPulse 0.9s ease-in-out infinite;
      visibility: visible;
    }
    @keyframes workflowPulse {
      0%, 100% { transform: scale(0.8); opacity: 0.55; }
      50% { transform: scale(1.25); opacity: 1; }
    }
    .panel-head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }
    h2 { margin: 0; font-size: 15px; }
    .meta { color: var(--muted); font-size: 12px; }
    .canvas-wrap { position: relative; padding: 10px; }
    canvas {
      width: 100%;
      display: block;
      border: 1px solid #e4e9f1;
      background: #fff;
      border-radius: 6px;
      aspect-ratio: 1 / 1;
    }
    #mapCanvas { aspect-ratio: 1.45 / 1; }
    #mapCanvas { cursor: crosshair; }
    #cloudCanvas { aspect-ratio: 1.45 / 1; cursor: grab; }
    #cloudCanvas:active { cursor: grabbing; }
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      padding: 10px;
    }
    .sensor-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #fbfcfe;
    }
    .sensor-card.hit { border-color: #fecaca; background: #fff7f7; }
    .sensor-title {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      font-weight: 700;
      margin-bottom: 6px;
    }
    .kv {
      display: grid;
      grid-template-columns: auto 1fr;
      gap: 4px 8px;
      color: var(--muted);
      font-size: 12px;
    }
    .kv strong { color: var(--text); font-weight: 600; text-align: right; }
    .readout {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      padding: 10px;
      border-top: 1px solid var(--line);
    }
    .toolbar {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
      padding: 0 10px 10px;
    }
    button {
      border: 1px solid #b9c6d8;
      background: #f8fafc;
      color: var(--text);
      border-radius: 6px;
      padding: 8px 12px;
      font-weight: 700;
      cursor: pointer;
    }
    button.primary {
      background: #2563eb;
      border-color: #1d4ed8;
      color: #fff;
    }
    button.danger {
      background: #fff7f7;
      border-color: #fecaca;
      color: #b91c1c;
    }
    button.active {
      background: #16a34a;
      border-color: #15803d;
      color: #fff;
    }
    button:disabled {
      cursor: not-allowed;
      opacity: 0.55;
    }
    .nav-hint {
      color: var(--muted);
      font-size: 12px;
      margin-left: auto;
    }
    .nav-mode-toggle {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      min-height: 38px;
      padding: 0 10px;
      border: 1px solid #f59e0b;
      border-radius: 6px;
      background: #fffbeb;
      color: #92400e;
      font-size: 12px;
      font-weight: 800;
      white-space: nowrap;
    }
    .nav-mode-toggle input {
      width: 16px;
      height: 16px;
      margin: 0;
      accent-color: #d97706;
    }
    .heading-input {
      width: 96px;
      border: 1px solid #b9c6d8;
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
    }
    .point-panel-body {
      display: grid;
      grid-template-columns: minmax(280px, 0.85fr) minmax(280px, 1fr);
      gap: 10px;
      padding: 10px;
    }
    .point-form {
      display: grid;
      gap: 8px;
      min-width: 0;
    }
    .point-form label {
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }
    .point-form input,
    .point-form textarea {
      width: 100%;
      border: 1px solid #b9c6d8;
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
      background: #fff;
      color: var(--text);
    }
    .point-form textarea {
      min-height: 62px;
      resize: vertical;
      font-family: ui-monospace, SFMono-Regular, Consolas, "Liberation Mono", monospace;
      font-size: 12px;
    }
    .point-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .point-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      padding: 0;
    }
    .point-message {
      min-height: 20px;
      color: var(--muted);
      font-size: 12px;
    }
    .point-message.bad { color: var(--warn); }
    .point-message.ok { color: var(--ok); }
    .point-list {
      display: grid;
      align-content: start;
      gap: 8px;
      max-height: 420px;
      overflow: auto;
      min-width: 0;
    }
    .point-item {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 9px;
      background: #fbfcfe;
      cursor: pointer;
    }
    .point-item.active {
      border-color: #0ea5e9;
      background: #f0f9ff;
    }
    .point-title {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      font-weight: 750;
    }
    .point-detail {
      margin-top: 4px;
      color: var(--muted);
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .nav-status {
      padding: 0 10px 10px;
      color: var(--muted);
      font-size: 12px;
    }
    .nav-status strong { color: var(--text); }
    .nav-status.warn strong { color: var(--amber); }
    .nav-status.bad strong { color: var(--warn); }
    .metric {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      background: #fbfcfe;
      min-height: 58px;
    }
    .metric .label { color: var(--muted); font-size: 12px; }
    .metric .value { font-size: 18px; font-weight: 750; margin-top: 4px; overflow-wrap: anywhere; }
    pre {
      margin: 0;
      padding: 10px;
      max-height: 180px;
      overflow: auto;
      background: #102033;
      color: #d8e6f7;
      font-size: 12px;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .map-workflow-grid { grid-template-columns: 1fr; }
      .readout { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .point-panel-body { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Base Sensor Dashboard</h1>
    <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">connecting</span></div>
  </header>
  <main>
    <section class="map-section">
      <div class="panel-head">
        <h2>SLAM Map + Odometry</h2>
        <span id="mapMeta" class="meta">waiting</span>
      </div>
      <div class="map-workflow-grid">
        <div>
      <div class="canvas-wrap"><canvas id="mapCanvas"></canvas></div>
      <div class="toolbar">
        <button id="undoWaypointBtn">撤销点</button>
        <button id="clearWaypointsBtn">清空点</button>
        <button id="setHeadingBtn">设置朝向</button>
        <button id="clearHeadingBtn">清除朝向</button>
        <input id="headingDegInput" class="heading-input" type="number" step="1" min="-180" max="180" placeholder="角度°" />
        <button id="applyHeadingDegBtn">应用角度</button>
        <label class="nav-mode-toggle" title="使用 Slamware KeyPoints 模式：按指定路径走，不自动绕障；遇到障碍会停止。">
          <input id="directNoAvoidanceMode" type="checkbox" />
          直连不绕障
        </label>
        <label class="nav-mode-toggle" title="绕开 Slamware 导航，直接发布 /cmd_vel；不会自动避障，请只在确认路径安全时使用。">
          <input id="rawCmdVelNoAvoidanceMode" type="checkbox" />
          裸控无避障
        </label>
        <span id="navHint" class="nav-hint">在地图上点击快速增加导航动作</span>
      </div>
      <div id="navStatus" class="nav-status">导航：等待选点</div>
      <div class="readout">
        <div class="metric"><div class="label">X</div><div id="odomX" class="value">--</div></div>
        <div class="metric"><div class="label">Y</div><div id="odomY" class="value">--</div></div>
        <div class="metric"><div class="label">Yaw</div><div id="odomYaw" class="value">--</div></div>
        <div class="metric"><div class="label">Track</div><div id="trackCount" class="value">--</div></div>
        <div class="metric"><div class="label">航点</div><div id="waypointCount" class="value">0</div></div>
        <div class="metric"><div class="label">规划路径</div><div id="planCount" class="value">--</div></div>
        <div class="metric"><div class="label">定位</div><div id="localizationState" class="value">--</div></div>
        <div class="metric"><div class="label">导航指令</div><div id="navCommand" class="value">--</div></div>
        <div class="metric"><div class="label">目标角度</div><div id="targetYaw" class="value">--</div></div>
      </div>
        </div>
        <aside class="workflow-panel">
          <div class="workflow-head">
            <strong>动作链</strong>
            <span id="workflowMeta" class="meta">待执行</span>
          </div>
          <div class="workflow-control-row">
            <button id="runWorkflowBtn" class="primary">执行动作链</button>
            <button id="startNavigationBtn">仅执行导航</button>
            <button id="stopNavigationBtn" class="danger">停止</button>
            <button id="clearWorkflowBtn">清空动作</button>
          </div>
          <div class="workflow-actions">
            <div class="action-builder">
              <label>动作类型
                <select id="newActionType">
                  <option value="navigate">导航</option>
                  <option value="arm_pick">机械臂抓取</option>
                  <option value="arm_place">机械臂放置</option>
                  <option value="arm_reset">机械臂复位</option>
                  <option value="column_height">立柱升降</option>
                </select>
              </label>
              <label class="nav-action-field">点位库
                <select id="newActionPointSelect">
                  <option value="">手动输入 / 地图点选</option>
                </select>
              </label>
              <label class="nav-action-field">X m
                <input id="newActionX" type="number" step="0.001" placeholder="地图 X" />
              </label>
              <label class="nav-action-field">Y m
                <input id="newActionY" type="number" step="0.001" placeholder="地图 Y" />
              </label>
              <label class="nav-action-field">Yaw deg
                <input id="newActionYawDeg" type="number" step="0.1" placeholder="当前" />
              </label>
              <label class="arm-action-field">抓取目标
                <select id="newArmTargetObject">
                  <option value="XiongMao">XiongMao / 熊猫烟</option>
                  <option value="Xizi_Liqun">Xizi_Liqun / 西子利群</option>
                </select>
              </label>
              <label class="arm-action-field">超时 s
                <input id="newArmTimeoutSec" type="number" step="1" min="1" max="600" value="120" />
              </label>
              <label class="column-action-field">目标高度 m
                <input id="newColumnTargetHeightM" type="number" step="0.001" min="-1" max="1" value="0" />
              </label>
              <label class="column-action-field">超时 s
                <input id="newColumnTimeoutSec" type="number" step="1" min="1" max="180" value="30" />
              </label>
            </div>
            <div class="action-builder-actions">
              <button id="fillCurrentPoseBtn">填当前位置</button>
              <button id="addWorkflowActionBtn" class="primary">增加动作</button>
              <button id="resetWorkflowBtn">重置状态</button>
            </div>
            <div class="workflow-detail">导航动作可以从点位库选，也可以手动填位姿或直接点地图；机械臂抓取会把目标标签发给手臂模块；立柱升降会调用 G1D 高度控制原始动作。动作卡片可拖拽排序。</div>
          </div>
          <div id="workflowList" class="workflow-list"></div>
        </aside>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>点位库 / 动作预留</h2>
        <span id="pointsMeta" class="meta">waiting</span>
      </div>
      <div class="point-panel-body">
        <div class="point-form">
          <div class="point-grid">
            <div>
              <label for="pointNameInput">名称</label>
              <input id="pointNameInput" type="text" placeholder="例如：货架 A 点" />
            </div>
            <div>
              <label for="pointYawDegInput">朝向 deg</label>
              <input id="pointYawDegInput" type="number" step="0.1" placeholder="0" />
            </div>
            <div>
              <label for="pointXInput">X m</label>
              <input id="pointXInput" type="number" step="0.001" placeholder="地图 X" />
            </div>
            <div>
              <label for="pointYInput">Y m</label>
              <input id="pointYInput" type="number" step="0.001" placeholder="地图 Y" />
            </div>
          </div>
          <div>
            <label for="pointNoteInput">备注</label>
            <textarea id="pointNoteInput" placeholder="可写用途、货架、调试说明"></textarea>
          </div>
          <div>
            <label for="pointActionsInput">动作 JSON（预留，不执行）</label>
            <textarea id="pointActionsInput" spellcheck="false">[]</textarea>
          </div>
          <div class="point-actions">
            <button id="recordCurrentPointBtn" class="primary">记录当前位置</button>
            <button id="newPointBtn">新建/清空</button>
            <button id="savePointBtn">保存点位</button>
            <button id="addPointToNavBtn">加入导航</button>
            <button id="deletePointBtn" class="danger">删除</button>
          </div>
          <div id="pointMessage" class="point-message">动作字段只保存，不执行；后续接机械臂时复用。</div>
        </div>
        <div id="savedPointList" class="point-list"></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Laser Scan</h2>
        <span id="scanMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="scanCanvas"></canvas></div>
      <div class="readout">
        <div class="metric"><div class="label">Valid</div><div id="scanValid" class="value">--</div></div>
        <div class="metric"><div class="label">Min Range</div><div id="scanMin" class="value">--</div></div>
        <div class="metric"><div class="label">Frame</div><div id="scanFrame" class="value">--</div></div>
        <div class="metric"><div class="label">Age</div><div id="scanAge" class="value">--</div></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Ultrasonic / Bumper Sensors</h2>
        <span id="sensorMeta" class="meta">waiting</span>
      </div>
      <div id="sensorGrid" class="sensor-grid"></div>
    </section>

    <section>
      <div class="panel-head">
        <h2>3D Point Cloud</h2>
        <span id="cloudMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="cloudCanvas"></canvas></div>
      <div class="readout">
        <div class="metric"><div class="label">Topic</div><div id="cloudTopic" class="value">--</div></div>
        <div class="metric"><div class="label">Points</div><div id="cloudPoints" class="value">--</div></div>
        <div class="metric"><div class="label">Frame</div><div id="cloudFrame" class="value">--</div></div>
        <div class="metric"><div class="label">Age</div><div id="cloudAge" class="value">--</div></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Raw State</h2>
        <span class="meta">/api/state</span>
      </div>
      <pre id="rawState">{}</pre>
    </section>
  </main>

  <script>
    const mapCanvas = document.getElementById('mapCanvas');
    const scanCanvas = document.getElementById('scanCanvas');
    const cloudCanvas = document.getElementById('cloudCanvas');
    let cachedMapSeq = -1;
    let cachedMapImage = null;
    let lastState = null;
    let currentMapGeom = null;
    let selectedWaypoints = [];
    let savedPoints = [];
    let editingPointId = null;
    let workflowActions = [];
    let draggedActionId = null;
    let workflowRun = {
      running: false,
      mode: 'idle',
      currentIndex: -1,
      completed: {},
      error: '',
      navigationStartedAt: 0,
      actionStartedAt: 0,
      actionDurationSec: 5,
      note: ''
    };
    let headingMode = false;
    let finalHeadingPoint = null;
    let manualHeadingDeg = null;
    let cloudYaw = -0.75;
    let cloudPitch = 0.65;
    let cloudDragging = false;
    let cloudLast = { x: 0, y: 0 };
    const NAV_REACH_DISTANCE_M = 0.18;
    const NAV_REACH_YAW_DEG = 4.0;
    const NAV_REACH_STABLE_MS = 1200;
    const NAV_IDLE_ACCEPT_DISTANCE_M = 0.22;
    const NAV_IDLE_ACCEPT_YAW_DEG = 12.0;
    const NAV_IDLE_STABLE_MS = 1800;
    const NAV_ODOM_STILL_M = 0.008;
    const NAV_ODOM_STILL_YAW_DEG = 0.8;

    function resizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(320, Math.floor(rect.width * dpr));
      const h = Math.max(240, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
        cachedMapSeq = -1;
      }
    }

    function fmt(value, digits = 2, suffix = '') {
      if (value === null || value === undefined || Number.isNaN(Number(value))) return '--';
      return `${Number(value).toFixed(digits)}${suffix}`;
    }

    function escapeHtml(value) {
      return String(value ?? '').replace(/[&<>"']/g, ch => ({
        '&': '&amp;',
        '<': '&lt;',
        '>': '&gt;',
        '"': '&quot;',
        "'": '&#39;'
      })[ch]);
    }

    function normalizeAngle(angle) {
      return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    function radToDeg(angle) {
      return normalizeAngle(angle) * 180 / Math.PI;
    }

    function makeActionId(prefix = 'act') {
      return `${prefix}-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
    }

    function getNavigationActions() {
      return workflowActions.filter(action => action.type === 'navigate');
    }

    function getLastNavigationAction() {
      for (let i = workflowActions.length - 1; i >= 0; i--) {
        if (workflowActions[i].type === 'navigate') return workflowActions[i];
      }
      return null;
    }

    function getSavedPointById(pointId) {
      return savedPoints.find(point => point.id === pointId) || null;
    }

    function syncSelectedWaypointsFromActions() {
      const navActions = getNavigationActions();
      selectedWaypoints = navActions.map(action => ({
        x: Number(action.x),
        y: Number(action.y),
        actionId: action.id
      }));
      const lastNav = navActions[navActions.length - 1];
      manualHeadingDeg = lastNav && Number.isFinite(Number(lastNav.yawDeg))
        ? Number(lastNav.yawDeg)
        : null;
    }

    function resetWorkflowAfterEdit() {
      syncSelectedWaypointsFromActions();
      resetWorkflowRun('待执行');
      refreshMapUi();
    }

    function setLastNavigationYaw(yawDeg) {
      const action = getLastNavigationAction();
      if (!action) return false;
      action.yawDeg = Number(radToDeg(normalizeAngle(Number(yawDeg) * Math.PI / 180)).toFixed(3));
      syncSelectedWaypointsFromActions();
      return true;
    }

    function addWorkflowAction(action) {
      workflowActions.push(action);
      resetWorkflowAfterEdit();
    }

    function removeWorkflowAction(actionId) {
      workflowActions = workflowActions.filter(action => action.id !== actionId);
      resetWorkflowAfterEdit();
    }

    function reorderWorkflowAction(dragId, targetId) {
      if (!dragId || !targetId || dragId === targetId) return;
      const from = workflowActions.findIndex(action => action.id === dragId);
      const to = workflowActions.findIndex(action => action.id === targetId);
      if (from < 0 || to < 0) return;
      const [item] = workflowActions.splice(from, 1);
      workflowActions.splice(to, 0, item);
      resetWorkflowAfterEdit();
    }

    function setStatus(ok, text) {
      document.getElementById('statusDot').classList.toggle('ok', ok);
      document.getElementById('statusText').textContent = text;
    }

    function mapToCanvas(map, x, y, geom) {
      const mx = (x - map.origin.x) / map.resolution;
      const my = (y - map.origin.y) / map.resolution;
      return {
        x: geom.ox + mx * geom.scale,
        y: geom.oy + (map.height - my) * geom.scale
      };
    }

    function canvasToMap(map, canvasX, canvasY, geom) {
      if (!map || !geom) return null;
      const mx = (canvasX - geom.ox) / geom.scale;
      const my = map.height - ((canvasY - geom.oy) / geom.scale);
      if (mx < 0 || my < 0 || mx >= map.width || my >= map.height) return null;
      return {
        x: map.origin.x + mx * map.resolution,
        y: map.origin.y + my * map.resolution
      };
    }

    function eventToCanvas(canvas, ev) {
      const rect = canvas.getBoundingClientRect();
      return {
        x: (ev.clientX - rect.left) * (canvas.width / rect.width),
        y: (ev.clientY - rect.top) * (canvas.height / rect.height)
      };
    }

    function drawMapPolyline(ctx, map, geom, points, color, width, dashed = false) {
      if (!points || points.length < 2) return;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = width * (window.devicePixelRatio || 1);
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      if (dashed) ctx.setLineDash([10 * (window.devicePixelRatio || 1), 6 * (window.devicePixelRatio || 1)]);
      ctx.beginPath();
      points.forEach((p, i) => {
        const c = mapToCanvas(map, p.x, p.y, geom);
        if (i === 0) ctx.moveTo(c.x, c.y);
        else ctx.lineTo(c.x, c.y);
      });
      ctx.stroke();
      ctx.restore();
    }

    function computeTargetYaw(state = lastState) {
      if (!selectedWaypoints.length) return null;
      const lastPoint = selectedWaypoints[selectedWaypoints.length - 1];
      if (manualHeadingDeg !== null && manualHeadingDeg !== undefined) {
        const yaw = normalizeAngle(Number(manualHeadingDeg) * Math.PI / 180);
        return { yaw, yawDeg: radToDeg(yaw), source: 'manual_heading_input_deg', label: '输入' };
      }
      if (finalHeadingPoint) {
        const dx = finalHeadingPoint.x - lastPoint.x;
        const dy = finalHeadingPoint.y - lastPoint.y;
        if (Math.hypot(dx, dy) > 0.001) {
          const yaw = normalizeAngle(Math.atan2(dy, dx));
          return { yaw, yawDeg: radToDeg(yaw), source: 'manual_heading_arrow', label: '手动' };
        }
      }
      if (selectedWaypoints.length >= 2) {
        const prev = selectedWaypoints[selectedWaypoints.length - 2];
        const dx = lastPoint.x - prev.x;
        const dy = lastPoint.y - prev.y;
        if (Math.hypot(dx, dy) > 0.001) {
          const yaw = normalizeAngle(Math.atan2(dy, dx));
          return { yaw, yawDeg: radToDeg(yaw), source: 'auto_last_segment', label: '自动' };
        }
      }
      if (state?.odom?.yaw !== null && state?.odom?.yaw !== undefined) {
        const yaw = normalizeAngle(Number(state.odom.yaw));
        return { yaw, yawDeg: radToDeg(yaw), source: 'current_odom', label: '当前' };
      }
      return null;
    }

    function drawArrow(ctx, start, end, color, label) {
      const dpr = window.devicePixelRatio || 1;
      const angle = Math.atan2(end.y - start.y, end.x - start.x);
      const head = 12 * dpr;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 4 * dpr;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(end.x, end.y);
      ctx.lineTo(end.x - head * Math.cos(angle - Math.PI / 6), end.y - head * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(end.x - head * Math.cos(angle + Math.PI / 6), end.y - head * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fill();
      if (label) {
        ctx.font = `${12 * dpr}px system-ui, sans-serif`;
        ctx.lineWidth = 4 * dpr;
        ctx.strokeStyle = '#ffffff';
        ctx.strokeText(label, end.x + 8 * dpr, end.y - 8 * dpr);
        ctx.fillText(label, end.x + 8 * dpr, end.y - 8 * dpr);
      }
      ctx.restore();
    }

    function drawTargetHeading(ctx, map, geom, state) {
      const target = computeTargetYaw(state);
      if (!target || !selectedWaypoints.length) return;
      const lastPoint = selectedWaypoints[selectedWaypoints.length - 1];
      const start = mapToCanvas(map, lastPoint.x, lastPoint.y, geom);
      let end;
      if (finalHeadingPoint && manualHeadingDeg === null) {
        end = mapToCanvas(map, finalHeadingPoint.x, finalHeadingPoint.y, geom);
      } else {
        const dpr = window.devicePixelRatio || 1;
        const len = 56 * dpr;
        end = {
          x: start.x + Math.cos(target.yaw) * len,
          y: start.y - Math.sin(target.yaw) * len
        };
      }
      drawArrow(ctx, start, end, '#16a34a', `${target.label} ${target.yawDeg.toFixed(1)}°`);
    }

    function drawSelectedWaypoints(ctx, map, geom) {
      drawMapPolyline(ctx, map, geom, selectedWaypoints, '#9333ea', 3, true);
      const dpr = window.devicePixelRatio || 1;
      selectedWaypoints.forEach((p, idx) => {
        const c = mapToCanvas(map, p.x, p.y, geom);
        ctx.fillStyle = '#9333ea';
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3 * dpr;
        ctx.beginPath();
        ctx.arc(c.x, c.y, 8 * dpr, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#ffffff';
        ctx.font = `${12 * dpr}px system-ui, sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(idx + 1), c.x, c.y);
      });
    }

    function drawSavedPoints(ctx, map, geom) {
      if (!savedPoints.length) return;
      const dpr = window.devicePixelRatio || 1;
      savedPoints.forEach(point => {
        if (point.x === null || point.x === undefined || point.y === null || point.y === undefined) return;
        const c = mapToCanvas(map, point.x, point.y, geom);
        const active = point.id === editingPointId;
        ctx.save();
        ctx.fillStyle = active ? '#f59e0b' : '#0ea5e9';
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3 * dpr;
        ctx.beginPath();
        ctx.arc(c.x, c.y, (active ? 9 : 7) * dpr, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        if (point.yaw !== null && point.yaw !== undefined) {
          const len = 28 * dpr;
          drawArrow(ctx, c, { x: c.x + Math.cos(point.yaw) * len, y: c.y - Math.sin(point.yaw) * len }, '#0f766e', '');
        }
        ctx.font = `${11 * dpr}px system-ui, sans-serif`;
        ctx.lineWidth = 3 * dpr;
        ctx.strokeStyle = '#ffffff';
        ctx.fillStyle = '#0f172a';
        const label = point.name || 'Point';
        ctx.strokeText(label, c.x + 10 * dpr, c.y - 10 * dpr);
        ctx.fillText(label, c.x + 10 * dpr, c.y - 10 * dpr);
        ctx.restore();
      });
    }

    function drawGlobalPlan(ctx, map, geom, nav) {
      const path = nav?.global_plan_path?.poses || [];
      drawMapPolyline(ctx, map, geom, path, '#d97706', 4, false);
    }

    function drawMap(state) {
      resizeCanvas(mapCanvas);
      const ctx = mapCanvas.getContext('2d');
      ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
      ctx.fillStyle = '#f8fafc';
      ctx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
      const map = state.map;
      if (!map || !map.data) {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for map', 16, 24);
        return;
      }

      if (cachedMapSeq !== map.seq) {
        const off = document.createElement('canvas');
        off.width = map.width;
        off.height = map.height;
        const offCtx = off.getContext('2d');
        const img = offCtx.createImageData(map.width, map.height);
        for (let y = 0; y < map.height; y++) {
          for (let x = 0; x < map.width; x++) {
            const srcIdx = y * map.width + x;
            const dstY = map.height - 1 - y;
            const dstIdx = (dstY * map.width + x) * 4;
            const v = map.data[srcIdx];
            let r = 224, g = 229, b = 236;
            if (v === 0) { r = 255; g = 255; b = 255; }
            else if (v > 70) { r = 28; g = 38; b = 52; }
            else if (v > 0) { r = 120; g = 132; b = 150; }
            img.data[dstIdx] = r;
            img.data[dstIdx + 1] = g;
            img.data[dstIdx + 2] = b;
            img.data[dstIdx + 3] = 255;
          }
        }
        offCtx.putImageData(img, 0, 0);
        cachedMapImage = off;
        cachedMapSeq = map.seq;
      }

      const scale = Math.min(mapCanvas.width / map.width, mapCanvas.height / map.height);
      const dw = map.width * scale;
      const dh = map.height * scale;
      const geom = { scale, ox: (mapCanvas.width - dw) / 2, oy: (mapCanvas.height - dh) / 2 };
      currentMapGeom = geom;
      ctx.drawImage(cachedMapImage, geom.ox, geom.oy, dw, dh);

      drawGlobalPlan(ctx, map, geom, state.navigation);

      if (state.track && state.track.length > 1) {
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = Math.max(2, 2 * (window.devicePixelRatio || 1));
        ctx.beginPath();
        state.track.forEach((p, i) => {
          const c = mapToCanvas(map, p.x, p.y, geom);
          if (i === 0) ctx.moveTo(c.x, c.y);
          else ctx.lineTo(c.x, c.y);
        });
        ctx.stroke();
      }

      if (state.odom) {
        const c = mapToCanvas(map, state.odom.x, state.odom.y, geom);
        const yaw = state.odom.yaw || 0;
        const size = 12 * (window.devicePixelRatio || 1);
        ctx.save();
        ctx.translate(c.x, c.y);
        ctx.rotate(-yaw);
        ctx.fillStyle = '#dc2626';
        ctx.beginPath();
        ctx.moveTo(size, 0);
        ctx.lineTo(-size * 0.75, size * 0.6);
        ctx.lineTo(-size * 0.45, 0);
        ctx.lineTo(-size * 0.75, -size * 0.6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      drawSavedPoints(ctx, map, geom);
      drawSelectedWaypoints(ctx, map, geom);
      drawTargetHeading(ctx, map, geom, state);

      document.getElementById('mapMeta').textContent =
        `${map.width}x${map.height}, ${fmt(map.resolution, 3, 'm/cell')}`;
    }

    function drawScan(state) {
      resizeCanvas(scanCanvas);
      const ctx = scanCanvas.getContext('2d');
      const w = scanCanvas.width, h = scanCanvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#fbfcfe';
      ctx.fillRect(0, 0, w, h);
      const scan = state.scan;
      const cx = w / 2, cy = h * 0.58;
      const maxM = scan?.range_max ? Math.min(scan.range_max, 8) : 8;
      const scale = Math.min(w, h) * 0.42 / maxM;

      ctx.strokeStyle = '#d8dee8';
      ctx.lineWidth = 1;
      for (let m = 1; m <= maxM; m++) {
        ctx.beginPath();
        ctx.arc(cx, cy, m * scale, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.strokeStyle = '#94a3b8';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - maxM * scale);
      ctx.stroke();

      ctx.fillStyle = '#2563eb';
      if (scan && scan.ranges) {
        for (let i = 0; i < scan.ranges.length; i++) {
          const r = scan.ranges[i];
          if (r === null) continue;
          const a = scan.angle_min + i * scan.angle_increment;
          const x = r * Math.cos(a);
          const y = r * Math.sin(a);
          const px = cx - y * scale;
          const py = cy - x * scale;
          ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
        }
      } else {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for laser scan', 16, 24);
      }
    }

    function renderSensors(state) {
      const grid = document.getElementById('sensorGrid');
      const sensors = state.sensors?.items || [];
      document.getElementById('sensorMeta').textContent =
        sensors.length ? `${sensors.length} sensors, age ${fmt(state.freshness_s.sensors, 2, 's')}` : 'waiting';
      grid.innerHTML = sensors.map(s => {
        const value = s.raw_value_is_finite ? fmt(s.value, 3) : 'inf';
        const hit = s.is_in_impact ? 'hit' : '';
        return `<div class="sensor-card ${hit}">
          <div class="sensor-title"><span>#${s.id} ${s.sensor_type_name}</span><span>${s.is_in_impact ? 'HIT' : 'OK'}</span></div>
          <div class="kv">
            <span>value</span><strong>${value}</strong>
            <span>impact</span><strong>${s.impact_type_name}</strong>
            <span>pose</span><strong>x=${fmt(s.pose.x, 3)}, y=${fmt(s.pose.y, 3)}, z=${fmt(s.pose.z, 3)}</strong>
            <span>freq</span><strong>${fmt(s.refresh_freq, 1, 'Hz')}</strong>
          </div>
        </div>`;
      }).join('') || '<div class="sensor-card">waiting for sensors</div>';
    }

    function rotatePoint(p) {
      const cy = Math.cos(cloudYaw), sy = Math.sin(cloudYaw);
      const cp = Math.cos(cloudPitch), sp = Math.sin(cloudPitch);
      const x1 = p[0] * cy - p[1] * sy;
      const y1 = p[0] * sy + p[1] * cy;
      const z1 = p[2];
      return [x1, y1 * cp - z1 * sp, y1 * sp + z1 * cp];
    }

    function drawCloudAxes(ctx, cx, cy, scale) {
      const axes = [
        { p: [0.6, 0, 0], c: '#dc2626', t: 'X' },
        { p: [0, 0.6, 0], c: '#16a34a', t: 'Y' },
        { p: [0, 0, 0.6], c: '#2563eb', t: 'Z' }
      ];
      ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
      axes.forEach(a => {
        const r = rotatePoint(a.p);
        const x = cx + r[0] * scale;
        const y = cy - r[1] * scale;
        ctx.strokeStyle = a.c;
        ctx.fillStyle = a.c;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(x, y);
        ctx.stroke();
        ctx.fillText(a.t, x + 4, y - 4);
      });
    }

    function drawCloud(state) {
      resizeCanvas(cloudCanvas);
      const ctx = cloudCanvas.getContext('2d');
      const w = cloudCanvas.width, h = cloudCanvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#fbfcfe';
      ctx.fillRect(0, 0, w, h);
      const cloud = state.point_cloud;
      const cx = w / 2, cy = h / 2;
      drawCloudAxes(ctx, cx, cy, Math.min(w, h) * 0.22);

      if (!cloud || !cloud.points || cloud.points.length === 0) {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for PointCloud2 data', 16, 24);
        document.getElementById('cloudMeta').textContent = 'waiting';
        return;
      }

      const pts = cloud.points;
      let maxAbs = 0.2;
      pts.forEach(p => {
        maxAbs = Math.max(maxAbs, Math.abs(p[0]), Math.abs(p[1]), Math.abs(p[2]));
      });
      const scale = Math.min(w, h) * 0.42 / maxAbs;
      const projected = pts.map(p => ({ p, r: rotatePoint(p) })).sort((a, b) => a.r[2] - b.r[2]);
      projected.forEach(item => {
        const x = cx + item.r[0] * scale;
        const y = cy - item.r[1] * scale;
        const zNorm = Math.max(0, Math.min(1, (item.r[2] / maxAbs + 1) / 2));
        const radius = Math.max(3, Math.min(8, 4 + zNorm * 4)) * (window.devicePixelRatio || 1);
        ctx.fillStyle = `rgb(${Math.round(40 + zNorm * 170)}, ${Math.round(110 + zNorm * 70)}, ${Math.round(230 - zNorm * 90)})`;
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fill();
      });

      document.getElementById('cloudMeta').textContent =
        `${cloud.sampled_points}/${cloud.total_points} points, age ${fmt(state.freshness_s.point_cloud, 2, 's')}`;
    }

    function updateReadouts(state) {
      document.getElementById('odomX').textContent = fmt(state.odom?.x, 3, 'm');
      document.getElementById('odomY').textContent = fmt(state.odom?.y, 3, 'm');
      document.getElementById('odomYaw').textContent = fmt(state.odom?.yaw_deg, 1, 'deg');
      document.getElementById('trackCount').textContent = state.track ? String(state.track.length) : '--';
      document.getElementById('waypointCount').textContent = String(selectedWaypoints.length);
      document.getElementById('planCount').textContent = state.navigation?.global_plan_path
        ? `${state.navigation.global_plan_path.total_poses}`
        : '--';
      const basic = state.navigation?.robot_basic_state;
      document.getElementById('localizationState').textContent = basic
        ? `${basic.is_localization_enabled ? 'ON' : 'OFF'} / ${basic.localization_quality}`
        : '--';
      const cmd = state.navigation?.last_command;
      document.getElementById('navCommand').textContent = cmd?.type ? `${cmd.type} #${cmd.seq || ''}` : '--';
      const targetYaw = computeTargetYaw(state);
      document.getElementById('targetYaw').textContent = targetYaw
        ? `${targetYaw.yawDeg.toFixed(1)}° ${targetYaw.label}`
        : '--';
      document.getElementById('scanValid').textContent = state.scan ? `${state.scan.valid_count}/${state.scan.count}` : '--';
      document.getElementById('scanMin').textContent = fmt(state.scan?.min_range, 3, 'm');
      document.getElementById('scanFrame').textContent = state.scan?.frame_id || '--';
      document.getElementById('scanAge').textContent = fmt(state.freshness_s?.scan, 2, 's');
      document.getElementById('scanMeta').textContent = state.scan
        ? `${state.scan.count} rays, age ${fmt(state.freshness_s.scan, 2, 's')}`
        : 'waiting';
      const cloud = state.point_cloud;
      document.getElementById('cloudTopic').textContent = cloud?.topic ? cloud.topic.split('/').filter(Boolean).slice(-1)[0] : '--';
      document.getElementById('cloudPoints').textContent = cloud ? `${cloud.sampled_points}/${cloud.total_points}` : '--';
      document.getElementById('cloudFrame').textContent = cloud?.frame_id || '--';
      document.getElementById('cloudAge').textContent = fmt(state.freshness_s?.point_cloud, 2, 's');
    }

    function updateNavigationStatus(state) {
      const navStatus = document.getElementById('navStatus');
      const last = state.navigation?.last_command;
      const basic = state.navigation?.robot_basic_state;
      const slamState = state.navigation?.slamware_state?.state || '--';
      const planCount = state.navigation?.global_plan_path?.total_poses || 0;
      const targetYaw = computeTargetYaw(state);
      const modeInput = document.getElementById('directNoAvoidanceMode');
      const rawModeInput = document.getElementById('rawCmdVelNoAvoidanceMode');
      const plannedMode = rawModeInput?.checked ? '裸控无避障' : (modeInput?.checked ? '直连不绕障' : '普通避障');
      const lastMode = last?.navigation_mode === 'raw_cmd_vel_no_obstacle_avoidance'
        ? '裸控无避障'
        : (last?.navigation_mode === 'direct_key_points_stop_on_obstacle'
        ? '直连不绕障'
        : (last?.navigation_mode === 'normal_slamware' ? '普通避障' : '--'));
      const parts = [
        `<strong>${selectedWaypoints.length}</strong> 个航点`,
        `目标角度: <strong>${targetYaw ? targetYaw.yawDeg.toFixed(1) + '° ' + targetYaw.label : '--'}</strong>`,
        `模式: <strong>${plannedMode}</strong>`,
        `上次模式: <strong>${lastMode}</strong>`,
        `Slamware: <strong>${slamState}</strong>`,
        `定位: <strong>${basic ? (basic.is_localization_enabled ? 'ON' : 'OFF') + ' / ' + basic.localization_quality : '--'}</strong>`,
        `规划路径: <strong>${planCount}</strong> 点`
      ];
      if (last?.type) parts.push(`上次指令: <strong>${last.type}</strong>`);
      navStatus.className = 'nav-status';
      if (basic && !basic.is_localization_enabled) navStatus.classList.add('bad');
      else if (basic && Number(basic.localization_quality) <= 0) navStatus.classList.add('warn');
      navStatus.innerHTML = `导航：${parts.join('　')}`;
    }

    async function tick() {
      try {
        const res = await fetch('/api/state', { cache: 'no-store' });
        const state = await res.json();
        lastState = state;
        setStatus(true, `online, uptime ${fmt(state.uptime_s, 1, 's')}`);
        drawMap(state);
        drawScan(state);
        drawCloud(state);
        renderSensors(state);
        updateReadouts(state);
        updateNavigationStatus(state);
        updateWorkflowProgress(state);
        document.getElementById('rawState').textContent = JSON.stringify({
          freshness_s: state.freshness_s,
          seq: state.seq,
          odom: state.odom,
          scan: state.scan ? { frame_id: state.scan.frame_id, min_range: state.scan.min_range, valid_count: state.scan.valid_count, count: state.scan.count } : null,
          point_cloud: state.point_cloud ? {
            topic: state.point_cloud.topic,
            frame_id: state.point_cloud.frame_id,
            total_points: state.point_cloud.total_points,
            sampled_points: state.point_cloud.sampled_points,
            bounds: state.point_cloud.bounds
          } : null,
          sensors: state.sensors,
          map: state.map ? { frame_id: state.map.frame_id, width: state.map.width, height: state.map.height, resolution: state.map.resolution, origin: state.map.origin } : null,
          navigation: state.navigation,
          arm_control: state.arm_control
        }, null, 2);
      } catch (err) {
        setStatus(false, `offline: ${err}`);
      }
    }

    function setNavButtonsBusy(busy) {
      document.getElementById('startNavigationBtn').disabled = busy;
      document.getElementById('runWorkflowBtn').disabled = busy;
      document.getElementById('clearWorkflowBtn').disabled = busy;
      document.getElementById('stopNavigationBtn').disabled = busy;
      document.getElementById('undoWaypointBtn').disabled = busy;
      document.getElementById('clearWaypointsBtn').disabled = busy;
      document.getElementById('setHeadingBtn').disabled = busy;
      document.getElementById('clearHeadingBtn').disabled = busy;
      document.getElementById('headingDegInput').disabled = busy;
      document.getElementById('applyHeadingDegBtn').disabled = busy;
    }

    function showNavMessage(kind, text) {
      const el = document.getElementById('navStatus');
      el.className = `nav-status ${kind || ''}`.trim();
      el.innerHTML = text;
    }

    async function postJson(url, payload = {}) {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
        cache: 'no-store'
      });
      const text = await res.text();
      let data;
      try { data = text ? JSON.parse(text) : {}; }
      catch (err) { data = { ok: false, error: text || String(err) }; }
      if (!res.ok) data.ok = false;
      return data;
    }

    function sleepMs(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
    }

    async function fetchStateOnce() {
      const res = await fetch('/api/state', { cache: 'no-store' });
      const state = await res.json();
      if (!res.ok || state?.ok === false) {
        throw new Error(state?.error || 'state request failed');
      }
      lastState = state;
      return state;
    }

    function getWorkflowModules() {
      return workflowActions.map((action, index) => {
        if (action.type === 'navigate') {
          return {
            ...action,
            title: action.title || '导航',
            index,
            navIndex: getNavigationActions().findIndex(nav => nav.id === action.id)
          };
        }
        if (action.type === 'arm_task') {
          return {
            ...action,
            title: action.title || armActionTitle(action),
            timeoutSec: action.timeoutSec || 120,
            index
          };
        }
        if (action.type === 'column_height') {
          return {
            ...action,
            title: action.title || '立柱升降',
            timeoutSec: action.timeoutSec || 30,
            index
          };
        }
        return {
          ...action,
          title: action.title || '拾取熊猫烟',
          durationSec: action.durationSec || 5,
          index
        };
      });
    }

    function resetWorkflowRun(note = '待执行') {
      workflowRun = {
        running: false,
        mode: 'idle',
        currentIndex: -1,
        completed: {},
        error: '',
        navigationStartedAt: 0,
        actionStartedAt: 0,
        actionDurationSec: 5,
        startPose: null,
        note
      };
      setWorkflowRunningUi(false);
      renderWorkflow();
    }

    function beginWorkflowRun(mode, currentIndex = 0) {
      workflowRun = {
        running: true,
        mode,
        currentIndex,
        completed: {},
        error: '',
        navigationStartedAt: Date.now(),
        actionStartedAt: 0,
        actionDurationSec: 5,
        startPose: lastState?.odom ? { x: lastState.odom.x, y: lastState.odom.y } : null,
        note: mode === 'chain' ? '动作链执行中' : '导航执行中'
      };
      renderWorkflow();
    }

    function markWorkflowError(message) {
      workflowRun.running = false;
      workflowRun.error = message || '执行失败';
      workflowRun.note = workflowRun.error;
      setWorkflowRunningUi(false);
      renderWorkflow();
    }

    function setWorkflowRunningUi(running) {
      const ids = [
        'startNavigationBtn',
        'runWorkflowBtn',
        'clearWorkflowBtn',
        'undoWaypointBtn',
        'clearWaypointsBtn',
        'setHeadingBtn',
        'clearHeadingBtn',
        'headingDegInput',
        'applyHeadingDegBtn',
        'directNoAvoidanceMode',
        'rawCmdVelNoAvoidanceMode',
        'addPointToNavBtn',
        'newActionType',
        'newActionPointSelect',
        'newActionX',
        'newActionY',
        'newActionYawDeg',
        'newArmTargetObject',
        'newArmTimeoutSec',
        'newColumnTargetHeightM',
        'newColumnTimeoutSec',
        'fillCurrentPoseBtn',
        'addWorkflowActionBtn'
      ];
      ids.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = Boolean(running);
      });
      const stopBtn = document.getElementById('stopNavigationBtn');
      if (stopBtn) stopBtn.disabled = false;
    }

    function distanceToAction(action, state = lastState) {
      if (!state?.odom || action?.x === undefined || action?.y === undefined) return null;
      return Math.hypot(Number(state.odom.x) - Number(action.x), Number(state.odom.y) - Number(action.y));
    }

    function angleDiffDeg(aDeg, bDeg) {
      if (!Number.isFinite(Number(aDeg)) || !Number.isFinite(Number(bDeg))) return null;
      return Math.abs(normalizeAngle((Number(aDeg) - Number(bDeg)) * Math.PI / 180) * 180 / Math.PI);
    }

    function navigationReachState(action, state = lastState) {
      const dist = distanceToAction(action, state);
      const distanceOk = dist !== null && dist <= NAV_REACH_DISTANCE_M;
      const hasTargetYaw = Number.isFinite(Number(action?.yawDeg));
      const yawErrorDeg = hasTargetYaw ? angleDiffDeg(state?.odom?.yaw_deg, action.yawDeg) : null;
      const yawOk = hasTargetYaw ? yawErrorDeg !== null && yawErrorDeg <= NAV_REACH_YAW_DEG : true;
      return {
        reached: Boolean(distanceOk && yawOk),
        distanceOk,
        yawOk,
        dist,
        yawErrorDeg,
      };
    }

    function isActionReached(action, state = lastState) {
      return navigationReachState(action, state).reached;
    }

    function previousNavigationPose(actionIndex) {
      for (let i = actionIndex - 1; i >= 0; i--) {
        const action = workflowActions[i];
        if (action?.type === 'navigate') return action;
      }
      return workflowRun.startPose;
    }

    function estimateNavProgress(action, state = lastState) {
      const dist = distanceToAction(action, state);
      if (dist === null) return 0;
      const start = previousNavigationPose(workflowActions.findIndex(item => item.id === action.id));
      const total = start ? Math.hypot(Number(action.x) - Number(start.x), Number(action.y) - Number(start.y)) : Math.max(dist, 0.01);
      if (total < 0.01) return isActionReached(action, state) ? 1 : 0;
      return Math.max(0, Math.min(1, 1 - dist / total));
    }

    function updateWorkflowProgress(state) {
      if (!workflowRun.running) {
        renderWorkflow();
        return;
      }
      const modules = getWorkflowModules();
      if (!modules.length) {
        workflowRun.running = false;
        workflowRun.note = '没有动作模块';
        renderWorkflow();
        return;
      }

      if (workflowRun.mode === 'navigation') {
        modules.forEach((action, index) => {
          if (action.type === 'navigate' && isActionReached(action, state)) {
            workflowRun.completed[action.id] = true;
          }
        });
        const nextIndex = modules.findIndex(action => action.type === 'navigate' && !workflowRun.completed[action.id]);
        if (nextIndex >= 0) {
          workflowRun.currentIndex = nextIndex;
        } else {
          workflowRun.running = false;
          let lastNavIndex = -1;
          modules.forEach((action, index) => {
            if (action.type === 'navigate') lastNavIndex = index;
          });
          workflowRun.currentIndex = Math.max(0, lastNavIndex);
          workflowRun.note = '导航完成';
          setWorkflowRunningUi(false);
        }
      }
      renderWorkflow();
    }

    function clearWorkflowActions() {
      if (workflowRun.running) {
        showNavMessage('bad', '动作链执行中：<strong>请先停止后再清空动作</strong>');
        return;
      }
      workflowActions = [];
      selectedWaypoints = [];
      finalHeadingPoint = null;
      manualHeadingDeg = null;
      draggedActionId = null;
      document.getElementById('headingDegInput').value = '';
      setHeadingMode(false);
      resetWorkflowRun('已清空动作');
      refreshMapUi();
      showNavMessage('', '动作链：<strong>已清空所有动作</strong>');
    }

    function workflowStepStatus(module, index) {
      if (workflowRun.error && index === workflowRun.currentIndex) return 'error';
      if (workflowRun.completed[module.id]) return 'done';
      if (workflowRun.running && index === workflowRun.currentIndex) return 'running';
      return 'queued';
    }

    function workflowStepProgress(module, index) {
      const status = workflowStepStatus(module, index);
      if (status === 'done') return 100;
      if (status !== 'running') return 0;
      if (module.type === 'navigate') return Math.round(estimateNavProgress(module, lastState) * 100);
      if (module.type === 'arm_task' || module.type === 'column_height' || module.type === 'fake_pick_xiongmao') {
        if (!workflowRun.actionStartedAt) return 0;
        const elapsed = (Date.now() - workflowRun.actionStartedAt) / 1000;
        const total = Math.max(0.1, workflowRun.actionDurationSec || module.timeoutSec || module.durationSec || 5);
        return Math.round(Math.max(0, Math.min(1, elapsed / total)) * 100);
      }
      return 0;
    }

    function armActionTitle(action) {
      const phase = String(action.phase || '').toUpperCase();
      const target = action.targetObject || action.target_object || '';
      if (phase === 'PICK') return `机械臂抓取 ${target || '目标'}`;
      if (phase === 'PLACE') return '机械臂放置';
      if (phase === 'RESET') return '机械臂复位';
      return '机械臂任务';
    }

    function actionTitle(action, index) {
      if (action.type === 'navigate') return `${index + 1}. ${action.title || '导航'}`;
      if (action.type === 'arm_task') return `${index + 1}. ${action.title || armActionTitle(action)}`;
      if (action.type === 'column_height') return `${index + 1}. ${action.title || '立柱升降'}`;
      if (action.type === 'fake_pick_xiongmao') return `${index + 1}. 拾取熊猫烟`;
      return `${index + 1}. ${action.title || action.type || '动作'}`;
    }

    function actionDetail(action) {
      if (action.type === 'navigate') {
        const yawText = Number.isFinite(Number(action.yawDeg)) ? `, yaw=${Number(action.yawDeg).toFixed(1)}°` : '';
        const pointText = action.pointName ? `，点位库：${action.pointName}` : '';
        return `目标 x=${Number(action.x).toFixed(3)}m, y=${Number(action.y).toFixed(3)}m${yawText}${pointText}`;
      }
      if (action.type === 'fake_pick_xiongmao') {
        return `假动作模块：后端休眠 ${action.durationSec || 5}s，后续可替换为机械臂动作`;
      }
      if (action.type === 'arm_task') {
        const phase = String(action.phase || '').toUpperCase();
        const target = action.targetObject || action.target_object || '';
        const targetText = target ? `，目标=${target}` : '';
        return `ROS 手臂任务：phase=${phase}${targetText}，超时 ${action.timeoutSec || 120}s`;
      }
      if (action.type === 'column_height') {
        return `G1D 立柱高度：target=${Number(action.targetHeightM || 0).toFixed(3)}m，超时 ${action.timeoutSec || 30}s`;
      }
      return '预留动作模块';
    }

    function renderWorkflow() {
      syncSelectedWaypointsFromActions();
      const list = document.getElementById('workflowList');
      const meta = document.getElementById('workflowMeta');
      if (!list || !meta) return;
      const modules = getWorkflowModules();
      const doneCount = modules.filter(module => workflowRun.completed[module.id]).length;
      meta.textContent = workflowRun.running
        ? `执行中 ${Math.min(workflowRun.currentIndex + 1, modules.length)}/${modules.length}`
        : `${doneCount}/${modules.length} 完成`;
      if (!modules.length) {
        list.innerHTML = '<div class="workflow-step"><div class="workflow-step-content"><div class="workflow-title">暂无动作模块</div><div class="workflow-detail">点击“增加动作”，或直接在地图上点击快速增加导航动作。</div></div></div>';
        return;
      }
      list.innerHTML = modules.map((module, index) => {
        const status = workflowStepStatus(module, index);
        const progress = workflowStepProgress(module, index);
        const badge = status === 'done' ? '完成' : status === 'running' ? '进行中' : status === 'error' ? '异常' : '等待';
        const draggable = workflowRun.running ? 'false' : 'true';
        const deleteButton = workflowRun.running ? '' : `<button class="workflow-delete" data-delete-action-id="${escapeHtml(module.id)}">删除</button>`;
        return `<div class="workflow-step ${status}" draggable="${draggable}" data-action-id="${escapeHtml(module.id)}" style="--progress:${progress}%">
          <div class="workflow-step-content">
            <div class="workflow-title">
              <span class="workflow-title-left"><span class="drag-handle">☰</span><span class="workflow-pulse"></span>${escapeHtml(actionTitle(module, index))}</span>
              <span class="workflow-badge">${badge}</span>
              ${deleteButton}
            </div>
            <div class="workflow-detail">${escapeHtml(actionDetail(module))}</div>
            <div class="workflow-progress-text">进度 ${progress}%</div>
          </div>
        </div>`;
      }).join('');
      bindWorkflowListEvents();
    }

    function bindWorkflowListEvents() {
      const list = document.getElementById('workflowList');
      if (!list || workflowRun.running) return;
      list.querySelectorAll('.workflow-delete').forEach(btn => {
        btn.addEventListener('click', ev => {
          ev.stopPropagation();
          removeWorkflowAction(btn.getAttribute('data-delete-action-id'));
        });
      });
      list.querySelectorAll('.workflow-step[data-action-id]').forEach(item => {
        item.addEventListener('dragstart', ev => {
          draggedActionId = item.getAttribute('data-action-id');
          item.classList.add('dragging');
          ev.dataTransfer.effectAllowed = 'move';
          ev.dataTransfer.setData('text/plain', draggedActionId);
        });
        item.addEventListener('dragover', ev => {
          ev.preventDefault();
          ev.dataTransfer.dropEffect = 'move';
        });
        item.addEventListener('drop', ev => {
          ev.preventDefault();
          const targetId = item.getAttribute('data-action-id');
          const sourceId = draggedActionId || ev.dataTransfer.getData('text/plain');
          reorderWorkflowAction(sourceId, targetId);
        });
        item.addEventListener('dragend', () => {
          item.classList.remove('dragging');
          draggedActionId = null;
        });
      });
    }

    async function waitForActionReached(action, timeoutMs = 180000) {
      const started = Date.now();
      let stableSince = null;
      let lastReach = null;
      while (workflowRun.running && Date.now() - started < timeoutMs) {
        lastReach = navigationReachState(action, lastState);
        if (lastReach.reached) {
          if (stableSince === null) stableSince = Date.now();
          if (Date.now() - stableSince >= NAV_REACH_STABLE_MS) return true;
        } else {
          stableSince = null;
        }
        await sleepMs(250);
      }
      if (!workflowRun.running) throw new Error('动作链已停止');
      const distText = lastReach?.dist === null || lastReach?.dist === undefined ? '--' : `${(lastReach.dist * 1000).toFixed(0)}mm`;
      const yawText = lastReach?.yawErrorDeg === null || lastReach?.yawErrorDeg === undefined ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`;
      throw new Error(`等待导航到达超时：距离偏差 ${distText}，yaw 偏差 ${yawText}`);
    }

    async function waitForSlamwareNavigationComplete(action, timeoutMs = 180000) {
      const started = Date.now();
      let strictStableSince = null;
      let planIdleSince = null;
      let odomStillSince = null;
      let lastReach = null;
      let lastOdom = null;
      let lastStateError = null;
      let sawNavigationEvidence = false;
      while (workflowRun.running && Date.now() - started < timeoutMs) {
        let state;
        try {
          state = await fetchStateOnce();
        } catch (err) {
          lastStateError = err;
          await sleepMs(300);
          continue;
        }

        const now = Date.now();
        lastReach = navigationReachState(action, state);
        if (lastReach.reached) {
          if (strictStableSince === null) strictStableSince = now;
          if (now - strictStableSince >= NAV_REACH_STABLE_MS) return true;
        } else {
          strictStableSince = null;
        }

        const plan = state?.navigation?.global_plan_path;
        const planCount = Number(plan?.total_poses || 0);
        const planFresh = Number(state?.freshness_s?.global_plan_path);
        const planFreshOk = !Number.isFinite(planFresh) || planFresh <= 2.0;
        if (planCount > 0) sawNavigationEvidence = true;
        if (plan && planFreshOk && planCount === 0) {
          if (planIdleSince === null) planIdleSince = now;
        } else {
          planIdleSince = null;
        }

        const odom = state?.odom;
        if (odom && lastOdom) {
          const moved = Math.hypot(Number(odom.x) - Number(lastOdom.x), Number(odom.y) - Number(lastOdom.y));
          const yawMoved = angleDiffDeg(odom.yaw_deg, lastOdom.yaw_deg);
          if (moved > 0.015 || (yawMoved !== null && yawMoved > 1.5)) sawNavigationEvidence = true;
          if (moved <= NAV_ODOM_STILL_M && (yawMoved === null || yawMoved <= NAV_ODOM_STILL_YAW_DEG)) {
            if (odomStillSince === null) odomStillSince = now;
          } else {
            odomStillSince = null;
          }
        }
        if (odom) lastOdom = { x: Number(odom.x), y: Number(odom.y), yaw_deg: Number(odom.yaw_deg) };

        const hasTargetYaw = Number.isFinite(Number(action?.yawDeg));
        const idleDistanceOk = lastReach?.dist !== null && lastReach?.dist !== undefined && lastReach.dist <= NAV_IDLE_ACCEPT_DISTANCE_M;
        const idleYawOk = hasTargetYaw
          ? lastReach?.yawErrorDeg !== null && lastReach?.yawErrorDeg !== undefined && lastReach.yawErrorDeg <= NAV_IDLE_ACCEPT_YAW_DEG
          : true;
        const planIdleOk = planIdleSince !== null && now - planIdleSince >= NAV_IDLE_STABLE_MS;
        const odomStillOk = odomStillSince !== null && now - odomStillSince >= NAV_IDLE_STABLE_MS;
        if (planIdleOk && odomStillOk && idleDistanceOk && idleYawOk && (sawNavigationEvidence || now - started > 3000)) {
          if (!lastReach.reached) {
            const distText = `${(lastReach.dist * 1000).toFixed(0)}mm`;
            const yawText = lastReach.yawErrorDeg === null || lastReach.yawErrorDeg === undefined ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`;
            showNavMessage('warn', `动作链：底层导航已停止，按兜底条件继续；距离 ${distText}，yaw ${yawText}`);
          }
          return true;
        }
        await sleepMs(250);
      }
      if (!workflowRun.running) throw new Error('动作链已停止');
      if (lastStateError && !lastReach) throw new Error(`读取底盘状态失败：${lastStateError.message || lastStateError}`);
      const distText = lastReach?.dist === null || lastReach?.dist === undefined ? '--' : `${(lastReach.dist * 1000).toFixed(0)}mm`;
      const yawText = lastReach?.yawErrorDeg === null || lastReach?.yawErrorDeg === undefined ? '--' : `${lastReach.yawErrorDeg.toFixed(1)}°`;
      throw new Error(`等待导航到达超时：距离偏差 ${distText}，yaw 偏差 ${yawText}`);
    }

    async function waitForRawNavigationComplete(command, action, timeoutMs = 180000) {
      const rawNavId = command?.raw_nav_id;
      if (!rawNavId) return waitForActionReached(action, timeoutMs);
      const started = Date.now();
      let lastRawCommand = null;
      let lastStateError = null;
      while (workflowRun.running && Date.now() - started < timeoutMs) {
        let state;
        try {
          state = await fetchStateOnce();
        } catch (err) {
          lastStateError = err;
          await sleepMs(300);
          continue;
        }
        const current = state?.navigation?.last_command;
        if (current?.raw_nav_id === rawNavId) {
          lastRawCommand = current;
          const status = String(current.raw_nav_status || '');
          if (status === 'done' || status === 'dry_run') return true;
          if (status === 'timeout' || status === 'cancelled' || status === 'error') {
            throw new Error(`裸控导航${status}：${current.raw_nav_error || 'no detail'}`);
          }
        } else if (current?.raw_cmd_vel && current?.raw_nav_id && current.raw_nav_id !== rawNavId) {
          throw new Error('裸控导航被新的 /cmd_vel 导航请求替换');
        }
        await sleepMs(250);
      }
      if (!workflowRun.running) throw new Error('动作链已停止');
      if (lastStateError && !lastRawCommand) throw new Error(`读取底盘状态失败：${lastStateError.message || lastStateError}`);
      const statusText = lastRawCommand?.raw_nav_status || '--';
      const distText = lastRawCommand?.raw_nav_distance_m === null || lastRawCommand?.raw_nav_distance_m === undefined
        ? '--'
        : `${(Number(lastRawCommand.raw_nav_distance_m) * 1000).toFixed(0)}mm`;
      const yawText = lastRawCommand?.raw_nav_yaw_error_deg === null || lastRawCommand?.raw_nav_yaw_error_deg === undefined
        ? '--'
        : `${Number(lastRawCommand.raw_nav_yaw_error_deg).toFixed(1)}°`;
      throw new Error(`等待裸控导航完成超时：status=${statusText}，距离 ${distText}，yaw ${yawText}`);
    }

    function setPointMessage(kind, text) {
      const el = document.getElementById('pointMessage');
      el.className = `point-message ${kind || ''}`.trim();
      el.textContent = text;
    }

    function setPointForm(point) {
      editingPointId = point?.id || null;
      document.getElementById('pointNameInput').value = point?.name || '';
      document.getElementById('pointXInput').value = point?.x !== undefined && point?.x !== null ? Number(point.x).toFixed(4) : '';
      document.getElementById('pointYInput').value = point?.y !== undefined && point?.y !== null ? Number(point.y).toFixed(4) : '';
      document.getElementById('pointYawDegInput').value = point?.yaw_deg !== undefined && point?.yaw_deg !== null ? Number(point.yaw_deg).toFixed(1) : '';
      document.getElementById('pointNoteInput').value = point?.note || '';
      document.getElementById('pointActionsInput').value = JSON.stringify(point?.actions || [], null, 2);
      renderSavedPoints();
      refreshMapUi();
    }

    function clearPointForm() {
      setPointForm(null);
      document.getElementById('pointActionsInput').value = '[]';
      setPointMessage('', '动作字段只保存，不执行；后续接机械臂时复用。');
    }

    function parseActionsInput() {
      const raw = document.getElementById('pointActionsInput').value.trim();
      if (!raw) return [];
      let actions;
      try {
        actions = JSON.parse(raw);
      } catch (err) {
        throw new Error(`动作 JSON 格式不对：${err.message}`);
      }
      if (!Array.isArray(actions)) {
        throw new Error('动作 JSON 必须是数组，例如 [{"type":"pick"}]');
      }
      return actions;
    }

    function buildPointPayload(includeId = true) {
      const x = Number(document.getElementById('pointXInput').value);
      const y = Number(document.getElementById('pointYInput').value);
      const yawDeg = Number(document.getElementById('pointYawDegInput').value || 0);
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        throw new Error('请填写有效的 X / Y 坐标');
      }
      if (!Number.isFinite(yawDeg)) {
        throw new Error('请填写有效的朝向角度');
      }
      const payload = {
        name: document.getElementById('pointNameInput').value.trim(),
        x,
        y,
        yaw_deg: yawDeg,
        note: document.getElementById('pointNoteInput').value,
        actions: parseActionsInput()
      };
      if (includeId && editingPointId) payload.id = editingPointId;
      return payload;
    }

    async function loadSavedPoints() {
      try {
        const res = await fetch('/api/points', { cache: 'no-store' });
        const data = await res.json();
        if (!data.ok) throw new Error(data.error || 'load points failed');
        savedPoints = data.points || [];
        if (editingPointId && !savedPoints.some(point => point.id === editingPointId)) {
          editingPointId = null;
        }
        renderSavedPoints();
        refreshActionPointOptions();
        document.getElementById('pointsMeta').textContent = `${savedPoints.length} saved`;
        refreshMapUi();
      } catch (err) {
        document.getElementById('pointsMeta').textContent = 'load failed';
        setPointMessage('bad', `点位列表读取失败：${err}`);
      }
    }

    function renderSavedPoints() {
      const list = document.getElementById('savedPointList');
      if (!savedPoints.length) {
        list.innerHTML = '<div class="point-item"><div class="point-title"><span>暂无保存点位</span></div><div class="point-detail">可以先点击“记录当前位置”，也可以手动填写 X/Y 后保存。</div></div>';
        return;
      }
      list.innerHTML = savedPoints.map(point => {
        const active = point.id === editingPointId ? 'active' : '';
        const actionsCount = Array.isArray(point.actions) ? point.actions.length : 0;
        return `<div class="point-item ${active}" data-point-id="${escapeHtml(point.id)}">
          <div class="point-title">
            <span>${escapeHtml(point.name || 'Point')}</span>
            <span>${Number(point.yaw_deg || 0).toFixed(1)}°</span>
          </div>
          <div class="point-detail">x=${Number(point.x).toFixed(3)}m, y=${Number(point.y).toFixed(3)}m · ${escapeHtml(point.source || 'manual')} · actions ${actionsCount}</div>
          ${point.note ? `<div class="point-detail">${escapeHtml(point.note)}</div>` : ''}
        </div>`;
      }).join('');
      list.querySelectorAll('.point-item[data-point-id]').forEach(item => {
        item.addEventListener('click', () => {
          const point = savedPoints.find(p => p.id === item.getAttribute('data-point-id'));
          if (point) {
            setPointForm(point);
            setPointMessage('', `正在编辑：${point.name || 'Point'}`);
          }
        });
      });
    }

    async function recordCurrentPoint() {
      let actions;
      try {
        actions = parseActionsInput();
      } catch (err) {
        setPointMessage('bad', err.message);
        return;
      }
      setPointMessage('', '正在记录当前机器人位置...');
      const data = await postJson('/api/points/record_current', {
        name: document.getElementById('pointNameInput').value.trim(),
        note: document.getElementById('pointNoteInput').value,
        actions
      });
      if (!data.ok) {
        setPointMessage('bad', `记录失败：${data.error || 'unknown error'}`);
        return;
      }
      editingPointId = data.point.id;
      await loadSavedPoints();
      const point = savedPoints.find(p => p.id === editingPointId);
      if (point) setPointForm(point);
      setPointMessage('ok', `已记录当前位置：${data.point.name}`);
    }

    async function savePoint() {
      let payload;
      try {
        payload = buildPointPayload(true);
      } catch (err) {
        setPointMessage('bad', err.message);
        return;
      }
      const data = await postJson('/api/points/upsert', payload);
      if (!data.ok) {
        setPointMessage('bad', `保存失败：${data.error || 'unknown error'}`);
        return;
      }
      editingPointId = data.point.id;
      await loadSavedPoints();
      const point = savedPoints.find(p => p.id === editingPointId);
      if (point) setPointForm(point);
      setPointMessage('ok', `已保存点位：${data.point.name}`);
    }

    async function deletePoint() {
      if (!editingPointId) {
        setPointMessage('bad', '请先在列表里选中一个点位');
        return;
      }
      if (!window.confirm('确定删除这个点位吗？')) return;
      const data = await postJson('/api/points/delete', { id: editingPointId });
      if (!data.ok) {
        setPointMessage('bad', `删除失败：${data.error || 'unknown error'}`);
        return;
      }
      clearPointForm();
      await loadSavedPoints();
      setPointMessage('ok', '点位已删除');
    }

    function addEditedPointToNav() {
      let payload;
      try {
        payload = buildPointPayload(false);
      } catch (err) {
        setPointMessage('bad', err.message);
        return;
      }
      addWorkflowAction({
        id: makeActionId('nav'),
        type: 'navigate',
        title: payload.name ? `导航到 ${payload.name}` : '导航',
        pointId: editingPointId || null,
        pointName: payload.name || '',
        x: Number(payload.x.toFixed(4)),
        y: Number(payload.y.toFixed(4)),
        yawDeg: Number(radToDeg(normalizeAngle(payload.yaw_deg * Math.PI / 180)).toFixed(3))
      });
      finalHeadingPoint = null;
      document.getElementById('headingDegInput').value = Number(payload.yaw_deg).toFixed(1);
      setHeadingMode(false);
      setPointMessage('ok', `已加入动作链：${payload.name || '导航'}，朝向 ${Number(payload.yaw_deg).toFixed(1)}°`);
    }

    function setActionPointSelection(pointId = '') {
      const select = document.getElementById('newActionPointSelect');
      if (select) select.value = pointId || '';
    }

    function refreshActionPointOptions() {
      const select = document.getElementById('newActionPointSelect');
      if (!select) return;
      const previous = select.value;
      const options = ['<option value="">手动输入 / 地图点选</option>'].concat(
        savedPoints.map(point => {
          const name = point.name || 'Point';
          const yaw = Number(point.yaw_deg || 0).toFixed(1);
          return `<option value="${escapeHtml(point.id)}">${escapeHtml(name)} (${Number(point.x).toFixed(3)}, ${Number(point.y).toFixed(3)}, ${yaw}°)</option>`;
        })
      );
      select.innerHTML = options.join('');
      if (previous && savedPoints.some(point => point.id === previous)) {
        select.value = previous;
      }
    }

    function setActionPoseInputs(x, y, yawDeg, options = {}) {
      if (!options.keepPointSelection) setActionPointSelection('');
      if (x !== null && x !== undefined) document.getElementById('newActionX').value = Number(x).toFixed(4);
      if (y !== null && y !== undefined) document.getElementById('newActionY').value = Number(y).toFixed(4);
      if (yawDeg !== null && yawDeg !== undefined) document.getElementById('newActionYawDeg').value = Number(yawDeg).toFixed(1);
    }

    function onActionPointSelected() {
      const select = document.getElementById('newActionPointSelect');
      const point = select ? getSavedPointById(select.value) : null;
      if (!point) return;
      setActionPoseInputs(point.x, point.y, point.yaw_deg || 0, { keepPointSelection: true });
      showNavMessage('', `动作链：已选择点位库 <strong>${escapeHtml(point.name || 'Point')}</strong>`);
    }

    function fillCurrentPoseForAction() {
      if (!lastState?.odom) {
        showNavMessage('bad', '动作链：<strong>当前没有 odom，不能填当前位置</strong>');
        return;
      }
      setActionPoseInputs(lastState.odom.x, lastState.odom.y, lastState.odom.yaw_deg || 0);
    }

    function updateActionBuilderVisibility() {
      const type = document.getElementById('newActionType').value;
      const isNav = type === 'navigate';
      const isArm = type.startsWith('arm_');
      const isArmPick = type === 'arm_pick';
      const isColumn = type === 'column_height';
      document.querySelectorAll('.nav-action-field').forEach(el => {
        el.style.display = isNav ? 'grid' : 'none';
      });
      document.querySelectorAll('.arm-action-field').forEach(el => {
        el.style.display = isArm ? 'grid' : 'none';
      });
      document.querySelectorAll('.column-action-field').forEach(el => {
        el.style.display = isColumn ? 'grid' : 'none';
      });
      document.getElementById('newArmTargetObject').disabled = !isArmPick;
      document.getElementById('fillCurrentPoseBtn').style.display = isNav ? '' : 'none';
    }

    function addActionFromBuilder() {
      const type = document.getElementById('newActionType').value;
      if (type === 'navigate') {
        const selectedPoint = getSavedPointById(document.getElementById('newActionPointSelect')?.value || '');
        const x = selectedPoint ? Number(selectedPoint.x) : Number(document.getElementById('newActionX').value);
        const y = selectedPoint ? Number(selectedPoint.y) : Number(document.getElementById('newActionY').value);
        const rawYaw = selectedPoint ? selectedPoint.yaw_deg : document.getElementById('newActionYawDeg').value;
        const yawDeg = rawYaw === '' || rawYaw === null || rawYaw === undefined ? (lastState?.odom?.yaw_deg || 0) : Number(rawYaw);
        if (!Number.isFinite(x) || !Number.isFinite(y)) {
          showNavMessage('bad', '动作链：<strong>导航动作需要有效的 X / Y</strong>');
          return;
        }
        if (!Number.isFinite(yawDeg)) {
          showNavMessage('bad', '动作链：<strong>导航动作需要有效的 yaw</strong>');
          return;
        }
        addWorkflowAction({
          id: makeActionId('nav'),
          type: 'navigate',
          title: selectedPoint ? `导航到 ${selectedPoint.name || '点位'}` : '导航',
          pointId: selectedPoint?.id || null,
          pointName: selectedPoint?.name || '',
          x: Number(x.toFixed(4)),
          y: Number(y.toFixed(4)),
          yawDeg: Number(radToDeg(normalizeAngle(yawDeg * Math.PI / 180)).toFixed(3))
        });
        const sourceText = selectedPoint ? `（点位库：${escapeHtml(selectedPoint.name || 'Point')}）` : '';
        showNavMessage('', `动作链：已增加导航动作${sourceText} x=${x.toFixed(3)}, y=${y.toFixed(3)}`);
        return;
      }
      if (type.startsWith('arm_')) {
        const phaseMap = { arm_pick: 'PICK', arm_place: 'PLACE', arm_reset: 'RESET' };
        const phase = phaseMap[type];
        const targetObject = phase === 'PICK' ? document.getElementById('newArmTargetObject').value : '';
        const timeoutSec = Number(document.getElementById('newArmTimeoutSec').value || 120);
        if (!phase) {
          showNavMessage('bad', '动作链：<strong>未知机械臂任务类型</strong>');
          return;
        }
        if (phase === 'PICK' && !targetObject) {
          showNavMessage('bad', '动作链：<strong>机械臂抓取需要选择目标标签</strong>');
          return;
        }
        if (!Number.isFinite(timeoutSec) || timeoutSec <= 0) {
          showNavMessage('bad', '动作链：<strong>机械臂任务需要有效超时时间</strong>');
          return;
        }
        const action = {
          id: makeActionId('arm'),
          type: 'arm_task',
          phase,
          targetObject,
          timeoutSec: Math.max(1, Math.min(600, Number(timeoutSec.toFixed(1))))
        };
        action.title = armActionTitle(action);
        addWorkflowAction(action);
        showNavMessage('', `动作链：已增加“${escapeHtml(action.title)}”`);
        return;
      }
      if (type === 'column_height') {
        const targetHeightM = Number(document.getElementById('newColumnTargetHeightM').value);
        const timeoutSec = Number(document.getElementById('newColumnTimeoutSec').value || 30);
        if (!Number.isFinite(targetHeightM)) {
          showNavMessage('bad', '动作链：<strong>立柱升降需要有效目标高度</strong>');
          return;
        }
        if (!Number.isFinite(timeoutSec) || timeoutSec <= 0) {
          showNavMessage('bad', '动作链：<strong>立柱升降需要有效超时时间</strong>');
          return;
        }
        const action = {
          id: makeActionId('column'),
          type: 'column_height',
          title: '立柱升降',
          targetHeightM: Number(targetHeightM.toFixed(4)),
          timeoutSec: Math.max(1, Math.min(180, Number(timeoutSec.toFixed(1))))
        };
        addWorkflowAction(action);
        showNavMessage('', `动作链：已增加“立柱升降” target=${action.targetHeightM.toFixed(3)}m`);
        return;
      }
      addWorkflowAction({
        id: makeActionId('act'),
        type: 'fake_pick_xiongmao',
        title: '拾取熊猫烟',
        durationSec: 5
      });
      showNavMessage('', '动作链：已增加动作“拾取熊猫烟”');
    }

    function setHeadingMode(enabled) {
      headingMode = Boolean(enabled);
      document.getElementById('setHeadingBtn').classList.toggle('active', headingMode);
      document.getElementById('navHint').textContent = headingMode
        ? '在地图上点击终点需要朝向的方向'
        : '在地图上点击添加航点';
    }

    function refreshMapUi() {
      if (lastState) {
        drawMap(lastState);
        updateReadouts(lastState);
        updateNavigationStatus(lastState);
      }
      renderWorkflow();
    }

    function applyHeadingDegFromInput() {
      if (!selectedWaypoints.length) {
        showNavMessage('bad', '导航：<strong>请先添加至少一个航点，再输入角度</strong>');
        return;
      }
      const input = document.getElementById('headingDegInput');
      const raw = Number(input.value);
      if (!Number.isFinite(raw)) {
        showNavMessage('bad', '导航：<strong>请输入有效角度</strong>');
        return;
      }
      const yaw = normalizeAngle(raw * Math.PI / 180);
      manualHeadingDeg = Number(radToDeg(yaw).toFixed(3));
      setLastNavigationYaw(manualHeadingDeg);
      input.value = manualHeadingDeg.toFixed(1);
      finalHeadingPoint = null;
      setHeadingMode(false);
      refreshMapUi();
      showNavMessage('', `导航：已输入终点朝向 <strong>${manualHeadingDeg.toFixed(1)}°</strong>`);
    }

    function buildNavigationPayload(waypoints = selectedWaypoints, yawDeg = null, yawSource = 'workflow_action') {
      const payload = { waypoints };
      const directNoAvoidanceMode = Boolean(document.getElementById('directNoAvoidanceMode')?.checked);
      const rawCmdVelNoAvoidanceMode = Boolean(document.getElementById('rawCmdVelNoAvoidanceMode')?.checked);
      if (rawCmdVelNoAvoidanceMode) {
        payload.raw_cmd_vel = true;
        payload.disable_obstacle_avoidance = true;
        payload.navigation_mode = 'raw_cmd_vel_no_obstacle_avoidance';
      } else if (directNoAvoidanceMode) {
        payload.direct_no_avoidance = true;
        payload.navigation_mode = 'direct_key_points_stop_on_obstacle';
      }
      if (yawDeg !== null && yawDeg !== undefined && Number.isFinite(Number(yawDeg))) {
        payload.yaw = Number(normalizeAngle(Number(yawDeg) * Math.PI / 180).toFixed(6));
        payload.yaw_source = yawSource;
      } else {
        const targetYaw = computeTargetYaw(lastState);
        if (targetYaw) {
          payload.yaw = Number(targetYaw.yaw.toFixed(6));
          payload.yaw_source = targetYaw.source;
        }
      }
      return payload;
    }

    async function startNavigation(options = {}) {
      const waypoints = options.waypoints || selectedWaypoints;
      if (!waypoints.length) {
        showNavMessage('bad', '导航：<strong>请先在地图上点击选择至少一个航点</strong>');
        return { ok: false, error: 'no waypoints' };
      }
      const targetYaw = options.yawDeg !== undefined && options.yawDeg !== null
        ? { yawDeg: Number(options.yawDeg), label: '动作' }
        : computeTargetYaw(lastState);
      if (!options.fromWorkflow) setNavButtonsBusy(true);
      const directNoAvoidanceMode = Boolean(document.getElementById('directNoAvoidanceMode')?.checked);
      const rawCmdVelNoAvoidanceMode = Boolean(document.getElementById('rawCmdVelNoAvoidanceMode')?.checked);
      const modeText = rawCmdVelNoAvoidanceMode ? '裸控无避障（/cmd_vel）' : (directNoAvoidanceMode ? '直连不绕障（遇障停止）' : '普通避障');
      showNavMessage('', `导航：正在发送航点和目标角度... ${targetYaw ? targetYaw.yawDeg.toFixed(1) + '° ' + targetYaw.label : ''}，模式 ${modeText}`);
      try {
        const data = await postJson('/api/navigation/start', buildNavigationPayload(waypoints, options.yawDeg, options.yawSource || 'workflow_action'));
        if (data.ok) {
          if (!options.fromWorkflow) {
            const firstNavIndex = getWorkflowModules().findIndex(action => action.type === 'navigate');
            beginWorkflowRun(options.workflowMode || 'navigation', Math.max(0, firstNavIndex));
          }
          const warnings = data.command?.safety?.warnings || [];
          const warningText = warnings.length ? `，警告：${warnings.join('；')}` : '';
          const yawText = data.command?.yaw_deg !== null && data.command?.yaw_deg !== undefined
            ? `，目标角度 ${Number(data.command.yaw_deg).toFixed(1)}°`
            : '';
          const commandModeText = data.command?.raw_cmd_vel
            ? '，裸控无避障（/cmd_vel）'
            : (data.command?.direct_no_avoidance ? '，直连不绕障（遇障停止）' : '，普通避障');
          showNavMessage(warnings.length ? 'warn' : '', `导航：<strong>已开始</strong>，航点 ${waypoints.length} 个${yawText}${commandModeText}${warningText}`);
        } else {
          const blockers = data.safety?.blockers || [];
          const details = blockers.length ? `：${blockers.join('；')}` : (data.error || 'unknown error');
          showNavMessage('bad', `导航：<strong>启动失败</strong>${details}`);
          if (!options.fromWorkflow) markWorkflowError(details);
        }
        return data;
      } catch (err) {
        showNavMessage('bad', `导航：<strong>请求失败</strong> ${err}`);
        if (!options.fromWorkflow) markWorkflowError(String(err));
        return { ok: false, error: String(err) };
      } finally {
        if (!options.fromWorkflow) {
          setNavButtonsBusy(false);
          if (workflowRun.running) setWorkflowRunningUi(true);
        }
        tick();
      }
    }

    async function runWorkflow() {
      const modules = getWorkflowModules();
      if (!modules.length) {
        showNavMessage('bad', '动作链：<strong>请先增加至少一个动作</strong>');
        return;
      }
      setWorkflowRunningUi(true);
      beginWorkflowRun('chain', 0);
      try {
        for (let index = 0; index < modules.length; index++) {
          const action = modules[index];
          workflowRun.currentIndex = index;
          workflowRun.actionStartedAt = 0;
          renderWorkflow();
          if (action.type === 'navigate') {
            showNavMessage('', `动作链：正在执行第 ${index + 1} 步导航...`);
            const navData = await startNavigation({
              fromWorkflow: true,
              workflowMode: 'chain',
              waypoints: [{ x: Number(action.x), y: Number(action.y) }],
              yawDeg: action.yawDeg,
              yawSource: 'workflow_action'
            });
            if (!navData.ok) {
              throw new Error(navData.error || '导航启动失败');
            }
            if (navData.command?.raw_cmd_vel && navData.command?.raw_nav_id) {
              await waitForRawNavigationComplete(navData.command, action);
            } else {
              await waitForSlamwareNavigationComplete(action);
            }
            workflowRun.completed[action.id] = true;
            continue;
          }
          if (action.type === 'arm_task') {
            workflowRun.actionStartedAt = Date.now();
            workflowRun.actionDurationSec = action.timeoutSec || 120;
            renderWorkflow();
            showNavMessage('', `动作链：正在执行第 ${index + 1} 步“${escapeHtml(action.title || armActionTitle(action))}”...`);
            const actionData = await postJson('/api/actions/execute', {
              type: 'arm_task',
              phase: action.phase,
              target_object: action.targetObject || '',
              timeout_sec: workflowRun.actionDurationSec,
              name: action.title || armActionTitle(action)
            });
            if (!actionData.ok) {
              const statusText = actionData.final_status?.status_text || actionData.last_status?.status_text || '';
              throw new Error(actionData.error || statusText || '机械臂任务执行失败');
            }
            workflowRun.completed[action.id] = true;
            continue;
          }
          if (action.type === 'column_height') {
            workflowRun.actionStartedAt = Date.now();
            workflowRun.actionDurationSec = action.timeoutSec || 30;
            renderWorkflow();
            showNavMessage('', `动作链：正在执行第 ${index + 1} 步“立柱升降” target=${Number(action.targetHeightM || 0).toFixed(3)}m...`);
            const actionData = await postJson('/api/actions/execute', {
              type: 'column_height',
              target_height_m: Number(action.targetHeightM || 0),
              timeout_sec: workflowRun.actionDurationSec,
              name: action.title || '立柱升降'
            });
            if (!actionData.ok) {
              throw new Error(actionData.error || '立柱升降执行失败');
            }
            workflowRun.completed[action.id] = true;
            continue;
          }
          if (action.type === 'fake_pick_xiongmao') {
            workflowRun.actionStartedAt = Date.now();
            workflowRun.actionDurationSec = action.durationSec || 5;
            renderWorkflow();
            showNavMessage('', `动作链：正在执行第 ${index + 1} 步“拾取熊猫烟”（${workflowRun.actionDurationSec} 秒）...`);
            const actionData = await postJson('/api/actions/execute', {
              type: 'fake_pick_xiongmao',
              name: '拾取熊猫烟',
              duration_sec: workflowRun.actionDurationSec
            });
            if (!actionData.ok) {
              throw new Error(actionData.error || '假动作执行失败');
            }
            workflowRun.completed[action.id] = true;
            continue;
          }
          throw new Error(`不支持的动作类型：${action.type}`);
        }
        workflowRun.running = false;
        workflowRun.note = '动作链完成';
        setWorkflowRunningUi(false);
        renderWorkflow();
        showNavMessage('', '动作链：<strong>已完成</strong>');
      } catch (err) {
        markWorkflowError(String(err.message || err));
        showNavMessage('bad', `动作链：<strong>执行失败</strong> ${err.message || err}`);
      }
    }

    async function stopNavigation() {
      setNavButtonsBusy(true);
      showNavMessage('', '停止：正在取消底盘导航，并发送机械臂停止/复位指令...');
      try {
        const data = await postJson('/api/actions/stop_all', {});
        if (data.ok) {
          const phases = Array.isArray(data.arm_stop_phases) ? data.arm_stop_phases.join(' / ') : '--';
          showNavMessage('', `停止：<strong>已发送</strong> 底盘取消 + 机械臂 ${phases}`);
        } else {
          showNavMessage('bad', `停止：<strong>失败</strong> ${data.error || ''}`);
        }
        workflowRun.running = false;
        workflowRun.note = '已停止';
        setWorkflowRunningUi(false);
        renderWorkflow();
      } catch (err) {
        showNavMessage('bad', `导航：<strong>停止请求失败</strong> ${err}`);
      } finally {
        setNavButtonsBusy(false);
        tick();
      }
    }

    mapCanvas.addEventListener('click', ev => {
      if (!lastState?.map || !currentMapGeom) return;
      if (workflowRun.running) {
        showNavMessage('bad', '动作链执行中：<strong>请先停止后再修改导航点</strong>');
        return;
      }
      const c = eventToCanvas(mapCanvas, ev);
      const p = canvasToMap(lastState.map, c.x, c.y, currentMapGeom);
      if (!p) {
        showNavMessage('bad', '导航：<strong>点在地图外</strong>');
        return;
      }
      if (headingMode) {
        if (!selectedWaypoints.length) {
          showNavMessage('bad', '导航：<strong>请先添加终点，再设置朝向</strong>');
          setHeadingMode(false);
          return;
        }
        finalHeadingPoint = { x: Number(p.x.toFixed(4)), y: Number(p.y.toFixed(4)) };
        manualHeadingDeg = null;
        document.getElementById('headingDegInput').value = '';
        setHeadingMode(false);
        refreshMapUi();
        const targetYaw = computeTargetYaw(lastState);
        if (targetYaw) {
          setLastNavigationYaw(targetYaw.yawDeg);
          document.getElementById('headingDegInput').value = targetYaw.yawDeg.toFixed(1);
        }
        showNavMessage('', `导航：已设置终点朝向 <strong>${targetYaw ? targetYaw.yawDeg.toFixed(1) : '--'}°</strong>`);
        return;
      }
      const yawDeg = lastState?.odom?.yaw_deg || 0;
      addWorkflowAction({
        id: makeActionId('nav'),
        type: 'navigate',
        title: '导航',
        x: Number(p.x.toFixed(4)),
        y: Number(p.y.toFixed(4)),
        yawDeg: Number(radToDeg(normalizeAngle(yawDeg * Math.PI / 180)).toFixed(3))
      });
      setActionPoseInputs(p.x, p.y, yawDeg);
      finalHeadingPoint = null;
      document.getElementById('headingDegInput').value = '';
    });

    document.getElementById('undoWaypointBtn').addEventListener('click', () => {
      const lastNav = getLastNavigationAction();
      if (lastNav) removeWorkflowAction(lastNav.id);
      finalHeadingPoint = null;
      manualHeadingDeg = null;
      document.getElementById('headingDegInput').value = '';
      if (!selectedWaypoints.length) setHeadingMode(false);
      refreshMapUi();
    });
    document.getElementById('clearWaypointsBtn').addEventListener('click', () => {
      workflowActions = workflowActions.filter(action => action.type !== 'navigate');
      syncSelectedWaypointsFromActions();
      finalHeadingPoint = null;
      manualHeadingDeg = null;
      document.getElementById('headingDegInput').value = '';
      setHeadingMode(false);
      resetWorkflowRun('待执行');
      refreshMapUi();
    });
    document.getElementById('setHeadingBtn').addEventListener('click', () => {
      if (!selectedWaypoints.length) {
        showNavMessage('bad', '导航：<strong>请先添加至少一个航点，再设置朝向</strong>');
        return;
      }
      setHeadingMode(!headingMode);
    });
    document.getElementById('clearHeadingBtn').addEventListener('click', () => {
      finalHeadingPoint = null;
      manualHeadingDeg = null;
      document.getElementById('headingDegInput').value = '';
      setHeadingMode(false);
      refreshMapUi();
    });
    document.getElementById('applyHeadingDegBtn').addEventListener('click', applyHeadingDegFromInput);
    document.getElementById('headingDegInput').addEventListener('keydown', ev => {
      if (ev.key === 'Enter') applyHeadingDegFromInput();
    });
    document.getElementById('directNoAvoidanceMode').addEventListener('change', () => {
      const enabled = Boolean(document.getElementById('directNoAvoidanceMode')?.checked);
      if (enabled) document.getElementById('rawCmdVelNoAvoidanceMode').checked = false;
      showNavMessage(enabled ? 'warn' : '', enabled
        ? '导航：已启用 <strong>直连不绕障</strong>，Slamware 会按指定路径走，遇障停止。'
        : '导航：已切回 <strong>普通避障</strong>。');
      tick();
    });
    document.getElementById('rawCmdVelNoAvoidanceMode').addEventListener('change', () => {
      const enabled = Boolean(document.getElementById('rawCmdVelNoAvoidanceMode')?.checked);
      if (enabled) document.getElementById('directNoAvoidanceMode').checked = false;
      showNavMessage(enabled ? 'bad' : '', enabled
        ? '导航：已启用 <strong>裸控无避障</strong>，将直接发布 /cmd_vel，请确认路径完全安全。'
        : '导航：已关闭 <strong>裸控无避障</strong>。');
      tick();
    });
    document.getElementById('startNavigationBtn').addEventListener('click', startNavigation);
    document.getElementById('runWorkflowBtn').addEventListener('click', runWorkflow);
    document.getElementById('stopNavigationBtn').addEventListener('click', stopNavigation);
    document.getElementById('clearWorkflowBtn').addEventListener('click', clearWorkflowActions);
    document.getElementById('newActionType').addEventListener('change', updateActionBuilderVisibility);
    document.getElementById('newActionPointSelect').addEventListener('change', onActionPointSelected);
    document.getElementById('fillCurrentPoseBtn').addEventListener('click', fillCurrentPoseForAction);
    document.getElementById('addWorkflowActionBtn').addEventListener('click', addActionFromBuilder);
    document.getElementById('resetWorkflowBtn').addEventListener('click', () => resetWorkflowRun('待执行'));
    document.getElementById('recordCurrentPointBtn').addEventListener('click', recordCurrentPoint);
    document.getElementById('newPointBtn').addEventListener('click', clearPointForm);
    document.getElementById('savePointBtn').addEventListener('click', savePoint);
    document.getElementById('addPointToNavBtn').addEventListener('click', addEditedPointToNav);
    document.getElementById('deletePointBtn').addEventListener('click', deletePoint);

    cloudCanvas.addEventListener('pointerdown', ev => {
      cloudDragging = true;
      cloudLast = { x: ev.clientX, y: ev.clientY };
      cloudCanvas.setPointerCapture(ev.pointerId);
    });
    cloudCanvas.addEventListener('pointermove', ev => {
      if (!cloudDragging) return;
      const dx = ev.clientX - cloudLast.x;
      const dy = ev.clientY - cloudLast.y;
      cloudLast = { x: ev.clientX, y: ev.clientY };
      cloudYaw += dx * 0.01;
      cloudPitch = Math.max(-1.35, Math.min(1.35, cloudPitch + dy * 0.01));
    });
    cloudCanvas.addEventListener('pointerup', () => { cloudDragging = false; });
    cloudCanvas.addEventListener('pointerleave', () => { cloudDragging = false; });

    window.addEventListener('resize', () => { cachedMapSeq = -1; tick(); });
    setInterval(tick, 500);
    updateActionBuilderVisibility();
    renderWorkflow();
    loadSavedPoints();
    tick();
  </script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    state: SharedState
    node_ref: BaseSensorNode
    point_store: SavedPointStore

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            self.write_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif parsed.path == "/api/state":
            self.write_json(self.state.snapshot())
        elif parsed.path == "/api/points":
            self.write_json(self.point_store.list_payload())
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
                }
            )
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
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--global-plan-path-topic", default="/slamware_ros_sdk_server_node/global_plan_path")
    parser.add_argument("--robot-basic-state-topic", default="/slamware_ros_sdk_server_node/robot_basic_state")
    parser.add_argument("--slamware-state-topic", default="/slamware_ros_sdk_server_node/state")
    parser.add_argument("--arm-command-topic", default="/arm_control_refactor/task_command")
    parser.add_argument("--arm-status-topic", default="/arm_control_refactor/task_status")
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
    parser.add_argument("--column-height-min-m", type=float, default=-1.0)
    parser.add_argument("--column-height-max-m", type=float, default=1.0)
    parser.add_argument("--raw-nav-linear-speed-mps", type=float, default=0.12)
    parser.add_argument("--raw-nav-angular-speed-radps", type=float, default=0.45)
    parser.add_argument("--raw-nav-position-tolerance-m", type=float, default=0.08)
    parser.add_argument("--raw-nav-yaw-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--points-file", default="data/nav_points.json")
    parser.add_argument(
        "--min-localization-quality",
        type=int,
        default=-1,
        help="Block navigation when localization_quality is below this value. -1 means warn only.",
    )
    parser.add_argument("--max-cloud-points", type=int, default=6000)
    parser.add_argument("--max-track", type=int, default=1200)
    args = parser.parse_args()

    state = SharedState(max_track=args.max_track)
    point_store = SavedPointStore(args.points_file)
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
        raw_nav_linear_speed_mps=args.raw_nav_linear_speed_mps,
        raw_nav_angular_speed_radps=args.raw_nav_angular_speed_radps,
        raw_nav_position_tolerance_m=args.raw_nav_position_tolerance_m,
        raw_nav_yaw_tolerance_deg=args.raw_nav_yaw_tolerance_deg,
        min_localization_quality=args.min_localization_quality,
        max_cloud_points=args.max_cloud_points,
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
