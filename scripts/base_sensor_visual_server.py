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
import signal
import struct
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import Point
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
                "freshness_s": {
                    "scan": self._age(self.scan, now),
                    "map": self._age(self.map, now),
                    "odom": self._age(self.odom, now),
                    "sensors": self._age(self.sensors, now),
                    "point_cloud": self._age(self.point_cloud, now),
                    "global_plan_path": self._age(self.global_plan_path, now),
                    "robot_basic_state": self._age(self.robot_basic_state, now),
                    "slamware_state": self._age(self.slamware_state, now),
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
        global_plan_path_topic: str,
        robot_basic_state_topic: str,
        slamware_state_topic: str,
        min_localization_quality: int,
        max_cloud_points: int,
    ) -> None:
        super().__init__("base_sensor_visual_server")
        self.state = state
        self.max_cloud_points = max(10, int(max_cloud_points))
        self.min_localization_quality = int(min_localization_quality)
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
        pointcloud_topics = [topic for topic in pointcloud_topics if topic]
        for topic in pointcloud_topics:
            self.create_subscription(PointCloud2, topic, self.make_point_cloud_cb(topic), qos)
        self.move_to_locations_pub = self.create_publisher(MoveToLocationsRequest, move_to_locations_topic, qos)
        self.cancel_action_pub = self.create_publisher(CancelActionRequest, cancel_action_topic, qos)
        self.get_logger().info(
            f"subscribed scan={scan_topic} map={map_topic} sensors={sensors_topic} "
            f"pointclouds={pointcloud_topics} plan={global_plan_path_topic}"
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

        request = MoveToLocationsRequest()
        request.locations = [Point(x=p["x"], y=p["y"], z=0.0) for p in waypoints]
        yaw = payload.get("yaw")
        if yaw is None:
            yaw = self.infer_final_yaw(waypoints)
        try:
            request.yaw = float(yaw)
        except (TypeError, ValueError):
            request.yaw = 0.0

        speed_ratio = payload.get("speed_ratio")
        if speed_ratio is not None:
            try:
                speed = max(0.05, min(1.0, float(speed_ratio)))
                request.options.speed_ratio.is_valid = True
                request.options.speed_ratio.value = speed
            except (TypeError, ValueError):
                return {"ok": False, "error": "invalid speed_ratio"}

        self.move_to_locations_pub.publish(request)
        now = time.time()
        command = {
            "received_at": now,
            "type": "move_to_locations",
            "waypoints": waypoints,
            "yaw": finite_or_none(request.yaw, 5),
            "yaw_deg": finite_or_none(math.degrees(float(request.yaw)), 2),
            "speed_ratio": finite_or_none(request.options.speed_ratio.value, 3)
            if request.options.speed_ratio.is_valid
            else None,
            "published_topic": "/slamware_ros_sdk_server_node/move_to_locations",
            "safety": safety,
        }
        with self.state.lock:
            self.state.seq["navigation_command"] += 1
            command["seq"] = self.state.seq["navigation_command"]
            self.state.last_navigation_command = command
        return {"ok": True, "navigation_started": True, "command": command}

    def cancel_navigation(self) -> Dict[str, Any]:
        self.cancel_action_pub.publish(CancelActionRequest())
        now = time.time()
        command = {
            "received_at": now,
            "type": "cancel_action",
            "published_topic": "/slamware_ros_sdk_server_node/cancel_action",
        }
        with self.state.lock:
            self.state.seq["navigation_command"] += 1
            command["seq"] = self.state.seq["navigation_command"]
            self.state.last_navigation_command = command
        return {"ok": True, "navigation_cancelled": True, "command": command}

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

    def infer_final_yaw(self, waypoints: List[Dict[str, float]]) -> float:
        if len(waypoints) >= 2:
            a = waypoints[-2]
            b = waypoints[-1]
            return math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        with self.state.lock:
            odom = self.state.odom
        if odom and odom.get("yaw") is not None:
            return float(odom["yaw"])
        return 0.0

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
    button:disabled {
      cursor: not-allowed;
      opacity: 0.55;
    }
    .nav-hint {
      color: var(--muted);
      font-size: 12px;
      margin-left: auto;
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
      .readout { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <h1>Base Sensor Dashboard</h1>
    <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">connecting</span></div>
  </header>
  <main>
    <section>
      <div class="panel-head">
        <h2>SLAM Map + Odometry</h2>
        <span id="mapMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="mapCanvas"></canvas></div>
      <div class="toolbar">
        <button id="undoWaypointBtn">撤销点</button>
        <button id="clearWaypointsBtn">清空点</button>
        <button id="startNavigationBtn" class="primary">开始导航</button>
        <button id="stopNavigationBtn" class="danger">停止导航</button>
        <span class="nav-hint">在地图上点击添加航点</span>
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
    let cloudYaw = -0.75;
    let cloudPitch = 0.65;
    let cloudDragging = false;
    let cloudLast = { x: 0, y: 0 };

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

      drawSelectedWaypoints(ctx, map, geom);

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
      const parts = [
        `<strong>${selectedWaypoints.length}</strong> 个航点`,
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
          navigation: state.navigation
        }, null, 2);
      } catch (err) {
        setStatus(false, `offline: ${err}`);
      }
    }

    function setNavButtonsBusy(busy) {
      document.getElementById('startNavigationBtn').disabled = busy;
      document.getElementById('stopNavigationBtn').disabled = busy;
      document.getElementById('undoWaypointBtn').disabled = busy;
      document.getElementById('clearWaypointsBtn').disabled = busy;
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

    async function startNavigation() {
      if (!selectedWaypoints.length) {
        showNavMessage('bad', '导航：<strong>请先在地图上点击选择至少一个航点</strong>');
        return;
      }
      setNavButtonsBusy(true);
      showNavMessage('', '导航：正在发送航点给 Slamware...');
      try {
        const data = await postJson('/api/navigation/start', { waypoints: selectedWaypoints });
        if (data.ok) {
          const warnings = data.command?.safety?.warnings || [];
          const warningText = warnings.length ? `，警告：${warnings.join('；')}` : '';
          showNavMessage(warnings.length ? 'warn' : '', `导航：<strong>已开始</strong>，航点 ${selectedWaypoints.length} 个${warningText}`);
        } else {
          const blockers = data.safety?.blockers || [];
          const details = blockers.length ? `：${blockers.join('；')}` : (data.error || 'unknown error');
          showNavMessage('bad', `导航：<strong>启动失败</strong>${details}`);
        }
      } catch (err) {
        showNavMessage('bad', `导航：<strong>请求失败</strong> ${err}`);
      } finally {
        setNavButtonsBusy(false);
        tick();
      }
    }

    async function stopNavigation() {
      setNavButtonsBusy(true);
      showNavMessage('', '导航：正在发送停止指令...');
      try {
        const data = await postJson('/api/navigation/cancel', {});
        if (data.ok) showNavMessage('', '导航：<strong>已发送停止指令</strong>');
        else showNavMessage('bad', `导航：<strong>停止失败</strong> ${data.error || ''}`);
      } catch (err) {
        showNavMessage('bad', `导航：<strong>停止请求失败</strong> ${err}`);
      } finally {
        setNavButtonsBusy(false);
        tick();
      }
    }

    mapCanvas.addEventListener('click', ev => {
      if (!lastState?.map || !currentMapGeom) return;
      const c = eventToCanvas(mapCanvas, ev);
      const p = canvasToMap(lastState.map, c.x, c.y, currentMapGeom);
      if (!p) {
        showNavMessage('bad', '导航：<strong>点在地图外</strong>');
        return;
      }
      selectedWaypoints.push({ x: Number(p.x.toFixed(4)), y: Number(p.y.toFixed(4)) });
      drawMap(lastState);
      updateReadouts(lastState);
      updateNavigationStatus(lastState);
    });

    document.getElementById('undoWaypointBtn').addEventListener('click', () => {
      selectedWaypoints.pop();
      if (lastState) {
        drawMap(lastState);
        updateReadouts(lastState);
        updateNavigationStatus(lastState);
      }
    });
    document.getElementById('clearWaypointsBtn').addEventListener('click', () => {
      selectedWaypoints = [];
      if (lastState) {
        drawMap(lastState);
        updateReadouts(lastState);
        updateNavigationStatus(lastState);
      }
    });
    document.getElementById('startNavigationBtn').addEventListener('click', startNavigation);
    document.getElementById('stopNavigationBtn').addEventListener('click', stopNavigation);

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
    tick();
  </script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    state: SharedState
    node_ref: BaseSensorNode

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            self.write_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif parsed.path == "/api/state":
            self.write_json(self.state.snapshot())
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
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

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
    parser.add_argument("--global-plan-path-topic", default="/slamware_ros_sdk_server_node/global_plan_path")
    parser.add_argument("--robot-basic-state-topic", default="/slamware_ros_sdk_server_node/robot_basic_state")
    parser.add_argument("--slamware-state-topic", default="/slamware_ros_sdk_server_node/state")
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
        global_plan_path_topic=args.global_plan_path_topic,
        robot_basic_state_topic=args.robot_basic_state_topic,
        slamware_state_topic=args.slamware_state_topic,
        min_localization_quality=args.min_localization_quality,
        max_cloud_points=args.max_cloud_points,
    )

    stop_event = threading.Event()

    def ros_spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=ros_spin, name="ros_spin", daemon=True)
    ros_thread.start()

    handler_cls = type("BoundDashboardHandler", (DashboardHandler,), {"state": state, "node_ref": node})
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
