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
