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


def navigation_mode_label(mode: str) -> str:
    if mode == "raw_cmd_vel_no_obstacle_avoidance":
        return "裸控不避障"
    if mode == "direct_key_points_stop_on_obstacle":
        return "直连少绕路"
    if mode == "normal_slamware":
        return "普通避障"
    return mode or "--"


def latest_navigation_fault(faults: Iterable[Dict[str, Any]], command: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not command:
        return None
    command_seq = command.get("seq")
    for fault in reversed(list(faults or [])):
        last = ((fault.get("navigation") or {}).get("last_command") or {})
        if command_seq is None or last.get("seq") == command_seq:
            reason = str(fault.get("reason") or "")
            if reason in {
                "navigation_start_blocked",
                "global_plan_zero",
                "odom_still_during_navigation",
                "laser_close_during_navigation",
                "sensor_impact",
                "frontend_navigation_stall",
                "frontend_navigation_timeout",
            }:
                return fault
    return None


def derive_navigation_debug_status(
    scan: Optional[Dict[str, Any]],
    odom: Optional[Dict[str, Any]],
    sensors: Optional[Dict[str, Any]],
    plan: Optional[Dict[str, Any]],
    command: Optional[Dict[str, Any]],
    robot_basic_state: Optional[Dict[str, Any]],
    faults: Iterable[Dict[str, Any]],
    now: float,
) -> Dict[str, Any]:
    output: Dict[str, Any] = {
        "level": "idle",
        "label": "等待导航",
        "detail": "暂无导航命令",
        "reason": "idle",
        "inferred": False,
    }
    if not command:
        return output

    command_type = str(command.get("type") or "")
    mode = str(command.get("navigation_mode") or "")
    mode_label = navigation_mode_label(mode)
    received_at = finite_or_none(command.get("received_at"))
    age_s = finite_or_none(now - received_at, 1) if received_at is not None else None
    output.update(
        {
            "level": "ok",
            "label": "导航中",
            "detail": f"{mode_label}，已运行 {age_s}s" if age_s is not None else mode_label,
            "reason": "running",
            "mode": mode,
            "mode_label": mode_label,
            "command_type": command_type,
            "command_seq": command.get("seq"),
            "age_s": age_s,
        }
    )

    if command.get("dry_run"):
        output.update({"level": "idle", "label": "Dry-run 已解析", "detail": "不会驱动机器人", "reason": "dry_run"})
        return output

    if command_type == "cancel_action":
        output.update({"level": "idle", "label": "已停止/取消导航", "detail": "最近一次命令是停止导航", "reason": "cancelled"})
        return output

    if command_type == "raw_cmd_vel_navigation":
        raw_status = str(command.get("raw_nav_status") or "running")
        labels = {
            "running": ("ok", "裸控导航中", "直接发布 /cmd_vel，不使用避障"),
            "final_yaw": ("ok", "裸控对齐终点", "正在对齐目标朝向"),
            "done": ("ok", "已到达目标点", "裸控导航完成"),
            "dry_run": ("idle", "Dry-run 已解析", "不会驱动机器人"),
            "cancelled": ("idle", "已停止/取消导航", "裸控导航已取消"),
            "timeout": ("bad", "导航超时", str(command.get("raw_nav_error") or "raw cmd_vel timeout")),
            "error": ("bad", "导航失败", str(command.get("raw_nav_error") or "raw cmd_vel error")),
        }
        level, label, detail = labels.get(raw_status, ("ok", "裸控导航中", raw_status))
        output.update({"level": level, "label": label, "detail": detail, "reason": f"raw_{raw_status}", "raw_nav_status": raw_status})
        return output

    if command_type != "move_to_locations":
        output.update({"label": command_type or "未知导航命令", "detail": "非标准导航命令"})
        return output

    plan_count = int((plan or {}).get("total_poses") or 0)
    min_range = finite_or_none((scan or {}).get("min_range"), 3)
    hits = [item for item in (sensors or {}).get("items", []) if item.get("is_in_impact")]
    goal_distance = None
    waypoints = command.get("waypoints") if isinstance(command.get("waypoints"), list) else []
    if odom and waypoints:
        goal = waypoints[-1]
        if isinstance(goal, dict) and goal.get("x") is not None and goal.get("y") is not None and odom.get("x") is not None and odom.get("y") is not None:
            goal_distance = math.hypot(float(goal["x"]) - float(odom["x"]), float(goal["y"]) - float(odom["y"]))
    detail_parts = [mode_label, f"规划点 {plan_count}"]
    if goal_distance is not None:
        output["distance_m"] = finite_or_none(goal_distance, 3)
        detail_parts.append(f"距目标 {goal_distance:.2f}m")
    if min_range is not None:
        output["min_scan_range_m"] = min_range
        detail_parts.append(f"最近障碍 {min_range:.2f}m")

    fault = latest_navigation_fault(faults, command)
    if fault:
        reason = str(fault.get("reason") or "")
        output["fault_reason"] = reason
        output["fault_seq"] = fault.get("seq")
        if reason == "navigation_start_blocked":
            output.update({"level": "bad", "label": "导航启动失败", "detail": "开始前安全检查未通过", "reason": reason})
            return output
        if reason == "global_plan_zero":
            output.update({"level": "bad", "label": "疑似避障失败", "detail": "规划路径为空，可能无可行路径", "reason": reason, "inferred": True})
            return output
        if reason in ("odom_still_during_navigation", "frontend_navigation_stall", "frontend_navigation_timeout"):
            output.update({"level": "bad", "label": "疑似导航卡住", "detail": "导航中里程计长时间不动或前端等待超时", "reason": reason, "inferred": True})
            return output
        if reason in ("laser_close_during_navigation", "sensor_impact"):
            output.update({"level": "warn", "label": "可能正在避障", "detail": "导航中检测到近障碍或传感器触发", "reason": reason, "inferred": True})
            return output

    if hits:
        output.update({"level": "bad", "label": "传感器触发", "detail": "导航附近传感器检测到触发", "reason": "sensor_impact", "inferred": True})
        return output
    if min_range is not None and min_range <= 0.45:
        output.update({"level": "warn", "label": "可能正在避障", "detail": "导航方向附近有近障碍", "reason": "laser_close", "inferred": True})
        return output
    if plan_count == 0 and age_s is not None and age_s > 2.0:
        output.update({"level": "bad", "label": "疑似避障失败", "detail": "导航已开始但规划路径为空", "reason": "plan_zero", "inferred": True})
        return output
    if goal_distance is not None and goal_distance <= 0.18:
        output.update({"level": "ok", "label": "已到达目标点", "detail": "距离目标已进入阈值", "reason": "arrived"})
        return output

    if robot_basic_state and robot_basic_state.get("is_localization_enabled") is False:
        detail_parts.append("定位未开启")
        output.update({"level": "bad", "label": "定位未开启", "reason": "localization_off"})
    elif robot_basic_state and finite_or_none(robot_basic_state.get("localization_quality")) is not None and float(robot_basic_state.get("localization_quality")) <= 0:
        detail_parts.append("定位质量低")
        output.update({"level": "warn", "label": "导航中，定位质量低", "reason": "localization_quality_low"})
    else:
        output.update({"level": "ok", "label": f"导航中（{mode_label}）", "reason": "running"})
    output["detail"] = "，".join(detail_parts)
    return output


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
            fault_snapshots = list(self.fault_snapshots)[-40:]
            debug_status = derive_navigation_debug_status(
                self.scan,
                self.odom,
                self.sensors,
                self.global_plan_path,
                self.last_navigation_command,
                self.robot_basic_state,
                fault_snapshots,
                now,
            )
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
                    "debug_status": debug_status,
                },
                "arm_control": {
                    "last_status": self.arm_task_status,
                    "last_command": self.last_arm_task_command,
                },
                "fault_snapshots": fault_snapshots,
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
