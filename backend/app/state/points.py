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
