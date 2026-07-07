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
