#!/usr/bin/env python3
"""Resident G1-D lift height service.

The G1-D column raw value is published on Unitree DDS topic
``rt/hispeed_state``.  This service converts that raw value into a physical
height that other frontends can poll over HTTP.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import parse_qs, urlparse


DEFAULT_BIND = "0.0.0.0"
DEFAULT_PORT = 28089
DEFAULT_DDS_INTERFACE = "eth0"
DEFAULT_DDS_HISPEED_TOPIC = "rt/hispeed_state"
DEFAULT_SDK2PY_PATH = "/home/unitree/unitree_sdk2_python"
DEFAULT_SDK_MIN_M = -0.1851
DEFAULT_SDK_MAX_M = 0.2469
DEFAULT_FULL_TRAVEL_M = 0.427
DEFAULT_CALIBRATION_PATH = "/home/unitree/.config/g1d_lift_height/calibration.json"


def finite_or_none(value: Any, ndigits: Optional[int] = None) -> Optional[float]:
    try:
        number = float(value)
    except Exception:
        return None
    if not math.isfinite(number):
        return None
    return round(number, ndigits) if ndigits is not None else number


def read_member(obj: Any, name: str, default: Any = None) -> Any:
    value = getattr(obj, name, default)
    if callable(value):
        return value()
    return value


def boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except Exception:
        return ""


def uptime_sec() -> Optional[float]:
    try:
        return float(Path("/proc/uptime").read_text().split()[0])
    except Exception:
        return None


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def load_json(path: Path) -> Optional[dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


class DdsHispeedReader:
    def __init__(self, *, network_interface: str, hispeed_topic: str, sdk2py_path: str) -> None:
        if sdk2py_path and Path(sdk2py_path).exists() and sdk2py_path not in sys.path:
            sys.path.insert(0, sdk2py_path)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_

        self.network_interface = network_interface
        self.hispeed_topic = hispeed_topic
        self.lock = threading.Lock()
        self.raw_y_m: Optional[float] = None
        self.updated_at = 0.0
        self.error: Optional[str] = None

        ChannelFactoryInitialize(0, network_interface)
        self.subscriber = ChannelSubscriber(hispeed_topic, Point32_)
        self.subscriber.Init(self._on_hispeed, 10)

    def _on_hispeed(self, msg: Any) -> None:
        raw_y = finite_or_none(read_member(msg, "y"))
        with self.lock:
            self.raw_y_m = raw_y
            self.updated_at = time.time()

    def latest(self, timeout_sec: float) -> tuple[Optional[float], float]:
        deadline = time.time() + max(0.05, float(timeout_sec))
        while time.time() < deadline:
            with self.lock:
                raw = self.raw_y_m
                updated = self.updated_at
            if raw is not None:
                return raw, updated
            time.sleep(0.01)
        with self.lock:
            return self.raw_y_m, self.updated_at


class LiftHeightService:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.reader: Optional[DdsHispeedReader] = None
        self.reader_error: Optional[str] = None
        self.lock = threading.Lock()
        self.calibration_path = Path(args.calibration_path).expanduser()

    def _reader_instance(self) -> Optional[DdsHispeedReader]:
        with self.lock:
            if self.reader is not None:
                return self.reader
            try:
                self.reader = DdsHispeedReader(
                    network_interface=self.args.dds_interface,
                    hispeed_topic=self.args.dds_hispeed_topic,
                    sdk2py_path=self.args.unitree_sdk2py_path,
                )
                self.reader_error = None
                return self.reader
            except Exception as exc:
                self.reader_error = str(exc)
                return None

    def latest_raw(self) -> tuple[Optional[float], float]:
        reader = self._reader_instance()
        if reader is not None:
            return reader.latest(self.args.wait_sec)
        return None, 0.0

    def calibration(self) -> tuple[float, str, Optional[dict[str, Any]]]:
        saved = load_json(self.calibration_path)
        if saved is not None:
            saved_min = finite_or_none(saved.get("sdk_min_m"))
            if saved_min is not None:
                return float(saved_min), "file", saved
        return float(self.args.sdk_min_m), "arg_default", saved

    def save_current_as_min(self, *, note: str = "") -> dict[str, Any]:
        raw_y_m, updated_at = self.latest_raw()
        if raw_y_m is None:
            return {
                "ok": False,
                "error": self.reader_error or "no hispeed frame",
                "calibration_path": str(self.calibration_path),
            }

        full_travel = float(self.args.full_travel_m)
        payload = {
            "sdk_min_m": raw_y_m,
            "sdk_max_m": raw_y_m + full_travel,
            "full_travel_m": full_travel,
            "source": "current_hispeed_y_as_physical_min",
            "note": note,
            "boot_id": boot_id(),
            "uptime_sec": uptime_sec(),
            "hispeed_y_m": raw_y_m,
            "data_age_sec": time.time() - updated_at if updated_at > 0 else None,
            "updated_at": now_iso(),
        }
        atomic_write_json(self.calibration_path, payload)
        return {
            "ok": True,
            "message": "current hispeed_y_m saved as physical zero/min height",
            "calibration_path": str(self.calibration_path),
            "calibration": payload,
        }

    def reset_calibration(self) -> dict[str, Any]:
        try:
            self.calibration_path.unlink()
        except FileNotFoundError:
            pass
        except Exception as exc:
            return {"ok": False, "error": str(exc), "calibration_path": str(self.calibration_path)}
        return {"ok": True, "message": "calibration removed; service will use argument defaults", "calibration_path": str(self.calibration_path)}

    def status(self) -> dict[str, Any]:
        started = time.time()
        raw_y_m, updated_at = self.latest_raw()

        full_travel = float(self.args.full_travel_m)
        sdk_min, calibration_source, calibration = self.calibration()
        saved_sdk_max = finite_or_none(calibration.get("sdk_max_m")) if calibration else None
        if calibration_source == "file":
            sdk_max = saved_sdk_max if saved_sdk_max is not None else sdk_min + full_travel
        else:
            sdk_max = float(self.args.sdk_max_m)
        lift_offset = sdk_min
        physical_height = None
        if raw_y_m is not None:
            physical_height = raw_y_m - lift_offset
            physical_height = max(0.0, min(full_travel, physical_height))

        now = time.time()
        data_age = (now - updated_at) if updated_at > 0 else None
        ok = physical_height is not None
        return {
            "ok": ok,
            "service": "g1d_lift_height_service",
            "source": "unitree_dds_hispeed",
            "error": None if ok else (self.reader_error or "no hispeed frame"),
            "hispeed_y_m": finite_or_none(raw_y_m, 6),
            "raw_height_m": finite_or_none(raw_y_m, 6),
            "lift_offset_m": finite_or_none(lift_offset, 6),
            "physical_height_m": finite_or_none(physical_height, 6),
            "full_travel_m": finite_or_none(full_travel, 6),
            "sdk_min_m": finite_or_none(sdk_min, 6),
            "sdk_max_m": finite_or_none(sdk_max, 6),
            "physical_min_m": 0.0,
            "physical_max_m": finite_or_none(full_travel, 6),
            "offset_valid": bool(calibration_source == "file" or self.args.allow_arg_default),
            "calibration_source": calibration_source,
            "calibration_path": str(self.calibration_path),
            "boot_id": boot_id(),
            "uptime_sec": finite_or_none(uptime_sec(), 3),
            "data_age_sec": finite_or_none(data_age, 3),
            "dds": {
                "network_interface": self.args.dds_interface,
                "hispeed_topic": self.args.dds_hispeed_topic,
                "unitree_sdk2py_path": self.args.unitree_sdk2py_path,
            },
            "timestamp": now,
            "updated_at": now_iso(),
            "elapsed_ms": round((time.time() - started) * 1000.0, 1),
        }


def make_handler(service: LiftHeightService) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "G1DLiftHeightService/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            if not service.args.quiet:
                super().log_message(fmt, *args)

        def write_json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "*")
            self.end_headers()

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/health":
                payload = service.status()
                self.write_json({"ok": payload.get("ok", False), "service": payload.get("service"), "error": payload.get("error")})
                return
            if path in ("/api/basic_status", "/api/lift_height", "/api/offset", "/api/status"):
                payload = service.status()
                self.write_json(payload, 200 if payload.get("ok") else 503)
                return
            if path == "/api/calibrate_min":
                query = parse_qs(parsed.query)
                note = str(query.get("note", [""])[0])
                result = service.save_current_as_min(note=note)
                payload = service.status()
                payload["calibration_result"] = result
                self.write_json(payload, 200 if result.get("ok") else 503)
                return
            if path == "/api/reset_calibration":
                result = service.reset_calibration()
                payload = service.status()
                payload["calibration_result"] = result
                self.write_json(payload, 200 if result.get("ok") else 503)
                return
            self.write_json({"ok": False, "error": f"not found: {path}"}, 404)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/api/calibrate_min":
                result = service.save_current_as_min(note="POST /api/calibrate_min")
                payload = service.status()
                payload["calibration_result"] = result
                self.write_json(payload, 200 if result.get("ok") else 503)
                return
            if path == "/api/reset_calibration":
                result = service.reset_calibration()
                payload = service.status()
                payload["calibration_result"] = result
                self.write_json(payload, 200 if result.get("ok") else 503)
                return
            self.write_json({"ok": False, "error": f"not found: {path}"}, 404)

    return Handler


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default=os.environ.get("G1D_LIFT_HEIGHT_BIND", DEFAULT_BIND))
    parser.add_argument("--port", type=int, default=int(os.environ.get("G1D_LIFT_HEIGHT_PORT", DEFAULT_PORT)))
    parser.add_argument("--dds-interface", default=os.environ.get("G1D_DDS_INTERFACE", DEFAULT_DDS_INTERFACE))
    parser.add_argument("--dds-hispeed-topic", default=os.environ.get("G1D_DDS_HISPEED_TOPIC", DEFAULT_DDS_HISPEED_TOPIC))
    parser.add_argument("--unitree-sdk2py-path", default=os.environ.get("UNITREE_SDK2PY_PATH", DEFAULT_SDK2PY_PATH))
    parser.add_argument("--sdk-min-m", type=float, default=float(os.environ.get("G1D_LIFT_SDK_MIN_M", DEFAULT_SDK_MIN_M)))
    parser.add_argument("--sdk-max-m", type=float, default=float(os.environ.get("G1D_LIFT_SDK_MAX_M", DEFAULT_SDK_MAX_M)))
    parser.add_argument("--full-travel-m", type=float, default=float(os.environ.get("G1D_LIFT_FULL_TRAVEL_M", DEFAULT_FULL_TRAVEL_M)))
    parser.add_argument("--calibration-path", default=os.environ.get("G1D_LIFT_CALIBRATION_PATH", DEFAULT_CALIBRATION_PATH))
    parser.add_argument("--allow-arg-default", action="store_true", default=os.environ.get("G1D_LIFT_ALLOW_ARG_DEFAULT", "1").lower() in ("1", "true", "yes"))
    parser.add_argument("--wait-sec", type=float, default=float(os.environ.get("G1D_LIFT_WAIT_SEC", 1.0)))
    parser.add_argument("--quiet", action="store_true", default=os.environ.get("G1D_LIFT_QUIET", "1").lower() in ("1", "true", "yes"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    service = LiftHeightService(args)
    httpd = ThreadingHTTPServer((args.bind, args.port), make_handler(service))
    print(
        f"G1D lift height service on http://{args.bind}:{args.port} "
        f"topic={args.dds_hispeed_topic} iface={args.dds_interface}",
        flush=True,
    )
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
