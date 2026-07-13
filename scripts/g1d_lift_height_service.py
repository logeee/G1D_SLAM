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
DEFAULT_AUTO_CALIBRATE_ON_BOOT = True
DEFAULT_AUTO_CALIBRATE_MAX_UPTIME_SEC = 300.0
DEFAULT_AUTO_CALIBRATE_TIMEOUT_SEC = 90.0
DEFAULT_AUTO_CALIBRATE_WARMUP_SEC = 2.0
DEFAULT_AUTO_CALIBRATE_STABLE_THRESHOLD_M = 0.002
DEFAULT_AUTO_CALIBRATE_STABLE_COUNT = 3
DEFAULT_AUTO_CALIBRATE_POLL_INTERVAL_SEC = 0.1
DEFAULT_AUTO_CALIBRATE_MIN_JUMP_RESET_M = 0.05


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
        self.auto_lock = threading.Lock()
        self.auto_calibration_state: dict[str, Any] = {
            "enabled": bool(args.auto_calibrate_on_boot),
            "state": "disabled" if not args.auto_calibrate_on_boot else "starting",
            "message": "",
            "stable_threshold_m": args.auto_calibrate_stable_threshold_m,
            "stable_count_required": args.auto_calibrate_stable_count,
            "boot_uptime_sec": uptime_sec(),
            "updated_at": now_iso(),
        }
        self.calibration_path = Path(args.calibration_path).expanduser()
        if args.auto_calibrate_on_boot:
            thread = threading.Thread(target=self._auto_calibrate_on_boot, daemon=True)
            thread.start()

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
                if not self.args.auto_calibrate_on_boot:
                    return float(saved_min), "file", saved
                saved_boot_id = str(saved.get("boot_id") or "")
                current_boot_id = boot_id()
                if saved_boot_id and current_boot_id and saved_boot_id == current_boot_id:
                    source = str(saved.get("source") or "file")
                    return float(saved_min), source, saved
                return float(self.args.sdk_min_m), "stale_file_pending_auto_boot", saved
        return float(self.args.sdk_min_m), "arg_default", saved

    def _set_auto_state(self, **updates: Any) -> None:
        with self.auto_lock:
            self.auto_calibration_state.update(updates)
            self.auto_calibration_state["updated_at"] = now_iso()

    def _auto_state(self) -> dict[str, Any]:
        with self.auto_lock:
            return dict(self.auto_calibration_state)

    def save_min(self, raw_y_m: float, *, source: str, note: str = "", updated_at: float = 0.0, extra: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        full_travel = float(self.args.full_travel_m)
        now = time.time()
        payload = {
            "sdk_min_m": raw_y_m,
            "sdk_max_m": raw_y_m + full_travel,
            "full_travel_m": full_travel,
            "source": source,
            "note": note,
            "boot_id": boot_id(),
            "uptime_sec": uptime_sec(),
            "hispeed_y_m": raw_y_m,
            "data_age_sec": now - updated_at if updated_at > 0 else None,
            "updated_at": now_iso(),
        }
        if extra:
            payload.update(extra)
        atomic_write_json(self.calibration_path, payload)
        return {
            "ok": True,
            "message": "hispeed_y_m saved as physical zero/min height",
            "calibration_path": str(self.calibration_path),
            "calibration": payload,
        }

    def save_current_as_min(self, *, note: str = "") -> dict[str, Any]:
        raw_y_m, updated_at = self.latest_raw()
        if raw_y_m is None:
            return {
                "ok": False,
                "error": self.reader_error or "no hispeed frame",
                "calibration_path": str(self.calibration_path),
            }
        return self.save_min(raw_y_m, source="manual_min_calibration", note=note, updated_at=updated_at)

    def _auto_calibrate_on_boot(self) -> None:
        saved = load_json(self.calibration_path)
        current_boot_id = boot_id()
        if saved and str(saved.get("boot_id") or "") == current_boot_id and finite_or_none(saved.get("sdk_min_m")) is not None:
            self._set_auto_state(state="skipped", message="current boot already calibrated", calibration_source=saved.get("source") or "file")
            return

        current_uptime = uptime_sec()
        max_uptime = float(self.args.auto_calibrate_max_uptime_sec)
        if current_uptime is not None and current_uptime > max_uptime:
            self._set_auto_state(
                state="skipped",
                message=f"uptime {current_uptime:.1f}s is greater than auto calibration window {max_uptime:.1f}s",
                uptime_sec=current_uptime,
            )
            return

        timeout = max(2.0, float(self.args.auto_calibrate_timeout_sec))
        warmup = max(0.0, float(self.args.auto_calibrate_warmup_sec))
        threshold = max(0.0001, float(self.args.auto_calibrate_stable_threshold_m))
        required_count = max(1, int(self.args.auto_calibrate_stable_count))
        poll_interval = max(0.02, float(self.args.auto_calibrate_poll_interval_sec))
        min_jump_reset = max(threshold * 2.0, float(self.args.auto_calibrate_min_jump_reset_m))

        start = time.monotonic()
        min_seen: Optional[float] = None
        stable_count = 0
        above_min_count = 0
        sample_count = 0
        latest_updated_at = 0.0
        self._set_auto_state(
            state="detecting_min",
            message="tracking hispeed_y minimum until it is stable",
            timeout_sec=timeout,
            warmup_sec=warmup,
            min_seen_y_m=None,
            stable_count=0,
            sample_count=0,
        )

        while time.monotonic() - start < timeout:
            raw_y_m, updated_at = self.latest_raw()
            elapsed = time.monotonic() - start
            if raw_y_m is None:
                self._set_auto_state(
                    state="detecting_min",
                    message=self.reader_error or "waiting for hispeed frame",
                    elapsed_sec=round(elapsed, 3),
                    min_seen_y_m=finite_or_none(min_seen, 6),
                    stable_count=stable_count,
                    sample_count=sample_count,
                )
                time.sleep(poll_interval)
                continue

            sample_count += 1
            latest_updated_at = max(latest_updated_at, updated_at)
            if min_seen is not None and raw_y_m > min_seen + min_jump_reset:
                above_min_count += 1
                if above_min_count >= required_count:
                    min_seen = raw_y_m
                    stable_count = 0
                    above_min_count = 0
            else:
                above_min_count = 0

            if min_seen is None or raw_y_m < min_seen:
                min_seen = raw_y_m
                stable_count = 0

            if min_seen is not None and abs(raw_y_m - min_seen) <= threshold:
                stable_count += 1
            else:
                stable_count = 0

            self._set_auto_state(
                state="detecting_min",
                message="tracking hispeed_y minimum until it is stable",
                elapsed_sec=round(elapsed, 3),
                current_y_m=finite_or_none(raw_y_m, 6),
                min_seen_y_m=finite_or_none(min_seen, 6),
                stable_count=stable_count,
                above_min_count=above_min_count,
                sample_count=sample_count,
                stable_threshold_m=threshold,
                stable_count_required=required_count,
                min_jump_reset_m=min_jump_reset,
            )

            if elapsed >= warmup and min_seen is not None and stable_count >= required_count:
                result = self.save_min(
                    min_seen,
                    source="auto_boot_min_calibration",
                    note="auto detected minimum hispeed_y after boot column auto-lower",
                    updated_at=latest_updated_at,
                    extra={
                        "auto_detection": {
                            "elapsed_sec": round(elapsed, 3),
                            "stable_threshold_m": threshold,
                            "stable_count_required": required_count,
                            "stable_count": stable_count,
                            "sample_count": sample_count,
                            "warmup_sec": warmup,
                            "timeout_sec": timeout,
                        }
                    },
                )
                self._set_auto_state(
                    state="done",
                    message="auto boot minimum detection done",
                    result=result,
                    min_seen_y_m=finite_or_none(min_seen, 6),
                    stable_count=stable_count,
                    sample_count=sample_count,
                    elapsed_sec=round(elapsed, 3),
                )
                return

            time.sleep(poll_interval)

        self._set_auto_state(
            state="failed",
            message="auto boot minimum detection timed out before minimum became stable",
            min_seen_y_m=finite_or_none(min_seen, 6),
            stable_count=stable_count,
            sample_count=sample_count,
            timeout_sec=timeout,
        )

    def reset_calibration(self) -> dict[str, Any]:
        try:
            self.calibration_path.unlink()
        except FileNotFoundError:
            pass
        except Exception as exc:
            return {"ok": False, "error": str(exc), "calibration_path": str(self.calibration_path)}
        return {"ok": True, "message": "calibration removed; service will use argument defaults until auto/manual calibration", "calibration_path": str(self.calibration_path)}

    def status(self) -> dict[str, Any]:
        started = time.time()
        raw_y_m, updated_at = self.latest_raw()

        full_travel = float(self.args.full_travel_m)
        sdk_min, calibration_source, calibration = self.calibration()
        saved_sdk_max = finite_or_none(calibration.get("sdk_max_m")) if calibration else None
        calibrated_sources = {"manual_min_calibration", "auto_boot_min_calibration", "current_hispeed_y_as_physical_min", "file"}
        if calibration_source in calibrated_sources:
            sdk_max = saved_sdk_max if saved_sdk_max is not None else sdk_min + full_travel
        else:
            sdk_max = float(self.args.sdk_max_m)
        offset_valid = bool(
            calibration_source in calibrated_sources
            or (calibration_source == "arg_default" and self.args.allow_arg_default)
        )
        lift_offset = sdk_min
        physical_height = None
        if raw_y_m is not None and offset_valid:
            physical_height = raw_y_m - lift_offset
            physical_height = max(0.0, min(full_travel, physical_height))

        now = time.time()
        data_age = (now - updated_at) if updated_at > 0 else None
        if raw_y_m is None:
            error = self.reader_error or "no hispeed frame"
        elif not offset_valid:
            error = f"lift offset is not calibrated for current boot: {calibration_source}"
        else:
            error = None
        ok = physical_height is not None and error is None
        auto_state = self._auto_state()
        detecting = bool(auto_state.get("state") in ("starting", "detecting_min"))
        return {
            "ok": ok,
            "service": "g1d_lift_height_service",
            "source": "unitree_dds_hispeed",
            "error": error,
            "hispeed_y_m": finite_or_none(raw_y_m, 6),
            "raw_height_m": finite_or_none(raw_y_m, 6),
            "lift_offset_m": finite_or_none(lift_offset, 6),
            "physical_height_m": finite_or_none(physical_height, 6),
            "offset": finite_or_none(lift_offset, 6) if offset_valid else None,
            "column_height": finite_or_none(raw_y_m, 6),
            "physical_height": finite_or_none(physical_height, 6) if physical_height is not None else -1.0,
            "detecting": detecting,
            "min_seen_y_m": finite_or_none(auto_state.get("min_seen_y_m"), 6),
            "stable_count": int(auto_state.get("stable_count") or 0),
            "full_travel_m": finite_or_none(full_travel, 6),
            "sdk_min_m": finite_or_none(sdk_min, 6),
            "sdk_max_m": finite_or_none(sdk_max, 6),
            "physical_min_m": 0.0,
            "physical_max_m": finite_or_none(full_travel, 6),
            "offset_valid": offset_valid,
            "calibration_source": calibration_source,
            "calibration_path": str(self.calibration_path),
            "auto_calibration": auto_state,
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
    parser.add_argument("--allow-arg-default", action="store_true", default=os.environ.get("G1D_LIFT_ALLOW_ARG_DEFAULT", "0").lower() in ("1", "true", "yes"))
    parser.add_argument("--auto-calibrate-on-boot", dest="auto_calibrate_on_boot", action="store_true", default=os.environ.get("G1D_LIFT_AUTO_CALIBRATE_ON_BOOT", str(int(DEFAULT_AUTO_CALIBRATE_ON_BOOT))).lower() in ("1", "true", "yes"))
    parser.add_argument("--no-auto-calibrate-on-boot", dest="auto_calibrate_on_boot", action="store_false")
    parser.add_argument("--auto-calibrate-max-uptime-sec", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_MAX_UPTIME_SEC", DEFAULT_AUTO_CALIBRATE_MAX_UPTIME_SEC)))
    parser.add_argument("--auto-calibrate-timeout-sec", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_TIMEOUT_SEC", DEFAULT_AUTO_CALIBRATE_TIMEOUT_SEC)))
    parser.add_argument("--auto-calibrate-warmup-sec", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_WARMUP_SEC", DEFAULT_AUTO_CALIBRATE_WARMUP_SEC)))
    parser.add_argument("--auto-calibrate-stable-threshold-m", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_STABLE_THRESHOLD_M", DEFAULT_AUTO_CALIBRATE_STABLE_THRESHOLD_M)))
    parser.add_argument("--auto-calibrate-stable-count", type=int, default=int(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_STABLE_COUNT", DEFAULT_AUTO_CALIBRATE_STABLE_COUNT)))
    parser.add_argument("--auto-calibrate-poll-interval-sec", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_POLL_INTERVAL_SEC", DEFAULT_AUTO_CALIBRATE_POLL_INTERVAL_SEC)))
    parser.add_argument("--auto-calibrate-min-jump-reset-m", type=float, default=float(os.environ.get("G1D_LIFT_AUTO_CALIBRATE_MIN_JUMP_RESET_M", DEFAULT_AUTO_CALIBRATE_MIN_JUMP_RESET_M)))
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
