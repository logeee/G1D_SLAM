"""Teleop jog controller for the G1-D base.

Design (ported from the standalone :18086 g1d_remote_control_server, plus a
server-side dead-man watchdog):

  * "Hold to move": while a direction button is held the frontend sends a jog
    command every ~0.35s. The first command launches
        g1d_simple_control <iface> <action> <speed> <hold_duration>
    as a long-lived subprocess (the binary keeps commanding motion). Repeat
    commands with the *same* action/speed only refresh a heartbeat timestamp;
    they do NOT restart the process (so motion stays smooth). A different
    action/speed replaces the running process.

  * Stop: on button release the frontend calls stop(); we terminate the hold
    process and issue an explicit `g1d_simple_control <iface> stop`, plus a
    pkill cleanup of any stale control processes.

  * Dead-man watchdog: a daemon thread checks every 0.2s. If a hold is active
    but no heartbeat arrived within `deadman_timeout_sec` (browser crash, tab
    closed, network loss), it stops the base automatically. This is the primary
    safety net and is entirely server-side.

Only one jog is ever active at a time. All public methods are thread-safe.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional


BASE_ACTIONS = {"forward", "back", "turn_left", "turn_right"}
STOP_ACTION = "stop"
# Frontend friendly aliases -> SDK action names.
ACTION_ALIASES = {
    "up": "forward",
    "down": "back",
    "left": "turn_left",
    "right": "turn_right",
    "ccw": "turn_left",
    "cw": "turn_right",
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


class JogController:
    def __init__(
        self,
        binary: str,
        interface: str,
        libdir: str = "",
        workdir: str = "",
        *,
        max_linear_mps: float = 0.3,
        max_angular_radps: float = 0.6,
        default_linear_mps: float = 0.15,
        default_angular_radps: float = 0.3,
        hold_duration_sec: float = 3600.0,
        deadman_timeout_sec: float = 1.0,
        command_timeout_extra_sec: float = 3.0,
    ) -> None:
        self.binary = str(binary)
        self.interface = str(interface)
        self.libdir = str(libdir or "")
        self.workdir = str(workdir or "") or None
        self.max_linear_mps = float(max_linear_mps)
        self.max_angular_radps = float(max_angular_radps)
        self.default_linear_mps = float(default_linear_mps)
        self.default_angular_radps = float(default_angular_radps)
        self.hold_duration_sec = float(hold_duration_sec)
        self.deadman_timeout_sec = float(deadman_timeout_sec)
        self.command_timeout_extra_sec = float(command_timeout_extra_sec)

        self._lock = threading.RLock()
        self._proc: Optional[subprocess.Popen] = None
        self._active_action: Optional[str] = None
        self._active_speed: Optional[float] = None
        self._last_heartbeat: float = 0.0

        self._stop_event = threading.Event()
        self._watchdog = threading.Thread(target=self._watchdog_loop, name="jog_watchdog", daemon=True)
        self._watchdog.start()

    # ---- helpers -------------------------------------------------------

    def _env(self) -> Dict[str, str]:
        env = os.environ.copy()
        if self.libdir:
            old = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = self.libdir if not old else f"{self.libdir}:{old}"
        return env

    def _normalize_action(self, raw: Any) -> str:
        action = str(raw or "").strip().lower()
        action = ACTION_ALIASES.get(action, action)
        if action not in BASE_ACTIONS:
            raise ValueError(f"unsupported jog action: {raw!r}")
        return action

    def _clamp_speed(self, action: str, raw: Any) -> float:
        if action in ("turn_left", "turn_right"):
            fallback, cap = self.default_angular_radps, self.max_angular_radps
        else:
            fallback, cap = self.default_linear_mps, self.max_linear_mps
        try:
            value = float(raw)
        except (TypeError, ValueError):
            value = fallback
        if value <= 0:
            value = fallback
        return round(_clamp(value, 0.0, cap), 4)

    def _binary_ok(self) -> bool:
        return bool(self.binary) and Path(self.binary).exists()

    # ---- process management (call under self._lock) --------------------

    def _launch(self, action: str, speed: float) -> subprocess.Popen:
        argv = [self.binary, self.interface, action, f"{speed:.4f}", f"{self.hold_duration_sec:.1f}"]
        return subprocess.Popen(
            argv,
            cwd=self.workdir,
            env=self._env(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _terminate_proc(self) -> Dict[str, Any]:
        proc = self._proc
        self._proc = None
        self._active_action = None
        self._active_speed = None
        if proc is None:
            return {"was_running": False}
        info: Dict[str, Any] = {"pid": proc.pid, "was_running": proc.poll() is None}
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
                info["killed"] = True
        return info

    def _issue_stop(self) -> Dict[str, Any]:
        """Send an explicit `stop` to the base and clean up stale processes."""
        result: Dict[str, Any] = {}
        try:
            completed = subprocess.run(
                [self.binary, self.interface, STOP_ACTION],
                cwd=self.workdir,
                env=self._env(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.command_timeout_extra_sec + 2.0,
                check=False,
            )
            result["stop_returncode"] = completed.returncode
        except Exception as exc:  # noqa: BLE001
            result["stop_error"] = str(exc)
        # Best-effort cleanup of any lingering control processes on this iface.
        try:
            subprocess.run(
                ["pkill", "-TERM", "-f", f"{Path(self.binary).name} {self.interface}"],
                check=False,
                capture_output=True,
                timeout=0.5,
            )
        except Exception:  # noqa: BLE001
            pass
        return result

    # ---- public API ----------------------------------------------------

    def move(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Start or refresh a held jog. Called repeatedly while a button is held."""
        try:
            action = self._normalize_action(payload.get("action"))
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        speed = self._clamp_speed(action, payload.get("speed"))
        if not self._binary_ok():
            return {"ok": False, "error": f"control binary not found: {self.binary}"}

        with self._lock:
            self._last_heartbeat = time.time()
            same = (
                self._proc is not None
                and self._proc.poll() is None
                and self._active_action == action
                and self._active_speed == speed
            )
            if same:
                return {"ok": True, "mode": "heartbeat", "action": action, "speed": speed}
            # Action/speed changed (or process died) -> replace.
            self._terminate_proc()
            try:
                self._proc = self._launch(action, speed)
            except Exception as exc:  # noqa: BLE001
                self._proc = None
                self._active_action = None
                self._active_speed = None
                return {"ok": False, "error": f"failed to launch control: {exc}"}
            self._active_action = action
            self._active_speed = speed
            return {"ok": True, "mode": "started", "action": action, "speed": speed, "pid": self._proc.pid}

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            terminated = self._terminate_proc()
            stop_info = self._issue_stop()
        return {"ok": True, "mode": "stopped", "terminated": terminated, **stop_info}

    def status(self) -> Dict[str, Any]:
        with self._lock:
            active = self._proc is not None and self._proc.poll() is None
            return {
                "ok": True,
                "active": active,
                "action": self._active_action,
                "speed": self._active_speed,
                "binary_ok": self._binary_ok(),
                "interface": self.interface,
                "max_linear_mps": self.max_linear_mps,
                "max_angular_radps": self.max_angular_radps,
                "default_linear_mps": self.default_linear_mps,
                "default_angular_radps": self.default_angular_radps,
                "deadman_timeout_sec": self.deadman_timeout_sec,
            }

    def shutdown(self) -> None:
        self._stop_event.set()
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass

    # ---- watchdog ------------------------------------------------------

    def _watchdog_loop(self) -> None:
        while not self._stop_event.wait(0.2):
            with self._lock:
                active = self._proc is not None and self._proc.poll() is None
                stale = active and (time.time() - self._last_heartbeat > self.deadman_timeout_sec)
            if stale:
                # Lost heartbeats while moving -> emergency stop.
                self.stop()
