"""Left-eye head-camera MJPEG streamer.

Pipeline:
  ZMQ SUB (tcp://host:zmq_port) -> latest side-by-side JPEG bytes
    -> decode -> crop left half -> re-encode JPEG -> MJPEG multipart

The SUB background thread is reference-counted: it starts when the first MJPEG
client connects and stops after the last one disconnects (plus a short linger),
so we never keep the camera subscription open when nobody is watching.

Config (zmq_port) is auto-discovered from the camera config REP server on
`request_port` (default 60000); a static `zmq_port` override skips discovery.
"""
from __future__ import annotations

import threading
import time
from typing import Optional


class HeadCameraStreamer:
    def __init__(
        self,
        host: str = "127.0.0.1",
        request_port: int = 60000,
        zmq_port: int = 0,
        jpeg_quality: int = 75,
        max_fps: float = 15.0,
        eye: str = "left",
        linger_sec: float = 3.0,
    ) -> None:
        self.host = str(host)
        self.request_port = int(request_port)
        self.static_zmq_port = int(zmq_port)
        self.jpeg_quality = int(jpeg_quality)
        self.max_fps = max(1.0, float(max_fps))
        self.eye = str(eye or "left").lower()
        self.linger_sec = float(linger_sec)

        self._lock = threading.Lock()
        self._clients = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest: Optional[bytes] = None  # latest full-frame JPEG bytes
        self._latest_at = 0.0
        self._resolved_port: Optional[int] = None
        self._binocular = True
        self._last_error: Optional[str] = None

    # ---- config discovery ---------------------------------------------

    def _discover_port(self) -> int:
        if self.static_zmq_port:
            self._binocular = True
            return self.static_zmq_port
        import zmq

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{self.host}:{self.request_port}")
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        try:
            sock.send(b"GET_DATA")
            if dict(poller.poll(2000)).get(sock) == zmq.POLLIN:
                cfg = sock.recv_json()
                head = (cfg or {}).get("head_camera", {}) or {}
                self._binocular = bool(head.get("binocular", True))
                port = int(head.get("zmq_port") or 0)
                if port:
                    return port
            raise RuntimeError("camera config REP gave no head_camera.zmq_port")
        finally:
            sock.close()

    # ---- SUB thread ----------------------------------------------------

    def _run(self) -> None:
        import zmq

        try:
            port = self._discover_port()
            self._resolved_port = port
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"discover: {exc}"
            return

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{self.host}:{port}")
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        try:
            while not self._stop.is_set():
                if dict(poller.poll(200)).get(sock) == zmq.POLLIN:
                    data = sock.recv()
                    self._latest = data
                    self._latest_at = time.time()
                    self._last_error = None
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"sub: {exc}"
        finally:
            sock.close()

    def _ensure_thread(self) -> None:
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._stop.clear()
                self._latest = None
                self._thread = threading.Thread(target=self._run, name="head_camera_sub", daemon=True)
                self._thread.start()

    def _stop_thread(self) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=1.0)

    # ---- frame cropping -----------------------------------------------

    def _left_jpeg(self, full_jpeg: bytes) -> Optional[bytes]:
        import cv2
        import numpy as np

        img = cv2.imdecode(np.frombuffer(full_jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        h, w = img.shape[:2]
        if self._binocular and w % 2 == 0:
            half = w // 2
            crop = img[:, :half] if self.eye != "right" else img[:, half:]
        else:
            crop = img
        ok, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return buf.tobytes() if ok else None

    # ---- public API ----------------------------------------------------

    def mjpeg_frames(self):
        """Generator yielding MJPEG multipart chunks. Ref-counts the SUB thread."""
        with self._lock:
            self._clients += 1
        self._ensure_thread()
        min_period = 1.0 / self.max_fps
        try:
            # Wait briefly for the first frame.
            deadline = time.time() + 4.0
            while self._latest is None and time.time() < deadline and not self._stop.is_set():
                time.sleep(0.05)
            while True:
                raw = self._latest
                if raw is not None:
                    left = self._left_jpeg(raw)
                    if left is not None:
                        yield (
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + left + b"\r\n"
                        )
                time.sleep(min_period)
        finally:
            with self._lock:
                self._clients = max(0, self._clients - 1)
                remaining = self._clients
            if remaining == 0:
                # Linger a moment so quick reconnects don't thrash the SUB socket.
                def _linger_stop():
                    time.sleep(self.linger_sec)
                    with self._lock:
                        if self._clients == 0:
                            self._stop_thread()

                threading.Thread(target=_linger_stop, daemon=True).start()

    def status(self) -> dict:
        with self._lock:
            active = self._thread is not None and self._thread.is_alive()
            clients = self._clients
        age = (time.time() - self._latest_at) if self._latest_at else None
        return {
            "ok": True,
            "active": active,
            "clients": clients,
            "host": self.host,
            "resolved_zmq_port": self._resolved_port,
            "has_frame": self._latest is not None,
            "frame_age_sec": round(age, 3) if age is not None else None,
            "eye": self.eye,
            "error": self._last_error,
        }

    def shutdown(self) -> None:
        with self._lock:
            self._clients = 0
        self._stop_thread()
