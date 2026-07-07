"""Head-camera left-eye streaming for mapping-mode collection.

  GET /api/camera/left_eye        -> MJPEG multipart stream (browser <img>)
  GET /api/camera/left_eye.jpg    -> single JPEG snapshot
  GET /api/camera/status          -> streamer status
"""
from __future__ import annotations

import time

from fastapi import APIRouter
from starlette.responses import Response, StreamingResponse

from ..context import ctx
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/camera/left_eye")
def camera_left_eye():
    streamer = ctx.camera
    if streamer is None:
        return StrictJSONResponse({"ok": False, "error": "camera streamer unavailable"}, status_code=503)
    return StreamingResponse(
        streamer.mjpeg_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store", "Pragma": "no-cache"},
    )


@router.get("/api/camera/left_eye.jpg")
def camera_left_eye_snapshot():
    streamer = ctx.camera
    if streamer is None:
        return StrictJSONResponse({"ok": False, "error": "camera streamer unavailable"}, status_code=503)
    streamer._ensure_thread()
    deadline = time.time() + 4.0
    while streamer._latest is None and time.time() < deadline:
        time.sleep(0.05)
    raw = streamer._latest
    if raw is None:
        return StrictJSONResponse({"ok": False, "error": "no frame"}, status_code=503)
    jpg = streamer._left_jpeg(raw)
    if jpg is None:
        return StrictJSONResponse({"ok": False, "error": "decode failed"}, status_code=500)
    return Response(content=jpg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@router.get("/api/camera/status")
def camera_status():
    streamer = ctx.camera
    if streamer is None:
        return StrictJSONResponse({"ok": False, "active": False, "error": "camera streamer unavailable"})
    return StrictJSONResponse(streamer.status())
