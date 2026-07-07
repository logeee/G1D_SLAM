"""Process-wide handles to the ROS node + shared stores.

Mirrors the original server's handler binding (state / node_ref / point_store).
main.py fills these in during startup; routers read them.
"""
from __future__ import annotations

import json
from typing import Any, Dict, Optional

from starlette.requests import Request
from starlette.exceptions import HTTPException


class AppContext:
    def __init__(self) -> None:
        self.state = None
        self.node = None
        self.point_store = None


ctx = AppContext()


async def read_json_body(request: Request) -> Dict[str, Any]:
    """Replicates the legacy read_json_body(): empty -> {}, >64KiB -> 413,
    invalid json -> 400, non-object -> 400."""
    raw = await request.body()
    if not raw:
        return {}
    if len(raw) > 65536:
        raise HTTPException(status_code=413, detail="request body too large")
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise HTTPException(status_code=400, detail=f"invalid json: {exc}")
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="json body must be an object")
    return payload
