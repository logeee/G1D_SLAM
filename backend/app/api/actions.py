"""Action execution: /api/actions/execute (+ stop_all) and lift height read.

execute_action() is ported verbatim from the legacy DashboardHandler.execute_action,
including the fake_pick_xiongmao simulation. All blocking work (arm task Condition
wait up to 600s, column subprocess up to 180s, fake sleep up to 30s) runs in the
threadpool so the event loop stays free.
"""
from __future__ import annotations

import time
from typing import Any, Dict

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse
from ..ros.helpers import finite_or_none, now_iso

router = APIRouter()


def execute_action(payload: Dict[str, Any]) -> Dict[str, Any]:
    action_type = str(payload.get("type") or payload.get("action") or "").strip()
    if action_type in ("column_height", "column_lift", "g1d_column_height"):
        return ctx.node.execute_column_height_action(payload)
    if action_type in ("arm_task", "arm_pick", "arm_place", "arm_reset"):
        arm_payload = dict(payload)
        if action_type == "arm_pick":
            arm_payload["phase"] = "PICK"
        elif action_type == "arm_place":
            arm_payload["phase"] = "PLACE"
        elif action_type == "arm_reset":
            arm_payload["phase"] = "RESET"
        return ctx.node.execute_arm_task(arm_payload)
    if action_type != "fake_pick_xiongmao":
        return {
            "ok": False,
            "error": "unsupported action type; expected arm_task, column_height, or fake_pick_xiongmao",
            "received": payload,
        }
    duration = finite_or_none(payload.get("duration_sec", 5), 3)
    if duration is None:
        duration = 5.0
    duration = max(0.0, min(30.0, float(duration)))
    started_at = now_iso()
    time.sleep(duration)
    return {
        "ok": True,
        "type": action_type,
        "name": payload.get("name") or "拾取熊猫烟",
        "started_at": started_at,
        "finished_at": now_iso(),
        "duration_sec": duration,
        "simulated": True,
        "message": "fake action finished; replace this endpoint with arm control later",
    }


@router.post("/api/actions/execute")
async def actions_execute(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(execute_action, payload))


@router.post("/api/actions/stop_all")
@router.post("/api/actions/stop")
async def actions_stop_all(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.stop_all_actions, payload))


@router.get("/api/lift_height")
@router.get("/api/column_height/current")
async def lift_height():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.read_lift_height_status))
