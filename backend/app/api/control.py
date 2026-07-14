"""Manual teleop control of the base (mapping-mode jog pad).

Endpoints:
  POST /api/control/jog    {action, speed}  -> start/refresh a held jog
  POST /api/control/stop                     -> stop any active jog
  GET  /api/control/status                   -> current jog state + limits

The frontend sends /jog repeatedly (~3 Hz) while a direction button is held and
/stop on release. A server-side dead-man watchdog stops the base automatically
if heartbeats stop arriving (see control/jog.py).
"""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.post("/api/control/jog")
async def control_jog(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.base_jog, payload))


@router.post("/api/control/stop")
async def control_stop():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.base_stop))


@router.get("/api/control/status")
async def control_status():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.base_jog_status))
