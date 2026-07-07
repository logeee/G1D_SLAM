"""Navigation start/cancel (with the legacy path aliases)."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.post("/api/navigation/start")
@router.post("/api/nav/start")
async def start(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.start_navigation, payload))


@router.post("/api/navigation/cancel")
@router.post("/api/nav/cancel")
@router.post("/api/navigation/stop")
@router.post("/api/nav/stop")
async def cancel(request: Request):
    return StrictJSONResponse(await run_in_threadpool(ctx.node.cancel_navigation))
