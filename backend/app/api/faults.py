"""Navigation fault snapshots: list (GET) + log / clear (POST)."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/fault_snapshots")
async def list_snapshots():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.fault_logger.list_payload))


@router.post("/api/fault_snapshots/log")
async def log_snapshot(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.fault_logger.capture_manual, payload))


@router.post("/api/fault_snapshots/clear")
async def clear_snapshots():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.fault_logger.clear))
