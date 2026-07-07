"""Relocalization: status (GET) + save_anchor / run (POST)."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/relocalization/status")
@router.get("/api/relocalization")
async def relocalization_status():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.relocalization_status))


@router.post("/api/relocalization/save_anchor")
async def save_anchor(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.save_relocalization_anchor, payload))


@router.post("/api/relocalization/run")
@router.post("/api/relocalization/start")
async def run_relocalization(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.run_relocalization, payload))
