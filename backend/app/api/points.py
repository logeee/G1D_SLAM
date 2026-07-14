"""Saved-point library: GET /api/points + record_current / upsert / delete."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/points")
async def get_points():
    return StrictJSONResponse(await run_in_threadpool(ctx.point_store.list_payload))


@router.post("/api/points/record_current")
async def record_current(request: Request):
    payload = await read_json_body(request)

    def _run():
        odom = ctx.state.snapshot().get("odom")
        return ctx.point_store.record_current(odom, payload)

    return StrictJSONResponse(await run_in_threadpool(_run))


@router.post("/api/points/upsert")
async def upsert(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.point_store.upsert, payload))


@router.post("/api/points/delete")
async def delete(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.point_store.delete, payload))
