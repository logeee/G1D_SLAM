"""Named action-chain library: GET /api/workflows + save / delete."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/workflows")
async def get_workflows():
    return StrictJSONResponse(await run_in_threadpool(ctx.workflow_store.list_payload))


@router.post("/api/workflows/save")
async def save_workflow(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.workflow_store.save, payload))


@router.post("/api/workflows/delete")
async def delete_workflow(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.workflow_store.delete, payload))
