"""Mapping mode: status/list (GET) + start/stop/save/load/list (POST)."""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


def _list_payload():
    return {"ok": True, "maps_dir": str(ctx.node.maps_dir), "saved_maps": ctx.node.list_saved_maps()}


@router.get("/api/mapping/status")
async def mapping_status():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.mapping_status))


@router.get("/api/mapping/list")
@router.get("/api/mapping/files")
async def mapping_list_get():
    return StrictJSONResponse(await run_in_threadpool(_list_payload))


@router.post("/api/mapping/list")
@router.post("/api/mapping/files")
async def mapping_list_post():
    return StrictJSONResponse(await run_in_threadpool(_list_payload))


@router.post("/api/mapping/start")
@router.post("/api/mapping/start_collection")
async def mapping_start(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.start_mapping, payload))


@router.post("/api/mapping/stop")
@router.post("/api/mapping/stop_collection")
async def mapping_stop(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.stop_mapping, payload))


@router.post("/api/mapping/save")
@router.post("/api/mapping/save_map")
async def mapping_save(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.save_map, payload))


@router.post("/api/mapping/load")
@router.post("/api/mapping/load_map")
async def mapping_load(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.load_map, payload))
