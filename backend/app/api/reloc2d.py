"""2D 重定位(LaserScan ↔ 占据栅格):三种方式 + 定时位姿读取。"""
from __future__ import annotations

from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool

from ..context import ctx, read_json_body
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/reloc2d/last_pose")
async def reloc2d_last_pose():
    pose = await run_in_threadpool(ctx.node.read_last_pose)
    return StrictJSONResponse({"ok": bool(pose), "last_pose": pose})


@router.get("/api/reloc2d/config")
async def reloc2d_get_config():
    return StrictJSONResponse(await run_in_threadpool(ctx.node.get_last_pose_config))


@router.post("/api/reloc2d/config")
async def reloc2d_set_config(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.set_last_pose_interval, payload))


@router.post("/api/reloc2d/run")
async def reloc2d_run(request: Request):
    payload = await read_json_body(request)
    return StrictJSONResponse(await run_in_threadpool(ctx.node.relocalize_2d, payload))
