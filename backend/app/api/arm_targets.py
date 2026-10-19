"""Mechanical-arm pickup targets derived from the active YOLO model."""
from fastapi import APIRouter
from starlette.concurrency import run_in_threadpool

from ..context import ctx
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/arm_targets")
@router.get("/api/arm/targets")
async def arm_targets():
    return StrictJSONResponse(await run_in_threadpool(ctx.arm_targets.read))
