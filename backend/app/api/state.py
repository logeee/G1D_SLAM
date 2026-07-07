"""GET /api/state, /api/health — read-only telemetry snapshots."""
from __future__ import annotations

from fastapi import APIRouter
from starlette.concurrency import run_in_threadpool

from ..context import ctx
from ..json_response import StrictJSONResponse

router = APIRouter()


@router.get("/api/state")
async def get_state():
    snap = await run_in_threadpool(ctx.state.snapshot)
    return StrictJSONResponse(snap)


@router.get("/api/health")
async def get_health():
    snap = await run_in_threadpool(ctx.state.snapshot)
    return StrictJSONResponse(
        {
            "ok": True,
            "uptime_s": snap["uptime_s"],
            "freshness_s": snap["freshness_s"],
            "seq": snap["seq"],
            "has_scan": snap["scan"] is not None,
            "has_map": snap["map"] is not None,
            "has_odom": snap["odom"] is not None,
            "has_sensors": snap["sensors"] is not None,
            "has_point_cloud": snap["point_cloud"] is not None,
            "has_global_plan_path": snap["navigation"]["global_plan_path"] is not None,
            "has_robot_basic_state": snap["navigation"]["robot_basic_state"] is not None,
            "has_arm_task_status": snap["arm_control"]["last_status"] is not None,
            "fault_snapshot_count": len(snap.get("fault_snapshots", [])),
        }
    )
