"""FastAPI entrypoint for the base-sensor dashboard backend.

Threading model (kept identical in spirit to the legacy ThreadingHTTPServer):
  - rclpy spins in a dedicated daemon thread (spin_once loop).
  - Every request handler offloads its (possibly blocking) node/store call to the
    anyio threadpool via run_in_threadpool, so the asyncio event loop is never
    blocked by long arm/column/mapping operations.
"""
from __future__ import annotations

import threading
from contextlib import asynccontextmanager
from pathlib import Path

import anyio
import uvicorn
from fastapi import FastAPI
from starlette.responses import FileResponse, HTMLResponse
from starlette.staticfiles import StaticFiles

from .config import parse_args
from .context import ctx
from .json_response import StrictJSONResponse
from .api import state as state_api
from .api import points as points_api
from .api import navigation as navigation_api
from .api import actions as actions_api
from .api import mapping as mapping_api
from .api import relocalization as relocalization_api
from .api import faults as faults_api


_DEV_PLACEHOLDER_HTML = """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<title>Base Sensor Dashboard (backend)</title></head>
<body style="font:14px system-ui;margin:40px;color:#152235">
<h1>后端已启动</h1>
<p>没有找到 <code>frontend/dist/index.html</code>。</p>
<p>开发模式：请运行 <code>npm run dev</code>（Vite dev server），它会把 <code>/api</code> 代理到本后端。</p>
<p>生产模式：先在 <code>frontend/</code> 执行 <code>npm run build</code> 生成 dist，再由本后端托管。</p>
</body></html>"""


def _build_node(args):
    """Construct SharedState + stores + BaseSensorNode exactly like the legacy main()."""
    import rclpy

    from .ros.node import BaseSensorNode
    from .state.points import SavedPointStore
    from .state.relocalization import RelocalizationAnchorStore
    from .state.shared_state import SharedState

    state = SharedState(max_track=args.max_track)
    point_store = SavedPointStore(args.points_file)
    relocalization_store = RelocalizationAnchorStore(args.relocalization_anchor_file)
    rclpy.init()
    node = BaseSensorNode(
        state=state,
        scan_topic=args.scan_topic,
        map_topic=args.map_topic,
        odom_topics=args.odom_topic,
        sensors_topic=args.sensors_topic,
        pointcloud_topics=args.pointcloud_topic,
        move_to_locations_topic=args.move_to_locations_topic,
        cancel_action_topic=args.cancel_action_topic,
        set_pose_topic=args.set_pose_topic,
        recover_localization_topic=args.recover_localization_topic,
        set_map_localization_topic=args.set_map_localization_topic,
        set_map_update_topic=args.set_map_update_topic,
        clear_map_topic=args.clear_map_topic,
        sync_get_stcm_service=args.sync_get_stcm_service,
        sync_set_stcm_service=args.sync_set_stcm_service,
        maps_dir=args.maps_dir,
        sync_get_stcm_timeout_sec=args.sync_get_stcm_timeout_sec,
        sync_set_stcm_timeout_sec=args.sync_set_stcm_timeout_sec,
        cmd_vel_topic=args.cmd_vel_topic,
        global_plan_path_topic=args.global_plan_path_topic,
        robot_basic_state_topic=args.robot_basic_state_topic,
        slamware_state_topic=args.slamware_state_topic,
        arm_command_topic=args.arm_command_topic,
        arm_status_topic=args.arm_status_topic,
        arm_task_timeout_sec=args.arm_task_timeout_sec,
        arm_stop_phases=args.arm_stop_phases.split(","),
        column_control_bin=args.column_control_bin,
        column_control_workdir=args.column_control_workdir,
        column_control_interface=args.column_control_interface,
        column_control_libdir=args.column_control_libdir,
        column_height_timeout_sec=args.column_height_timeout_sec,
        column_height_min_m=args.column_height_min_m,
        column_height_max_m=args.column_height_max_m,
        lift_height_url=args.lift_height_url,
        lift_height_timeout_sec=args.lift_height_timeout_sec,
        raw_nav_linear_speed_mps=args.raw_nav_linear_speed_mps,
        raw_nav_angular_speed_radps=args.raw_nav_angular_speed_radps,
        raw_nav_position_tolerance_m=args.raw_nav_position_tolerance_m,
        raw_nav_yaw_tolerance_deg=args.raw_nav_yaw_tolerance_deg,
        min_localization_quality=args.min_localization_quality,
        relocalization_store=relocalization_store,
        relocalization_search_radius_m=args.relocalization_search_radius_m,
        relocalization_max_time_ms=args.relocalization_max_time_ms,
        relocalization_movement=args.relocalization_movement,
        max_cloud_points=args.max_cloud_points,
        fault_log_path=args.fault_log_path,
    )
    return state, point_store, node


def create_app(args) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        import rclpy

        state, point_store, node = _build_node(args)
        ctx.state = state
        ctx.node = node
        ctx.point_store = point_store

        stop_event = threading.Event()

        def ros_spin() -> None:
            while rclpy.ok() and not stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)

        ros_thread = threading.Thread(target=ros_spin, name="ros_spin", daemon=True)
        ros_thread.start()

        # Long arm tasks can hold a worker thread for minutes; give the pool room
        # so high-frequency /api/state polling never starves behind them.
        try:
            anyio.to_thread.current_default_thread_limiter().total_tokens = 256
        except Exception:  # noqa: BLE001
            pass

        print(f"base sensor dashboard (fastapi) ready on http://{args.bind}:{args.port}", flush=True)
        try:
            yield
        finally:
            stop_event.set()
            ros_thread.join(timeout=2.0)
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            if rclpy.ok():
                rclpy.shutdown()

    app = FastAPI(title="Base Sensor Dashboard", lifespan=lifespan, default_response_class=StrictJSONResponse)

    for module in (
        state_api,
        points_api,
        navigation_api,
        actions_api,
        mapping_api,
        relocalization_api,
        faults_api,
    ):
        app.include_router(module.router)

    dist_dir = Path(args.frontend_dist)
    index_file = dist_dir / "index.html"

    @app.get("/", include_in_schema=False)
    @app.get("/dashboard", include_in_schema=False)
    async def index():
        if index_file.exists():
            return FileResponse(str(index_file), headers={"Cache-Control": "no-store"})
        return HTMLResponse(_DEV_PLACEHOLDER_HTML)

    if dist_dir.exists():
        # Registered last so /api/* and / take precedence; serves built assets.
        app.mount("/", StaticFiles(directory=str(dist_dir), html=True), name="static")

    return app


def main() -> int:
    args = parse_args()
    app = create_app(args)
    uvicorn.run(app, host=args.bind, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
