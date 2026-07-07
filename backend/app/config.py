"""CLI / settings for the FastAPI backend.

The argument list is ported verbatim from the original monolith's main(), so all
existing flags keep working. Only two changes vs. the legacy server:
  - default --port is 18090 (was 18083)
  - added --frontend-dist to locate the built Vue app (frontend/dist)
"""
from __future__ import annotations

import argparse
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FastAPI web dashboard for base laser/map/odom/basic sensors.")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18090)
    parser.add_argument("--scan-topic", default="/slamware_ros_sdk_server_node/scan")
    parser.add_argument("--map-topic", default="/slamware_ros_sdk_server_node/map")
    parser.add_argument("--odom-topic", action="append", default=["/slamware_ros_sdk_server_node/odom"])
    parser.add_argument("--sensors-topic", default="/slamware_ros_sdk_server_node/basic_sensors_values")
    parser.add_argument("--pointcloud-topic", action="append", default=["/ele_clouds"])
    parser.add_argument("--move-to-locations-topic", default="/slamware_ros_sdk_server_node/move_to_locations")
    parser.add_argument("--cancel-action-topic", default="/slamware_ros_sdk_server_node/cancel_action")
    parser.add_argument("--set-pose-topic", default="/slamware_ros_sdk_server_node/set_pose")
    parser.add_argument("--recover-localization-topic", default="/slamware_ros_sdk_server_node/recover_localization")
    parser.add_argument("--set-map-localization-topic", default="/slamware_ros_sdk_server_node/set_map_localization")
    parser.add_argument("--set-map-update-topic", default="/slamware_ros_sdk_server_node/set_map_update")
    parser.add_argument("--clear-map-topic", default="/slamware_ros_sdk_server_node/clear_map")
    parser.add_argument("--sync-get-stcm-service", default="/sync_get_stcm")
    parser.add_argument("--sync-set-stcm-service", default="/sync_set_stcm")
    parser.add_argument("--maps-dir", default="data/map")
    parser.add_argument("--sync-get-stcm-timeout-sec", type=float, default=30.0)
    parser.add_argument("--sync-set-stcm-timeout-sec", type=float, default=30.0)
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--global-plan-path-topic", default="/slamware_ros_sdk_server_node/global_plan_path")
    parser.add_argument("--robot-basic-state-topic", default="/slamware_ros_sdk_server_node/robot_basic_state")
    parser.add_argument("--slamware-state-topic", default="/slamware_ros_sdk_server_node/state")
    parser.add_argument("--arm-command-topic", default="/arm_control/task_command")
    parser.add_argument("--arm-status-topic", default="/arm_control/task_status")
    parser.add_argument("--arm-task-timeout-sec", type=float, default=120.0)
    parser.add_argument(
        "--arm-stop-phases",
        default="RESET,SUCTION_STOP,MOTION_STOP",
        help="Comma-separated arm phases published by the Stop button.",
    )
    parser.add_argument("--column-control-bin", default="/home/unitree/unitree_sdk2/build/bin/g1d_height_control")
    parser.add_argument("--column-control-workdir", default="/home/unitree/unitree_sdk2/build")
    parser.add_argument("--column-control-interface", default="eth0")
    parser.add_argument("--column-control-libdir", default="/home/unitree/unitree_sdk2/thirdparty/lib/aarch64")
    # Manual teleop jog of the base (mirrors the proven :18086 g1d_simple_control path).
    parser.add_argument("--base-control-bin", default="/home/unitree/unitree_sdk2/build/bin/g1d_simple_control")
    parser.add_argument("--jog-max-linear-mps", type=float, default=0.15)
    parser.add_argument("--jog-max-angular-radps", type=float, default=0.6)
    parser.add_argument("--jog-default-linear-mps", type=float, default=0.1)
    parser.add_argument("--jog-default-angular-radps", type=float, default=0.3)
    parser.add_argument("--jog-hold-duration-sec", type=float, default=3600.0)
    parser.add_argument("--jog-deadman-timeout-sec", type=float, default=1.0)
    # Left-eye head camera MJPEG stream (mapping-mode collection view).
    parser.add_argument("--head-camera-host", default="127.0.0.1")
    parser.add_argument("--head-camera-request-port", type=int, default=60000)
    parser.add_argument("--head-camera-zmq-port", type=int, default=0, help="0 = auto-discover via request port")
    parser.add_argument("--head-camera-jpeg-quality", type=int, default=75)
    parser.add_argument("--head-camera-max-fps", type=float, default=15.0)
    parser.add_argument("--head-camera-eye", default="left", choices=["left", "right"])
    parser.add_argument("--column-height-timeout-sec", type=float, default=30.0)
    parser.add_argument("--column-height-min-m", type=float, default=-0.053)
    parser.add_argument("--column-height-max-m", type=float, default=0.376)
    parser.add_argument("--lift-height-url", default="http://127.0.0.1:28089/api/basic_status")
    parser.add_argument("--lift-height-timeout-sec", type=float, default=1.0)
    parser.add_argument("--raw-nav-linear-speed-mps", type=float, default=0.12)
    parser.add_argument("--raw-nav-angular-speed-radps", type=float, default=0.45)
    parser.add_argument("--raw-nav-position-tolerance-m", type=float, default=0.08)
    parser.add_argument("--raw-nav-yaw-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--points-file", default="data/nav_points.json")
    parser.add_argument("--relocalization-anchor-file", default="data/relocalization_anchor.json")
    parser.add_argument("--last-pose-file", default="data/last_pose.json")
    parser.add_argument("--last-pose-save-interval-sec", type=float, default=10.0)
    parser.add_argument("--relocalization-search-radius-m", type=float, default=0.6)
    parser.add_argument("--relocalization-max-time-ms", type=int, default=8000)
    parser.add_argument("--relocalization-movement", default="NO_MOVE", choices=["NO_MOVE", "ROTATE_ONLY", "ANY"])
    parser.add_argument(
        "--min-localization-quality",
        type=int,
        default=-1,
        help="Block navigation when localization_quality is below this value. -1 means warn only.",
    )
    parser.add_argument("--max-cloud-points", type=int, default=6000)
    parser.add_argument("--max-track", type=int, default=1200)
    parser.add_argument("--fault-log-path", default="data/navigation_fault_snapshots.jsonl")
    parser.add_argument(
        "--frontend-dist",
        default=str(Path(__file__).resolve().parent.parent.parent / "frontend" / "dist"),
        help="Directory of the built Vue app (frontend/dist). Served at / when present.",
    )
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    return build_parser().parse_args(argv)
