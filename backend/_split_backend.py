#!/usr/bin/env python3
"""One-off codegen: carve the ROS/business logic out of the original monolith
(scripts/base_sensor_visual_server.py) into the backend/app package, VERBATIM.

Only the non-HTML, non-HTTP-handler, non-main portions are extracted:
  - constants + helper functions        -> app/ros/helpers.py
  - class SharedState                    -> app/state/shared_state.py
  - class SavedPointStore                -> app/state/points.py
  - class RelocalizationAnchorStore      -> app/state/relocalization.py
  - class FaultSnapshotLogger            -> app/state/faults.py
  - class BaseSensorNode                 -> app/ros/node.py

The FastAPI layer (json_response / config / api routers / main) is hand-written
separately; it is intentionally NOT generated here.
"""
from __future__ import annotations

from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "scripts" / "base_sensor_visual_server.py"
APP = Path(__file__).resolve().parent / "app"


def find_line(lines, needle, start=0):
    for i in range(start, len(lines)):
        if lines[i].startswith(needle):
            return i
    raise SystemExit(f"marker not found: {needle!r}")


def slice_body(lines, start_idx, end_idx):
    """Return joined text for lines[start_idx:end_idx] with trailing blanks trimmed."""
    seg = lines[start_idx:end_idx]
    while seg and seg[-1].strip() == "":
        seg.pop()
    return "\n".join(seg) + "\n"


HELPERS_HEADER = '''# Auto-extracted from scripts/base_sensor_visual_server.py (verbatim).
# Constants + pure helpers + QoS factory. No ROS message imports here so the
# state/* modules can depend on it without pulling in the ROS message types.
from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


'''

STORE_HEADER = '''# Auto-extracted from scripts/base_sensor_visual_server.py (verbatim).
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path as FsPath
from typing import Any, Dict, Iterable, List, Optional, Tuple
import urllib.error
import urllib.request

from ..ros.helpers import (
    SENSOR_TYPE_NAMES,
    IMPACT_TYPE_NAMES,
    SLAMWARE_MOVE_OPTION_WITH_YAW,
    SLAMWARE_MOVE_OPTION_KEY_POINTS,
    finite_or_none,
    yaw_from_quaternion,
    quaternion_from_yaw,
    normalize_angle_rad,
    now_iso,
    make_reliable_qos,
)


'''

NODE_HEADER = '''# Auto-extracted from scripts/base_sensor_visual_server.py (verbatim).
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path as FsPath
from typing import Any, Dict, Iterable, List, Optional, Tuple
import urllib.error
import urllib.request

from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from slamware_ros_sdk.msg import (
    BasicSensorValueDataArray,
    CancelActionRequest,
    ClearMapRequest,
    LocalizationMovement,
    MapKind,
    MoveToLocationsRequest,
    RecoverLocalizationRequest,
    RobotBasicState,
    SetMapLocalizationRequest,
    SetMapUpdateRequest,
    SyncMapRequest,
)
from slamware_ros_sdk.srv import SyncGetStcm, SyncSetStcm
from std_msgs.msg import String

from ..ros.helpers import (
    SENSOR_TYPE_NAMES,
    IMPACT_TYPE_NAMES,
    SLAMWARE_MOVE_OPTION_WITH_YAW,
    SLAMWARE_MOVE_OPTION_KEY_POINTS,
    finite_or_none,
    yaw_from_quaternion,
    quaternion_from_yaw,
    normalize_angle_rad,
    now_iso,
    make_reliable_qos,
)
from ..state.shared_state import SharedState
from ..state.relocalization import RelocalizationAnchorStore
from ..state.faults import FaultSnapshotLogger


'''


def main():
    lines = SRC.read_text(encoding="utf-8").split("\n")

    i_const = find_line(lines, "SENSOR_TYPE_NAMES = {")
    i_shared = find_line(lines, "class SharedState:")
    i_points = find_line(lines, "class SavedPointStore:")
    i_reloc = find_line(lines, "class RelocalizationAnchorStore:")
    i_fault = find_line(lines, "class FaultSnapshotLogger:")
    i_node = find_line(lines, "class BaseSensorNode(Node):")
    i_html = find_line(lines, 'HTML = r"""')

    helpers_body = slice_body(lines, i_const, i_shared)
    shared_body = slice_body(lines, i_shared, i_points)
    points_body = slice_body(lines, i_points, i_reloc)
    reloc_body = slice_body(lines, i_reloc, i_fault)
    fault_body = slice_body(lines, i_fault, i_node)
    node_body = slice_body(lines, i_node, i_html)

    (APP / "ros").mkdir(parents=True, exist_ok=True)
    (APP / "state").mkdir(parents=True, exist_ok=True)
    (APP / "api").mkdir(parents=True, exist_ok=True)

    (APP / "ros" / "helpers.py").write_text(HELPERS_HEADER + helpers_body, encoding="utf-8")
    (APP / "state" / "shared_state.py").write_text(STORE_HEADER + shared_body, encoding="utf-8")
    (APP / "state" / "points.py").write_text(STORE_HEADER + points_body, encoding="utf-8")
    (APP / "state" / "relocalization.py").write_text(STORE_HEADER + reloc_body, encoding="utf-8")
    (APP / "state" / "faults.py").write_text(STORE_HEADER + fault_body, encoding="utf-8")
    (APP / "ros" / "node.py").write_text(NODE_HEADER + node_body, encoding="utf-8")

    for pkg in ("", "ros", "state", "api"):
        initf = (APP / pkg / "__init__.py") if pkg else (APP / "__init__.py")
        if not initf.exists():
            initf.write_text("", encoding="utf-8")

    print("extracted:")
    for rel in ("ros/helpers.py", "state/shared_state.py", "state/points.py",
                "state/relocalization.py", "state/faults.py", "ros/node.py"):
        p = APP / rel
        print(f"  {rel}: {len(p.read_text(encoding='utf-8').splitlines())} lines")


if __name__ == "__main__":
    main()
