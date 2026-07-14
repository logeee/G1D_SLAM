#!/usr/bin/env python3
"""Optional ROS1 shim for x_nav topic compatibility.

Run this only when the work platform talks to ROS1 topics directly. It keeps
the ROS1 names that x_nav documents expose, then calls the HTTP compatibility
adapter on port 9000.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HttpClient:
    def __init__(self, base_url: str, timeout_sec: float) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = float(timeout_sec)

    def get(self, path: str, query: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        url = self.base_url + path
        if query:
            url += "?" + urlencode(query)
        return self._request("GET", url, None)

    def post(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._request("POST", self.base_url + path, payload)

    def _request(self, method: str, url: str, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = Request(url, data=data, headers=headers, method=method)
        try:
            with urlopen(req, timeout=self.timeout_sec) as response:
                raw = response.read().decode("utf-8", errors="replace")
                return json.loads(raw) if raw else {"success": True, "ok": True}
        except HTTPError as exc:
            return {"success": False, "ok": False, "error": "HTTP %s" % exc.code, "body": exc.read().decode("utf-8", errors="replace")}
        except (URLError, TimeoutError, ValueError) as exc:
            return {"success": False, "ok": False, "error": str(exc)}


class XNavRos1CompatNode:
    def __init__(self) -> None:
        self.client = HttpClient(
            rospy.get_param("~compat_base_url", "http://127.0.0.1:9000"),
            rospy.get_param("~http_timeout_sec", 3.0),
        )
        self.last_nav_state = "-1"
        self.last_slam_state = "3"
        self.last_master_state = "0"
        self.state_pub = rospy.Publisher("/x_nav/state", String, queue_size=10, latch=True)
        self.slam_state_pub = rospy.Publisher("/x_nav/slam/state", String, queue_size=10, latch=True)
        self.master_state_pub = rospy.Publisher("/x_nav/master/state", String, queue_size=10, latch=True)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal, queue_size=10)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.on_initialpose, queue_size=10)
        rospy.Subscriber("/planner/cmd", String, self.on_planner_cmd, queue_size=10)
        rospy.Subscriber("/node_cmd", String, self.on_node_cmd, queue_size=10)
        rospy.Subscriber(rospy.get_param("~path_topic", "/topological_path"), Path, self.on_path, queue_size=10)

        rate_hz = float(rospy.get_param("~state_rate_hz", 5.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(0.2, rate_hz)), self.publish_state_from_http)
        rospy.loginfo("x_nav ROS1 compatibility node started; compat_base_url=%s", self.client.base_url)

    def pose_payload(self, pose: Any) -> Dict[str, Any]:
        q = pose.orientation
        return {
            "positionX": pose.position.x,
            "positionY": pose.position.y,
            "positionZ": pose.position.z,
            "orientationX": q.x,
            "orientationY": q.y,
            "orientationZ": q.z,
            "orientationW": q.w,
        }

    def on_goal(self, msg: PoseStamped) -> None:
        payload = self.pose_payload(msg.pose)
        payload["inputValue"] = "ros1_goal"
        result = self.client.post("/api/extra/nav_custom", payload)
        self.apply_result_state(result, success_state="14", failure_state="15")
        rospy.loginfo("forwarded /move_base_simple/goal result=%s", self.short(result))

    def on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        result = self.client.post("/api/extra/set_pose", self.pose_payload(msg.pose.pose))
        self.apply_result_state(result, success_state="-1", failure_state="3")
        rospy.loginfo("forwarded /initialpose result=%s", self.short(result))

    def on_path(self, msg: Path) -> None:
        if not msg.poses:
            rospy.logwarn("received empty path on /topological_path")
            return
        waypoints: List[Dict[str, float]] = []
        for stamped in msg.poses:
            waypoints.append({"x": stamped.pose.position.x, "y": stamped.pose.position.y})
        final_pose = msg.poses[-1].pose
        q = final_pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        seq = int(msg.header.seq or 0)
        payload = {
            "waypoints": waypoints,
            "yaw": yaw,
            "yaw_source": "ros1_path",
            "direct_no_avoidance": seq in (0, 1),
            "path_mode_seq": seq,
        }
        result = self.client.post("/api/compat/navigation/start", payload)
        self.apply_result_state(result, success_state="14", failure_state="15")
        rospy.loginfo("forwarded path with %d poses result=%s", len(waypoints), self.short(result))

    def on_planner_cmd(self, msg: String) -> None:
        cmd = (msg.data or "").strip()
        if cmd == "cancel_nav":
            result = self.client.get("/api/extra/nav_work/cancel", {"stop": "1"})
            self.apply_result_state(result, success_state="-1", failure_state=self.last_nav_state)
        elif cmd in (
            "replan_goal",
            "disable_localplanner",
            "enable_localplanner",
            "enable_replan",
            "disable_replan",
            "enable_forward",
            "enable_backward",
            "enable_stop_obstacle",
            "disable_stop_obstacle",
        ):
            result = {"success": True, "status": "compat_noop", "msg": "planner cmd accepted as no-op", "cmd": cmd}
        else:
            result = {"success": False, "status": "unsupported_planner_cmd", "msg": "unknown planner cmd", "cmd": cmd}
        rospy.loginfo("handled /planner/cmd=%s result=%s", cmd, self.short(result))

    def on_node_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        action, _, arg = raw.partition("#")
        if action == "launch_mapping":
            result = self.client.post("/api/extra/add_map", {"map_name": arg})
            self.apply_result_state(result, success_state="-1", failure_state=self.last_nav_state)
        elif action == "save_map":
            result = self.client.post("/api/extra/save_map", {"map_name": arg})
            self.apply_result_state(result, success_state="-4#%s" % arg, failure_state=self.last_nav_state)
        elif action == "launch_navigation":
            result = self.client.post("/api/extra/switch_map", {"map_name": arg, "package": arg})
            self.apply_result_state(result, success_state="-2", failure_state=self.last_nav_state)
        elif action in ("exit_mapping", "exit_navigation"):
            result = self.client.get("/api/extra/nav_work/cancel", {"stop": "1"})
            self.apply_result_state(result, success_state="-1", failure_state=self.last_nav_state)
        elif action in ("launch", "exit"):
            result = {"success": True, "status": "compat_noop", "msg": "node lifecycle cmd accepted as no-op", "cmd": raw}
        else:
            result = {"success": False, "status": "unsupported_node_cmd", "msg": "unknown node cmd", "cmd": raw}
        rospy.loginfo("handled /node_cmd=%s result=%s", raw, self.short(result))

    def apply_result_state(self, result: Dict[str, Any], success_state: str, failure_state: str) -> None:
        if result.get("success") or result.get("ok"):
            self.last_nav_state = success_state
        else:
            self.last_nav_state = failure_state
        self.publish_cached_state()

    def publish_state_from_http(self, _event: Any) -> None:
        result = self.client.get("/api/compat/x_nav_state")
        if result.get("success"):
            self.last_nav_state = str(result.get("nav_state") or self.last_nav_state)
            self.last_slam_state = str(result.get("slam_state") or self.last_slam_state)
            self.last_master_state = str(result.get("master_state") or self.last_master_state)
        self.publish_cached_state()

    def publish_cached_state(self) -> None:
        self.state_pub.publish(String(data=self.last_nav_state))
        self.slam_state_pub.publish(String(data=self.last_slam_state))
        self.master_state_pub.publish(String(data=self.last_master_state))

    @staticmethod
    def short(value: Dict[str, Any]) -> str:
        text = json.dumps(value, ensure_ascii=False, sort_keys=True)
        return text[:500] + ("..." if len(text) > 500 else "")


def main() -> None:
    rospy.init_node("x_nav_ros1_compat_node")
    XNavRos1CompatNode()
    rospy.spin()


if __name__ == "__main__":
    main()
