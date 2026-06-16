#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight web dashboard for Unitree base sensors.

The server subscribes to existing ROS2 topics and exposes a read-only browser
view.  It is intentionally dependency-light: rclpy + Python stdlib only.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import struct
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Iterable, List, Optional
from urllib.parse import urlparse

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from slamware_ros_sdk.msg import BasicSensorValueDataArray


SENSOR_TYPE_NAMES = {
    -1: "UNKNOWN",
    0: "BUMPER",
    1: "CLIFF",
    2: "SONAR",
    3: "DEPTH_CAMERA",
    4: "WALL_SENSOR",
    5: "MAG_TAPE_DETECTOR",
}

IMPACT_TYPE_NAMES = {
    -1: "UNKNOWN",
    0: "DIGITAL",
    1: "ANALOG",
}


def finite_or_none(value: Any, digits: Optional[int] = None) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return round(f, digits) if digits is not None else f


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class SharedState:
    def __init__(self, max_track: int) -> None:
        self.lock = threading.RLock()
        self.started_at = time.time()
        self.scan: Optional[Dict[str, Any]] = None
        self.map: Optional[Dict[str, Any]] = None
        self.odom: Optional[Dict[str, Any]] = None
        self.sensors: Optional[Dict[str, Any]] = None
        self.point_cloud: Optional[Dict[str, Any]] = None
        self.track: deque[Dict[str, float]] = deque(maxlen=max_track)
        self.seq = {"scan": 0, "map": 0, "odom": 0, "sensors": 0, "point_cloud": 0}

    def snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self.lock:
            return {
                "ok": True,
                "server_time": now,
                "uptime_s": round(now - self.started_at, 3),
                "seq": dict(self.seq),
                "scan": self.scan,
                "map": self.map,
                "odom": self.odom,
                "track": list(self.track),
                "sensors": self.sensors,
                "point_cloud": self.point_cloud,
                "freshness_s": {
                    "scan": self._age(self.scan, now),
                    "map": self._age(self.map, now),
                    "odom": self._age(self.odom, now),
                    "sensors": self._age(self.sensors, now),
                    "point_cloud": self._age(self.point_cloud, now),
                },
            }

    @staticmethod
    def _age(item: Optional[Dict[str, Any]], now: float) -> Optional[float]:
        if not item:
            return None
        stamp = item.get("received_at")
        if stamp is None:
            return None
        return round(now - float(stamp), 3)


class BaseSensorNode(Node):
    def __init__(
        self,
        state: SharedState,
        scan_topic: str,
        map_topic: str,
        odom_topics: Iterable[str],
        sensors_topic: str,
        pointcloud_topics: Iterable[str],
        max_cloud_points: int,
    ) -> None:
        super().__init__("base_sensor_visual_server")
        self.state = state
        self.max_cloud_points = max(10, int(max_cloud_points))
        qos = make_reliable_qos(depth=10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, qos)
        for topic in odom_topics:
            if topic:
                self.create_subscription(Odometry, topic, self.make_odom_cb(topic), qos)
        self.create_subscription(BasicSensorValueDataArray, sensors_topic, self.on_sensors, qos)
        pointcloud_topics = [topic for topic in pointcloud_topics if topic]
        for topic in pointcloud_topics:
            self.create_subscription(PointCloud2, topic, self.make_point_cloud_cb(topic), qos)
        self.get_logger().info(
            f"subscribed scan={scan_topic} map={map_topic} sensors={sensors_topic} "
            f"pointclouds={pointcloud_topics}"
        )

    def on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        ranges: List[Optional[float]] = []
        valid_ranges: List[float] = []
        for raw in msg.ranges:
            value = finite_or_none(raw, 4)
            if value is None or value < msg.range_min or value > msg.range_max:
                ranges.append(None)
            else:
                ranges.append(value)
                valid_ranges.append(value)

        payload = {
            "received_at": now,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "angle_min": finite_or_none(msg.angle_min, 6),
            "angle_max": finite_or_none(msg.angle_max, 6),
            "angle_increment": finite_or_none(msg.angle_increment, 8),
            "range_min": finite_or_none(msg.range_min, 4),
            "range_max": finite_or_none(msg.range_max, 4),
            "count": len(ranges),
            "valid_count": len(valid_ranges),
            "min_range": round(min(valid_ranges), 4) if valid_ranges else None,
            "ranges": ranges,
        }
        with self.state.lock:
            self.state.seq["scan"] += 1
            payload["seq"] = self.state.seq["scan"]
            self.state.scan = payload

    def on_map(self, msg: OccupancyGrid) -> None:
        now = time.time()
        info = msg.info
        payload = {
            "received_at": now,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "resolution": finite_or_none(info.resolution, 6),
            "width": int(info.width),
            "height": int(info.height),
            "origin": {
                "x": finite_or_none(info.origin.position.x, 6),
                "y": finite_or_none(info.origin.position.y, 6),
                "z": finite_or_none(info.origin.position.z, 6),
                "yaw": finite_or_none(
                    yaw_from_quaternion(
                        info.origin.orientation.x,
                        info.origin.orientation.y,
                        info.origin.orientation.z,
                        info.origin.orientation.w,
                    ),
                    6,
                ),
            },
            "data": list(msg.data),
        }
        with self.state.lock:
            self.state.seq["map"] += 1
            payload["seq"] = self.state.seq["map"]
            self.state.map = payload

    def make_odom_cb(self, topic: str):
        def on_odom(msg: Odometry) -> None:
            now = time.time()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            payload = {
                "received_at": now,
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
                "x": finite_or_none(p.x, 5),
                "y": finite_or_none(p.y, 5),
                "z": finite_or_none(p.z, 5),
                "yaw": finite_or_none(yaw, 6),
                "yaw_deg": finite_or_none(math.degrees(yaw), 3),
            }
            track_point = {
                "x": float(p.x),
                "y": float(p.y),
                "yaw": float(yaw),
                "t": now,
            }
            with self.state.lock:
                self.state.seq["odom"] += 1
                payload["seq"] = self.state.seq["odom"]
                self.state.odom = payload
                if math.isfinite(track_point["x"]) and math.isfinite(track_point["y"]):
                    self.state.track.append(track_point)

        return on_odom

    def on_sensors(self, msg: BasicSensorValueDataArray) -> None:
        now = time.time()
        sensors = []
        for item in msg.values_data:
            info = item.info
            pose = info.install_pose
            stype = int(info.sensor_type.type)
            itype = int(info.impact_type.type)
            raw_value = float(item.value.value)
            value_is_finite = math.isfinite(raw_value) and abs(raw_value) < 1.0e30
            sensors.append(
                {
                    "id": int(info.id),
                    "sensor_type": stype,
                    "sensor_type_name": SENSOR_TYPE_NAMES.get(stype, str(stype)),
                    "impact_type": itype,
                    "impact_type_name": IMPACT_TYPE_NAMES.get(itype, str(itype)),
                    "refresh_freq": finite_or_none(info.refresh_freq, 3),
                    "pose": {
                        "x": finite_or_none(pose.position.x, 5),
                        "y": finite_or_none(pose.position.y, 5),
                        "z": finite_or_none(pose.position.z, 5),
                    },
                    "is_in_impact": bool(item.value.is_in_impact),
                    "value": round(raw_value, 5) if value_is_finite else None,
                    "raw_value_is_finite": value_is_finite,
                }
            )

        payload = {"received_at": now, "count": len(sensors), "items": sensors}
        with self.state.lock:
            self.state.seq["sensors"] += 1
            payload["seq"] = self.state.seq["sensors"]
            self.state.sensors = payload

    def make_point_cloud_cb(self, topic: str):
        def on_point_cloud(msg: PointCloud2) -> None:
            payload = self.point_cloud_to_payload(topic, msg)
            with self.state.lock:
                self.state.seq["point_cloud"] += 1
                payload["seq"] = self.state.seq["point_cloud"]
                self.state.point_cloud = payload

        return on_point_cloud

    def point_cloud_to_payload(self, topic: str, msg: PointCloud2) -> Dict[str, Any]:
        now = time.time()
        fields = {field.name: field for field in msg.fields}
        points: List[List[float]] = []
        bounds = {
            "min_x": None,
            "max_x": None,
            "min_y": None,
            "max_y": None,
            "min_z": None,
            "max_z": None,
        }
        total_points = int(msg.width) * int(msg.height)
        xyz_ok = all(name in fields and int(fields[name].datatype) == 7 for name in ("x", "y", "z"))

        if xyz_ok and total_points > 0 and msg.point_step > 0:
            endian = ">" if msg.is_bigendian else "<"
            data = bytes(msg.data)
            x_off = int(fields["x"].offset)
            y_off = int(fields["y"].offset)
            z_off = int(fields["z"].offset)
            step = max(1, math.ceil(total_points / self.max_cloud_points))
            for idx in range(0, total_points, step):
                row = idx // int(msg.width) if msg.width else 0
                col = idx % int(msg.width) if msg.width else 0
                base = row * int(msg.row_step) + col * int(msg.point_step)
                try:
                    x = struct.unpack_from(endian + "f", data, base + x_off)[0]
                    y = struct.unpack_from(endian + "f", data, base + y_off)[0]
                    z = struct.unpack_from(endian + "f", data, base + z_off)[0]
                except (struct.error, ValueError):
                    continue
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                points.append([round(float(x), 4), round(float(y), 4), round(float(z), 4)])
                for axis, value in (("x", x), ("y", y), ("z", z)):
                    lo = "min_" + axis
                    hi = "max_" + axis
                    bounds[lo] = value if bounds[lo] is None else min(bounds[lo], value)
                    bounds[hi] = value if bounds[hi] is None else max(bounds[hi], value)

        return {
            "received_at": now,
            "topic": topic,
            "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "width": int(msg.width),
            "height": int(msg.height),
            "total_points": total_points,
            "sampled_points": len(points),
            "fields": [field.name for field in msg.fields],
            "point_step": int(msg.point_step),
            "row_step": int(msg.row_step),
            "is_dense": bool(msg.is_dense),
            "xyz_supported": bool(xyz_ok),
            "bounds": {key: finite_or_none(value, 4) for key, value in bounds.items()},
            "points": points,
        }


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Base Sensor Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fa;
      --panel: #ffffff;
      --line: #d8dee8;
      --text: #152235;
      --muted: #66758a;
      --ok: #16a34a;
      --warn: #dc2626;
      --blue: #2563eb;
      --amber: #d97706;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 18px;
      border-bottom: 1px solid var(--line);
      background: #fff;
      position: sticky;
      top: 0;
      z-index: 5;
    }
    h1 { margin: 0; font-size: 20px; letter-spacing: 0; }
    .status {
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      white-space: nowrap;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--warn); }
    .dot.ok { background: var(--ok); }
    main {
      display: grid;
      grid-template-columns: minmax(360px, 1.2fr) minmax(340px, 1fr);
      gap: 14px;
      padding: 14px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
      overflow: hidden;
    }
    .panel-head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }
    h2 { margin: 0; font-size: 15px; }
    .meta { color: var(--muted); font-size: 12px; }
    .canvas-wrap { position: relative; padding: 10px; }
    canvas {
      width: 100%;
      display: block;
      border: 1px solid #e4e9f1;
      background: #fff;
      border-radius: 6px;
      aspect-ratio: 1 / 1;
    }
    #mapCanvas { aspect-ratio: 1.45 / 1; }
    #cloudCanvas { aspect-ratio: 1.45 / 1; cursor: grab; }
    #cloudCanvas:active { cursor: grabbing; }
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      padding: 10px;
    }
    .sensor-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #fbfcfe;
    }
    .sensor-card.hit { border-color: #fecaca; background: #fff7f7; }
    .sensor-title {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      font-weight: 700;
      margin-bottom: 6px;
    }
    .kv {
      display: grid;
      grid-template-columns: auto 1fr;
      gap: 4px 8px;
      color: var(--muted);
      font-size: 12px;
    }
    .kv strong { color: var(--text); font-weight: 600; text-align: right; }
    .readout {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      padding: 10px;
      border-top: 1px solid var(--line);
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      background: #fbfcfe;
      min-height: 58px;
    }
    .metric .label { color: var(--muted); font-size: 12px; }
    .metric .value { font-size: 18px; font-weight: 750; margin-top: 4px; overflow-wrap: anywhere; }
    pre {
      margin: 0;
      padding: 10px;
      max-height: 180px;
      overflow: auto;
      background: #102033;
      color: #d8e6f7;
      font-size: 12px;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .readout { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <h1>Base Sensor Dashboard</h1>
    <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">connecting</span></div>
  </header>
  <main>
    <section>
      <div class="panel-head">
        <h2>SLAM Map + Odometry</h2>
        <span id="mapMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="mapCanvas"></canvas></div>
      <div class="readout">
        <div class="metric"><div class="label">X</div><div id="odomX" class="value">--</div></div>
        <div class="metric"><div class="label">Y</div><div id="odomY" class="value">--</div></div>
        <div class="metric"><div class="label">Yaw</div><div id="odomYaw" class="value">--</div></div>
        <div class="metric"><div class="label">Track</div><div id="trackCount" class="value">--</div></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Laser Scan</h2>
        <span id="scanMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="scanCanvas"></canvas></div>
      <div class="readout">
        <div class="metric"><div class="label">Valid</div><div id="scanValid" class="value">--</div></div>
        <div class="metric"><div class="label">Min Range</div><div id="scanMin" class="value">--</div></div>
        <div class="metric"><div class="label">Frame</div><div id="scanFrame" class="value">--</div></div>
        <div class="metric"><div class="label">Age</div><div id="scanAge" class="value">--</div></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Ultrasonic / Bumper Sensors</h2>
        <span id="sensorMeta" class="meta">waiting</span>
      </div>
      <div id="sensorGrid" class="sensor-grid"></div>
    </section>

    <section>
      <div class="panel-head">
        <h2>3D Point Cloud</h2>
        <span id="cloudMeta" class="meta">waiting</span>
      </div>
      <div class="canvas-wrap"><canvas id="cloudCanvas"></canvas></div>
      <div class="readout">
        <div class="metric"><div class="label">Topic</div><div id="cloudTopic" class="value">--</div></div>
        <div class="metric"><div class="label">Points</div><div id="cloudPoints" class="value">--</div></div>
        <div class="metric"><div class="label">Frame</div><div id="cloudFrame" class="value">--</div></div>
        <div class="metric"><div class="label">Age</div><div id="cloudAge" class="value">--</div></div>
      </div>
    </section>

    <section>
      <div class="panel-head">
        <h2>Raw State</h2>
        <span class="meta">/api/state</span>
      </div>
      <pre id="rawState">{}</pre>
    </section>
  </main>

  <script>
    const mapCanvas = document.getElementById('mapCanvas');
    const scanCanvas = document.getElementById('scanCanvas');
    const cloudCanvas = document.getElementById('cloudCanvas');
    let cachedMapSeq = -1;
    let cachedMapImage = null;
    let cloudYaw = -0.75;
    let cloudPitch = 0.65;
    let cloudDragging = false;
    let cloudLast = { x: 0, y: 0 };

    function resizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(320, Math.floor(rect.width * dpr));
      const h = Math.max(240, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
        cachedMapSeq = -1;
      }
    }

    function fmt(value, digits = 2, suffix = '') {
      if (value === null || value === undefined || Number.isNaN(Number(value))) return '--';
      return `${Number(value).toFixed(digits)}${suffix}`;
    }

    function setStatus(ok, text) {
      document.getElementById('statusDot').classList.toggle('ok', ok);
      document.getElementById('statusText').textContent = text;
    }

    function mapToCanvas(map, x, y, geom) {
      const mx = (x - map.origin.x) / map.resolution;
      const my = (y - map.origin.y) / map.resolution;
      return {
        x: geom.ox + mx * geom.scale,
        y: geom.oy + (map.height - my) * geom.scale
      };
    }

    function drawMap(state) {
      resizeCanvas(mapCanvas);
      const ctx = mapCanvas.getContext('2d');
      ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
      ctx.fillStyle = '#f8fafc';
      ctx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
      const map = state.map;
      if (!map || !map.data) {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for map', 16, 24);
        return;
      }

      if (cachedMapSeq !== map.seq) {
        const off = document.createElement('canvas');
        off.width = map.width;
        off.height = map.height;
        const offCtx = off.getContext('2d');
        const img = offCtx.createImageData(map.width, map.height);
        for (let y = 0; y < map.height; y++) {
          for (let x = 0; x < map.width; x++) {
            const srcIdx = y * map.width + x;
            const dstY = map.height - 1 - y;
            const dstIdx = (dstY * map.width + x) * 4;
            const v = map.data[srcIdx];
            let r = 224, g = 229, b = 236;
            if (v === 0) { r = 255; g = 255; b = 255; }
            else if (v > 70) { r = 28; g = 38; b = 52; }
            else if (v > 0) { r = 120; g = 132; b = 150; }
            img.data[dstIdx] = r;
            img.data[dstIdx + 1] = g;
            img.data[dstIdx + 2] = b;
            img.data[dstIdx + 3] = 255;
          }
        }
        offCtx.putImageData(img, 0, 0);
        cachedMapImage = off;
        cachedMapSeq = map.seq;
      }

      const scale = Math.min(mapCanvas.width / map.width, mapCanvas.height / map.height);
      const dw = map.width * scale;
      const dh = map.height * scale;
      const geom = { scale, ox: (mapCanvas.width - dw) / 2, oy: (mapCanvas.height - dh) / 2 };
      ctx.drawImage(cachedMapImage, geom.ox, geom.oy, dw, dh);

      if (state.track && state.track.length > 1) {
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = Math.max(2, 2 * (window.devicePixelRatio || 1));
        ctx.beginPath();
        state.track.forEach((p, i) => {
          const c = mapToCanvas(map, p.x, p.y, geom);
          if (i === 0) ctx.moveTo(c.x, c.y);
          else ctx.lineTo(c.x, c.y);
        });
        ctx.stroke();
      }

      if (state.odom) {
        const c = mapToCanvas(map, state.odom.x, state.odom.y, geom);
        const yaw = state.odom.yaw || 0;
        const size = 12 * (window.devicePixelRatio || 1);
        ctx.save();
        ctx.translate(c.x, c.y);
        ctx.rotate(-yaw);
        ctx.fillStyle = '#dc2626';
        ctx.beginPath();
        ctx.moveTo(size, 0);
        ctx.lineTo(-size * 0.75, size * 0.6);
        ctx.lineTo(-size * 0.45, 0);
        ctx.lineTo(-size * 0.75, -size * 0.6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      document.getElementById('mapMeta').textContent =
        `${map.width}x${map.height}, ${fmt(map.resolution, 3, 'm/cell')}`;
    }

    function drawScan(state) {
      resizeCanvas(scanCanvas);
      const ctx = scanCanvas.getContext('2d');
      const w = scanCanvas.width, h = scanCanvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#fbfcfe';
      ctx.fillRect(0, 0, w, h);
      const scan = state.scan;
      const cx = w / 2, cy = h * 0.58;
      const maxM = scan?.range_max ? Math.min(scan.range_max, 8) : 8;
      const scale = Math.min(w, h) * 0.42 / maxM;

      ctx.strokeStyle = '#d8dee8';
      ctx.lineWidth = 1;
      for (let m = 1; m <= maxM; m++) {
        ctx.beginPath();
        ctx.arc(cx, cy, m * scale, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.strokeStyle = '#94a3b8';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - maxM * scale);
      ctx.stroke();

      ctx.fillStyle = '#2563eb';
      if (scan && scan.ranges) {
        for (let i = 0; i < scan.ranges.length; i++) {
          const r = scan.ranges[i];
          if (r === null) continue;
          const a = scan.angle_min + i * scan.angle_increment;
          const x = r * Math.cos(a);
          const y = r * Math.sin(a);
          const px = cx - y * scale;
          const py = cy - x * scale;
          ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
        }
      } else {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for laser scan', 16, 24);
      }
    }

    function renderSensors(state) {
      const grid = document.getElementById('sensorGrid');
      const sensors = state.sensors?.items || [];
      document.getElementById('sensorMeta').textContent =
        sensors.length ? `${sensors.length} sensors, age ${fmt(state.freshness_s.sensors, 2, 's')}` : 'waiting';
      grid.innerHTML = sensors.map(s => {
        const value = s.raw_value_is_finite ? fmt(s.value, 3) : 'inf';
        const hit = s.is_in_impact ? 'hit' : '';
        return `<div class="sensor-card ${hit}">
          <div class="sensor-title"><span>#${s.id} ${s.sensor_type_name}</span><span>${s.is_in_impact ? 'HIT' : 'OK'}</span></div>
          <div class="kv">
            <span>value</span><strong>${value}</strong>
            <span>impact</span><strong>${s.impact_type_name}</strong>
            <span>pose</span><strong>x=${fmt(s.pose.x, 3)}, y=${fmt(s.pose.y, 3)}, z=${fmt(s.pose.z, 3)}</strong>
            <span>freq</span><strong>${fmt(s.refresh_freq, 1, 'Hz')}</strong>
          </div>
        </div>`;
      }).join('') || '<div class="sensor-card">waiting for sensors</div>';
    }

    function rotatePoint(p) {
      const cy = Math.cos(cloudYaw), sy = Math.sin(cloudYaw);
      const cp = Math.cos(cloudPitch), sp = Math.sin(cloudPitch);
      const x1 = p[0] * cy - p[1] * sy;
      const y1 = p[0] * sy + p[1] * cy;
      const z1 = p[2];
      return [x1, y1 * cp - z1 * sp, y1 * sp + z1 * cp];
    }

    function drawCloudAxes(ctx, cx, cy, scale) {
      const axes = [
        { p: [0.6, 0, 0], c: '#dc2626', t: 'X' },
        { p: [0, 0.6, 0], c: '#16a34a', t: 'Y' },
        { p: [0, 0, 0.6], c: '#2563eb', t: 'Z' }
      ];
      ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
      axes.forEach(a => {
        const r = rotatePoint(a.p);
        const x = cx + r[0] * scale;
        const y = cy - r[1] * scale;
        ctx.strokeStyle = a.c;
        ctx.fillStyle = a.c;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(x, y);
        ctx.stroke();
        ctx.fillText(a.t, x + 4, y - 4);
      });
    }

    function drawCloud(state) {
      resizeCanvas(cloudCanvas);
      const ctx = cloudCanvas.getContext('2d');
      const w = cloudCanvas.width, h = cloudCanvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#fbfcfe';
      ctx.fillRect(0, 0, w, h);
      const cloud = state.point_cloud;
      const cx = w / 2, cy = h / 2;
      drawCloudAxes(ctx, cx, cy, Math.min(w, h) * 0.22);

      if (!cloud || !cloud.points || cloud.points.length === 0) {
        ctx.fillStyle = '#66758a';
        ctx.fillText('waiting for PointCloud2 data', 16, 24);
        document.getElementById('cloudMeta').textContent = 'waiting';
        return;
      }

      const pts = cloud.points;
      let maxAbs = 0.2;
      pts.forEach(p => {
        maxAbs = Math.max(maxAbs, Math.abs(p[0]), Math.abs(p[1]), Math.abs(p[2]));
      });
      const scale = Math.min(w, h) * 0.42 / maxAbs;
      const projected = pts.map(p => ({ p, r: rotatePoint(p) })).sort((a, b) => a.r[2] - b.r[2]);
      projected.forEach(item => {
        const x = cx + item.r[0] * scale;
        const y = cy - item.r[1] * scale;
        const zNorm = Math.max(0, Math.min(1, (item.r[2] / maxAbs + 1) / 2));
        const radius = Math.max(3, Math.min(8, 4 + zNorm * 4)) * (window.devicePixelRatio || 1);
        ctx.fillStyle = `rgb(${Math.round(40 + zNorm * 170)}, ${Math.round(110 + zNorm * 70)}, ${Math.round(230 - zNorm * 90)})`;
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fill();
      });

      document.getElementById('cloudMeta').textContent =
        `${cloud.sampled_points}/${cloud.total_points} points, age ${fmt(state.freshness_s.point_cloud, 2, 's')}`;
    }

    function updateReadouts(state) {
      document.getElementById('odomX').textContent = fmt(state.odom?.x, 3, 'm');
      document.getElementById('odomY').textContent = fmt(state.odom?.y, 3, 'm');
      document.getElementById('odomYaw').textContent = fmt(state.odom?.yaw_deg, 1, 'deg');
      document.getElementById('trackCount').textContent = state.track ? String(state.track.length) : '--';
      document.getElementById('scanValid').textContent = state.scan ? `${state.scan.valid_count}/${state.scan.count}` : '--';
      document.getElementById('scanMin').textContent = fmt(state.scan?.min_range, 3, 'm');
      document.getElementById('scanFrame').textContent = state.scan?.frame_id || '--';
      document.getElementById('scanAge').textContent = fmt(state.freshness_s?.scan, 2, 's');
      document.getElementById('scanMeta').textContent = state.scan
        ? `${state.scan.count} rays, age ${fmt(state.freshness_s.scan, 2, 's')}`
        : 'waiting';
      const cloud = state.point_cloud;
      document.getElementById('cloudTopic').textContent = cloud?.topic ? cloud.topic.split('/').filter(Boolean).slice(-1)[0] : '--';
      document.getElementById('cloudPoints').textContent = cloud ? `${cloud.sampled_points}/${cloud.total_points}` : '--';
      document.getElementById('cloudFrame').textContent = cloud?.frame_id || '--';
      document.getElementById('cloudAge').textContent = fmt(state.freshness_s?.point_cloud, 2, 's');
    }

    async function tick() {
      try {
        const res = await fetch('/api/state', { cache: 'no-store' });
        const state = await res.json();
        setStatus(true, `online, uptime ${fmt(state.uptime_s, 1, 's')}`);
        drawMap(state);
        drawScan(state);
        drawCloud(state);
        renderSensors(state);
        updateReadouts(state);
        document.getElementById('rawState').textContent = JSON.stringify({
          freshness_s: state.freshness_s,
          seq: state.seq,
          odom: state.odom,
          scan: state.scan ? { frame_id: state.scan.frame_id, min_range: state.scan.min_range, valid_count: state.scan.valid_count, count: state.scan.count } : null,
          point_cloud: state.point_cloud ? {
            topic: state.point_cloud.topic,
            frame_id: state.point_cloud.frame_id,
            total_points: state.point_cloud.total_points,
            sampled_points: state.point_cloud.sampled_points,
            bounds: state.point_cloud.bounds
          } : null,
          sensors: state.sensors,
          map: state.map ? { frame_id: state.map.frame_id, width: state.map.width, height: state.map.height, resolution: state.map.resolution, origin: state.map.origin } : null
        }, null, 2);
      } catch (err) {
        setStatus(false, `offline: ${err}`);
      }
    }

    cloudCanvas.addEventListener('pointerdown', ev => {
      cloudDragging = true;
      cloudLast = { x: ev.clientX, y: ev.clientY };
      cloudCanvas.setPointerCapture(ev.pointerId);
    });
    cloudCanvas.addEventListener('pointermove', ev => {
      if (!cloudDragging) return;
      const dx = ev.clientX - cloudLast.x;
      const dy = ev.clientY - cloudLast.y;
      cloudLast = { x: ev.clientX, y: ev.clientY };
      cloudYaw += dx * 0.01;
      cloudPitch = Math.max(-1.35, Math.min(1.35, cloudPitch + dy * 0.01));
    });
    cloudCanvas.addEventListener('pointerup', () => { cloudDragging = false; });
    cloudCanvas.addEventListener('pointerleave', () => { cloudDragging = false; });

    window.addEventListener('resize', () => { cachedMapSeq = -1; tick(); });
    setInterval(tick, 500);
    tick();
  </script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    state: SharedState

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/dashboard"):
            self.write_bytes(HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif parsed.path == "/api/state":
            self.write_json(self.state.snapshot())
        elif parsed.path == "/api/health":
            snap = self.state.snapshot()
            self.write_json(
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
                }
            )
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def write_json(self, obj: Dict[str, Any]) -> None:
        data = json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.write_bytes(data, "application/json; charset=utf-8")

    def write_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Web dashboard for base laser/map/odom/basic sensors.")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18083)
    parser.add_argument("--scan-topic", default="/slamware_ros_sdk_server_node/scan")
    parser.add_argument("--map-topic", default="/slamware_ros_sdk_server_node/map")
    parser.add_argument("--odom-topic", action="append", default=["/slamware_ros_sdk_server_node/odom"])
    parser.add_argument("--sensors-topic", default="/slamware_ros_sdk_server_node/basic_sensors_values")
    parser.add_argument("--pointcloud-topic", action="append", default=["/ele_clouds"])
    parser.add_argument("--max-cloud-points", type=int, default=6000)
    parser.add_argument("--max-track", type=int, default=1200)
    args = parser.parse_args()

    state = SharedState(max_track=args.max_track)
    rclpy.init()
    node = BaseSensorNode(
        state=state,
        scan_topic=args.scan_topic,
        map_topic=args.map_topic,
        odom_topics=args.odom_topic,
        sensors_topic=args.sensors_topic,
        pointcloud_topics=args.pointcloud_topic,
        max_cloud_points=args.max_cloud_points,
    )

    stop_event = threading.Event()

    def ros_spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=ros_spin, name="ros_spin", daemon=True)
    ros_thread.start()

    handler_cls = type("BoundDashboardHandler", (DashboardHandler,), {"state": state})
    server = ThreadingHTTPServer((args.bind, args.port), handler_cls)

    def shutdown(_signum: int, _frame: Any) -> None:
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(f"serving base sensor dashboard on http://{args.bind}:{args.port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        stop_event.set()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
