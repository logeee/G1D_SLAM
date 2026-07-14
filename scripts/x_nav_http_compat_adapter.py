#!/usr/bin/env python3
"""HTTP compatibility adapter for x_nav-style /api/extra calls.

This process is intentionally thin: it listens on the x_nav default HTTP port
and forwards supported calls to the existing G1D_SLAM dashboard service.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen


CONTROL_PAGE_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>x_nav 兼容层控制台</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f6f8;
      --panel: #ffffff;
      --panel-2: #eef3f8;
      --text: #14202b;
      --muted: #667383;
      --line: #d9e1ea;
      --accent: #1167b1;
      --accent-2: #0f766e;
      --danger: #b42318;
      --warn: #b54708;
      --ok: #067647;
      --shadow: 0 8px 28px rgba(16, 24, 40, 0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "Microsoft YaHei", sans-serif;
      letter-spacing: 0;
    }
    .shell {
      width: min(1120px, calc(100vw - 32px));
      margin: 0 auto;
      padding: 24px 0 36px;
    }
    header {
      display: flex;
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 18px;
    }
    h1 {
      margin: 0 0 6px;
      font-size: 26px;
      line-height: 1.2;
      font-weight: 720;
    }
    .sub {
      color: var(--muted);
      font-size: 14px;
    }
    .top-actions {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    button, a.button {
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: #fff;
      color: var(--text);
      min-height: 38px;
      padding: 8px 12px;
      font-size: 14px;
      font-weight: 650;
      cursor: pointer;
      text-decoration: none;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 7px;
      min-width: 92px;
      white-space: nowrap;
    }
    button:hover, a.button:hover { border-color: #a8b6c6; }
    button.primary {
      background: var(--accent);
      border-color: var(--accent);
      color: #fff;
    }
    button.danger {
      background: var(--danger);
      border-color: var(--danger);
      color: #fff;
    }
    button:disabled {
      opacity: 0.55;
      cursor: wait;
    }
    body.is-busy button {
      cursor: progress;
    }
    body.is-busy .mode-card {
      pointer-events: none;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 16px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
      padding: 16px;
      min-width: 0;
    }
    .section-title {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      margin-bottom: 14px;
      min-height: 30px;
    }
    h2 {
      margin: 0;
      font-size: 17px;
      line-height: 1.25;
    }
    .pill {
      border-radius: 999px;
      padding: 4px 9px;
      font-size: 12px;
      font-weight: 720;
      background: var(--panel-2);
      color: var(--muted);
      min-width: 76px;
      min-height: 24px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      white-space: nowrap;
    }
    .pill.ok { background: #dcfae6; color: var(--ok); }
    .pill.warn { background: #fef0c7; color: var(--warn); }
    .pill.danger { background: #fee4e2; color: var(--danger); }
    .current {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8fafc;
      padding: 14px;
      margin-bottom: 14px;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 110px;
      gap: 10px;
      align-items: center;
      min-height: 88px;
    }
    .mode-name {
      font-size: 24px;
      line-height: 1.2;
      font-weight: 760;
      word-break: keep-all;
      overflow-wrap: anywhere;
      min-height: 30px;
    }
    .mode-desc {
      color: var(--muted);
      margin-top: 4px;
      font-size: 13px;
      line-height: 1.45;
      min-height: 38px;
    }
    .mode-list {
      display: grid;
      gap: 10px;
      grid-template-columns: repeat(3, minmax(220px, 1fr));
      grid-auto-rows: minmax(92px, auto);
    }
    .mode-card {
      width: 100%;
      height: 100%;
      min-height: 92px;
      justify-content: flex-start;
      text-align: left;
      background: #fff;
      padding: 12px;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 86px;
      gap: 12px;
      align-items: center;
      min-width: 0;
    }
    .mode-card.active {
      border-color: var(--accent);
      box-shadow: inset 0 0 0 1px var(--accent);
      background: #f0f7ff;
    }
    .mode-card.raw.active {
      border-color: var(--danger);
      box-shadow: inset 0 0 0 1px var(--danger);
      background: #fff6f5;
    }
    .mode-card strong {
      display: block;
      font-size: 15px;
      margin-bottom: 4px;
      white-space: nowrap;
    }
    .mode-card > div > span {
      color: var(--muted);
      display: block;
      font-weight: 500;
      line-height: 1.35;
      white-space: normal;
      max-height: 38px;
      overflow: hidden;
    }
    .settings {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin-top: 14px;
      max-width: 520px;
    }
    label {
      display: grid;
      gap: 6px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 650;
    }
    input {
      width: 100%;
      min-height: 38px;
      border: 1px solid var(--line);
      border-radius: 7px;
      padding: 8px 10px;
      color: var(--text);
      font-size: 14px;
      background: #fff;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .metric {
      background: #f8fafc;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 11px;
      min-height: 82px;
    }
    .metric .key {
      color: var(--muted);
      font-size: 12px;
      font-weight: 720;
      margin-bottom: 6px;
    }
    .metric .value {
      font-size: 15px;
      font-weight: 720;
      overflow-wrap: anywhere;
    }
    .notice {
      margin-top: 12px;
      border: 1px solid #fedf89;
      background: #fffbeb;
      color: #93370d;
      border-radius: 8px;
      padding: 10px 12px;
      font-size: 13px;
      line-height: 1.45;
    }
    .log {
      display: grid;
      gap: 8px;
      max-height: 360px;
      overflow: auto;
      padding-right: 2px;
    }
    .log-item {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 9px;
      background: #f8fafc;
      font-size: 12px;
      line-height: 1.35;
    }
    .log-row {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 4px;
    }
    code {
      font-family: "Cascadia Mono", Consolas, monospace;
      font-size: 12px;
      color: #344054;
    }
    .toast {
      position: fixed;
      left: 50%;
      bottom: 22px;
      transform: translateX(-50%);
      background: #101828;
      color: #fff;
      border-radius: 7px;
      padding: 10px 13px;
      box-shadow: var(--shadow);
      font-size: 14px;
      display: none;
      z-index: 20;
      max-width: min(520px, calc(100vw - 32px));
    }
    .toast.show { display: block; }
    @media (max-width: 820px) {
      .shell { width: min(100vw - 20px, 640px); padding-top: 14px; }
      header { align-items: stretch; flex-direction: column; }
      .top-actions { justify-content: stretch; }
      .top-actions > * { flex: 1; }
      .grid { grid-template-columns: 1fr; }
      .current { grid-template-columns: 1fr; }
      .mode-list { grid-template-columns: 1fr; }
      .settings, .status-grid { grid-template-columns: 1fr; }
      h1 { font-size: 22px; }
      .mode-name { font-size: 21px; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header>
      <div>
        <h1>x_nav 兼容层控制台</h1>
        <div class="sub">机器人侧模式开关，作业平台不用改请求。</div>
      </div>
      <div class="top-actions">
        <a id="dashboardLink" class="button" href="#" target="_blank" rel="noreferrer">打开 18083</a>
        <button id="refreshBtn" type="button">刷新</button>
        <button id="stopBtn" class="danger" type="button">停止导航</button>
      </div>
    </header>

    <div class="grid">
      <section>
        <div class="section-title">
          <h2>导航模式</h2>
          <span id="saveState" class="pill">未连接</span>
        </div>

        <div class="current">
          <div>
            <div id="currentMode" class="mode-name">读取中...</div>
            <div id="currentDesc" class="mode-desc">正在连接机器人侧 9000 adapter。</div>
          </div>
          <span id="currentBadge" class="pill">unknown</span>
        </div>

        <div class="mode-list">
          <button class="mode-card" data-mode="normal" type="button">
            <div>
              <strong>普通避障</strong>
              <span>平台下发目标点后走 Slamware 正常导航，适合常规作业。</span>
            </div>
            <span class="pill">normal</span>
          </button>
          <button class="mode-card" data-mode="direct_no_avoidance" type="button">
            <div>
              <strong>直连少绕路</strong>
              <span>偏向直达目标点，遇到障碍仍由底层停止或失败。</span>
            </div>
            <span class="pill warn">direct</span>
          </button>
          <button class="mode-card raw" data-mode="raw_cmd_vel_no_obstacle_avoidance" type="button">
            <div>
              <strong>裸控不避障</strong>
              <span>adapter 直接用 /cmd_vel 走向目标点，仅限空旷现场和有人急停。</span>
            </div>
            <span class="pill danger">raw</span>
          </button>
        </div>

        <div class="settings">
          <label>
            裸控线速度 m/s
            <input id="linearSpeed" inputmode="decimal" type="number" min="0.05" max="1.2" step="0.01" value="0.35">
          </label>
          <label>
            裸控角速度 rad/s
            <input id="angularSpeed" inputmode="decimal" type="number" min="0.1" max="3" step="0.05" value="1.2">
          </label>
        </div>

        <div class="notice">裸控不避障会绕开 Slamware 避障能力。切换前请确认现场空旷，人员能立即按急停。</div>
      </section>

      <section>
        <div class="section-title">
          <h2>运行状态</h2>
          <span id="healthBadge" class="pill">checking</span>
        </div>
        <div class="status-grid">
          <div class="metric">
            <div class="key">9000 adapter</div>
            <div id="adapterStatus" class="value">读取中...</div>
          </div>
          <div class="metric">
            <div class="key">x_nav state</div>
            <div id="navState" class="value">读取中...</div>
          </div>
          <div class="metric">
            <div class="key">task_status</div>
            <div id="taskStatus" class="value">读取中...</div>
          </div>
          <div class="metric">
            <div class="key">last_update</div>
            <div id="lastUpdate" class="value">读取中...</div>
          </div>
        </div>

        <div class="section-title" style="margin-top: 18px;">
          <h2>最近请求</h2>
          <span class="pill">tail</span>
        </div>
        <div id="log" class="log"></div>
      </section>
    </div>
  </main>

  <div id="toast" class="toast"></div>

  <script>
    const MODES = {
      normal: {
        name: "普通避障",
        desc: "平台请求保持原样，底层走 Slamware 正常导航。",
        badge: "normal"
      },
      direct_no_avoidance: {
        name: "直连少绕路",
        desc: "平台请求保持原样，adapter 默认改成 direct_key_points_stop_on_obstacle。",
        badge: "direct"
      },
      raw_cmd_vel_no_obstacle_avoidance: {
        name: "裸控不避障",
        desc: "平台请求保持原样，adapter 默认改成 raw_cmd_vel 裸控。",
        badge: "raw"
      }
    };

    const $ = (id) => document.getElementById(id);
    let busy = false;
    let refreshInFlight = false;

    function toast(text) {
      const el = $("toast");
      el.textContent = text;
      el.classList.add("show");
      clearTimeout(window.__toastTimer);
      window.__toastTimer = setTimeout(() => el.classList.remove("show"), 2600);
    }

    async function getJson(url, options) {
      const res = await fetch(url, options);
      const text = await res.text();
      let data;
      try {
        data = text ? JSON.parse(text) : {};
      } catch (err) {
        throw new Error(`非 JSON 响应: ${text.slice(0, 120)}`);
      }
      if (!res.ok) {
        throw new Error(data.msg || data.error || `HTTP ${res.status}`);
      }
      return data;
    }

    function setBusy(value) {
      busy = value;
      document.body.classList.toggle("is-busy", value);
      $("saveState").textContent = value ? "保存中" : "已连接";
      $("saveState").className = value ? "pill warn" : "pill ok";
    }

    function renderConfig(config) {
      const mode = config.default_navigation_mode || "normal";
      const meta = MODES[mode] || { name: mode, desc: "未知模式", badge: mode };
      $("currentMode").textContent = meta.name;
      $("currentDesc").textContent = meta.desc;
      $("currentBadge").textContent = meta.badge;
      $("currentBadge").className = mode.includes("raw") ? "pill danger" : mode.includes("direct") ? "pill warn" : "pill ok";
      if (document.activeElement !== $("linearSpeed")) {
        $("linearSpeed").value = config.raw_linear_speed_mps ?? 0.35;
      }
      if (document.activeElement !== $("angularSpeed")) {
        $("angularSpeed").value = config.raw_angular_speed_radps ?? 1.2;
      }
      document.querySelectorAll(".mode-card").forEach((btn) => {
        btn.classList.toggle("active", btn.dataset.mode === mode);
      });
    }

    function renderState(state) {
      $("navState").textContent = state.nav_state ?? "-";
      $("taskStatus").textContent = state.task_status ?? "-";
      const compat = state.compat_state || {};
      const ts = compat.last_update_at ? new Date(compat.last_update_at * 1000) : null;
      $("lastUpdate").textContent = ts ? ts.toLocaleString() : "-";
    }

    function escapeHtml(value) {
      return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
        "&": "&amp;",
        "<": "&lt;",
        ">": "&gt;",
        '"': "&quot;",
        "'": "&#39;"
      }[ch]));
    }

    function renderLog(items) {
      const log = $("log");
      if (!items || !items.length) {
        log.innerHTML = '<div class="log-item">暂无请求记录</div>';
        return;
      }
      log.innerHTML = items.slice().reverse().map((item) => {
        const status = item.success === false ? "danger" : "ok";
        const body = item.request_body ? escapeHtml(JSON.stringify(item.request_body).slice(0, 160)) : "";
        const method = escapeHtml(item.method || "");
        const path = escapeHtml(item.path || "");
        const compatStatus = escapeHtml(item.compat_status || item.http_status || "");
        const time = escapeHtml(item.time || "");
        return `
          <div class="log-item">
            <div class="log-row">
              <strong>${method} <code>${path}</code></strong>
              <span class="pill ${status}">${compatStatus}</span>
            </div>
            <div><code>${time}</code></div>
            ${body ? `<div><code>${body}</code></div>` : ""}
          </div>
        `;
      }).join("");
    }

    function renderLogError(message) {
      $("log").innerHTML = `
        <div class="log-item">
          <div class="log-row">
            <strong>最近请求读取失败</strong>
            <span class="pill danger">error</span>
          </div>
          <div><code>${escapeHtml(message)}</code></div>
        </div>
      `;
    }

    async function refresh(options = {}) {
      if (busy && !options.force) return;
      if (refreshInFlight) return;
      refreshInFlight = true;
      try {
        const [healthResult, configResult, xstateResult, logResult] = await Promise.allSettled([
          getJson("/api/health"),
          getJson("/api/compat/config"),
          getJson("/api/compat/x_nav_state"),
          getJson("/api/compat/request_log/tail?limit=8")
        ]);
        const coreFailures = [];
        if (healthResult.status === "fulfilled") {
          const health = healthResult.value;
          $("adapterStatus").textContent = health.ok ? `OK, ${health.uptime_s}s` : "异常";
          $("healthBadge").textContent = health.ok ? "online" : "error";
          $("healthBadge").className = health.ok ? "pill ok" : "pill danger";
        } else {
          coreFailures.push(healthResult.reason);
          $("adapterStatus").textContent = "读取失败";
          $("healthBadge").textContent = "error";
          $("healthBadge").className = "pill danger";
        }
        if (configResult.status === "fulfilled") {
          renderConfig(configResult.value.config || {});
        } else {
          coreFailures.push(configResult.reason);
        }
        if (xstateResult.status === "fulfilled") {
          renderState(xstateResult.value || {});
        } else {
          coreFailures.push(xstateResult.reason);
        }
        if (logResult.status === "fulfilled") {
          renderLog(logResult.value.items || []);
        } else {
          renderLogError(logResult.reason?.message || String(logResult.reason));
        }
        if (coreFailures.length >= 3) {
          throw coreFailures[0];
        }
        if (!busy) {
          $("saveState").textContent = "已连接";
          $("saveState").className = "pill ok";
        }
      } catch (err) {
        $("healthBadge").textContent = "offline";
        $("healthBadge").className = "pill danger";
        $("saveState").textContent = "连接失败";
        $("saveState").className = "pill danger";
        toast(err.message || String(err));
      } finally {
        refreshInFlight = false;
      }
    }

    async function setMode(mode) {
      if (busy) return;
      setBusy(true);
      try {
        const payload = { default_navigation_mode: mode };
        if (mode === "raw_cmd_vel_no_obstacle_avoidance") {
          payload.raw_linear_speed_mps = Number($("linearSpeed").value || 0.35);
          payload.raw_angular_speed_radps = Number($("angularSpeed").value || 1.2);
        }
        const data = await getJson("/api/compat/config", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });
        renderConfig(data.config || payload);
        toast("模式已切换");
        await refresh({ force: true });
      } catch (err) {
        toast(err.message || String(err));
      } finally {
        setBusy(false);
      }
    }

    async function stopNavigation() {
      if (!confirm("确认停止当前导航？")) return;
      setBusy(true);
      try {
        await getJson("/api/extra/nav_work/cancel?stop=1");
        toast("已发送停止导航");
        await refresh({ force: true });
      } catch (err) {
        toast(err.message || String(err));
      } finally {
        setBusy(false);
      }
    }

    document.querySelectorAll(".mode-card").forEach((btn) => {
      btn.addEventListener("click", () => setMode(btn.dataset.mode));
    });
    $("refreshBtn").addEventListener("click", refresh);
    $("stopBtn").addEventListener("click", stopNavigation);
    $("dashboardLink").href = `${location.protocol}//${location.hostname}:18083`;

    refresh();
    setInterval(refresh, 3000);
  </script>
</body>
</html>
"""


def finite_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(number):
        return default
    return number


def strip_stcm(name: str) -> str:
    return name[:-5] if name.lower().endswith(".stcm") else name


def yaw_from_quaternion(qx: Any, qy: Any, qz: Any, qw: Any) -> float:
    x = finite_float(qx, 0.0) or 0.0
    y = finite_float(qy, 0.0) or 0.0
    z = finite_float(qz, 0.0) or 0.0
    w = finite_float(qw, 1.0) or 1.0
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def page_items(items: List[Dict[str, Any]], page: int, page_size: int) -> Tuple[List[Dict[str, Any]], int]:
    page = max(1, int(page or 1))
    page_size = max(1, int(page_size or 10))
    total_pages = max(1, math.ceil(len(items) / page_size))
    start = (page - 1) * page_size
    return items[start : start + page_size], total_pages


def compact_for_log(value: Any, max_text: int = 2000, max_items: int = 30) -> Any:
    """Keep request logs readable and bounded."""
    if value is None or isinstance(value, (bool, int, float, str)):
        if isinstance(value, str) and len(value) > max_text:
            return value[:max_text] + "...<truncated>"
        return value
    if isinstance(value, list):
        compacted = [compact_for_log(item, max_text=max_text, max_items=max_items) for item in value[:max_items]]
        if len(value) > max_items:
            compacted.append({"truncated_items": len(value) - max_items})
        return compacted
    if isinstance(value, dict):
        compacted: Dict[str, Any] = {}
        for index, (key, item) in enumerate(value.items()):
            if index >= max_items:
                compacted["truncated_keys"] = len(value) - max_items
                break
            compacted[str(key)] = compact_for_log(item, max_text=max_text, max_items=max_items)
        return compacted
    return str(value)


def should_log_path(path: str) -> bool:
    # The built-in control page polls these endpoints frequently. Logging them
    # makes the JSONL file grow fast and can recursively capture log responses.
    quiet_paths = {
        "/api/health",
        "/api/compat/health",
        "/api/compat/config",
        "/api/compat/x_nav_state",
        "/api/compat/request_log/tail",
        "/api/compat/capabilities",
        "/api/s1-agent/v1/task/query",
    }
    return path not in quiet_paths


class UpstreamClient:
    def __init__(self, base_url: str, timeout_sec: float = 3.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def request(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        url = self.base_url + path
        data = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = Request(url, data=data, headers=headers, method=method.upper())
        try:
            with urlopen(req, timeout=self.timeout_sec) as response:
                raw = response.read()
                if not raw:
                    return {"ok": True}
                text = raw.decode("utf-8", errors="replace")
                try:
                    parsed = json.loads(text)
                except json.JSONDecodeError:
                    return {"ok": False, "error": "upstream returned non-json response", "body": text}
                if isinstance(parsed, dict):
                    return parsed
                return {"ok": True, "data": parsed}
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            return {"ok": False, "error": f"upstream HTTP {exc.code}", "body": body}
        except URLError as exc:
            return {"ok": False, "error": f"upstream unavailable: {exc.reason}"}
        except TimeoutError:
            return {"ok": False, "error": "upstream timeout"}


class CompatServer(ThreadingHTTPServer):
    def __init__(self, addr: Tuple[str, int], handler: type[BaseHTTPRequestHandler], args: argparse.Namespace) -> None:
        super().__init__(addr, handler)
        self.args = args
        self.upstream = UpstreamClient(args.upstream, args.upstream_timeout_sec)
        self.started_at = time.time()
        self.log_lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.compat_state: Dict[str, Any] = {
            "nav_state": "-1",
            "slam_state": "3",
            "master_state": "0",
            "mode": "idle",
            "map_name": "",
            "task_status": "无任务",
            "last_goal": None,
            "last_nav_started_at": None,
            "last_update_at": time.time(),
        }
        self.config: Dict[str, Any] = {
            "default_navigation_mode": args.default_navigation_mode,
            "raw_linear_speed_mps": args.raw_linear_speed_mps,
            "raw_angular_speed_radps": args.raw_angular_speed_radps,
        }
        self.s1_tasks: Dict[str, Dict[str, Any]] = {}
        if args.request_log_path:
            log_path = Path(args.request_log_path)
            if not log_path.is_absolute():
                log_path = Path.cwd() / log_path
            self.request_log_path = log_path
            self.request_log_path.parent.mkdir(parents=True, exist_ok=True)
        else:
            self.request_log_path = None

    def set_compat_state(self, **updates: Any) -> Dict[str, Any]:
        with self.state_lock:
            self.compat_state.update(updates)
            self.compat_state["last_update_at"] = time.time()
            return dict(self.compat_state)

    def get_compat_state(self) -> Dict[str, Any]:
        with self.state_lock:
            return dict(self.compat_state)

    def update_config(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        allowed_modes = {"normal", "direct_no_avoidance", "raw_cmd_vel_no_obstacle_avoidance"}
        with self.state_lock:
            if "default_navigation_mode" in updates or "mode" in updates or "navigation_mode" in updates:
                mode = str(updates.get("default_navigation_mode", updates.get("mode", updates.get("navigation_mode")))).strip()
                if mode not in allowed_modes:
                    raise ValueError("default_navigation_mode must be one of: " + ", ".join(sorted(allowed_modes)))
                self.config["default_navigation_mode"] = mode
            for key in ("raw_linear_speed_mps", "raw_angular_speed_radps"):
                if key in updates:
                    value = finite_float(updates.get(key))
                    if value is None:
                        raise ValueError(f"{key} must be a finite number")
                    self.config[key] = value
            return dict(self.config)

    def get_config(self) -> Dict[str, Any]:
        with self.state_lock:
            return dict(self.config)

    def record_s1_task(self, task_id: str, command_resp_list: List[Dict[str, Any]], status: str = "executing") -> Dict[str, Any]:
        now = time.time()
        task = {
            "task_id": task_id,
            "task_status": status,
            "command_resp_list": command_resp_list,
            "created_at": now,
            "updated_at": now,
        }
        with self.state_lock:
            self.s1_tasks[task_id] = task
            return json.loads(json.dumps(task, ensure_ascii=False))

    def get_s1_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        with self.state_lock:
            task = self.s1_tasks.get(task_id)
            return json.loads(json.dumps(task, ensure_ascii=False)) if task else None

    def update_s1_task(self, task_id: str, *, status: Optional[str] = None, description: Optional[str] = None) -> Optional[Dict[str, Any]]:
        with self.state_lock:
            task = self.s1_tasks.get(task_id)
            if not task:
                return None
            if status:
                task["task_status"] = status
                for command in task.get("command_resp_list", []):
                    command["status"] = status
            if description is not None:
                for command in task.get("command_resp_list", []):
                    command["description"] = description
            task["updated_at"] = time.time()
            return json.loads(json.dumps(task, ensure_ascii=False))

    def terminate_active_s1_navigation_tasks(self, description: str = "terminated by emergency_stop") -> None:
        with self.state_lock:
            for task in self.s1_tasks.values():
                if task.get("task_status") in ("issued", "executing", "unprocess"):
                    commands = task.get("command_resp_list") or []
                    if any(command.get("task_command_code") == "navigation" for command in commands):
                        task["task_status"] = "terminated"
                        for command in commands:
                            command["status"] = "terminated"
                            command["description"] = description
                        task["updated_at"] = time.time()

    def append_request_log(self, entry: Dict[str, Any]) -> None:
        if not self.request_log_path:
            return
        entry = compact_for_log(entry)
        line = json.dumps(entry, ensure_ascii=False, separators=(",", ":"))
        with self.log_lock:
            with self.request_log_path.open("a", encoding="utf-8") as f:
                f.write(line + "\n")

    def tail_request_log(self, limit: int = 50) -> List[Dict[str, Any]]:
        if not self.request_log_path or not self.request_log_path.exists():
            return []
        limit = max(1, min(500, int(limit or 50)))
        max_bytes = 2 * 1024 * 1024
        block_size = 64 * 1024
        with self.log_lock:
            with self.request_log_path.open("rb") as f:
                f.seek(0, os.SEEK_END)
                pos = f.tell()
                chunks: List[bytes] = []
                bytes_read = 0
                newline_count = 0
                while pos > 0 and newline_count <= limit and bytes_read < max_bytes:
                    read_size = min(block_size, pos, max_bytes - bytes_read)
                    pos -= read_size
                    f.seek(pos)
                    chunk = f.read(read_size)
                    chunks.append(chunk)
                    bytes_read += len(chunk)
                    newline_count += chunk.count(b"\n")
                raw = b"".join(reversed(chunks))
            lines = raw.decode("utf-8", errors="replace").splitlines()[-limit:]
        items: List[Dict[str, Any]] = []
        for line in lines[-limit:]:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                item = {"raw": line[:2000], "truncated": len(line) > 2000}
            items.append(item)
        return items


class XNavCompatHandler(BaseHTTPRequestHandler):
    server: CompatServer

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.server.args.quiet:
            return
        super().log_message(fmt, *args)

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def write_json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        parsed = urlparse(self.path)
        if not should_log_path(parsed.path):
            return
        self.server.append_request_log(
            {
                "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "method": self.command,
                "path": parsed.path,
                "query": parse_qs(parsed.query),
                "client": self.client_address[0] if self.client_address else "",
                "request_body": getattr(self, "_last_json_body", None),
                "http_status": int(status),
                "response": payload,
                "unknown_endpoint": bool(payload.get("unknown_endpoint")),
                "compat_status": payload.get("status"),
                "success": payload.get("success"),
            }
        )

    def write_html(self, html: str, status: int = HTTPStatus.OK) -> None:
        body = html.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json_body(self) -> Optional[Dict[str, Any]]:
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0:
            self._last_json_body = {}
            return {}
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self._last_json_body = raw.decode("utf-8", errors="replace")
            self.write_json({"success": False, "msg": f"invalid json: {exc}", "status": "error"}, HTTPStatus.BAD_REQUEST)
            return None
        if not isinstance(payload, dict):
            self._last_json_body = payload
            self.write_json({"success": False, "msg": "json body must be an object", "status": "error"}, HTTPStatus.BAD_REQUEST)
            return None
        self._last_json_body = payload
        return payload

    @staticmethod
    def success(msg: str = "操作成功", **extra: Any) -> Dict[str, Any]:
        payload = {"success": True, "status": "success", "msg": msg}
        payload.update(extra)
        return payload

    @staticmethod
    def failure(msg: str, *, status: str = "error", upstream: Optional[Dict[str, Any]] = None, **extra: Any) -> Dict[str, Any]:
        payload = {"success": False, "status": status, "msg": msg}
        if upstream is not None:
            payload["upstream"] = upstream
        payload.update(extra)
        return payload

    @staticmethod
    def s1_success(result: Any = None) -> Dict[str, Any]:
        return {"success": True, "errcode": "SUCCESS", "errmsg": "ok", "result": result}

    @staticmethod
    def s1_failure(errmsg: str, errcode: str = "ERROR", result: Any = None) -> Dict[str, Any]:
        return {"success": False, "errcode": errcode, "errmsg": errmsg, "result": result}

    def upstream(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return self.server.upstream.request(method, path, payload)

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        path = parsed.path
        if path in ("/", "/compat", "/compat/", "/api/compat/control", "/api/compat/ui"):
            self.write_html(CONTROL_PAGE_HTML)
            return
        if path in ("/api/health", "/api/compat/health"):
            self.write_json(
                {
                    "success": True,
                    "ok": True,
                    "status": "success",
                    "adapter": "x_nav_http_compat_adapter",
                    "upstream": self.server.args.upstream,
                    "uptime_s": round(time.time() - self.server.started_at, 3),
                }
            )
            return
        if path == "/api/compat/capabilities":
            self.write_json(self.capabilities())
            return
        if path == "/api/compat/config":
            self.write_json({"success": True, "status": "success", "config": self.server.get_config()})
            return
        if path == "/api/compat/request_log/tail":
            limit = int((query.get("limit") or ["50"])[0] or 50)
            self.write_json(
                {
                    "success": True,
                    "status": "success",
                    "log_path": str(self.server.request_log_path) if self.server.request_log_path else "",
                    "items": self.server.tail_request_log(limit),
                }
            )
            return
        if path in ("/api/compat/x_nav_state", "/api/extra/get_nav_state"):
            self.write_json(self.handle_get_x_nav_state())
            return
        if path == "/api/extra/get_status":
            self.write_json(self.handle_get_status())
            return
        if path == "/api/extra/get_map_data":
            self.write_json(self.handle_get_map_data(query))
            return
        if path == "/api/extra/get_nav_data":
            self.write_json(self.handle_get_nav_data(query))
            return
        if path == "/api/extra/get_obstacle_data":
            self.write_json({"data": [], "totalPages": 1, "success": True, "status": "success", "msg": "虚拟障碍物兼容接口暂为空"})
            return
        if path == "/api/extra/refresh_point_cloud":
            health = self.upstream("GET", "/api/health")
            ok = bool(health.get("ok"))
            self.write_json(self.success("刷新成功", upstream=health) if ok else self.failure("底层服务不可用", upstream=health))
            return
        if path == "/api/extra/nav_work/cancel":
            stop = (query.get("stop") or [""])[0]
            if stop == "1":
                self.write_json(self.cancel_navigation("停止导航作业"))
            else:
                self.write_json(
                    self.success(
                        "继续导航作业",
                        compat_noop=True,
                        note="G1D_SLAM 当前没有暂停/继续语义；该接口作为兼容 no-op 返回成功",
                    )
                )
            return
        if path == "/api/extra/target_angle":
            angle = (query.get("angle") or [""])[0]
            self.write_json(
                self.failure(
                    f"web_cmd/cmd{angle} 暂未映射到 G1D 底盘动作",
                    status="not_implemented",
                )
            )
            return
        self.write_json(
            self.failure(
                f"unknown GET endpoint: {path}",
                status="unsupported_endpoint",
                unknown_endpoint=True,
                method="GET",
                request_path=path,
                query=query,
            )
        )

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        payload = self.read_json_body()
        if payload is None:
            return
        if path == "/api/s1-agent/v1/task/submit":
            self.write_json(self.handle_s1_task_submit(payload))
            return
        if path == "/api/s1-agent/v1/task/query":
            self.write_json(self.handle_s1_task_query(payload))
            return
        if path == "/api/extra/nav_custom":
            self.write_json(self.handle_nav_custom(payload))
            return
        if path == "/api/compat/config":
            try:
                config = self.server.update_config(payload)
            except ValueError as exc:
                self.write_json(self.failure(str(exc), status="bad_config"))
                return
            self.write_json(self.success("配置已更新", config=config))
            return
        if path in ("/api/navigation/start", "/api/nav/start", "/api/compat/navigation/start"):
            self.write_json(self.handle_navigation_start(payload))
            return
        if path == "/api/extra/mark_nav_point":
            self.write_json(self.handle_mark_nav_point(payload))
            return
        if path == "/api/extra/delete_nav_area":
            self.write_json(
                self.failure(
                    "虚拟障碍物接口暂未映射；思岚底层当前兼容层不写入 x_nav MarkerArray 障碍物",
                    status="not_implemented",
                )
            )
            return
        if path == "/api/extra/set_pose":
            self.write_json(self.handle_set_pose(payload))
            return
        if path == "/api/extra/add_map":
            self.write_json(self.handle_add_map(payload))
            return
        if path == "/api/extra/switch_map":
            self.write_json(self.handle_switch_map(payload))
            return
        if path in ("/api/extra/save_map", "/api/extra/save_maps"):
            self.write_json(self.handle_save_map(payload))
            return
        if path in ("/api/navigation/cancel", "/api/nav/cancel", "/api/extra/cancel_nav"):
            self.write_json(self.cancel_navigation("停止导航作业"))
            return
        self.write_json(
            self.failure(
                f"unknown POST endpoint: {path}",
                status="unsupported_endpoint",
                unknown_endpoint=True,
                method="POST",
                request_path=path,
                request_body=payload,
            )
        )

    def capabilities(self) -> Dict[str, Any]:
        return {
            "success": True,
            "status": "success",
            "adapter": "x_nav_http_compat_adapter",
            "upstream": self.server.args.upstream,
            "config": self.server.get_config(),
            "supported": [
                "GET /",
                "GET /api/compat/control",
                "GET /api/extra/get_status",
                "GET /api/extra/get_nav_state",
                "GET /api/compat/request_log/tail",
                "GET /api/compat/config",
                "POST /api/compat/config",
                "POST /api/s1-agent/v1/task/submit",
                "POST /api/s1-agent/v1/task/query",
                "GET /api/extra/get_map_data",
                "GET /api/extra/get_nav_data",
                "GET /api/extra/get_obstacle_data",
                "GET /api/extra/refresh_point_cloud",
                "GET /api/extra/nav_work/cancel?stop=1",
                "POST /api/extra/add_map",
                "POST /api/extra/save_map",
                "POST /api/extra/switch_map",
                "POST /api/extra/nav_custom",
                "POST /api/extra/mark_nav_point",
                "POST /api/extra/set_pose",
            ],
            "not_implemented": [
                "virtual obstacle write/delete is acknowledged as not_implemented",
                "web_cmd gait/action commands are not mapped",
                "pause/resume navigation is not mapped; cancel + resend instead",
                "ROS1 topics/services are not provided by this HTTP-only adapter",
            ],
        }

    def handle_get_x_nav_state(self) -> Dict[str, Any]:
        state = self.upstream("GET", "/api/state")
        mapping_status = self.upstream("GET", "/api/mapping/status")
        derived = self.derive_x_nav_state(state, mapping_status)
        return {
            "success": True,
            "status": "success",
            "nav_state": derived["nav_state"],
            "slam_state": derived["slam_state"],
            "master_state": derived["master_state"],
            "task_status": derived["task_status"],
            "control_status": derived["control_status"],
            "compat_state": self.server.get_compat_state(),
        }

    def handle_get_status(self) -> Dict[str, Any]:
        state = self.upstream("GET", "/api/state")
        if not state.get("uptime_s") and not state.get("ok", True):
            return self.failure("底层状态读取失败", upstream=state)
        mapping_status = self.upstream("GET", "/api/mapping/status")
        odom = state.get("odom") or {}
        nav = (state.get("navigation") or {}).get("last_command") or {}
        derived = self.derive_x_nav_state(state, mapping_status)
        x = finite_float(odom.get("x"), 0.0) or 0.0
        y = finite_float(odom.get("y"), 0.0) or 0.0
        yaw_deg = finite_float(odom.get("yaw_deg"), 0.0) or 0.0
        age = finite_float(((state.get("freshness_s") or {}).get("odom")), 0.0) or 0.0
        return {
            "battery": "",
            "control_status": derived["control_status"],
            "cpu_temperature": "",
            "cpu_usage": "",
            "map_name": self.current_map_name(state, mapping_status),
            "memory_percent": "",
            "package": "",
            "position_msg": f"({x:.3f}, {y:.3f}, {yaw_deg:.1f}) age={age:.2f}s",
            "pub_vel_msg": "",
            "status": "success",
            "success": True,
            "task_status": derived["task_status"],
            "version": "x_nav_compat_http_v1",
            "x_nav_state": derived["nav_state"],
            "x_nav_slam_state": derived["slam_state"],
            "x_nav_master_state": derived["master_state"],
            "upstream_navigation": nav,
            "upstream_mapping": mapping_status,
        }

    def derive_x_nav_state(self, state: Dict[str, Any], mapping_status: Optional[Dict[str, Any]] = None) -> Dict[str, str]:
        compat = self.server.get_compat_state()
        nav = (state.get("navigation") or {}).get("last_command") or {}
        basic = (state.get("navigation") or {}).get("robot_basic_state") or {}
        odom = state.get("odom") or {}
        freshness = state.get("freshness_s") or {}
        map_name = self.current_map_name(state, mapping_status)
        nav_state = str(compat.get("nav_state") or "-1")
        slam_state = str(compat.get("slam_state") or "3")
        master_state = str(compat.get("master_state") or "0")
        task_status = str(compat.get("task_status") or "无任务")
        control_status = "导航模式" if map_name else "空闲"

        if map_name and master_state in ("0", ""):
            master_state = f"2#{map_name}"
        if basic.get("is_localization_enabled") is False:
            control_status = "导航模式 请重定位初始化"
            slam_state = "3"

        odom_age = finite_float(freshness.get("odom"))
        if odom_age is not None and odom_age > self.server.args.odom_stale_sec:
            nav_state = "16"
            task_status = "未接受到base_link/odom，无法进行导航，请重定位"

        command_type = str(nav.get("type") or "")
        if command_type == "cancel_action":
            nav_state = "-1"
            task_status = "无任务"
        elif command_type == "raw_cmd_vel_navigation":
            raw_status = str(nav.get("raw_nav_status") or "running")
            if raw_status == "done":
                nav_state = "13"
                task_status = "无任务"
            elif raw_status in ("cancelled", "dry_run"):
                nav_state = "-1"
                task_status = "无任务"
            elif raw_status in ("timeout", "error"):
                nav_state = "15"
                task_status = "导航失败"
            elif raw_status == "final_yaw":
                nav_state = "12"
                task_status = "即将到达目标点，正在对齐目标终点"
            else:
                nav_state = "0"
                task_status = "执行导航任务中"
        elif command_type == "move_to_locations":
            arrived = self.is_arrived(odom, nav)
            if arrived:
                nav_state = "13"
                task_status = "无任务"
            else:
                started_at = finite_float(nav.get("received_at"))
                if started_at and time.time() - started_at < 2.0:
                    nav_state = "14"
                    task_status = "全局路径规划成功"
                else:
                    nav_state = "0"
                    task_status = "执行导航任务中"

        if nav_state == "-2":
            control_status = "导航模式 地图初始化成功"
        elif nav_state == "-1" and map_name:
            control_status = "导航模式"

        self.server.set_compat_state(
            nav_state=nav_state,
            slam_state=slam_state,
            master_state=master_state,
            mode="navigation" if map_name else compat.get("mode", "idle"),
            map_name=map_name,
            task_status=task_status,
        )
        return {
            "nav_state": nav_state,
            "slam_state": slam_state,
            "master_state": master_state,
            "task_status": task_status,
            "control_status": control_status,
        }

    def is_arrived(self, odom: Dict[str, Any], nav: Dict[str, Any]) -> bool:
        waypoints = nav.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            return False
        goal = waypoints[-1]
        if not isinstance(goal, dict):
            return False
        x = finite_float(odom.get("x"))
        y = finite_float(odom.get("y"))
        gx = finite_float(goal.get("x"))
        gy = finite_float(goal.get("y"))
        if x is None or y is None or gx is None or gy is None:
            return False
        distance = math.hypot(float(gx) - float(x), float(gy) - float(y))
        if distance > self.server.args.arrival_distance_m:
            return False
        yaw = finite_float(odom.get("yaw"))
        goal_yaw = finite_float(nav.get("yaw"))
        if yaw is None or goal_yaw is None:
            return True
        yaw_error = abs(math.atan2(math.sin(float(goal_yaw) - float(yaw)), math.cos(float(goal_yaw) - float(yaw))))
        return math.degrees(yaw_error) <= self.server.args.arrival_yaw_deg

    @staticmethod
    def current_map_name(state: Dict[str, Any], mapping_status: Optional[Dict[str, Any]] = None) -> str:
        mapping = mapping_status if mapping_status and mapping_status.get("ok") else state.get("mapping") or {}
        loaded = mapping.get("last_map_load") or {}
        if not loaded:
            loaded = mapping.get("last_map_save") or {}
        name = str(loaded.get("name") or "")
        return strip_stcm(name)

    def handle_get_map_data(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        page = int((query.get("page") or ["1"])[0] or 1)
        upstream = self.upstream("GET", "/api/mapping/list")
        if not upstream.get("ok"):
            return self.failure("地图列表读取失败", upstream=upstream)
        rows = []
        for item in upstream.get("saved_maps", []):
            raw_name = str(item.get("name") or "")
            if not raw_name:
                continue
            rows.append({"col1": strip_stcm(raw_name), "col2": "切换", "col3": "删除", "raw": item})
        data, total_pages = page_items(rows, page, self.server.args.page_size)
        return {"data": data, "totalPages": total_pages, "success": True, "status": "success"}

    def handle_get_nav_data(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        page = int((query.get("page") or ["1"])[0] or 1)
        upstream = self.upstream("GET", "/api/points")
        if not upstream.get("ok"):
            return self.failure("导航点位读取失败", upstream=upstream)
        rows = []
        for index, point in enumerate(upstream.get("points", []), start=1):
            name = str(point.get("name") or point.get("id") or index)
            rows.append(
                {
                    "col1": name,
                    "col2": "导航",
                    "col3": "重定位",
                    "col4": "删除",
                    "id": point.get("id"),
                    "x": point.get("x"),
                    "y": point.get("y"),
                    "yaw_deg": point.get("yaw_deg"),
                    "raw": point,
                }
            )
        data, total_pages = page_items(rows, page, self.server.args.page_size)
        return {"data": data, "totalPages": total_pages, "success": True, "status": "success"}

    def parse_pose_payload(self, payload: Dict[str, Any]) -> Tuple[Optional[Dict[str, float]], Optional[str]]:
        x = finite_float(payload.get("positionX", payload.get("x")))
        y = finite_float(payload.get("positionY", payload.get("y")))
        z = finite_float(payload.get("positionZ", payload.get("z")), 0.0) or 0.0
        if x is None or y is None:
            return None, "positionX/positionY is required"
        yaw = payload.get("yaw")
        yaw_deg = finite_float(payload.get("yaw_deg", payload.get("yawDeg")))
        if yaw is not None:
            yaw_value = finite_float(yaw)
            if yaw_value is None:
                return None, "invalid yaw"
        elif yaw_deg is not None:
            yaw_value = math.radians(float(yaw_deg))
        else:
            yaw_value = yaw_from_quaternion(
                payload.get("orientationX", payload.get("qx", 0.0)),
                payload.get("orientationY", payload.get("qy", 0.0)),
                payload.get("orientationZ", payload.get("qz", 0.0)),
                payload.get("orientationW", payload.get("qw", 1.0)),
            )
        return {"x": float(x), "y": float(y), "z": float(z), "yaw": float(yaw_value), "yaw_deg": math.degrees(float(yaw_value))}, None

    @staticmethod
    def s1_command_response(
        command_id: str,
        command_code: str,
        status: str,
        description: str = "ok",
        result: Any = None,
    ) -> Dict[str, Any]:
        return {
            "task_command_id": command_id,
            "task_command_code": command_code,
            "status": status,
            "result": result,
            "description": description,
        }

    def handle_s1_task_submit(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            return self.s1_failure("task_id is required", "BAD_REQUEST")
        commands = payload.get("task_command_info")
        if not isinstance(commands, list) or not commands:
            return self.s1_failure("task_command_info is required", "BAD_REQUEST")
        if len(commands) != 1:
            return self.s1_failure("only one task_command_info item is supported currently", "UNSUPPORTED_COMMAND")
        command = commands[0]
        if not isinstance(command, dict):
            return self.s1_failure("task_command_info item must be an object", "BAD_REQUEST")
        command_id = str(command.get("command_id") or task_id).strip() or task_id
        command_code = str(command.get("command_code") or "").strip()
        if command_code == "navigation":
            return self.handle_s1_navigation_submit(task_id, command_id, command)
        if command_code == "emergency_stop":
            return self.handle_s1_emergency_stop_submit(task_id, command_id)
        return self.s1_failure(f"unsupported command_code: {command_code}", "UNSUPPORTED_COMMAND")

    def handle_s1_navigation_submit(self, task_id: str, command_id: str, command: Dict[str, Any]) -> Dict[str, Any]:
        params = command.get("command_param")
        if not isinstance(params, dict):
            return self.s1_failure("command_param is required", "BAD_REQUEST")
        position = params.get("position")
        if not isinstance(position, dict):
            return self.s1_failure("command_param.position is required", "BAD_REQUEST")
        orientation = params.get("orientation") if isinstance(params.get("orientation"), dict) else {}
        nav_payload = {
            "inputValue": task_id,
            "positionX": position.get("x"),
            "positionY": position.get("y"),
            "positionZ": position.get("z", 0.0),
            "orientationX": orientation.get("x", 0.0),
            "orientationY": orientation.get("y", 0.0),
            "orientationZ": orientation.get("z", 0.0),
            "orientationW": orientation.get("w", 1.0),
            "s1_task_id": task_id,
            "s1_command_id": command_id,
        }
        yaw = params.get("yaw", orientation.get("yaw"))
        yaw_deg = params.get("yaw_deg", params.get("yawDeg", orientation.get("yaw_deg", orientation.get("yawDeg"))))
        if yaw is not None:
            nav_payload["yaw"] = yaw
        elif yaw_deg is not None:
            nav_payload["yaw_deg"] = yaw_deg
        if params.get("dry_run"):
            nav_payload["dry_run"] = True
        result = self.handle_nav_custom(nav_payload)
        if result.get("success"):
            self.server.record_s1_task(
                task_id,
                [
                    self.s1_command_response(
                        command_id,
                        "navigation",
                        "executing",
                        "navigation command accepted",
                    )
                ],
                status="executing",
            )
            return self.s1_success(None)
        description = str(result.get("msg") or "navigation command failed")
        self.server.record_s1_task(
            task_id,
            [self.s1_command_response(command_id, "navigation", "error", description, result=result)],
            status="error",
        )
        return self.s1_failure(description, "NAVIGATION_START_FAILED")

    def handle_s1_emergency_stop_submit(self, task_id: str, command_id: str) -> Dict[str, Any]:
        result = self.cancel_navigation("emergency_stop")
        if result.get("success"):
            self.server.terminate_active_s1_navigation_tasks()
            self.server.record_s1_task(
                task_id,
                [self.s1_command_response(command_id, "emergency_stop", "finished", "ok")],
                status="finished",
            )
            return self.s1_success(None)
        description = str(result.get("msg") or "emergency_stop failed")
        self.server.record_s1_task(
            task_id,
            [self.s1_command_response(command_id, "emergency_stop", "error", description, result=result)],
            status="error",
        )
        return self.s1_failure(description, "EMERGENCY_STOP_FAILED")

    def handle_s1_task_query(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            return self.s1_failure("task_id is required", "BAD_REQUEST")
        task = self.server.get_s1_task(task_id)
        if not task:
            return self.s1_success({"task_id": task_id, "task_status": "unknown", "command_resp_list": []})
        task = self.refresh_s1_task_status(task)
        return self.s1_success(
            {
                "task_id": task_id,
                "task_status": task.get("task_status", "unknown"),
                "command_resp_list": task.get("command_resp_list", []),
            }
        )

    def refresh_s1_task_status(self, task: Dict[str, Any]) -> Dict[str, Any]:
        status = str(task.get("task_status") or "unknown")
        if status not in ("issued", "executing", "unprocess"):
            return task
        commands = task.get("command_resp_list") or []
        if not any(command.get("task_command_code") == "navigation" for command in commands):
            return task
        state = self.upstream("GET", "/api/state")
        mapping_status = self.upstream("GET", "/api/mapping/status")
        derived = self.derive_x_nav_state(state, mapping_status)
        nav_state = str(derived.get("nav_state") or "")
        description = str(derived.get("task_status") or "")
        new_status = "executing"
        new_description = description or "executing"
        if nav_state == "13":
            new_status = "finished"
            new_description = "ok"
        elif nav_state in ("15", "16"):
            new_status = "error"
            new_description = description or "navigation failed"
        elif nav_state == "-1":
            nav = (state.get("navigation") or {}).get("last_command") or {}
            if str(nav.get("type") or "") == "cancel_action":
                new_status = "terminated"
                new_description = "terminated"
        updated = self.server.update_s1_task(str(task.get("task_id")), status=new_status, description=new_description)
        return updated or task

    def handle_nav_custom(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose, err = self.parse_pose_payload(payload)
        if err:
            return self.failure(err)
        assert pose is not None
        request = {
            "waypoints": [{"x": pose["x"], "y": pose["y"]}],
            "yaw": pose["yaw"],
            "yaw_source": payload.get("yaw_source") or payload.get("yawSource") or "x_nav_compat",
        }
        passthrough_fields = (
            "speed_ratio",
            "speedRatio",
            "raw_cmd_vel",
            "rawCmdVel",
            "disable_obstacle_avoidance",
            "disableObstacleAvoidance",
            "navigation_mode",
            "direct_no_avoidance",
            "directNoAvoidance",
            "key_points_mode",
            "keyPointsMode",
            "raw_linear_speed_mps",
            "raw_angular_speed_radps",
            "raw_position_tolerance_m",
            "raw_yaw_tolerance_deg",
            "timeout_sec",
            "timeoutSec",
        )
        for field in passthrough_fields:
            if field in payload:
                request[field] = payload[field]
        explicit_mode = any(
            key in payload
            for key in (
                "raw_cmd_vel",
                "rawCmdVel",
                "disable_obstacle_avoidance",
                "disableObstacleAvoidance",
                "direct_no_avoidance",
                "directNoAvoidance",
                "key_points_mode",
                "keyPointsMode",
                "navigation_mode",
            )
        )
        configured_mode = str(self.server.get_config().get("default_navigation_mode") or "normal")
        effective_mode = str(payload.get("navigation_mode") or "")
        if not explicit_mode:
            effective_mode = configured_mode

        if (
            payload.get("raw_cmd_vel")
            or payload.get("rawCmdVel")
            or effective_mode == "raw_cmd_vel_no_obstacle_avoidance"
        ):
            request["raw_cmd_vel"] = True
            request["disable_obstacle_avoidance"] = True
            request["navigation_mode"] = "raw_cmd_vel_no_obstacle_avoidance"
            request.setdefault("raw_linear_speed_mps", self.server.get_config().get("raw_linear_speed_mps", 0.35))
            request.setdefault("raw_angular_speed_radps", self.server.get_config().get("raw_angular_speed_radps", 1.2))
        elif (
            payload.get("direct_no_avoidance")
            or payload.get("directNoAvoidance")
            or effective_mode in ("direct_no_avoidance", "direct_key_points_stop_on_obstacle")
        ):
            request["direct_no_avoidance"] = True
            request["navigation_mode"] = "direct_key_points_stop_on_obstacle"
        if payload.get("dry_run"):
            request["dry_run"] = True
        upstream = self.upstream("POST", "/api/navigation/start", request)
        if upstream.get("ok"):
            self.server.set_compat_state(
                nav_state="14",
                task_status="全局路径规划成功",
                mode="navigation",
                last_goal=pose,
                last_nav_started_at=time.time(),
            )
            return self.success("全局路径规划成功", command=upstream.get("command"), upstream=upstream)
        self.server.set_compat_state(nav_state="15", task_status="全局路径规划失败")
        return self.failure("全局路径规划失败", upstream=upstream)

    def handle_navigation_start(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        waypoints = payload.get("waypoints") or payload.get("points")
        if not isinstance(waypoints, list) or not waypoints:
            return self.failure("waypoints is required")
        upstream = self.upstream("POST", "/api/navigation/start", payload)
        if upstream.get("ok"):
            goal = None
            if isinstance(waypoints[-1], dict):
                goal = waypoints[-1]
            self.server.set_compat_state(
                nav_state="14",
                task_status="全局路径规划成功",
                mode="navigation",
                last_goal=goal,
                last_nav_started_at=time.time(),
            )
            return self.success("全局路径规划成功", command=upstream.get("command"), upstream=upstream)
        self.server.set_compat_state(nav_state="15", task_status="全局路径规划失败")
        return self.failure("全局路径规划失败", upstream=upstream)

    def handle_mark_nav_point(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose, err = self.parse_pose_payload(payload)
        if err:
            return self.failure(err)
        assert pose is not None
        name = str(payload.get("inputValue") or payload.get("name") or "").strip()
        if not name or name == "null":
            name = f"Point {time.strftime('%Y-%m-%d %H:%M:%S')}"
        upstream = self.upstream(
            "POST",
            "/api/points/upsert",
            {
                "name": name,
                "x": pose["x"],
                "y": pose["y"],
                "yaw": pose["yaw"],
                "yaw_deg": pose["yaw_deg"],
                "note": "created by x_nav compatibility adapter",
                "actions": payload.get("actions", []),
            },
        )
        if upstream.get("ok"):
            return self.success("操作成功", point=upstream.get("point"), upstream=upstream)
        return self.failure("保存导航点位失败", upstream=upstream)

    def handle_set_pose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose, err = self.parse_pose_payload(payload)
        if err:
            return self.failure(err)
        assert pose is not None
        upstream = self.upstream(
            "POST",
            "/api/relocalization/run",
            {
                "anchor": {
                    "x": pose["x"],
                    "y": pose["y"],
                    "z": pose["z"],
                    "yaw": pose["yaw"],
                    "yaw_deg": pose["yaw_deg"],
                    "source": "x_nav_compat_set_pose",
                },
                "set_pose": True,
            },
        )
        if upstream.get("ok"):
            self.server.set_compat_state(slam_state="5#0.0", nav_state="-1", task_status="无任务")
            return self.success("操作成功", upstream=upstream)
        return self.failure("重定位初始化失败", upstream=upstream)

    def handle_add_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        upstream = self.upstream("POST", "/api/mapping/start", {"clear": payload.get("clear", True)})
        if upstream.get("ok"):
            map_name = str(payload.get("map_name") or "")
            self.server.set_compat_state(
                nav_state="-1",
                slam_state="0#0",
                master_state=f"1#{map_name}" if map_name else "1",
                mode="mapping",
                map_name=map_name,
                task_status="无任务",
            )
            return self.success("启动成功", map_name=payload.get("map_name"), upstream=upstream)
        return self.failure("启动建图失败", upstream=upstream)

    def handle_save_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        name = payload.get("map_name") or payload.get("name") or payload.get("package")
        upstream = self.upstream("POST", "/api/mapping/save", {"map_name": name})
        if upstream.get("ok"):
            map_name = strip_stcm(str(upstream.get("name") or name or ""))
            self.server.set_compat_state(nav_state=f"-4#{map_name}", slam_state=f"-4#{map_name}", map_name=map_name)
            return self.success("保存成功", map_name=map_name, upstream=upstream)
        return self.failure("保存地图失败", upstream=upstream)

    def handle_switch_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        name = payload.get("map_name") or payload.get("package") or payload.get("name")
        if not str(name or "").strip():
            return self.failure("map_name is required")
        upstream = self.upstream("POST", "/api/mapping/load", {"map_name": name})
        if upstream.get("ok"):
            map_name = strip_stcm(str(upstream.get("name") or name))
            self.server.set_compat_state(
                nav_state="-2",
                slam_state="0#1",
                master_state=f"2#{map_name}",
                mode="navigation",
                map_name=map_name,
                task_status="无任务",
            )
            return self.success(f"地图  {map_name} 切换成功", map_name=map_name, upstream=upstream)
        return self.failure("地图切换失败", upstream=upstream)

    def cancel_navigation(self, msg: str) -> Dict[str, Any]:
        upstream = self.upstream("POST", "/api/navigation/cancel", {})
        if upstream.get("ok"):
            self.server.set_compat_state(nav_state="-1", task_status="无任务")
            return self.success(msg, upstream=upstream)
        return self.failure("停止导航失败", upstream=upstream)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="x_nav HTTP compatibility adapter for G1D_SLAM.")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--upstream", default="http://127.0.0.1:18083")
    parser.add_argument("--upstream-timeout-sec", type=float, default=3.0)
    parser.add_argument("--page-size", type=int, default=10)
    parser.add_argument("--request-log-path", default="data/x_nav_compat_requests.jsonl")
    parser.add_argument(
        "--default-navigation-mode",
        default="normal",
        choices=["normal", "direct_no_avoidance", "raw_cmd_vel_no_obstacle_avoidance"],
        help="Mode used for x_nav nav_custom requests that do not explicitly specify an avoidance mode.",
    )
    parser.add_argument("--raw-linear-speed-mps", type=float, default=0.35)
    parser.add_argument("--raw-angular-speed-radps", type=float, default=1.2)
    parser.add_argument("--arrival-distance-m", type=float, default=0.18)
    parser.add_argument("--arrival-yaw-deg", type=float, default=4.0)
    parser.add_argument("--odom-stale-sec", type=float, default=2.0)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    server = CompatServer((args.bind, args.port), XNavCompatHandler, args)
    print(
        f"x_nav compatibility adapter listening on http://{args.bind}:{args.port}, "
        f"forwarding to {args.upstream}; request_log={args.request_log_path or 'disabled'}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
